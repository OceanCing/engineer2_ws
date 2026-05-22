//
// Created by koonyu on 23-12-2.
//

#pragma once
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <future>
#include <chrono>
#include <ctime>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <exception>
#include <deep_exchanger/dynamicConfig.h>
#include <rm_msgs/ExchangerMsg.h>
#include <inference_engine.hpp>
#include <memory>
#include "../include/rm_cgal_tools_polygon_simplification_2.h"
#include "../include/deep_exchanger/lsd.h"

namespace deep_exchanger
{
    typedef struct BoxInfo {
        float x1;
        float y1;
        float x2;
        float y2;
        float x3;
        float y3;
        float x4;
        float y4;
        double score;
        int label;
    } BoxInfo;

    typedef struct HeadInfo {
        std::string cls_layer;
        std::string dis_layer;
        int stride;
    } HeadInfo;

    class DeepExchanger : public nodelet::Nodelet
    {
    public:
        DeepExchanger() = default;
        ~DeepExchanger() override
        {
            if (this->my_thread_.joinable())
                my_thread_.join();
        }
        void initialize(ros::NodeHandle &nh);
        void onInit() override;

        InferenceEngine::ExecutableNetwork network_;
        InferenceEngine::InferRequest infer_request_;
        std::string input_name_;
        std::vector<HeadInfo> heads_info_{
                // cls_pred|dis_pred|stride
                {"transpose_0.tmp_0", "transpose_1.tmp_0", 8},
                {"transpose_2.tmp_0", "transpose_3.tmp_0", 16},
                {"transpose_4.tmp_0", "transpose_5.tmp_0", 32},
                {"transpose_6.tmp_0", "transpose_7.tmp_0", 64},
        };

    private:
        template <const int Reserved>
        static void colorEnhance(cv::Mat& in_out_src)
        {
            static_assert(Reserved >= 0 && Reserved <= 2, "");
            cv::medianBlur(in_out_src, in_out_src, 3);
            cv::Size origin_size = in_out_src.size();
            cv::pyrDown(in_out_src, in_out_src);
            cv::resize(in_out_src, in_out_src, in_out_src.size() / 2, cv::InterpolationFlags::INTER_AREA);
            typedef cv::Vec3b Pixel;
            // Parallel execution using C++11 lambda
            in_out_src.forEach<Pixel>([](Pixel& px, __attribute__((unused)) const int* position) -> void {
                // min = min(b, g, r)
                uchar min = px(1) < px(2) ? px(1) : px(2);
                min = min < px(0) ? min : px(0);
                constexpr int a = (2 - Reserved) * (Reserved != 1);
                constexpr int b = (a > 0) && (Reserved != 1) ? a - 1 : 2 - (Reserved == 2);
                px(a) = 0;
                px(b) = 0;
                px(Reserved) -= min;
                // px -= Pixel(255, 255, min);  // do not use. Relatively slow because calling constructor everytime.
            });
            cv::bitwise_not(in_out_src, in_out_src);
            cv::resize(in_out_src, in_out_src, origin_size);
        }

        typedef cv::Scalar Scalar;
        template <Scalar* const LowerBoundary, Scalar* const UpperBoundary>
        static void colorCvt(cv::Mat& src, cv::Mat1b& sel)
        {
            cv::Mat3b hsv(src.size());
            cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, *LowerBoundary, *UpperBoundary, sel);
            cv::filter2D(sel, sel, sel.depth(), cv::Mat_<uchar>(cv::Size(3, 3), 1));
        }

        void(*color_enhance_)(cv::Mat&) = nullptr;
        void(*convert_color_)(cv::Mat&, cv::Mat1b&) = nullptr;

        void dynamicCallback(deep_exchanger::dynamicConfig& config);
        void receiveFromCam(const sensor_msgs::ImageConstPtr &image);
        void receiveFromEng(const std_msgs::BoolConstPtr &signal);
        void threading();
        void modelProcess(const cv::Mat &image);
        void cvProcess(const cv::Mat &image);
        void detect(cv::Mat image, double score_threshold);
        void preProcess(cv::Mat &image, InferenceEngine::Blob::Ptr &blob);
        void decodeInfer(const float *&cls_pred, const float *&dis_pred, int stride,
                         double threshold,
                         std::vector<std::vector<BoxInfo>> &results);
        BoxInfo disPred2Bbox(const float *&box_det, int label, double score, int x, int y, int stride);
        static void nms(std::vector<BoxInfo> &result);
        void drawBboxes(const cv::Mat &bgr, const std::vector<BoxInfo> &bboxes);
        void poseNonSensePnP();
        void getPnP(cv::Mat &rvec, cv::Mat &tvec);
        void quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);
        double square(double in);
        cv::Mat gammaTrans(cv::Mat& m_img, double gamma, int n_c);
        void selectboxes(const cv::Mat &bgr,
                         std::vector<BoxInfo> &bboxes);
        void Lsd_LineDetect(cv::Mat image, std::vector<cv::Vec4f> &lines);

        static Scalar upper4blue_;  // for blue, not its exact value
        static Scalar lower4blue_;  // for blue, not its exact value
        static Scalar upper4red_;   // for red, not its exact value
        static Scalar lower4red_;   // for red, not its exact valueqa

        std::thread my_thread_;
        ros::NodeHandle nh_;
        cv_bridge::CvImagePtr cv_image_;
        ros::Subscriber img_subscriber_;
        ros::Subscriber tf_updated_subscriber_;
        ros::Publisher binary_publisher_;
        ros::Publisher segmentation_publisher_;
        ros::Publisher camera_pose_publisher_;
        dynamic_reconfigure::Server<deep_exchanger::dynamicConfig> server_;
        dynamic_reconfigure::Server<deep_exchanger::dynamicConfig>::CallbackType callback_;

        cv::Mat camera_matrix_;
        cv::Mat distortion_coefficients_;

        std::vector<cv::Point> temp_triangle_hull_;

        int target_is_red_{};

        bool tf_update_{};
        bool is_show_center_{};
        bool shape_signal_{};
        bool pose_signal_{};
        bool direction_signal_{};

        rm_msgs::ExchangerMsg prev_msg_;
        ros::Publisher pnp_publisher_;
        tf2_ros::Buffer tf_buffer_;
        tf::TransformBroadcaster tf_broadcaster_;
        rm_msgs::ExchangerMsg exchange_msg_;

        std::vector<cv::Point3f> world_points_;
        std::vector<cv::Point3f> arrow_left_points;
        std::vector<cv::Point3f> arrow_right_points;
        std::vector<cv::Point2f> imagePoints_;
        std::vector<cv::Point3f> tf_trans_points;
        cv::Mat exchanger_rvec_;
        cv::Mat exchanger_tvec_;
        cv::Mat arrow_rvec_;
        cv::Mat arrow_tvec_;


        double small_offset_{};
        double side_small_offset_{};
        double x_offset_{};
        double y_offset_{};
        double z_offset_{};
        double roll_offset_{};
        double pitch_offset_{};
        double yaw_offset_{};

        int morph_type_;
        int morph_iterations_;
        int morph_size_;

        int box_select_area_;
        int side_triangle_select_area_;
        double similarity_;
        int select_lineDist_;

        int min_triangle_threshold_{};
        double triangle_approx_epsilon_{};
        double triangle_moment_bias_{};

        double score_thresh_{};
        int num_class_ = 2;
        int image_size_ = 416;
        std::vector<BoxInfo> box_result_vec_;
        std::vector<std::vector<cv::Point>> side_triangle_vec_;

        bool gamma_;
        double gamma_y_;

        cv::Mat look_up_table_ = cv::Mat::ones(1, 256, CV_8U);


        ros::Publisher trans_publisher_;
        ros::Publisher hough_publisher_;

    };
}