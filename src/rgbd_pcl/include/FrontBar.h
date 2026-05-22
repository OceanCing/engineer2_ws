#pragma once

#include "VisionExchanger_Base.h"
#include <ros/callback_queue.h>
#include <thread>
#include <atomic>
#include <dynamic_reconfigure/server.h>
#include <rgbd_pcl/SideBarCfgConfig.h>

namespace vision_exchanger
{
    class FrontBar{
    public:
        DetectStatus status = DetectStatus::NONE;
        bool is_pnp_success = false;
        bool is_detection_success = false;
        //这里这么写是为了让这个变量跳过编译器优化，否则oninit中循环检测get_new_frame的那个while(1)会被优化成死循环，永远无法接收到下一帧。
        std::atomic<bool> get_new_frame{false};


        image_transport::CameraSubscriber Camera_sub;

        FrontBar(const ros::NodeHandle& nh);
        ~FrontBar();

        void PreProcess();
        void Detection();
        void get_NormalLine();
        void PoseSolver();
        void SendTF();
        void draw_debug_img();

        //初始化并启动订阅
        void initialization();
        void SC_CB(const sensor_msgs::ImageConstPtr& image_msg,
                            const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

        void SideBarDynamicReconfigure(rgbd_pcl::SideBarCfgConfig &config, uint32_t level);

        std::shared_ptr<image_transport::ImageTransport> it;

    private:
        ros::NodeHandle nh_;
        cv_bridge::CvImageConstPtr cv_ptr;

        // 独立回调队列，让SC_CB在单独线程跑
        ros::CallbackQueue sc_queue_;
        std::shared_ptr<std::thread> sc_spinner_thread_;
        std::atomic<bool> sc_spinner_running_{false};

        image_transport::Publisher debugimage_pub;

        cv::Mat raw_img;
        cv::Mat debug_img;
        cv::Mat hsv;
        cv::Mat mask;
        cv::Mat morp;

        cv::Mat_<double> cam_front_K;
        std::vector<double> cam_front_D;

        cv::Matx<double, 4, 2> points_2d_mat_;
        cv::Matx<double, 4, 3> points_3d_mat_;
        cv::Mat_<double> r_vec,t_vec;

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> select_bar_contours;
        double select_bar_area = 0;

        //tags
        int g_debug_view = 0 ;
    };
}

