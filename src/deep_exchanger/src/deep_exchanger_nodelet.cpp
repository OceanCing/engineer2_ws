//
// Created by koonyu on 23-12-2.
//

#include "../include/deep_exchanger/deep_exchanger.h"
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

PLUGINLIB_EXPORT_CLASS(deep_exchanger::DeepExchanger, nodelet::Nodelet)

namespace deep_exchanger
{
    cv::Scalar DeepExchanger::lower4blue_{26, 80, 80};
    cv::Scalar DeepExchanger::upper4blue_{34, 255, 255};
    cv::Scalar DeepExchanger::lower4red_{78, 80, 80};
    cv::Scalar DeepExchanger::upper4red_{99, 255, 255};

    void DeepExchanger::onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        static ros::CallbackQueue my_queue;
        nh.setCallbackQueue(&my_queue);
        initialize(nh);
        my_thread_ = std::thread([](){
            ros::SingleThreadedSpinner spinner;
            spinner.spin(&my_queue);
        });
    }

    void DeepExchanger::initialize(ros::NodeHandle &nh)
    {
        std::string model_path = " ";
        nh_ = ros::NodeHandle(nh, "deep_exchanger");
        img_subscriber_= nh_.subscribe("/hk_camera/image_raw", 1, &DeepExchanger::receiveFromCam,this);
        tf_updated_subscriber_ = nh_.subscribe("/is_update_exchanger", 1, &DeepExchanger::receiveFromEng, this);
        binary_publisher_ = nh_.advertise<sensor_msgs::Image>("/exchanger_binary_publisher", 1);
        segmentation_publisher_ = nh_.advertise<sensor_msgs::Image>("/exchanger_segmentation_publisher", 1);
        camera_pose_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/camera_pose_publisher", 1);
        pnp_publisher_ = nh_.advertise<rm_msgs::ExchangerMsg>("/pnp_publisher", 1);

        trans_publisher_ = nh_.advertise<sensor_msgs::Image>("/exchanger_transform_publisher",1);
        hough_publisher_=nh.advertise<sensor_msgs::Image>("/exchanger_houghlines_publisher",1);

        callback_ = boost::bind(&DeepExchanger::dynamicCallback, this, _1);
        server_.setCallback(callback_);

//        camera_matrix_ = (cv::Mat_<float>(3, 3) << 1811.208049,    0.     ,  692.262792,
//                0.     , 1811.768707,  576.194205,
//                0.     ,    0.     ,    1.     );
        camera_matrix_ = (cv::Mat_<float>(3, 3) << 1196.21895,    0.     ,  720.33579,
                          0.     , 1195.29136,  542.55074,
                          0.     ,    0.     ,    1.     );
//        distortion_coefficients_=(cv::Mat_<float>(1,5) << -0.079091 ,0.108809 ,-0.000094, -0.000368, 0.000000);
        distortion_coefficients_=(cv::Mat_<float>(1,5) << -0.097805, 0.069704, -0.000301, -0.000453, 0.000000);

        nh.getParam("model_path", model_path);
        nh.getParam("red", target_is_red_);
        InferenceEngine::Core ie;
        InferenceEngine::CNNNetwork model = ie.ReadNetwork(model_path);
        // prepare input settings
        InferenceEngine::InputsDataMap inputs_map(model.getInputsInfo());
        input_name_ = inputs_map.begin()->first;
        InferenceEngine::InputInfo::Ptr input_info = inputs_map.begin()->second;
        // prepare output settings
        InferenceEngine::OutputsDataMap outputs_map(model.getOutputsInfo());
        for (auto &output_info : outputs_map) {
            output_info.second->setPrecision(InferenceEngine::Precision::FP32);
        }
        std::map<std::string, std::string> config = {
                {InferenceEngine::PluginConfigParams::KEY_PERF_COUNT,
                        InferenceEngine::PluginConfigParams::NO},
                {InferenceEngine::PluginConfigParams::KEY_CPU_BIND_THREAD,
                        InferenceEngine::PluginConfigParams::NUMA},
                {InferenceEngine::PluginConfigParams::KEY_CPU_THROUGHPUT_STREAMS,
                        InferenceEngine::PluginConfigParams::CPU_THROUGHPUT_NUMA},
                //            { InferenceEngine::PluginConfigParams::KEY_CPU_THREADS_NUM,
                //            "16" },
        };

        // get network
        network_ = ie.LoadNetwork(model, "CPU", config);
        infer_request_ = network_.CreateInferRequest();

        // Find the triangle on the side of the exchange mine
        cv::Mat temp_triangle=cv::imread(ros::package::getPath("deep_exchanger")+"/arrow.png",cv::IMREAD_GRAYSCALE);
        cv::Mat binary_1;
        cv::threshold(temp_triangle,binary_1,0, 255, CV_THRESH_BINARY_INV + cv::THRESH_OTSU);

        std::vector<std::vector<cv::Point>> tri_temp_contour;
        std::vector<cv::Point> tri_hull;
        cv::findContours(binary_1,tri_temp_contour,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        std::sort(tri_temp_contour.begin(), tri_temp_contour.end(),
                  [](const auto &v1, const auto &v2){ return cv::contourArea(v1) > cv::contourArea(v2);});

        cv::convexHull(tri_temp_contour[0],tri_hull, true);
        temp_triangle_hull_ = tri_hull;

        prev_msg_.pose.orientation.w = 1;
        static tf2_ros::TransformListener tfListener(tf_buffer_);
        tf_update_ = true;

        

        // select color
        if (target_is_red_==0)
        {
            color_enhance_ = DeepExchanger::colorEnhance<0>;
            convert_color_ = DeepExchanger::colorCvt<&DeepExchanger::lower4blue_, &DeepExchanger::upper4blue_>;
        }
        else
        {
            color_enhance_ = DeepExchanger::colorEnhance<2>;
            convert_color_ = DeepExchanger::colorCvt<&DeepExchanger::lower4red_, &DeepExchanger::upper4red_>;
        }

        std::cout<<"initialize successed!"<<std::endl;
    }

    void DeepExchanger::dynamicCallback(deep_exchanger::dynamicConfig &config)
    {
        *(lower4blue_.val)/* */= config.blue_lower_hsv_h;
        *(lower4blue_.val + 1) = config.blue_lower_hsv_s;
        *(lower4blue_.val + 2) = config.blue_lower_hsv_v;
        *(upper4blue_.val)/* */= config.blue_upper_hsv_h;
        *(upper4blue_.val + 1) = config.blue_upper_hsv_s;
        *(upper4blue_.val + 2) = config.blue_upper_hsv_v;

        *(lower4red_.val)/* */= config.red_lower_hsv_h;
        *(lower4red_.val + 1) = config.red_lower_hsv_s;
        *(lower4red_.val + 2) = config.red_lower_hsv_v;
        *(upper4red_.val)/* */= config.red_upper_hsv_h;
        *(upper4red_.val + 1) = config.red_upper_hsv_s;
        *(upper4red_.val + 2) = config.red_upper_hsv_v;

        morph_type_ = 2 * config.morph_type + 1;
        morph_iterations_ = config.morph_iterations;
        morph_size_ = config.morph_size;

        small_offset_ = config.small_offset;
        side_small_offset_ = config.side_small_offset;
//        std::cout<<side_small_offset_<<"----"<<-0.0995 - side_small_offset_<<std::endl;

        x_offset_ = config.x_offset;
        y_offset_ = config.y_offset;
        z_offset_ = config.z_offset;
        roll_offset_ = config.roll_offset;
        pitch_offset_ = config.pitch_offset;
        yaw_offset_ = config.yaw_offset;

        min_triangle_threshold_ = config.min_triangle_threshold;
        triangle_approx_epsilon_ = config.triangle_approx_epsilon;
        triangle_moment_bias_ = config.triangle_moment_bias;

        score_thresh_ = config.score_thresh;

        is_show_center_ = config.is_show_center;

        box_select_area_ = config.box_select_area;
        side_triangle_select_area_ = config.side_triangle_select_area;
        similarity_ = config.similarity;
        select_lineDist_ = config.select_lineDist;

        gamma_ = config.gamma;
        gamma_y_ = config.gamma_y;

        uchar *p = look_up_table_.ptr();
        for (int i = 0; i < 256; ++i)
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma_y_) * 255.0);
    }

    void DeepExchanger::receiveFromCam(const sensor_msgs::ImageConstPtr &image)
    {
        world_points_.clear();
        arrow_left_points.clear();
        arrow_right_points.clear();

        world_points_.reserve(4);
        world_points_.emplace_back(0.144,0.144,0);
        world_points_.emplace_back(0.144 + small_offset_,-0.144 - small_offset_,0);
        world_points_.emplace_back(-0.144,-0.144,0);
        world_points_.emplace_back(-0.144,0.144,0);

        tf_trans_points.reserve(3);
        tf_trans_points.emplace_back(0.144,0,0);
        tf_trans_points.emplace_back(0,0.144,0);
        tf_trans_points.emplace_back(00,0,0.144);

        arrow_left_points.reserve(4);
//        arrow_left_points.emplace_back(0.0605 - side_small_offset_,0,0);
//        arrow_left_points.emplace_back(-0.0395 - side_small_offset_,-0.100 - side_small_offset_,0);
//        arrow_left_points.emplace_back(0.0705,0,0);
//        arrow_left_points.emplace_back(-0.0395 - side_small_offset_,0.100 + side_small_offset_,0);
        arrow_left_points.emplace_back(-0.0950 - side_small_offset_,0,0);
        arrow_left_points.emplace_back(0.0005 - side_small_offset_,-0.0995 - side_small_offset_,0);
        arrow_left_points.emplace_back(0.0955,0,0);
        arrow_left_points.emplace_back(0.0005 - side_small_offset_,0.0995 + side_small_offset_,0);


        arrow_right_points.reserve(4);
//        arrow_right_points.emplace_back(-0.0605 + side_small_offset_,0,0);
//        arrow_right_points.emplace_back(0.0395 + side_small_offset_,0.100 + side_small_offset_,0);
//        arrow_right_points.emplace_back(-0.0705,0,0);
//        arrow_right_points.emplace_back(0.0395 + side_small_offset_,-0.100 - side_small_offset_,0);
        arrow_right_points.emplace_back(0.0950 + side_small_offset_,0,0);
        arrow_right_points.emplace_back(-0.0005 + side_small_offset_,0.0995 + side_small_offset_,0);
        arrow_right_points.emplace_back(-0.0955,0,0);
        arrow_right_points.emplace_back(-0.0005 + side_small_offset_,-0.0995 - side_small_offset_,0);


        cv_image_ = cv_bridge::toCvCopy(image, "bgr8");
        if (gamma_) cv::LUT(cv_image_->image, look_up_table_, cv_image_->image);
        cv::resize(cv_image_->image, cv_image_->image,
                   cv::Size(image_size_, image_size_));
        threading();
        segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , cv_image_->image).toImageMsg());
        ros::Duration(0.1).sleep();
    }

    void DeepExchanger::receiveFromEng(const std_msgs::BoolConstPtr &signal)
    {
        bool is_update = signal->data;
        tf_update_ = is_update;
    }
}