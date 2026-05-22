#include "FrontBar.h"

// HSV 阈值（可通过 ROS 私有参数覆盖：~h_min ~s_min ~v_min ~h_max ~s_max ~v_max）
static int g_h_min = 103;
static int g_s_min = 140;
static int g_v_min = 73;
static int g_h_max = 179;
static int g_s_max = 255;
static int g_v_max = 255;


// 0=raw, 1=morphology, 2=Contours, 3=Contours_select

void vision_exchanger::FrontBar::SideBarDynamicReconfigure(rgbd_pcl::SideBarCfgConfig &config, uint32_t level) {
    // level 未使用也没关系，但显式忽略可避免部分编译器告警
    (void)level;
    // g_h_min = config.h_min;
    // g_h_max = config.h_max;
    // g_s_min = config.s_min;
    // g_s_max = config.s_max;
    // g_v_min = config.v_min;
    // g_v_max = config.v_max;
    g_debug_view = config.debug_view;
    ROS_INFO("Dynamic reconfigure updated: debug_view=%d", g_debug_view);
}

void vision_exchanger::FrontBar::SC_CB(const sensor_msgs::ImageConstPtr& image_msg,
                            const sensor_msgs::CameraInfoConstPtr& cam_info_msg) {
    // //触发识别侧灯条的图像和图像信息传递
    // while (!image_msg->data.empty()) {
    //     ROS_WARN("waiting cam");
    // }

    cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::RGB8);
    memcpy(cam_front_K.data,cam_info_msg->K.data(),9 * sizeof(double));
    cam_front_D= cam_info_msg->D;
    get_new_frame.store(true);

}

//构造函数
vision_exchanger::FrontBar::FrontBar(const ros::NodeHandle &nh):nh_(nh) {
    // 设置独立回调队列
    nh_.setCallbackQueue(&sc_queue_);

    // 启动独立线程处理 sc_queue_
    sc_spinner_running_ = true;
    sc_spinner_thread_ = std::make_shared<std::thread>([this]() {
        while (sc_spinner_running_ && ros::ok()) {
            sc_queue_.callAvailable(ros::WallDuration(0.01));
        }
    });

    it=std::make_shared<image_transport::ImageTransport>(nh_);
    // points_3d_mat_ ={-SIDEBAR_HEIGBT/2,SIDEBAR_WIDTH/2,0,
    //                     -SIDEBAR_HEIGBT/2,-SIDEBAR_WIDTH/2,0,
    //                     SIDEBAR_HEIGBT/2,-SIDEBAR_WIDTH/2,0,
    //                     SIDEBAR_HEIGBT/2,SIDEBAR_WIDTH/2,0};

    cam_front_K.create(3, 3);
    r_vec(3, 1),t_vec(3, 1);
    //!!!可能有错！
    // cv::Mat_<double> r_vec(3, 1),
    // t_vec(3, 1);
    debugimage_pub = it->advertise("/debug_image_frontbar", 1);


}

vision_exchanger::FrontBar::~FrontBar() {
    sc_spinner_running_ = false;
    if (sc_spinner_thread_ && sc_spinner_thread_->joinable()) {
        sc_spinner_thread_->join();
    }
}

void vision_exchanger::FrontBar::PreProcess() {

    ROS_INFO("PreProcess start");
    //转HSV
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_RGB2HSV);

    //二值化
    cv::inRange(hsv,
                cv::Scalar(g_h_min, g_s_min, g_v_min),
                cv::Scalar(g_h_max, g_s_max, g_v_max),
                mask);
    //形态学操作
    cv::morphologyEx(mask, morp, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)),
                     cv::Point(-1,-1), 3);

}

void vision_exchanger::FrontBar::Detection() {

    cv::findContours(morp,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    //四个角标
    std::vector<std::vector<cv::Point>> right_angle_contours;

    //找角标
    //筛选步骤：
    /*
        轮廓近似
        轮廓边数
        轮廓面积
        是否为凹多边形

     */
    for (const auto & contour : contours) {
        std::vector<cv::Point> after_approxPolyDP;
        //轮廓近似
        cv::approxPolyDP(contour,after_approxPolyDP,0.05*cv::arcLength(contour,true),1);
        // convexHull(contour,after_approxPolyDP,true,false);
        double this_area = contourArea(after_approxPolyDP);
        if (after_approxPolyDP.size() == 6 && this_area >= 10) {
            if  (!cv::isContourConvex(after_approxPolyDP)) {
                right_angle_contours.push_back(after_approxPolyDP);
            }
        }
        cv::Mat temp_debug_img = cv_ptr->image.clone();

        if (!contours.empty()) {
            cv::drawContours(temp_debug_img, right_angle_contours, -1, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("right_angle_contours", temp_debug_img);
        cv::waitKey(1);
        // if (after_approxPolyDP.size() == 4 && this_area >= 3000) {
        //     // 先规范点的顺序，因为不规范无法进行后面的判断：0-上左，1-下左，2-下右，3-上右
        //     if (cv::norm(after_approxPolyDP[0]-after_approxPolyDP[1]) > cv::norm(after_approxPolyDP[0]-after_approxPolyDP[3])) {
        //         cv::Point temp = after_approxPolyDP[0];
        //         after_approxPolyDP[0] = after_approxPolyDP[1];
        //         after_approxPolyDP[1] = after_approxPolyDP[2];
        //         after_approxPolyDP[2] = after_approxPolyDP[3];
        //         after_approxPolyDP[3] = temp;
        //     }
        // } else
        //     continue;

    }
    //for end



    if (select_bar_contours.empty())
        is_detection_success = false;
    else
        is_detection_success = true;
}

void vision_exchanger::FrontBar::PoseSolver() {
    //将轮廓填入points_2d_mat_
    for (int i = 0; i < 4; ++i) {
        points_2d_mat_(i, 0) = static_cast<double>(select_bar_contours[i].x);
        points_2d_mat_(i, 1) = static_cast<double>(select_bar_contours[i].y);
    }
    //!!!判断pnp是否成功！
    //SOLVEPNP_IPPE

    is_pnp_success = cv::solvePnP(points_3d_mat_,points_2d_mat_,
       cam_front_K,cam_front_D,
       r_vec,t_vec,false,cv::SOLVEPNP_IPPE);



    //________________________________________



}
//PoseSolver

void vision_exchanger::FrontBar::SendTF() {
    // 1. 创建广播器对象（通常设为 static 以避免每次调用函数都重新创建连接）
    static tf2_ros::TransformBroadcaster sidebar_broadcaster;

    // 2. 创建一个变换Stamped消息对象，用来填充数据
    geometry_msgs::TransformStamped sidebar_transformStamped;

    // 3. 设置时间戳，使用当前时间，表示这个变换此刻有效
    sidebar_transformStamped.header.stamp = ros::Time::now();

    // 4. 设置父坐标系名字，即你想依附的那个 Frame
    sidebar_transformStamped.header.frame_id = "gemini_336l_color_optical_frame";

    // 5. 设置子坐标系名字，即你新加入的 Frame
    sidebar_transformStamped.child_frame_id = "sidebar";


     // 6. 设置平移 (Translation)：相机相对于机器底座的 x, y, z 距离（单位：米）
    sidebar_transformStamped.transform.translation.x = t_vec.at<double>(0);
    sidebar_transformStamped.transform.translation.y = t_vec.at<double>(1);
    sidebar_transformStamped.transform.translation.z = t_vec.at<double>(2);

//-----------------------------------------
    // 4. 处理旋转 (Rotation)
    // 4.1 将 OpenCV 的旋转向量 r_vec 转换为 3x3 旋转矩阵
    cv::Mat R_mat;
    cv::Rodrigues(r_vec, R_mat);

    // 4.2 将 OpenCV 矩阵转换为 tf2::Matrix3x3
    tf2::Matrix3x3 tf2_rot(
        R_mat.at<double>(0, 0), R_mat.at<double>(0, 1), R_mat.at<double>(0, 2),
        R_mat.at<double>(1, 0), R_mat.at<double>(1, 1), R_mat.at<double>(1, 2),
        R_mat.at<double>(2, 0), R_mat.at<double>(2, 1), R_mat.at<double>(2, 2)
    );

    // 4.3 将矩阵转换为四元数
    tf2::Quaternion tf2_quat;
    tf2_rot.getRotation(tf2_quat);

    // 4.4 填充到消息中
    sidebar_transformStamped.transform.rotation.x = tf2_quat.x();
    sidebar_transformStamped.transform.rotation.y = tf2_quat.y();
    sidebar_transformStamped.transform.rotation.z = tf2_quat.z();
    sidebar_transformStamped.transform.rotation.w = tf2_quat.w();
//-----------------------------------------
    //
    // sidebar_transformStamped.transform.translation.x = 0;
    // sidebar_transformStamped.transform.translation.y = 0;
    // sidebar_transformStamped.transform.translation.z = 0.0; // 二维实现，pose 中没有z，z 是 0
    // //  |--------- 四元数设置
    // tf2::Quaternion qtn;
    // qtn.setRPY(0,0,3);
    // sidebar_transformStamped.transform.rotation.x = qtn.getX();
    // sidebar_transformStamped.transform.rotation.y = qtn.getY();
    // sidebar_transformStamped.transform.rotation.z = qtn.getZ();
    // sidebar_transformStamped.transform.rotation.w = qtn.getW();
    //
    //

    // 9. 发送变换
    sidebar_broadcaster.sendTransform(sidebar_transformStamped);

}

void vision_exchanger::FrontBar::draw_debug_img() {
    cv::Mat output_img = cv_ptr->image.clone();
    std::string out_encoding = sensor_msgs::image_encodings::RGB8;;

//!!!改成可以根据选择的类型判断使用哪个draw
    if (g_debug_view == 0) {
        // raw

    } else if (g_debug_view == 1) {
        // morphology
        output_img = morp;
        out_encoding = sensor_msgs::image_encodings::MONO8;

    } else if (g_debug_view == 2) {
        // Contours：绘制 findContours 得到的所有轮廓
        if (!contours.empty()) {
            cv::drawContours(output_img, contours, -1, cv::Scalar(0, 255, 0), 2);
        }

    } else if (g_debug_view == 3) {
        // Contours_select：仅绘制 select_bar_contours
        if (!select_bar_contours.empty()) {
            cv::drawContours(output_img, std::vector<std::vector<cv::Point>>{select_bar_contours}, -1,
                             cv::Scalar(0, 255, 0), 2);
            for (size_t i = 0; i < select_bar_contours.size(); ++i) {
                const cv::Point &pt = select_bar_contours[i];
                cv::putText(output_img,
                            std::to_string(i),
                            pt,
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            cv::Scalar(0, 255, 0),
                            2);
            }
        }

    } else if (g_debug_view == 4) {
        if (!select_bar_contours.empty()) {
            std::vector<cv::Point3f> xyz_define;//定义坐标轴
            std::vector<cv::Point2f> xyz_img;//定义图像中点

            xyz_define.emplace_back(0,0,0);
            xyz_define.emplace_back(100,0,0);
            xyz_define.emplace_back(0,100,0);
            xyz_define.emplace_back(0,0,100);
            cv::projectPoints(xyz_define,r_vec,t_vec,cam_front_K,cam_front_D,xyz_img);
            cv::line(output_img,xyz_img[0],xyz_img[1],cv::Scalar(0,0,255),3);
            cv::line(output_img,xyz_img[0],xyz_img[2],cv::Scalar(0,255,0),3);
            cv::line(output_img,xyz_img[0],xyz_img[3],cv::Scalar(255,0,0),3);

            cv::drawContours(output_img, std::vector<std::vector<cv::Point>>{select_bar_contours}, -1,
                 cv::Scalar(0, 255, 0), 2);
            for (size_t i = 0; i < select_bar_contours.size(); ++i) {
                const cv::Point &pt = select_bar_contours[i];
                cv::putText(output_img,
                            std::to_string(i),
                            pt,
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.7,
                            cv::Scalar(0, 255, 0),
                            2);
            }
        }
    }

    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(cv_ptr->header,
                                                       out_encoding,
                                                       output_img).toImageMsg();
    debugimage_pub.publish(out_msg);
    //-------------------------------------
}


void vision_exchanger::FrontBar::initialization() {

    // raw_img.release();
    // debug_img.release();
    // hsv.release();
    // mask.release();
    // morp.release();

    contours.clear();
    select_bar_contours.clear();
    select_bar_area = 0;

    is_detection_success = false;
    is_pnp_success = false;

    r_vec = cv::Mat::zeros(3, 1, CV_64F);
    t_vec = cv::Mat::zeros(3, 1, CV_64F);

    //!!!可能有错！
    // cv::Mat_<double> r_vec(3, 1),
    // t_vec(3, 1);
    Camera_sub = it->subscribeCamera("/hk_camera/image_raw", 1, &FrontBar::SC_CB, this);
    // FC_sub = it->subscribeCamera("/hk_camera/image_raw", 1, &SideBar::FC_CB, this);
}
