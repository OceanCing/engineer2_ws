//
// Created by hiling on 2026/2/4.
//
#include "AssemblyPost.h"



void vision_exchanger::AssemblyPost::onInit() {
    ros::NodeHandle& nh = this->getNodeHandle();
    ros::NodeHandle& pnh = this->getPrivateNodeHandle();

    //-----------------------------------------深度相机 Init---------------------------------------------
    //构建对象
    sidebar_ptr = std::make_shared<vision_exchanger::SideBar>(nh);

    //SideBar动态调参服务器（使用私有NodeHandle，这样rqt_reconfigure可以找到）
    dr_server_ = std::make_shared<dynamic_reconfigure::Server<rgbd_pcl::SideBarCfgConfig>>(pnh);
    dynamic_reconfigure::Server<rgbd_pcl::SideBarCfgConfig>::CallbackType sidebar_reconfigure_cb;
    sidebar_reconfigure_cb = boost::bind(&vision_exchanger::SideBar::SideBarDynamicReconfigure, sidebar_ptr.get(), boost::placeholders::_1, boost::placeholders::_2);
    dr_server_->setCallback(sidebar_reconfigure_cb);

    //server callback
    SideBarService = nh.advertiseService("RequestInfo",&vision_exchanger::AssemblyPost::SideBarServiceCB,this);
    ROS_WARN("start Sidebar service!");

    //-----------------------测试服务代码-----------------------
    sleep(2);
    ros::ServiceClient client = nh.serviceClient<rgbd_pcl::RequestInfo>("RequestInfo");
    ros::service::waitForService("RequestInfo");
    rgbd_pcl::RequestInfo request;
    request.request.request_type = 2;
    bool ret = client.call(request);

    //连续debug模式 记得关掉shutdown
    while (1) {
        ret = client.call(request);
        //1hz debug模式
        // sleep(1);
    }

//-----------------------------------------臂上小相机 Init---------------------------------------------
    frontbar_ptr = std::make_shared<vision_exchanger::FrontBar>(nh);
    // frontbar_ptr->
}

//server callback
//接收到server后，调用A

// SideBarServiceCB
bool vision_exchanger::AssemblyPost::SideBarServiceCB(rgbd_pcl::RequestInfo::Request& req,
          rgbd_pcl::RequestInfo::Response& res) {
    ROS_INFO("Into SideBarServiceCB...");
    bool is_sueccess = true;
//!!!改！
    //tradition
    this->status = DetectStatus::FC_SideBar;
    sidebar_ptr->status = DetectStatus::FC_SideBar;
    // ROS_WARN("into service callback");

    sidebar_ptr->initialization();
//！！写一个连续debug的模式，持续生效cb函数而不是每次调用后释放！

    //等待相机图像，超时3秒判断为硬件问题
    ros::Time start_time = ros::Time::now();
    const double timeout_sec = 3.0;
    while (!sidebar_ptr->get_new_frame.load()) {
        if ((ros::Time::now() - start_time).toSec() > timeout_sec) {
            ROS_WARN("Camera timeout! No image received within %.1f seconds. Check camera.", timeout_sec);
            this->status = DetectStatus::NONE;
            sidebar_ptr->status = DetectStatus::NONE;
            sidebar_ptr->FC_sub.shutdown();
            return false;
        }
        ros::Duration(0.01).sleep();  // 10ms轮询间隔，避免CPU占用过高
    }
    sidebar_ptr->get_new_frame.store(false);

//！！！加入preprocess不成功的判断，避免程序褒姒！++++++++++++++

    sidebar_ptr->PreProcess();
    sidebar_ptr->Detection();
    if (!sidebar_ptr->is_detection_success) {
        ROS_WARN("SideBar detection empty!");
        is_sueccess = false;
    } else {
        sidebar_ptr->PoseSolver();
        if (!sidebar_ptr->is_pnp_success) {
            ROS_WARN("SideBar PnP failed");
            is_sueccess = false;
        }
    }
    if (is_sueccess) {
        sidebar_ptr->SendTF();
    }


    sidebar_ptr->draw_debug_img();
    this->status = DetectStatus::NONE;
    sidebar_ptr->status = DetectStatus::NONE;
    // sidebar_ptr->FC_sub.shutdown();
    return is_sueccess;


 //RGBD Camera
// ROS_WARN("INTO SERVER!");
     return true;

 }


bool vision_exchanger::AssemblyPost::FrontBarServiceCB(rgbd_pcl::RequestInfo::Request& req,
          rgbd_pcl::RequestInfo::Response& res) {
    ROS_INFO("Into FrontBarServiceCB...");
    bool is_sueccess = true;
    //!!!改！
    //tradition
    this->status = DetectStatus::SC_FrontBar;
    frontbar_ptr->status = DetectStatus::SC_FrontBar;
    // ROS_WARN("into service callback");

    frontbar_ptr->initialization();
    //！！写一个连续debug的模式，持续生效cb函数而不是每次调用后释放！

    //等待相机图像，超时3秒判断为硬件问题
    ros::Time start_time = ros::Time::now();
    const double timeout_sec = 3.0;
    while (!frontbar_ptr->get_new_frame.load()) {
        if ((ros::Time::now() - start_time).toSec() > timeout_sec) {
            ROS_WARN("Camera timeout! No image received within %.1f seconds. Check camera.", timeout_sec);
            this->status = DetectStatus::NONE;
            frontbar_ptr->status = DetectStatus::NONE;
            frontbar_ptr->Camera_sub.shutdown();
            return false;
        }
        ros::Duration(0.01).sleep();  // 10ms轮询间隔，避免CPU占用过高
    }
    frontbar_ptr->get_new_frame.store(false);

    //！！！加入preprocess不成功的判断，避免程序褒姒！++++++++++++++

    frontbar_ptr->PreProcess();
    frontbar_ptr->Detection();
    // if (!frontbar_ptr->is_detection_success) {
    //     ROS_WARN("SideBar detection empty!");
    //     is_sueccess = false;
    // } else {
    //     frontbar_ptr->PoseSolver();
    //     if (!frontbar_ptr->is_pnp_success) {
    //         ROS_WARN("SideBar PnP failed");
    //         is_sueccess = false;
    //     }
    // }
    // if (is_sueccess) {
    //     frontbar_ptr->SendTF();
    // }


    // frontbar_ptr->draw_debug_img();
    this->status = DetectStatus::NONE;
    frontbar_ptr->status = DetectStatus::NONE;
    // frontbar_ptr->FC_sub.shutdown();
    return is_sueccess;


    //RGBD Camera
    // ROS_WARN("INTO SERVER!");
    return true;
}
