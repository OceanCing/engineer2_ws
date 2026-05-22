//
// Created by hiling on 2026/2/4.
//
#pragma once
#include "VisionExchanger_Base.h"
#include "SideBar.h"
#include "FrontBar.h"
#include "rgbd_pcl/RequestInfo.h"
#include <pluginlib/class_list_macros.h>

namespace vision_exchanger {
    // ros::NodeHandle nh;

    class AssemblyPost : public nodelet::Nodelet {
    public:
        // AssemblyPost() {};
        // ~AssemblyPost() {};
        DetectStatus status;
        void onInit() override;
        bool SideBarServiceCB(rgbd_pcl::RequestInfo::Request&,
          rgbd_pcl::RequestInfo::Response&);
        ros::ServiceServer SideBarService;

        bool FrontBarServiceCB(rgbd_pcl::RequestInfo::Request& req,
          rgbd_pcl::RequestInfo::Response& res);
        ros::ServiceServer FrontBarService;



    private:
        std::shared_ptr<vision_exchanger::SideBar> sidebar_ptr;
        std::shared_ptr<vision_exchanger::FrontBar> frontbar_ptr;
        std::shared_ptr<dynamic_reconfigure::Server<rgbd_pcl::SideBarCfgConfig>> dr_server_;
        ros::Publisher status_pub;
        cv::Mat debug_img;
        bool is_pnp_success = false;
        bool sidebar_debug = false;

    };
}