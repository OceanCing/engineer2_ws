#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/filters/passthrough.h>
#include <pcl-1.10/pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.10/pcl/filters/voxel_grid.h>
#include <pcl-1.10/pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl-1.10/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.10/pcl/sample_consensus/lmeds.h>
#include <pcl-1.10/pcl/features/normal_3d.h>
#include <pcl-1.10/pcl/sample_consensus/ransac.h>
#include <pcl-1.10/pcl/filters/extract_indices.h>
#include <pcl-1.10/pcl/filters/statistical_outlier_removal.h>


#include <sys/wait.h>
#include <dynamic_reconfigure/server.h>

#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>

#include <locale.h>


using namespace std;
using namespace cv;
using namespace pcl;

namespace rgbd_pcl {

class RecordPointCloudNodelet : public nodelet::Nodelet {
public:
    RecordPointCloudNodelet() = default;

private:
    void onInit() override {
        setlocale(LC_ALL, "");

        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        pnh.param("x_min", x_min_, -0.2);
        pnh.param("x_max", x_max_, 0.35);
        pnh.param("y_min", y_min_, -0.45);
        pnh.param("y_max", y_max_, 0.3);
        pnh.param("z_min", z_min_, 0.1);
        pnh.param("z_max", z_max_, 0.9);
        pnh.param("leaf", leaf_, 0.001);

        std::string input_topic;
        std::string output_topic;
        std::string marker_topic;
        pnh.param<std::string>("input_topic", input_topic, "/gemini_336l/depth/points");
        pnh.param<std::string>("output_topic", output_topic, "/rgbd_pcl/points");
        pnh.param<std::string>("marker_topic", marker_topic, "/rgbd_pcl/cylinder_marker");

        pointcloud_cylinder_pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 2);
        cylinder_info_pub_ = nh.advertise<visualization_msgs::Marker>(marker_topic, 1);
        pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 2, &RecordPointCloudNodelet::rgbcCb, this);

        NODELET_WARN("record_pointcloud nodelet init done");
    }

    void rgbcCb(const sensor_msgs::PointCloud2::ConstPtr& msg_p) {
        pcl::PCLPointCloud2 cloud_after_preprocess;

        pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*msg_p, *pcl_pc2);

        pcl::PCLPointCloud2ConstPtr cloud_raw(pcl_pc2);
        pcl::PCLPointCloud2 cloud_pass_filtered;

        // 创建三个方向的直通滤波器，按 X -> Y -> Z 串联
        pcl::PCLPointCloud2::Ptr cloud_after_x(new pcl::PCLPointCloud2);
        pcl::PCLPointCloud2::Ptr cloud_after_xy(new pcl::PCLPointCloud2);

        pcl::PassThrough<pcl::PCLPointCloud2> pass_x;
        pcl::PassThrough<pcl::PCLPointCloud2> pass_y;
        pcl::PassThrough<pcl::PCLPointCloud2> pass_z;

        // X 方向滤波
        pass_x.setInputCloud(cloud_raw);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min_, x_max_);
        pass_x.filter(*cloud_after_x);

        // Y 方向滤波（以 X 方向滤波结果为输入）
        pass_y.setInputCloud(cloud_after_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min_, y_max_);
        pass_y.filter(*cloud_after_xy);

        // Z 方向滤波（以 X+Y 结果为输入）
        pass_z.setInputCloud(cloud_after_xy);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_min_, z_max_);
        pass_z.filter(cloud_pass_filtered);

        // 体素滤波
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
        pcl::PCLPointCloud2::Ptr sor_cloud = boost::make_shared<pcl::PCLPointCloud2>(cloud_pass_filtered);
        voxel_grid.setInputCloud(sor_cloud);
        voxel_grid.setLeafSize(leaf_, leaf_, leaf_);
        pcl::PCLPointCloud2::Ptr cloud_after_Voxl(new pcl::PCLPointCloud2);
        voxel_grid.filter(*cloud_after_Voxl);

        cloud_after_preprocess = *cloud_after_Voxl;

        sensor_msgs::PointCloud2 out_msg;
        pcl_conversions::fromPCL(cloud_after_preprocess, out_msg);
        out_msg.header = msg_p->header;
        pointcloud_cylinder_pub_.publish(out_msg);
    }

private:
    ros::Publisher pointcloud_cylinder_pub_;
    ros::Publisher cylinder_info_pub_;
    ros::Subscriber pointcloud_sub_;

    double x_min_ = -0.2;
    double x_max_ = 0.35;
    double y_min_ = -0.45;
    double y_max_ = 0.3;
    double z_min_ = 0.30;
    double z_max_ = 0.9;
    double leaf_ = 0.001;

    int sor_MeanK_ = 51;
    double sor_StddevMulThresh_ = 1.0;
    bool enable_sor_ = true;

    int cylinder_KSearch_num_ = 50;
    bool enable_cylinder_filter_ = true;
    int MaxIterations_ = 10000;
    int seg_Method_Type_ = 0;
    double cylinder_Radius_min_ = 0.01;
    double cylinder_Radius_max_ = 0.03;

    bool enable_visualization_ = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_cylinder_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::ModelCoefficients::Ptr coefficients_cylinder_{new pcl::ModelCoefficients};
    pcl::ModelCoefficients::Ptr pre_coefficients_cylinder_{new pcl::ModelCoefficients};
};

}  // namespace rgbd_pcl

PLUGINLIB_EXPORT_CLASS(rgbd_pcl::RecordPointCloudNodelet, nodelet::Nodelet)


