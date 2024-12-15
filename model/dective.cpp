#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

// 全局变量
ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out_with_normals(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_icp_with_normals(new pcl::PointCloud<pcl::PointNormal>);
pcl::VoxelGrid<pcl::PointXYZ> grid;
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
pcl::visualization::CloudViewer viewer("Cloud Viewer");

// 目标点云，用于匹配
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::fromROSMsg(*msg, *cloud_in);
    cloud_in->header.frame_id = msg->header.frame_id;
    cloud_in->header.stamp = msg->header.stamp;
    std::cout << "Received a point cloud with " << cloud_in->size() << " data points." << std::endl;

    // Voxel Grid Downsampling
    grid.setInputCloud(cloud_in);
    grid.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    grid.filter(*cloud_filtered);
    std::cout << "Filtered cloud contains " << cloud_filtered->size() << " data points." << std::endl;

    // PassThrough Filter
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0f, 1.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_pass);
    std::cout << "PassThrough filtered cloud contains " << cloud_pass->size() << " data points." << std::endl;

    // Normal Estimation
    ne.setInputCloud(cloud_pass);
    ne.setSearchMethod(tree);
    ne.setKSearch(50);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);
    std::cout << "Estimated " << cloud_normals->size() << " normals." << std::endl;

    // ICP
    cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_pass, *cloud_normals, *cloud_with_normals);
    cloud_icp_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    icp.setInputSource(cloud_with_normals);
    icp.setInputTarget(cloud_out_with_normals); // 使用目标点云
    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(0.01f);
    icp.setRANSACIterations(1000);
    icp.setTransformationEstimation(pcl::registration::TransformationEstimationPointToPoint(false));
    icp.align(*cloud_icp_with_normals);
    std::cout << "Has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

    // Publish ICP result
    pcl::toROSMsg(*cloud_icp_with_normals, cloud_out->header, *cloud_out);
    cloud_out->header.frame_id = msg->header.frame_id;
    cloud_out->header.stamp = msg->header.stamp;
    pub.publish(cloud_out);

    // Visualize
    viewer.showCloud(cloud_icp_with_normals);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_matching");
    ros::NodeHandle nh;

    // 读取PCD文件
    std::string pcd_file_path = "/path/to/your/file.pcd"; // 替换为目标PCD文件路径
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *target_cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    // 将目标点云转为带法向量的点云以进行ICP
    pcl::copyPointCloud(*target_cloud, *cloud_out);
    // 这里可以添加法向量计算，如有需要

    // 订阅和发布
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 1);
    ros::spin();
    return 0;
}
