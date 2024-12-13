//在ros中读取3d点云pcd文件，在从话题中读取点云数据，让pcd去匹配从话题中读取的点云数据，并将匹配结果发布到新的话题中。
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

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out_with_normals;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_icp_with_normals;
pcl::VoxelGrid<pcl::PointXYZ> grid;
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
pcl::visualization::CloudViewer viewer("Cloud Viewer");

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *cloud_in);
    cloud_in->header.frame_id = msg->header.frame_id;
    cloud_in->header.stamp = msg->header.stamp;
    std::cout << "Received a point cloud with " << cloud_in->size() << " data points." << std::endl;

    // 1. Voxel Grid Downsampling
    grid.setInputCloud(cloud_in);
    grid.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    grid.filter(*cloud_filtered);
    std::cout << "Filtered cloud contains " << cloud_filtered->size() << " data points." << std::endl;

    // 2. PassThrough Filter
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0f, 1.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_pass);
    std::cout << "PassThrough filtered cloud contains " << cloud_pass->size() << " data points." << std::endl;

    // 3. Normal Estimation
    ne.setInputCloud(cloud_pass);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setKSearch(50);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);
    std::cout << "Estimated " << cloud_normals->size() << " normals." << std::endl;

    // 4. ICP
    cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_pass, *cloud_normals, *cloud_with_normals);
    cloud_out_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    cloud_icp_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    icp.setInputSource(cloud_with_normals);
    icp.setInputTarget(cloud_out_with_normals);
    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(0.01f);
    icp.setRANSACIterations(100000);
    icp.setTransformationEstimation(pcl::registration::TransformationEstimationPointToPoint(false));
    icp.align(*cloud_icp_with_normals);
    std::cout << "Has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

    // 5. Publish ICP result
    pcl::toROSMsg(*cloud_icp_with_normals, cloud_out->header, *cloud_out);
    cloud_out->header.frame_id = msg->header.frame_id;
    cloud_out->header.stamp = msg->header.stamp;
    pub.publish(cloud_out);

    // 6. Visualize
    viewer.showCloud(cloud_icp_with_normals);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_matching");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 1);
    ros::spin();
    return 0;
}