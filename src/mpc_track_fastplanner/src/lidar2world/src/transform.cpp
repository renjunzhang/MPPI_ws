#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher points_pub;
tf2_ros::Buffer tfBuffer;
laser_geometry::LaserProjection projector_;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in) {
    try {
        // 1. LaserScan 转 PointCloud2 (在 scan 的 frame 中)
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*scan_in, cloud);
        
        // 2. 转换到 odom frame
        geometry_msgs::TransformStamped transformStamped = 
            tfBuffer.lookupTransform("odom", cloud.header.frame_id, ros::Time(0));
        
        sensor_msgs::PointCloud2 transformed_cloud;
        tf2::doTransform(cloud, transformed_cloud, transformStamped);
        
        transformed_cloud.header.frame_id = "odom";
        transformed_cloud.header.stamp = ros::Time::now();
        points_pub.publish(transformed_cloud);
        
    } catch(tf2::TransformException &e) {
        ROS_WARN_THROTTLE(1.0, "[lidar2world] Failed to transform: %s", e.what());
    }
}

int main(int argc, char **argv) 
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "point_cloud_transform_node");
    ros::NodeHandle nh("~");

    // === 参数化配置（适配 hf_platform） ===
    std::string input_scan_topic, output_cloud_topic, target_frame;
    
    nh.param<std::string>("input_scan_topic", input_scan_topic, "/scan_full_filtered");
    nh.param<std::string>("output_cloud_topic", output_cloud_topic, "/point_cloud_map");
    nh.param<std::string>("target_frame", target_frame, "odom");
    
    ros::Subscriber scan_sub = nh.subscribe(input_scan_topic, 10, laserScanCallback);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>(output_cloud_topic, 10);

    tf2_ros::TransformListener tfListener(tfBuffer);
    
    ROS_INFO("=================================================");
    ROS_INFO("[lidar2world] Node started with configuration:");
    ROS_INFO("  input_scan: %s", input_scan_topic.c_str());
    ROS_INFO("  output_cloud: %s", output_cloud_topic.c_str());
    ROS_INFO("  target_frame: %s", target_frame.c_str());
    ROS_INFO("  Converting /scan_full_filtered (filtered LaserScan)");
    ROS_INFO("  No additional PCL filtering applied");
    ROS_INFO("  Waiting for TF: * -> odom");
    ROS_INFO("=================================================");
    
    ros::spin();

    return 0;

}