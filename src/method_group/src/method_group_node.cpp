#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

int main(int argc, char** argv) {
    ros::init(argc, argv, "method_group_node");
    ros::NodeHandle nh;

    // Путь к PCD файлу
    std::string filename = "/home/stud/Desktop/project/build/milk_cartoon_all_small_clorox.pcd";

    PointCloud::Ptr cloud(new PointCloud);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1) { // load the file
        PCL_ERROR("Couldn't read file %s\n", filename.c_str());
        return -1;
    }

    ROS_INFO("Loaded %lu data points from %s", cloud->points.size(), filename.c_str());

    // Фильтр PassThrough по оси Z
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.6, 1.1);  // Установите нужные пределы фильтрации по оси Z здесь
    pass.filter(*cloud);




    ROS_INFO("Filtered cloud has %lu data points", cloud->points.size());

    // Преобразование в sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map"; // Установите нужный frame_id

    // Публикация облака точек
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
