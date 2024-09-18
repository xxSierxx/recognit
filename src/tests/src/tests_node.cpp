#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

ros::Subscriber cloud_sub;
ros::Publisher cloud_pub;
PointCloud::Ptr accumulated_cloud(new PointCloud); // Объединенное облако точек
int count = 0;
const int TOTAL_CLOUDS = 7; // Общее количество облаков в bag файле

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    ROS_INFO("Point cloud received with %lu points", cloud->points.size());

    *accumulated_cloud += *cloud;
    count++;

    // Публикация только при последнем облаке точек из bag файла
    if (count == TOTAL_CLOUDS) {
        ROS_INFO("All point clouds received. Processing...");

        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(accumulated_cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        PointCloud::Ptr filtered_cloud(new PointCloud);
        vg.filter(*filtered_cloud);

        // Фильтр по оси Z, чтобы убрать пол и потолок
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(filtered_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.7, 5); // Пределы фильтрации по оси Z
//        pass.setFilterLimits(-10, 10); // Пределы фильтрации по оси Z
        pass.filter(*filtered_cloud);
//0.69788 do 1.594
        // Сегментация плоскостей (стены)
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.01); // Пороговое значение для сегментации

        seg.setInputCloud(filtered_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            ROS_WARN("No planar model found.");
            return;
        }

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(filtered_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*filtered_cloud);

        // Преобразование обратно в sensor_msgs/PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header = cloud_msg->header;
        output.header.frame_id = "map"; 

        // Публикация отфильтрованного облака точек
        cloud_pub.publish(output);

        ROS_INFO("Point cloud processed and published.");

        accumulated_cloud.reset(new PointCloud);
        count = 0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tests_node");
    ros::NodeHandle nh;

    cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);

    ros::spin();
    return 0;
}
