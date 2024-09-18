#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// Координаты многоугольника
struct Point2D {
    float x;
    float y;
};

std::vector<Point2D> polygon = {
    {-0.147f, 0.105f},
    {-0.178335f, 0.100313f},
    {-0.180756f, -0.0008f},
    {-0.148327f, -0.00171476f}
};

ros::Subscriber cloud_sub;
ros::Publisher cloud_pub;
PointCloud::Ptr accumulated_cloud(new PointCloud);
int count = 0;
const int TOTAL_CLOUDS = 1;

// Проверка, находится ли точка внутри многоугольника
bool isInsidePolygon(const pcl::PointXYZRGB& pt, const std::vector<Point2D>& polygon) {
    int i, j;
    bool result = false;
    for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if ((polygon[i].y > pt.y) != (polygon[j].y > pt.y) &&
            (pt.x < (polygon[j].x - polygon[i].x) * (pt.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)) {
            result = !result;
        }
    }
    return result;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    ROS_INFO("Point cloud received with %lu points", cloud->points.size());

    *accumulated_cloud += *cloud;
    count++;

    if (count == TOTAL_CLOUDS) {
        ROS_INFO("All point clouds received. Processing...");

        // Воксельный фильтр для уменьшения количества точек
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(accumulated_cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        PointCloud::Ptr filtered_cloud(new PointCloud);
        vg.filter(*filtered_cloud);

        // Фильтрация точек внутри многоугольника
        PointCloud::Ptr polygon_filtered_cloud(new PointCloud);
        for (const auto& point : filtered_cloud->points) {
            if (isInsidePolygon(point, polygon)) {
                polygon_filtered_cloud->points.push_back(point);
            }
        }

        polygon_filtered_cloud->width = polygon_filtered_cloud->points.size();
        polygon_filtered_cloud->height = 1;
        polygon_filtered_cloud->is_dense = true;

        // Сохранение результата в файл PCD
        pcl::io::savePCDFileASCII("/home/stud/Desktop/tests_point/filtered_polygon.pcd", *polygon_filtered_cloud);

        // Преобразование обратно в sensor_msgs/PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*polygon_filtered_cloud, output);
        output.header = cloud_msg->header;
        output.header.frame_id = "map";

        // Публикация отфильтрованного облака точек
        cloud_pub.publish(output);

        ROS_INFO("Point cloud processed, saved, and published.");

        accumulated_cloud.reset(new PointCloud);
        count = 0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "template_model_node");
    ros::NodeHandle nh;

    cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);

    ros::spin();
    return 0;
}
