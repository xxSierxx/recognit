#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>

// Параметры для изображения RGB
const int image_width = 640;
const int image_height = 480;

void createSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius, uint8_t r, uint8_t g, uint8_t b) {
    pcl::PointXYZRGB point;
    point.r = r;
    point.g = g;
    point.b = b;
    for (float phi = 0.0; phi <= M_PI; phi += M_PI / 20.0) {
        for (float theta = 0.0; theta <= 2.0 * M_PI; theta += M_PI / 20.0) {
            point.x = radius * sin(phi) * cos(theta);
            point.y = radius * sin(phi) * sin(theta);
            point.z = radius * cos(phi);
            cloud->points.push_back(point);
        }
    }
}

void createCube(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float size, uint8_t r, uint8_t g, uint8_t b) {
    pcl::PointXYZRGB point;
    point.r = r;
    point.g = g;
    point.b = b;

    // Верхняя и нижняя стороны
    for (float x = -size; x <= size; x += 0.02) {
        for (float y = -size; y <= size; y += 0.02) {
            point.x = x;
            point.y = y;
            point.z = size;
            cloud->points.push_back(point);

            point.z = -size;
            cloud->points.push_back(point);
        }
    }

    // Стороны
    for (float y = -size; y <= size; y += 0.02) {
        for (float z = -size; z <= size; z += 0.02) {
            point.y = y;
            point.z = z;
            point.x = size;
            cloud->points.push_back(point);

            point.x = -size;
            cloud->points.push_back(point);
        }
    }

    for (float x = -size; x <= size; x += 0.02) {
        for (float z = -size; z <= size; z += 0.02) {
            point.x = x;
            point.z = z;
            point.y = size;
            cloud->points.push_back(point);

            point.y = -size;
            cloud->points.push_back(point);
        }
    }
}

void createCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius, float height, uint8_t r, uint8_t g, uint8_t b) {
    pcl::PointXYZRGB point;
    point.r = r;
    point.g = g;
    point.b = b;
    for (float z = -height / 2.0; z <= height / 2.0; z += 0.02) {
        for (float theta = 0.0; theta <= 2.0 * M_PI; theta += M_PI / 20.0) {
            point.x = radius * cos(theta);
            point.y = radius * sin(theta);
            point.z = z;
            cloud->points.push_back(point);
        }
    }
}

void saveImages(const cv::Mat& img_rgb, const cv::Mat& img_depth, const std::string& save_folder) {
    std::string rgb_filename = save_folder + "rgb_image.jpg";
    std::string depth_filename = save_folder + "depth_image.png";

    cv::imwrite(rgb_filename, img_rgb);
    cv::imwrite(depth_filename, img_depth);
}

int main(int argc, char** argv) {
    // Инициализация ROS
    ros::init(argc, argv, "point_cloud_generator");
    ros::NodeHandle nh;

    // Создание облака точек
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Создание сферы
    createSphere(cloud, 0.1, 255, 0, 0);  // красная сфера

    // Создание куба
    createCube(cloud, 0.1, 0, 255, 0);   // зеленый куб

    // Создание цилиндра
    createCylinder(cloud, 0.05, 0.2, 0, 0, 255);  // синий цилиндр

    // Конвертация облака точек в сообщение ROS
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "base_link";  // Предполагаем, что frame_id - "base_link"

    // Создание изображения RGB и изображения глубины
    cv::Mat img_rgb(image_height, image_width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat img_depth(image_height, image_width, CV_16UC1, cv::Scalar(1000));

    std::string save_folder = "/home/stud/Desktop/tests_point/";
    saveImages(img_rgb, img_depth, save_folder);

    // Публикация облака точек
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        cloud_msg.header.stamp = ros::Time::now();
        pub_cloud.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
