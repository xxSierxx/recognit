#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/features/shot_omp.h>

#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>


typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;

pcl::PointCloud<pcl::PointXYZ>::Ptr createCube(double size);
pcl::PointCloud<pcl::PointXYZ>::Ptr createSphere(double radius);
pcl::PointCloud<pcl::PointXYZ>::Ptr createCylinder(double radius, double height);

pcl::PointCloud<pcl::PointXYZ>::Ptr cube_model = createCube(5); // Здесь задается размер куба

void computeKeypointsAndNormals(pcl::PointCloud<PointType>::Ptr& cloud,
                                pcl::PointCloud<PointType>::Ptr& keypoints,
                                pcl::PointCloud<NormalType>::Ptr& normals,
                                float uniform_sampling_size = 0.01)
{
    // Uniform Sampling для создания ключевых точек
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(uniform_sampling_size);
    uniform_sampling.filter(*keypoints);

    // Оценка нормалей для ключевых точек
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setInputCloud(keypoints);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30); // Используем k-ближайших соседей для оценки нормалей
    norm_est.compute(*normals);
}

void ros_fun(const pcl::PointCloud<PointType>::ConstPtr& cloud, sensor_msgs::PointCloud2 &pointCloud) {
    pcl::toROSMsg(*cloud, pointCloud);
    pointCloud.header.stamp = ros::Time::now();
    pointCloud.header.frame_id = "map";
}

pcl::PointCloud<PointType>::Ptr point_fun(cv::Mat& img_rgb, const cv::Mat& img_dep, int stepReduced = 1) {
    float fx = 570.3422241210938;
    float fy = 570.3422241210938;
    float cx = 314.5;
    float cy = 235.5;

    pcl::PointCloud<PointType>::Ptr pointCloud(new pcl::PointCloud<PointType>);

    // Преобразование изображения в цветовое пространство HSV
    cv::Mat img_hsv;
    cv::cvtColor(img_rgb, img_hsv, cv::COLOR_BGR2HSV);

    // Создание маски для желтого цвета в HSV
    cv::Mat mask;
    cv::inRange(img_hsv, cv::Scalar(100, 70, 50), cv::Scalar(130, 255, 255), mask);

    // Нахождение контуров
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Преобразование изображения в облако точек
    for (int i = 0; i < img_dep.rows; i += stepReduced) {
        for (int j = 0; j < img_dep.cols; j += stepReduced) {
            unsigned short d = img_dep.at<uint16_t>(i, j);
            if (d == 0) continue; // Пропуск точек с нулевой глубиной

            PointType point;
            point.z = d / 1000.0;
            point.x = ((float)j - cx) * point.z / fx;
            point.y = ((float)i - cy) * point.z / fy;

            cv::Vec3b pixel = img_rgb.at<cv::Vec3b>(i, j);
            point.r = pixel[2];
            point.g = pixel[1];
            point.b = pixel[0];

            pointCloud->points.push_back(point);
        }
    }

    // Рисуем прямоугольники вокруг контуров и выделяем точки в облаке точек
    for (const auto &contour : contours) {
        cv::Rect bounding_rect = cv::boundingRect(contour);
        cv::rectangle(img_rgb, bounding_rect, cv::Scalar(192, 192, 192), 2); // зеленый прямоугольник

        // Проецирование контуров на облако точек
        for (int i = bounding_rect.y; i < bounding_rect.y + bounding_rect.height; i++) {
            for (int j = bounding_rect.x; j < bounding_rect.x + bounding_rect.width; j++) {
                if (i >= img_dep.rows || j >= img_dep.cols) continue;
                unsigned short d = img_dep.at<uint16_t>(i, j);
                if (d == 0) continue;

                PointType point;
                point.z = d / 1000.0;
                point.x = ((float)j - cx) * point.z / fx;
                point.y = ((float)i - cy) * point.z / fy;

                cv::Vec3b pixel = img_rgb.at<cv::Vec3b>(i, j);
                point.r = pixel[2];
                point.g = pixel[1];
                point.b = pixel[0];

                pointCloud->points.push_back(point);
            }
        }
    }

    return pointCloud;
}

void computeSHOTDescriptors(pcl::PointCloud<PointType>::Ptr& keypoints,
                            pcl::PointCloud<NormalType>::Ptr& normals,
                            pcl::PointCloud<PointType>::Ptr& model_keypoints,
                            pcl::PointCloud<NormalType>::Ptr& model_normals,
                            pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors)
{
    pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> shot;
    shot.setInputCloud(keypoints);
    shot.setInputNormals(normals);
    shot.setSearchSurface(model_keypoints);
   // shot.setSearchMethod(model_keypoints->makeShared()); // Используем makeShared() для преобразования в shared_ptr
    shot.setRadiusSearch(0.02); // Радиус поиска для сопоставления дескрипторов
    shot.compute(*descriptors);
}


//Исполняющий код. Здесь происходит считывания информации о сцене вызов функции и публикация

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_detection_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("ros_cloud", 1);

    cv::Mat img_rgb, img_depth;
    ros::Rate r(1);
    std::vector<cv::Mat> img_rgb_array, img_depth_array;

    // Загрузка изображений из файлов
//    for (int num = 5975; num < 5976; num++) {
        std::string path_img = "/home/stud/Desktop/tests_point/rgb_image.jpg";
//        for (int d = 0; d < 6 - std::to_string(num).size(); d++) {
//            path_img += "0";
//        }
//        path_img += std::to_string(num);
//        path_img += ".jpg";
        ROS_INFO_STREAM("Loading " << path_img);
        img_rgb = cv::imread(path_img, cv::IMREAD_COLOR);
        ROS_INFO_STREAM("Img: " << img_rgb.cols << " x " << img_rgb.rows);
        img_rgb_array.push_back(img_rgb);
        std::string path_depth = "/home/stud/Desktop/tests_point/depth_image.png";
//        for (int d = 0; d < 6 - std::to_string(num).size(); d++) {
//            path_depth += "0";
//        }
//        path_depth += std::to_string(num);
//        path_depth += ".png";
        ROS_INFO_STREAM("Loading " << path_depth);
        img_depth = cv::imread(path_depth, cv::IMREAD_ANYDEPTH);
        ROS_INFO_STREAM("Img depth: " << img_depth.cols << " x " << img_depth.rows);
        img_depth_array.push_back(img_depth);
//    }

    // Создание и заполнение облака точек
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    for (size_t num = 0; num < img_rgb_array.size(); ++num) {
        ROS_INFO_STREAM("Building cloud from IMG " << num);
        pcl::PointCloud<PointType>::Ptr temp_cloud = point_fun(img_rgb_array[num], img_depth_array[num], 1); // Полное разрешение для облака точек
        *cloud += *temp_cloud;
    }

    sensor_msgs::PointCloud2 ros_cloud;
    ros_fun(cloud, ros_cloud);

    ROS_INFO_STREAM("Finish processing, publishing...");



    while (ros::ok()) {
        pub.publish(ros_cloud);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

// Дальше идет код, который генерирует три базовые модели (цилиндр, куб и сферу),
// которые в дальнейшем будут использоваться для выделения по фигуре
// Состоит из трех одноименных функции с различными п-ми.
// Тип возвращаемых данных - Облако точек.

pcl::PointCloud<pcl::PointXYZ>::Ptr createCube(double size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cube(new pcl::PointCloud<pcl::PointXYZ>());

    // Вершины куба
    pcl::PointXYZ pt1(-size / 2, -size / 2, -size / 2);
    pcl::PointXYZ pt2(size / 2, -size / 2, -size / 2);
    pcl::PointXYZ pt3(size / 2, size / 2, -size / 2);
    pcl::PointXYZ pt4(-size / 2, size / 2, -size / 2);
    pcl::PointXYZ pt5(-size / 2, -size / 2, size / 2);
    pcl::PointXYZ pt6(size / 2, -size / 2, size / 2);
    pcl::PointXYZ pt7(size / 2, size / 2, size / 2);
    pcl::PointXYZ pt8(-size / 2, size / 2, size / 2);

    cube->push_back(pt1);
    cube->push_back(pt2);
    cube->push_back(pt3);
    cube->push_back(pt4);
    cube->push_back(pt5);
    cube->push_back(pt6);
    cube->push_back(pt7);
    cube->push_back(pt8);

    return cube;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr createSphere(double radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphere(new pcl::PointCloud<pcl::PointXYZ>());

    double resolution = 50; // разрешение сферы

    for (double phi = 0.0; phi <= M_PI; phi += M_PI / resolution)
    {
        for (double theta = 0.0; theta <= 2.0 * M_PI; theta += M_PI / resolution)
        {
            pcl::PointXYZ pt;
            pt.x = radius * sin(phi) * cos(theta);
            pt.y = radius * sin(phi) * sin(theta);
            pt.z = radius * cos(phi);
            sphere->push_back(pt);
        }
    }

    return sphere;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr createCylinder(double radius, double height)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZ>());

    double resolution = 50; // разрешение цилиндра

    for (double angle = 0.0; angle <= 2.0 * M_PI; angle += M_PI / resolution)
    {
        for (double z = -height / 2; z <= height / 2; z += height / resolution)
        {
            pcl::PointXYZ pt;
            pt.x = radius * cos(angle);
            pt.y = radius * sin(angle);
            pt.z = z;
            cylinder->push_back(pt);
        }
    }

    return cylinder;
}
