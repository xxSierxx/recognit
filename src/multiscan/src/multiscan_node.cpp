#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Path.h>


nav_msgs::Path path;
void fun_path (Eigen::Matrix4f transform_msgs){

  geometry_msgs:: PoseStamped pose;
  Eigen::Matrix3f rot_mat;
  for(int i = 0; i<3; i++){
    for (int j = 0; j<3; j++){
      rot_mat(i,j) = transform_msgs(i,j);
    }
  }

  Eigen::Quaternionf quat(rot_mat);

  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  pose.pose.orientation.w = quat.w();

  pose.pose.position.x = transform_msgs(0, 3);
  pose.pose.position.y = transform_msgs(1, 3);
  pose.pose.position.z = transform_msgs(2, 3);

  path.poses.push_back(pose);
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenatePointCloudsWithTransformations(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_array,
    const std::vector<Eigen::Matrix4f>& transform,
    size_t stepReduced = 1) {

  // Создание пустого объединенного облака точек
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // Итерация по всем облакам точек и соответствующим трансформациям
  for (size_t i = 0; i < cloud_array.size(); ++i) {
    // Применение текущей трансформации к текущему облаку точек
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_array.at(i), *transformed_cloud, transform.at(i));

    // Добавление преобразованного облака точек к объединенному облаку
    *combined_cloud += *transformed_cloud;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(size_t i = 0; i < combined_cloud->points.size(); i+=stepReduced) {
    reduced_combined_cloud->points.push_back(combined_cloud->points.at(i));
  }

  return reduced_combined_cloud;
}

Eigen::Matrix4f icp_fun(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud2){
  // Применяем алгоритм ICP
  pcl::IterativeClosestPoint<pcl::PointXYZRGB,
      pcl::PointXYZRGB> icp;
  //Настройка ICP
 //icp.setMaximumIterations(1000); //количество итераций 25 30
  icp.setTransformationEpsilon(1e-9); //терминальная ошибка -8
  icp.setMaxCorrespondenceDistance(0.08); //максимальное расстояние

  //Загружаем первое облако точек
  icp.setInputSource(cloud2);

  //Загружаем второе облако точек
  icp.setInputTarget(cloud1);

  //Создаем облако точек, куда будет помещен результат
  //сопоставления
  pcl::PointCloud<pcl::PointXYZRGB>final_cloud;

  //Сопоставляем
  icp.align(final_cloud);

  //Получаем оценки сопоставления
  ROS_INFO_STREAM("Score: "<<icp.getFitnessScore());
  ROS_INFO_STREAM("Transformation");
  Eigen::Matrix4f transform = icp.getFinalTransformation();


  ROS_INFO_STREAM("\n"<<transform);

  return transform;
}

void ros_fun (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, sensor_msgs::PointCloud2 &pointCloud){
  pointCloud.width = cloud->width;
  pointCloud.height = cloud->height;
  pointCloud.is_bigendian = false;
  pointCloud.is_dense = false;
  sensor_msgs:: PointCloud2Modifier modifier(pointCloud);
  modifier.setPointCloud2FieldsByString(2,"xyz","rgb");
  modifier.resize(cloud->points.size());

  sensor_msgs::PointCloud2Iterator<float>out_x(pointCloud,"x");
  sensor_msgs::PointCloud2Iterator<float>out_y(pointCloud,"y");
  sensor_msgs::PointCloud2Iterator<float>out_z(pointCloud,"z");
  sensor_msgs::PointCloud2Iterator<uint8_t>out_r(pointCloud,"r");
  sensor_msgs::PointCloud2Iterator<uint8_t>out_g(pointCloud,"g");
  sensor_msgs::PointCloud2Iterator<uint8_t>out_b(pointCloud,"b");
  for(size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZRGB point = cloud->points[i];
    *out_x = point.x;
    *out_y = point.y;
    *out_z = point.z;

    *out_b = point.b;
    *out_g = point.g;
    *out_r = point.r;

    ++out_x;
    ++out_y;
    ++out_z;

    ++out_b;
    ++out_g;
    ++out_r;

  }
}

pcl::PointCloud<pcl::PointXYZRGB> point_fun (cv::Mat img_rgb, cv::Mat img_dep, int stepReduced = 1){

  float fx = 570.3422241210938;
  float fy = 570.3422241210938;
  float cx = 314.5;
  float cy = 235.5;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //  pointCloud->width = img_dep.cols;

  //  pointCloud->height = img_dep.rows;


  for(int i = 0; i < img_dep.rows; i+=stepReduced) {
    for(int j = 0; j < img_dep.cols; j+=stepReduced) {
      pcl::PointXYZRGB point;
      unsigned short d = img_dep.at<uint16_t>(i,j);
      // unsigned short d = (img_dep.data[i*img_dep.step+j]) + (img_dep.data[i*img_dep.step+j+1]*256);
      point.z = d/1000.0;
      point.x = ((float)i-cx) * point.z/fx;
      point.y = ((float)j-cy) * point.z/fy;
      cv::Vec3b pixel = img_rgb.at<cv::Vec3b>(i, j);
      point.r = pixel[2];
      point.g = pixel[1];
      point.b = pixel[0];
      pointCloud->points.push_back(point);


    }
  }
  return *pointCloud;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "imread_node");
  ros::NodeHandle nh;

  ros::Publisher pub =
      nh.advertise<sensor_msgs::PointCloud2>("combinedPoint", 1);
  ros::Publisher path_pub =
      nh.advertise<nav_msgs::Path>("path", 8);
  cv::Mat img_rgb,img_depth, img_rgb1, img_dep1;
  std::vector<sensor_msgs::PointCloud2> pointClouds;

  sensor_msgs::PointCloud2 point1,point2, combinedPointCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p1,p2, point_finish;
  Eigen::Matrix4f transform;
  ros::Rate r(1);
  int numF = 0;
  std::vector<Eigen::Matrix4f> all_transform;
  std::vector<cv::Mat> img_rgb_array, img_depth_array;

  //Загрузка изображений из файлов
  for(int num = 700; num < 800; num+=2) {
    std::string path_img = "/home/stud/Desktop/bedroom/image/";
    for(int d = 0; d < 6 - std::to_string(num).size(); d++) {
      path_img += "0";
    }
    path_img += std::to_string(num);
    path_img += ".jpg";
    ROS_INFO_STREAM("Loading "<<path_img);
    img_rgb = cv::imread(path_img, cv::IMREAD_COLOR);
    ROS_INFO_STREAM("Img: "<<img_rgb.cols<<" x "<<img_rgb.rows);
    img_rgb_array.push_back(img_rgb);
    std::string path_depth = "/home/stud/Desktop/bedroom/depth/";
    for(int d = 0; d < 6 - std::to_string(num).size(); d++) {
      path_depth += "0";
    }
    path_depth += std::to_string(num);
    path_depth += ".png";
    ROS_INFO_STREAM("Loading "<<path_depth);
    img_depth = cv::imread(path_depth, cv::IMREAD_ANYDEPTH);
    ROS_INFO_STREAM("Img depth: "<<img_depth.cols<<" x "<<img_depth.rows);
    img_depth_array.push_back(img_depth);

  }



  //Сопоставление

  //Получение облаков точек
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_array;
  for(int num = 0; num < img_rgb_array.size(); num++) {
    ROS_INFO_STREAM("Building cloud from IMG "<<num);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud = point_fun(img_rgb_array.at(num),  img_depth_array.at(num), 4);
    ROS_INFO_STREAM("Point cloud size: "<<cloud->points.size());

    if(num > 0){
      transform *= icp_fun (point_finish,cloud);
      all_transform.push_back(transform);
      cloud_array.push_back(cloud);
      point_finish = concatenatePointCloudsWithTransformations(cloud_array, all_transform, 4); //шаг
    }

    else{
      *cloud1 = point_fun(img_rgb_array.at(num+1),  img_depth_array.at(num+1), 4); //шаг
      cloud_array.push_back(cloud);
      cloud_array.push_back(cloud1);
      //Создании нулевой трансформации

      for(int i = 0; i < 4;i++) {
        for(int j = 0; j < 4; j++) {
          if(i == j) {
            transform(i,j) = 1;
          }
          else {
            transform(i,j) = 0;
          }
        }
      }
      all_transform.push_back(transform);
      all_transform.push_back(transform);

      point_finish = concatenatePointCloudsWithTransformations(cloud_array, all_transform, 4);
      transform *= icp_fun(point_finish,cloud);
    }

    all_transform.push_back(transform);
    fun_path(transform);
  }

  ros_fun(point_finish, combinedPointCloud);

  ROS_INFO_STREAM("Finish processing, publishing...");


  while (ros::ok()){

    combinedPointCloud.header.frame_id = "/base_link";
    combinedPointCloud.header.stamp = ros::Time::now();

    path.header.frame_id = "/base_link";
    path.header.stamp =  ros::Time::now();
    pub.publish(combinedPointCloud);

    path_pub.publish(path);
    ros::spinOnce();
    r.sleep();

  }

}
//  transform<<1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
