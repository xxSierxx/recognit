#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB PointType;

pcl::PointCloud<pcl::PointXYZRGB> point_fun (cv::Mat img_rgb, cv::Mat img_dep, int stepReduced = 1){

  float fx = 570.3422241210938;
  float fy = 570.3422241210938;
  float cx = 314.5;
  float cy = 235.5;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for(int i = 0; i < img_dep.rows; i+=stepReduced) {
    for(int j = 0; j < img_dep.cols; j+=stepReduced) {
      pcl::PointXYZRGB point;
      unsigned short d = img_dep.at<uint16_t>(i,j);
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

void ros_fun(const pcl::PointCloud<PointType>::ConstPtr& cloud, sensor_msgs::PointCloud2& pointCloud)
{
  pcl::toROSMsg(*cloud, pointCloud);
  pointCloud.header.stamp = ros::Time::now();
  pointCloud.header.frame_id = "map";
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_point_cloud_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("ros_cloud", 1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cv::Mat img_rgb, img_depth;
  sensor_msgs::PointCloud2 ros_cloud;
  ros::Rate r(1);

  // Загрузка изображений из файлов
  std::string path_img_rgb = "/home/stud/Desktop/tests_point/rgb_image.jpg";
  std::string path_img_depth = "/home/stud/Desktop/tests_point/depth_image.png";

  ROS_INFO_STREAM("Loading RGB image: " << path_img_rgb);
  img_rgb = cv::imread(path_img_rgb, cv::IMREAD_COLOR);
  ROS_INFO_STREAM("Img RGB: " << img_rgb.cols << " x " << img_rgb.rows);

  ROS_INFO_STREAM("Loading Depth image: " << path_img_depth);
  img_depth = cv::imread(path_img_depth, cv::IMREAD_ANYDEPTH);
  ROS_INFO_STREAM("Img Depth: " << img_depth.cols << " x " << img_depth.rows);

  // Создание и заполнение облака точек
  *cloud = point_fun(img_rgb, img_depth);


  ros_fun(cloud, ros_cloud);

  ROS_INFO_STREAM("Finish processing, publishing...");

  while (ros::ok()) {
    pub.publish(ros_cloud);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

