#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


static bool isNewData;
static sensor_msgs::LaserScan scan_data;


int i = 0;

void funCallBack (const sensor_msgs::LaserScan &msg){

  isNewData = true;
  scan_data = msg;
  i++;
}

void scan_filling (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

  for(int j = 1; j < scan_data.ranges.size(); j++) {
    if(scan_data.ranges.at(j) <= 6 && scan_data.ranges.at(j)>= 0.3){
      pcl::PointXYZ point;
      float range = scan_data.ranges.at(j);
      float theta = scan_data.angle_min + j * scan_data.angle_increment;
      point.x = range * cos(theta);
      point.y = range * sin(theta);
      point.z = 0.0;
      cloud->points.push_back(point);
    }
  }
  cloud->width = cloud->points.size();
}

void ros_fun (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::PointCloud &cloud_ros){

  for(int i=0; i < cloud->points.size(); i++) {
    geometry_msgs::Point32 point;
    point.x = cloud->points.at(i).x;
    point.y = cloud->points.at(i).y;
    point.z = cloud->points.at(i).z;
    cloud_ros.points.push_back(point);
  }
}

Eigen::Matrix4f icp_fun(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2){
  // Применяем алгоритм ICP
  pcl::IterativeClosestPoint<pcl::PointXYZ,
      pcl::PointXYZ> icp;
  //Настройка ICP
  icp.setMaximumIterations(50); //количество итераций
  icp.setTransformationEpsilon(1e-10); //терминальная ошибка
  icp.setMaxCorrespondenceDistance(0.15); //максимальное расстояние
  //Загружаем первое облако точек
  icp.setInputSource(cloud1);

  //Загружаем второе облако точек
  icp.setInputTarget(cloud2);

  //Создаем облако точек, куда будет помещен результат
  //сопоставления
  pcl::PointCloud<pcl::PointXYZ>final_cloud;

  //Сопоставляем
  icp.align(final_cloud);

  //Получаем оценки сопоставления
  ROS_INFO_STREAM("Score: "<<icp.getFitnessScore());
  ROS_INFO_STREAM("Transformation");
  Eigen::Matrix4f transform = icp.getFinalTransformation();

  ROS_INFO_STREAM("\n"<<transform);

  return transform;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "test_icp_node");
  ros::NodeHandle nh;
  //Два облака точек
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new
                                               pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new
                                                pcl::PointCloud<pcl::PointXYZ>);


  ros::Publisher path_pub =
      nh.advertise<nav_msgs::Path>("path", 8);

  ros::Publisher c_pub_in =
      nh.advertise<sensor_msgs::PointCloud>("cloud_in",8);

  ros::Publisher c_pub_out =
      nh.advertise<sensor_msgs::PointCloud>("cloud_out",8);

  ros::Publisher pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/pose",8);

  ros::Subscriber scan =
      nh.subscribe("/scan", 8, funCallBack);

  nav_msgs::Path path;


  //Траектория
  //Положение 1
  geometry_msgs:: PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  path.poses.push_back(pose);

  Eigen::Matrix4f transform1, transform_test;;
  transform1<<1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  transform_test<<1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

  ros::Rate r(10);
  while (ros::ok()){
    ros::spinOnce();

    sensor_msgs::PointCloud cloud_in_ros, cloud_out_ros;

    switch (i == 1? 1:2){

    case 1:
      scan_filling(cloud_in);
      break;

    case 2:

      sensor_msgs::ChannelFloat32 channel,channel_out;
      channel.name = "intense";
      channel_out.name = "intense";

      cloud_out->points.clear();
      *cloud_out = *cloud_in;
      cloud_in->points.clear();
      scan_filling(cloud_in);

      Eigen::Matrix3f rot_mat;

      for(int i = 0; i<3; i++){
        for (int j = 0; j<3; j++){
          rot_mat(i,j) = transform1(i,j);
        }
      }
      Eigen::Quaternionf quat(rot_mat);


      //Взаимодействие с ROS
      ros_fun(cloud_in,cloud_in_ros);
      channel.values.resize(cloud_in->points.size(), 10);
      cloud_in_ros.channels.push_back(channel);

      ros_fun(cloud_out, cloud_out_ros);
      channel_out.values.resize(cloud_out->points.size(), 20);
      cloud_out_ros.channels.push_back(channel_out);

      cloud_in_ros.header.frame_id = "map";
      cloud_out_ros.header.frame_id = "map";
      path.header.frame_id = "map";

      //Положение 1

      transform1 *= icp_fun(cloud_in,cloud_out);

      //Положение 2
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();
      pose.pose.orientation.w = quat.w();

      pose.pose.position.x = transform1(0, 3);
      pose.pose.position.y = transform1(1, 3);
      pose.pose.position.z = transform1(2, 3);
      path.poses.push_back(pose);

      c_pub_in.publish(cloud_in_ros);
      c_pub_out.publish(cloud_out_ros);
      path_pub.publish(path);
      pose_pub.publish(pose);
      isNewData = false;
      break;

    }


    r.sleep();
  }
}



//1)принять данные с лаз.скан
//2)получить облако точек
//3)обработать ОТ с помощью pcl
//4)сравнить с помощью icp
//5)вывести и отстроить траекторию



//Заполняем данными первое облако
//  cloud_in->width = 5;
//  cloud_in->height = 1;

//  for(int i =0; i<5; i++){
//    pcl::PointXYZ point;
//    point.x = 1024 * rand() / (RAND_MAX+1.0);
//    point.y = 1024 * rand() / (RAND_MAX+1.0);
//    point.z = 1024 * rand() / (RAND_MAX+1.0);
//    cloud_in->points.push_back(point);
//  }

//  ROS_INFO_STREAM("Cloud in coordinates");
//  for(int i = 0; i< 5; i++){

//    ROS_INFO_STREAM("point" <<i<<" "<<
//                    cloud_in->points.at(i).x<<" "<<
//                    cloud_in->points.at(i).y<<" "<<
//                    cloud_in->points.at(i).z);
//  }
//Копируем первое облако во второе
//  *cloud_out = *cloud_in;

//  //Трансформируем второе облако точек
//  for(int i = 0; i< cloud_out->points.size(); i++){
//    cloud_out->points.at(i).x +=0.7;
//  }
//  ROS_INFO_STREAM("Cloud out coordinates");
//  for(int i = 0; i< 5; i++){

//    ROS_INFO_STREAM("point" <<i<<" "<<
//                    cloud_out->points.at(i).x<<" "<<
//                    cloud_out->points.at(i).y<<" "<<
//                    cloud_out->points.at(i).z);
//  }

//Eigen (angle Eyler)
//pcl for icp transmation
//[r11 r12 r13]tx
//[r r r] ty
//[r r r] tz
//[sx sy sz] 1]
//Eiler -> Quaternion
//Eigen rotation matrix to quaternion
//MAtix3f iz Matrix4f
//Quat imeet 4 поля выдаем в сообщаение в Pose.poseStam
//как получили кватернион в узле калмана eigen quaternion to eulerangle
//auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
//кватарнион описывает только вращение;
