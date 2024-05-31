#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <iostream>
using namespace std;

vector<geometry_msgs::Pose2D> path_odom, path_icp;
nav_msgs::Path nav_path_odom, nav_path_icp,nav_path_kalman;
bool isNewOdom, isNewICP;


void odomCallBack (const nav_msgs::Odometry &msg){

  geometry_msgs::Pose2D pose;
  Eigen::Quaternionf q;
  q.x() = msg.pose.pose.orientation.x;
  q.y() = msg.pose.pose.orientation.y;
  q.z() = msg.pose.pose.orientation.z;
  q.w() = msg.pose.pose.orientation.w;
  auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
  pose.theta = euler(2);
  path_odom.push_back(pose);
  isNewOdom = true;

}

void icpCallBack (const geometry_msgs::PoseStamped &msg){

  geometry_msgs:: Pose2D pose;
  Eigen::Quaternionf q;
  q.x() = msg.pose.orientation.x;
  q.y() = msg.pose.orientation.y;
  q.z() = msg.pose.orientation.z;
  q.w() = msg.pose.orientation.w;
  auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
  pose.x = msg.pose.position.x;
  pose.y = msg.pose.position.y;
  pose.theta = euler(2);
  //  ROS_INFO_STREAM("Pose "<<pose.x<<"  "<<pose.y<<" "<<pose.theta);
  path_icp.push_back(pose);
  isNewICP = true;

}

void kalman_fun (){

  nav_path_icp.poses.clear();
  nav_path_kalman.poses.clear();
  nav_path_odom.poses.clear();

  /////----------------------Фильтр Калмана------------------------
  vector<geometry_msgs::Pose2D> path_kalman;

  //Матрица ошибки предсказания
  Eigen::Matrix3f P = Eigen::Matrix3f::Identity()*0.5;
  //Модель системы;
  Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
  //Усиление Калмана
  Eigen::Matrix3f G = Eigen::Matrix3f::Zero();
  //Матрица ковариации шума измерений
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity()*0.001;
  //Матрица ковариции шума процесса
  Eigen::Matrix3f Q = Eigen::Matrix3f::Identity() * 0.001;
  //Модель управления
  Eigen::Matrix3f B = Eigen::Matrix3f::Identity();
  //Предыдущий вектор состояние системы
  Eigen::Vector3f X_prev(0,0,0);
  //Добавляем начальную позицию в траекторию
  geometry_msgs::Pose2D pose;
  pose.x=0;
  pose.y = 0;
  pose.theta = 0;
  path_kalman.push_back(pose);


  //Проходим по всем полученным позициям
  for (int i = 1; i < path_odom.size();i++) {

    //Этап предсказания
    //Считаем, что в качестве управляющего воздействия
    //используются данные одометрии
    Eigen::Vector3f U(path_odom.at(i).x - path_odom.at(i-1).x,
                      path_odom.at(i).y - path_odom.at(i-1).y,
                      path_odom.at(i).theta - path_odom.at(i-1).theta);
    //Даем оценку вектора состояния
    Eigen::Vector3f X = A * X_prev + B * U;
    //Вычисляем ошибку предсказания
    P =  A * P * A.transpose() + Q;

    //Этап коррекции
    G =  P * ((P+R).inverse());
    //Под текущими измерениями считаем данные оценки положения
    //По алгоритму ICP
    Eigen::Vector3f Z(path_icp.at(i).x,
                      path_icp.at(i).y,
                      path_icp.at(i).theta);
    //Основное уравнение
    X = X + G * (Z - X);
    //Обновляем ошибку предсказания
    P = (Eigen::Matrix3f::Identity()-G)*P;
    //Переносим текущий вектор состояния в предыдущий
    X_prev = X;
    //Добавляем точку в траекторию

    pose.x=X(0);
    pose.y = X(1);
    pose.theta = X(2);
    path_kalman.push_back(pose);


  }

  ///------------------------ROS---------------------------------

  nav_path_icp.header.frame_id = "map";
  nav_path_odom.header.frame_id = "map";
  nav_path_kalman.header.frame_id = "map";

  for(int i = 0; i < path_odom.size(); i++){
    geometry_msgs::Pose2D current_pose_odom = path_odom.at(i);
    geometry_msgs::Pose2D current_pose_icp = path_icp.at(i);
    geometry_msgs::Pose2D current_pose_kalman = path_kalman.at(i);


    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = current_pose_odom.x;
    pose.pose.position.y = current_pose_odom.y;
    nav_path_odom.poses.push_back(pose);

    pose.pose.position.x = current_pose_icp.x;
    pose.pose.position.y = current_pose_icp.y;
    nav_path_icp.poses.push_back(pose);

    pose.pose.position.x = current_pose_kalman.x;
    pose.pose.position.y = current_pose_kalman.y;
    nav_path_kalman.poses.push_back(pose);

  }

}

int main(int argc, char **argv){

  ros::init(argc, argv, "test_kalman_node");
  ros::NodeHandle nh;
  ros::Subscriber pos_sub
      = nh.subscribe("/odom", 8, odomCallBack);
  ros::Subscriber icp_pub
      = nh.subscribe("/pose", 8, icpCallBack);

  ros::Publisher path_odom_pub =
      nh.advertise<nav_msgs::Path>("path_odom",8);
  ros::Publisher path_icp_pub =
      nh.advertise<nav_msgs::Path>("path_icp",8);
  ros::Publisher path_kalman_pub =
      nh.advertise<nav_msgs::Path>("path_kalman",8);

  ros::Rate r(1);

  while(ros::ok()){

    ROS_INFO_STREAM("isNewIcp "<<isNewICP<<"\n isNewOdom "<<isNewOdom);

    if(isNewICP && isNewOdom) {

      kalman_fun();
      isNewICP = false;
      isNewOdom = false;

    }

    path_odom_pub.publish(nav_path_odom);
    path_icp_pub.publish(nav_path_icp);
    path_kalman_pub.publish(nav_path_kalman);
    ros::spinOnce();
    r.sleep();
  }
}

//fstream file_with_path("/home/robot31-4/Desktop/path_example.txt");
//vector<geometry_msgs::Pose2D> path_odom;
//vector <geometry_msgs::Pose2D> path_icp;

//Eigen:: Quaternionf q;
//geometry_msgs::Pose2D pose_odom, pose_icp;


////Считываем из файла
//while(file_with_path>>pose_odom.x>>pose_odom.y>>pose_odom.theta>>pose_icp.x>>pose_icp.y>>pose_icp.theta){
//  path_odom.push_back(pose_odom);
//  path_icp.push_back(pose_icp);

//}

//ROS_INFO_STREAM("Load: "<<path_odom.size()<<" positions");

