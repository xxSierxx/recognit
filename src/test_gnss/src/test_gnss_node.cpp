#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <geodesy/utm.h>
#include <nav_msgs/Path.h>
#include <cmath>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GNSS_node");
  ros::NodeHandle nh;
  fstream nmea_file("/home/stud/Downloads/nmea_scan_Den1.txt");

  ros::Publisher path_pub =
      nh.advertise<nav_msgs::Path>("gnss_path", 8);

  char c;
  vector<string> sentances;
  string current_sentance;
  vector<sensor_msgs::NavSatFix> coordinates;

  while((c = nmea_file.get()) != EOF) {
    current_sentance.push_back(c);
    if (c==10) {
      if(current_sentance.size()>1) {
        sentances.push_back(current_sentance);
      }
      current_sentance.clear();
    }
  }
  for(int i = 0; i < sentances.size(); i++) {
    size_t f_pose = sentances.at(i).find("GGA");
    if(f_pose != std::string::npos) {
      cout<<sentances.at(i)<<endl;
      string rmc_sentance = sentances.at(i);
      vector<string> words;
      string current_word;
      int w_pose = 0;

      while (w_pose < rmc_sentance.size()){
        if(rmc_sentance.at(w_pose)==','){
          words.push_back(current_word);
          current_word.clear();
        }
        else {
          current_word.push_back(rmc_sentance.at(w_pose));
        }
        w_pose++;
      }

      string lat_s = words.at(2);
      string lat_p = words.at(3);
      string long_s = words.at(4);
      string long_p = words.at(5);
      string alt_s = words.at(9);
      string alt_p = words.at(10);
      // cout<<"latitude "<<lat_s<<" "<<lat_p<<" longitude "<<long_s<<" "<<long_p<<" altitude "<<alt_s<<" "<<alt_p<<endl;

      //Переводим в числовое значение

      //Широта
      string grad = lat_s.substr(0, 2);
      string minutes = lat_s.substr(2, lat_s.size()-2);
      //         cout<<"Grad "<<grad<<" minutes "<<minutes<<endl;
      double latitude = stoi(grad)+stod(minutes)/60.0;
      //Для южного полушария отрицательная
      if (lat_p == "S"){
        latitude *= -1;
      }

      //Долгота
      grad = long_s.substr(0, 3);
      minutes = long_s.substr(3, long_s.size()-3);
      double longitude = stoi(grad)+stod(minutes)/60.0;
      if (lat_p == "W"){
        longitude *= -1;
      }

      //Высота
      double altitude = stod(alt_s);

      cout<<"Latitude "<<latitude<<" Longitude "<<longitude<<" Altitude "<<altitude<<endl;

      //TO DO: получить высоту
      //В NMEA высота находится в GGA на 9, 10 позициях

      sensor_msgs::NavSatFix coord;
      coord.header.frame_id = "world";
      coord.latitude = latitude;
      coord.longitude = longitude;
      coord.altitude = altitude;
      coordinates.push_back(coord);
    }
  }

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose,pose_old;
  path.header.frame_id = "map";

  for (int c = 0; c < coordinates.size(); c++) {
    //        cout<<coordinates.at(c).latitude<<" "
    //           <<coordinates.at(c).longitude<<endl;
    geographic_msgs::GeoPoint current_point;
    current_point.latitude = coordinates.at(c).latitude;
    current_point.longitude = coordinates.at(c).longitude;
    current_point.altitude = coordinates.at(c).altitude;

    // ROS_INFO_STREAM(current_point.altitude);

    //To UTM
    geodesy::UTMPoint utm_point(current_point);

    //Координаты в метрах
    cout<<"UTM: easting(x) "<<utm_point.easting<<
          " northing(y) "<<utm_point.northing<<
          " altitude "<<utm_point.altitude<<
          " zone "<<(int)utm_point.zone<<
          " band "<<utm_point.band<<endl;
    ROS_INFO_STREAM(utm_point.altitude);

    if (path.poses.size()<1) {
      pose_old.pose.position.x = utm_point.easting;
      pose_old.pose.position.y = utm_point.northing;
      pose_old.pose.position.z = utm_point.altitude;

      pose.pose.position.x = utm_point.easting - pose_old.pose.position.x;
      pose.pose.position.y = utm_point.northing - pose_old.pose.position.y;
      pose.pose.position.z = utm_point.altitude - pose_old.pose.position.z;
      path.poses.push_back(pose);
    }
    else if (path.poses.size()==1){
      pose.pose.position.x = utm_point.easting - pose_old.pose.position.x;
      pose.pose.position.y = utm_point.northing - pose_old.pose.position.y;
      pose.pose.position.z = utm_point.altitude - pose_old.pose.position.z;
      path.poses.push_back(pose);

    }
    else {
      pose.pose.position.x = utm_point.easting - pose_old.pose.position.x;
      pose.pose.position.y = utm_point.northing - pose_old.pose.position.y;
      pose.pose.position.z = utm_point.altitude - pose_old.pose.position.z;
      path.poses.push_back(pose);

      double d = sqrt (pow ( (path.poses.at(path.poses.size()-1).pose.position.x - path.poses.at(path.poses.size()-2).pose.position.x),2 ) +
                       pow ( (path.poses.at(path.poses.size()-1).pose.position.y - path.poses.at(path.poses.size()-2).pose.position.y),2 ) );

      double D = D + sqrt (pow((0.9996*d),2) + pow( (path.poses.at(path.poses.size()-1).pose.position.z - path.poses.at(path.poses.size()-2).pose.position.z),2 )) *
          (((path.poses.at(path.poses.size()-1).pose.position.z + path.poses.at(path.poses.size()-2).pose.position.z)/2 + 6371302) / 6371302);

      ROS_INFO_STREAM("D "<<D);
    }
  }

  ros::Rate r(10);
  while (ros::ok()) {
    path_pub.publish(path);
    ros::spinOnce();
    r.sleep();
  }
}


