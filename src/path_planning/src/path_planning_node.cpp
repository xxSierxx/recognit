#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>

int getMapCellValue(const nav_msgs::OccupancyGrid &map, int i = 0, int j = 0) {
  int value = map.data.at(i * map.info.width + j);
  return value; // добавлено возвращение значения
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_planning_node");
  ros::NodeHandle nh;

  // Получение карты из сервиса
  ros::ServiceClient map_client =
      nh.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap srv;
  nav_msgs::OccupancyGrid main_map;
  if (map_client.call(srv)) {
    main_map = srv.response.map;
    ROS_INFO_STREAM("Map received: " << main_map.info.height
                    << " x " << main_map.info.width);
  } else {
    ROS_ERROR_STREAM("Error while receiving map");
    return 1;
  }

  nav_msgs::OccupancyGrid transformed_map = main_map;
  nav_msgs::Path planned_path;
  float robot_size = 0.35;
  float s_dist = robot_size / 2 * sqrt(2) / transformed_map.info.resolution;
  float th_incr =
      asin(1 / (transformed_map.info.height * transformed_map.info.resolution));

  int pre_current_col, pre_current_row; // перемещено перед циклом while

  for (int i = 0; i < main_map.info.height; i++) {
    for (int j = 0; j < main_map.info.width; j++) {
      if (getMapCellValue(main_map, i, j) == 100) {
        int krug = 2 * 3.14 / th_incr;
        float theta = 0;

        for (int incr = 0; incr < krug; incr++) {
          float x1 = s_dist * cos(theta);
          float y1 = s_dist * sin(theta);

          int prep_point_col = x1 + j;
          int prep_point_row = y1 + i;

          if (prep_point_col < main_map.info.height &&
              prep_point_row < main_map.info.width) {
            int index = prep_point_row * transformed_map.info.width + prep_point_col;
            transformed_map.data[index] = 100;
          }
          theta = theta + th_incr;
        }
      }
    }
  }



  geometry_msgs::Pose2D start_pose, finish_pose;



  // Публикация планируемой траектории
  ros::Publisher tr_map_pub =
      nh.advertise<nav_msgs::OccupancyGrid>("/tr_map", 10);
  transformed_map.header.frame_id = "map";
  transformed_map.header.stamp = ros::Time::now();

  int i = 0;
  while (i < 10) {
    tr_map_pub.publish(transformed_map);
    ros::spinOnce(); // исправлено на ros::spinOnce()
    sleep(1);
    i++; // добавлена точка с запятой
  }

  return 0; // добавлено возвращение 0 в конце main
}

