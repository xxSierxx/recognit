#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>

int getMapCellValue(const nav_msgs::OccupancyGrid &map, int i = 0,int j = 0) {
  int value = map.data.at(i * map.info.width + j);
  return value;
}

nav_msgs::OccupancyGrid fun_map (nav_msgs::OccupancyGrid&main_map){

  nav_msgs::OccupancyGrid transformed_map = main_map;
  float robot_size = 0.35;
  float safe_distance = robot_size/2 * sqrt(2);   //радиус окружности

  float th_incr = asin(1/(transformed_map.info.height * transformed_map.info.resolution));

  for(int i = 0; i < main_map.info.height; i++){
    for (int j = 0; j < main_map.info.width; j++) {
      if(getMapCellValue(main_map,i,j) == 100){
        float th = 0;
        float circle = 2 * M_PI / th_incr;

        for(int k = 0; k<circle; k++){

          int x =(safe_distance/main_map.info.resolution) * cos(th);
          int y = (safe_distance/main_map.info.resolution) * sin(th);

          int prep_point_col = j + x;
          int prep_point_row = i + y;

          int index = prep_point_row * transformed_map.info.width + prep_point_col;
          transformed_map.data[index] = 100;
          th += th_incr;

        }
      }
    }
  }

  return transformed_map;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "test_path_planning_node");
  ros::NodeHandle nh;

  //Получить карту из сервиса
  ros::ServiceClient map_client =
      nh.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap srv;
  nav_msgs::OccupancyGrid main_map, copy_map;
  ros::Publisher tr_map_pub =
      nh.advertise<nav_msgs::OccupancyGrid>("/tf_map", 10);
  ros::Publisher pub_path =
      nh.advertise<nav_msgs::Path>("/path",10);
  geometry_msgs::Pose2D start_pose, finish_pose,second_pose;
  nav_msgs::Path path;
  geometry_msgs:: PoseStamped pose;

  //Задаем координаты начальной и конечной точек
  start_pose.x = 0;
  start_pose.y = 0;

  finish_pose.x = 5;
  finish_pose.y = 1;

  if(map_client.call(srv)){
    main_map = srv.response.map;
    ROS_INFO_STREAM("Map received: "<<main_map.info.height
                    <<" x "<<main_map.info.width);
  }
  else {
    ROS_ERROR_STREAM("Error \"while\" receiving map");
    return 1;
  }

  copy_map.header.frame_id = "map";
  copy_map.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();

  copy_map = fun_map(main_map);

  pose.pose.position.x = start_pose.x;
  pose.pose.position.y = start_pose.y;
  pose.pose.position.z = 0;
  path.poses.push_back(pose);





  for(int i = 0; i < copy_map.info.height; i++){
    for (int j = 0; j < copy_map.info.width; j++) {
      if(getMapCellValue(copy_map,i,j) == 0){
//        int x =(safe_distance/main_map.info.resolution) * cos(th);
//        int y = (safe_distance/main_map.info.resolution) * sin(th);
        int prep_point_col = j + 2;
        int prep_point_row = i + 2;
           int index = prep_point_row * copy_map.info.width + prep_point_col;
           copy_map.data[index] = 0;
      }}}
//        if(finish_pose.x>copy_map.data[i,j]&&finish_pose.y&&j<=0){
//          second_pose.x = i;
//          second_pose.y = j;
//          continue;
//        }
//        if(second_pose.x>i; second_pose.y>j)

//      }

      ros::Rate r(1);
      while(ros::ok()){
        pub_path.publish(path);
        tr_map_pub.publish(copy_map);
        ros::spinOnce();
        r.sleep();

      }
    }
//  }
//}


//TODO Реализовать алгоритм планирования
//Общая задача - последовательно добавлять точки
//в траекторию типа nav_msgs::Path до тех пор,пока
//не будет достигнута конечная точка

//На каждом шаге следовать правилам
//1. Среди 8 соседей ячейки выбрать ту, которая
//находится ближе всех к конечной и при этом
//не занята препятствием
//2. В ходе планирования могут встречаться ловушки.
//Ловушками являются вогнутые участки вблизи
//препятсвий, в которых алгоритм планирования
//будет "ходить по кругу"
//Для решения проблем ловуке рекомендуется
//помечать ранее посещенные ячейки значением -2
//а если окрестности были полностью посещены
//то стоит вернуться назад

//После того, как траектория будет полностью
//построена, ее необходимо оптимизировать
//Суть оптимизации заключается в срезании участков
//где робот может пройти, не врезавшись в
//перпятствие
//Для решения этой задачи можно последовательно
//искать путь между точками n + 1 и n - 1
//Для каждой точки n, полученного маршрута
//и в случае наличии такого пути точку n
//исключать из маршрута. Процесс повторять до
//тех пор, пока не останется участков, где можно
//срезать
//КРУЖНОСТЬ карта планирование маршрута по белым точка. Черными точками рисуем окружность. ИДем по карте находим черную точку и идем в полярную систем координат и обрисовываем всю окружность. 0.45 если карта небольшая то угол можно взять 0.01 в радиане. две карты делать одну исходну другая расширенная. На исходной находим точки и отображаем на черной
//запрашиваем карту в мап сервере!
