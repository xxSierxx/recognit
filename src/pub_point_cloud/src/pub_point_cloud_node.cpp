#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

typedef pcl::PointXYZRGB PointType;

// Функция для создания куба
pcl::PointCloud<PointType>::Ptr createCube(float size, float x_offset, float y_offset, float z_offset, uint8_t r, uint8_t g, uint8_t b) {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  float half_size = size / 2.0;

  float step = 0.02;
  for (float x = -half_size; x <= half_size; x += step) {
    for (float y = -half_size; y <= half_size; y += step) {

      // Верхняя и нижняя грани
      for (float z : {-half_size, half_size}) {
        PointType point;
        point.x = x + x_offset;
        point.y = y + y_offset;
        point.z = z + z_offset;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud->points.push_back(point);
      }
    }
  }

  for (float x = -half_size; x <= half_size; x += step) {
    for (float z = -half_size; z <= half_size; z += step) {

      // Передняя и задняя грани
      for (float y : {-half_size, half_size}) {
        PointType point;
        point.x = x + x_offset;
        point.y = y + y_offset;
        point.z = z + z_offset;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud->points.push_back(point);
      }
    }
  }

  for (float y = -half_size; y <= half_size; y += step) {
    for (float z = -half_size; z <= half_size; z += step) {

      // Левая и правая грани
      for (float x : {-half_size, half_size}) {
        PointType point;
        point.x = x + x_offset;
        point.y = y + y_offset;
        point.z = z + z_offset;
        point.r = r;
        point.g = g;
        point.b = b;
        cloud->points.push_back(point);
      }
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

// Функция для создания цилиндра
pcl::PointCloud<PointType>::Ptr createCylinder(float radius, float height, float x_offset, float y_offset, float z_offset, uint8_t r, uint8_t g, uint8_t b) {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

  float step = 0.02; // Уменьшил шаг для плотности точек
  // Боковая поверхность цилиндра
  for (float z = -height / 2; z <= height / 2; z += step) {
    for (float angle = 0; angle <= 2 * M_PI; angle += M_PI / 72.0) {
      PointType point;
      point.x = radius * cos(angle) + x_offset;
      point.y = radius * sin(angle) + y_offset;
      point.z = z + z_offset;
      point.r = r;
      point.g = g;
      point.b = b;
      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  return cloud;
}



// Функция для создания сферы
pcl::PointCloud<PointType>::Ptr createSphere(float radius, float x_offset, float y_offset, float z_offset, uint8_t r, uint8_t g, uint8_t b) {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

  float step = M_PI / 36.0; // Уменьшил шаг для плотности точек
  for (float phi = 0; phi <= M_PI; phi += step) {
    for (float theta = 0; theta < 2 * M_PI; theta += step) {
      PointType point;
      point.x = radius * sin(phi) * cos(theta) + x_offset;
      point.y = radius * sin(phi) * sin(theta) + y_offset;
      point.z = radius * cos(phi) + z_offset;
      point.r = r;
      point.g = g;
      point.b = b;
      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void ros_fun(const pcl::PointCloud<PointType>::ConstPtr& cloud, sensor_msgs::PointCloud2& pointCloud) {
  pcl::toROSMsg(*cloud, pointCloud);
  pointCloud.header.stamp = ros::Time::now();
  pointCloud.header.frame_id = "map";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pub_point_cloud_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("ros_cloud", 1);
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  sensor_msgs::PointCloud2 ros_cloud;
  ros::Rate r(1);

  // Создание куба, цилиндра и сферы

//   pcl::PointCloud<PointType>::Ptr cube3 = createCube(2.2, 3, 3, 0, 255, 0, 200);

  // pcl::PointCloud<PointType>::Ptr sphere1 = createSphere(2.2, 3, 3, 0, 0, 255, 0);

  // pcl::PointCloud<PointType>::Ptr cylinder1 = createCylinder(1, 2, 3, 3, 0, 255, 80, 100);


//  pcl::PointCloud<PointType>::Ptr cube1 = createCube(1, -0, 0, 0, 0, 0, 255);

//  pcl::PointCloud<PointType>::Ptr cube2 = createCube(1, -4, 0, 0, 0, 255, 0);

//  pcl::PointCloud<PointType>::Ptr cube = createCube(1, -2, 0, 0, 255, 0, 0);

//  pcl::PointCloud<PointType>::Ptr sphere1 = createSphere(0.8, 0.5, 1, 0, 255, 0, 200);

  pcl::PointCloud<PointType>::Ptr cylinder1 = createCylinder(0.8, 1, 0.5, 1, 0, 255, 0, 200);


    pcl::PointCloud<PointType>::Ptr cube = createCube(0.5, -2, 0, 0, 255, 0, 0);

    pcl::PointCloud<PointType>::Ptr sphere = createSphere(0.5, -3, 2, 0, 0, 0, 255);

    pcl::PointCloud<PointType>::Ptr cylinder = createCylinder(0.2, 0.5, -0.8, 0, 0, 255, 200, 0);
//       pcl::PointCloud<PointType>::Ptr cube3 = createCube(1, 0.5, 1, 0, 255, 0, 200);

  // Объединение облаков точек
      *cloud = *cube + *cylinder ;
      *cloud += *sphere;
      *cloud += *cylinder1;

//  *cloud = *cube +*cube1 ;
//  *cloud += *sphere1;


  //Публикация облака
  ros_fun(cloud, ros_cloud);

  ROS_INFO_STREAM("Finish processing, publishing...");

  while (ros::ok()) {
    pub.publish(ros_cloud);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
