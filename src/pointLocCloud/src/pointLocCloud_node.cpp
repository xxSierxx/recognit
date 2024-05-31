#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>


static sensor_msgs::PointCloud2 currentCloud;
bool isNewData;
int th = 58;
float l = -0.16 + 0.15;


void pixelTo3DPoint(const sensor_msgs::PointCloud2 &pCloud, const int u, const int v, geometry_msgs::Point &p) {

    int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;
    int arrayPosX = arrayPosition + pCloud.fields[0].offset;
    int arrayPosY = arrayPosition + pCloud.fields[1].offset;
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset;

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;
}

void pointCloud2Callback(const sensor_msgs::PointCloud2 &msg)
{
    currentCloud = msg;
    isNewData = true;
}

//y - z

nav_msgs::OccupancyGrid mapkin(sensor_msgs::PointCloud2 &cloud ) {

    //Заполнение карты проходимости
    nav_msgs::OccupancyGrid lmap;
    geometry_msgs::Point point;

    lmap.info.resolution = 0.1;
    float map_size_m = 20;
    lmap.info.width = map_size_m / lmap.info.resolution;
    lmap.info.height = map_size_m / lmap.info.resolution;
    lmap.info.origin.position.x = -map_size_m/2.0;
    lmap.info.origin.position.y = -map_size_m/2.0;
    lmap.header.frame_id = "camera_depth_optical_frame";
    lmap.header.stamp = ros::Time::now();
    lmap.data.resize(lmap.info.width * lmap.info.height);

    for (int row = 0; row<lmap.info.height; row++){
        for (int col = 0; col<lmap.info.width; col++){
            int index = row * lmap.info.width + col;
            lmap.data[index] = -1;
        }
    }

    //Препятствия
    for(int i = 0; i < cloud.width; i++) {
        for(int j = 0; j < cloud.height; j++) {

            pixelTo3DPoint(cloud, i, j, point);
            if (-point.y>l) {
                int point_col = (-lmap.info.origin.position.x + point.x)/lmap.info.resolution;
                int point_row = (-lmap.info.origin.position.y + point.z)/lmap.info.resolution;
                int index = point_row * lmap.info.width + point_col;

                if ( (index >= 0) && (index < lmap.info.width * lmap.info.height)) lmap.data[index]=100;

            }
        }
    }

    //Трассировка

    float th_incr = asin(lmap.info.resolution/(map_size_m*lmap.info.resolution));

    float th_max = th * M_PI / 180;
    int luch = th_max / th_incr;

    for(int i = 0; i < luch; i++) {

        //Вычисляем угол для поворота лучей.
        float theta = -th_max/2 + i * th_incr + 1.57;
        float itr=0;
        int prep_point_col = (-lmap.info.origin.position.x)/lmap.info.resolution;
        int prep_point_row = (-lmap.info.origin.position.y)/lmap.info.resolution;
        int index = prep_point_row * lmap.info.width + prep_point_col;

        while ( (lmap.data[index] != 100) && (prep_point_col <= lmap.info.width) && (prep_point_row <= lmap.info.height)){
            lmap.data[index] = 0;
            itr+=lmap.info.resolution;
            float x1 = itr*cos(theta);
            float y1 = itr*sin(theta);
            prep_point_col = (-lmap.info.origin.position.x+x1)/lmap.info.resolution;
            prep_point_row = (-lmap.info.origin.position.y+y1)/lmap.info.resolution;
            index = prep_point_row * lmap.info.width + prep_point_col;
        }
    }
    return lmap;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "pointLocCloud_node");
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub =
            nh.subscribe("/camera/depth/points", 8, pointCloud2Callback);
    ros::Publisher pub =
            nh.advertise<nav_msgs::OccupancyGrid>("/pointLocalMap", 8);

    ros::Rate r(10);
    while (ros::ok()) {

        if (isNewData){
            pub.publish(mapkin(currentCloud));
            isNewData = false;
        }

        ros::spinOnce();
        r.sleep();
    }
}
