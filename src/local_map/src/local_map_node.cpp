#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

static bool isNewScan;
static sensor_msgs::LaserScan scan_data;
void laserDataCallback(
        const sensor_msgs::LaserScan &msg) {
    scan_data = msg;
    isNewScan = true;
}

nav_msgs::OccupancyGrid formMap(sensor_msgs::LaserScan &scan) {


    //Заполнение карты проходимости

    nav_msgs::OccupancyGrid local_map, global_map;
    local_map.info.resolution = 0.04;
    float map_size_m = 12;

    local_map.info.width = map_size_m / local_map.info.resolution;
    local_map.info.height =  local_map.info.width;
    local_map.info.origin.position.x = - map_size_m/2.0;
    local_map.info.origin.position.y = - map_size_m/2.0;

    local_map.header.frame_id = "map";
    local_map.header.stamp = ros::Time::now();
    local_map.data.resize(local_map.info.width * local_map.info.height);


    for(int row = 0; row < local_map.info.height; row++) {
        for(int col = 0; col < local_map.info.width; col++) {
            int index = row * local_map.info.width + col;
            local_map.data[index] = -1;
        }
    }


    for(int i = 0; i < scan.ranges.size(); i++) {

        if(scan.ranges.at(i)<=6){
            float angle = scan.angle_min + i * scan.angle_increment;
            //Координаты препятствия
            float x = scan.ranges.at(i) * cos(angle);
            float y = scan.ranges.at(i) * sin(angle);
            int point_col = (-local_map.info.origin.position.x + x)
                    / local_map.info.resolution ;
            int point_row = (-local_map.info.origin.position.y + y)
                    / local_map.info.resolution ;
            int index = point_row * local_map.info.width + point_col;
            local_map.data[index] = 100;

            float r = 0;

            for(int j = 0; j<scan.ranges.at(i)/local_map.info.resolution; j++) {
                r +=  local_map.info.resolution;
                float x1 = r * cos(angle);
                ROS_INFO_STREAM("x: "<<x);
                float y1 = r * sin(angle);
                int point_col1 = (-local_map.info.origin.position.x + x1)
                        / local_map.info.resolution ;
                int point_row1 = (-local_map.info.origin.position.y + y1)
                        / local_map.info.resolution ;
                int index1 = point_row1 * local_map.info.width + point_col1;
                local_map.data[index1] = 0;

            }

        }
    }
    return local_map;

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_map_pkg_node");
    ros::NodeHandle nh;
    isNewScan = false;


    //Указать частоту, подписчиков, публикаторов
    //и другие данные


    ros::Subscriber scan_sub = nh.subscribe("/scan",8, laserDataCallback);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>
            ("/local_map",8);
    ros::Rate r(1);
    while(ros::ok()) {
        ros::spinOnce();
        if(isNewScan) {
            map_pub.publish(formMap(scan_data));
            isNewScan = false;
        }
        r.sleep();
    }
}
