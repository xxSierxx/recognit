#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
float height, width, fx, fy, cx, cy, px,py,pz, s = 1000;
sensor_msgs::Image img;

void funImage(const sensor_msgs::Image &info){
    img = info;
    ROS_INFO_STREAM("image recived");
}

void funCamera(const sensor_msgs::CameraInfo &info){
    height = info.height;
    width = info.width;
    fx = info.K[0];
    fy = info.K[4];
    cx = info.K[2];
    cy = info.K[5];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointCloud_node");
    ros::NodeHandle nh;
    ros::Subscriber depth =
            nh.subscribe  ( "/camera/depth/image_raw", 10,funImage);
    ros::Subscriber camera =
            nh.subscribe  ( "/camera/depth/camera_info", 10,funCamera);
    ros::Publisher pub =
            nh.advertise <sensor_msgs::PointCloud>("/pointCloud",10);
    ros::Publisher pub1 =
            nh.advertise <sensor_msgs::PointCloud2>("/pointCloud1",10);
    ros::Rate r(10);


    while (ros::ok()){
        sensor_msgs::PointCloud pointCloud;
        sensor_msgs::ChannelFloat32 channel;
        sensor_msgs::PointCloud2 cloud2;
        channel.name = "channel 1";
        pointCloud.channels.push_back(channel);
        for(int i = 0; i < height; i++) {
            for(int j = 0; j < width*2; j+=2) {
                geometry_msgs::Point32 point;
                unsigned short d = (img.data[i*img.step+j]) + (img.data[i*img.step+j+1]*256);
                point.z = d/s;
                point.x = (i - cx)*point.z/fx;
                point.y = (j/2 - cy)*point.z/fy;
                channel.values.push_back(point.z);
                pointCloud.points.push_back(point);
            }
        }
           pointCloud.header.frame_id = "cld";

           //second type

            int numpoints = pointCloud.points.size();
            cloud2.header.frame_id = "cld2";
            cloud2.width = numpoints;
            cloud2.height = 1;
            cloud2.is_bigendian = false;
            cloud2.is_dense = false;

            sensor_msgs::PointCloud2Modifier modifier(cloud2);
            modifier.setPointCloud2FieldsByString(2,"xyz","rgb");
            modifier.resize(numpoints);
            sensor_msgs::PointCloud2Iterator<float> out_x(cloud2, "x");
            sensor_msgs::PointCloud2Iterator<float> out_y(cloud2, "y");
            sensor_msgs::PointCloud2Iterator<float> out_z(cloud2, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud2, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud2, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud2, "b");
            for (int idx=0; idx < pointCloud.points.size(); idx++)
            {
            *out_x = pointCloud.points.at(idx).x;
            *out_y = pointCloud.points.at(idx).y;
            *out_z = pointCloud.points.at(idx).z;
            *out_r = 15;
            *out_g = 45;
            *out_b = 190;
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_r;
            ++out_g;
            ++out_b;
            }
            pub.publish(pointCloud);
            pub1.publish(cloud2);
            ros::spinOnce();
            r.sleep();
    }



    return 0;
}




