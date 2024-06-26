#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

class WeedRealPose{
    ros::NodeHandle nh;
    ros::Subscriber points_sub;
    ros::Subscriber xy_sub;
    ros::Publisher pose_pub;

    int x = 0; 
    int y = 0;

    public: 
        WeedRealPose(){
            points_sub = nh.subscribe("/camera/depth/points",1, &WeedRealPose::cloudCb, this);
            pose_pub = nh.advertise<geometry_msgs::Point>("/weed_real_xyz", 10);
            xy_sub = nh.subscribe("/weed_xy",1, &WeedRealPose::xyCb, this);
        }
        void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
            if((cloud_msg->width * cloud_msg->height) == 0){
                return;
            }
            try {
                pcl::PointCloud<pcl::PointXYZ> pose;
                pcl::fromROSMsg(*cloud_msg, pose);
                pcl::PointXYZ weed_pose = pose.at(x,y);
                geometry_msgs::Point pose_msg;
                pose_msg.x = weed_pose.x;
                pose_msg.y = weed_pose.y;
                pose_msg.z = weed_pose.z;
                pose_pub.publish(pose_msg);
            } 
            catch (std::exception &e) {
                ROS_ERROR("Error: %s", e.what());
            }
        }
        void xyCb(const std_msgs::Int16MultiArray &msg){
            x = msg.data[0];
            y = msg.data[1];
        }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "weed_real_pose");
    WeedRealPose weed_real_pose;
    ros::spin();
    return 0;
}