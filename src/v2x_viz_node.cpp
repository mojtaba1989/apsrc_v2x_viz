#include "ros/ros.h"
#include <apsrc_msgs/BasicSafetyMessage.h>
#include "geodesy/utm.h"
#include <visualization_msgs/Marker.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

struct BSM_node
{
  std::string id;
  double x;
  double y;
  double yaw;
};

class BsmSubscriber {
public:
  BsmSubscriber(){
    nh_ = ros::NodeHandle();
    loadParams();
    bsm_sub_ = nh_.subscribe(bsm_topic_, 10, &BsmSubscriber::bsmCallback, this);
    gps_sub_ = nh_.subscribe(gps_topic_, 10, &BsmSubscriber::gpsCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 10, &BsmSubscriber::imuCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("v2x/viz/markers", 1, true);   
  }

  void loadParams()
  {
    nh_.param<std::string>("bsm_topic", bsm_topic_, "/v2x/BasicSafetyMessage");
    nh_.param<std::string>("gps_topic", gps_topic_, "/gps/gps");
    nh_.param<std::string>("imu_topic", imu_topic_, "/gps/imu");
    nh_.param("MA_gain", MA_gain_, 0.1);
    ROS_INFO("Parameters Loaded");
  }

  void gpsCallback(const gps_common::GPSFix::ConstPtr& msg){
    loc_ = *msg;
    geographic_msgs::GeoPoint geo_ego = {};
    geo_ego.altitude = static_cast<double>(msg->altitude);
    geo_ego.latitude = static_cast<double>(msg->latitude);
    geo_ego.longitude = static_cast<double>(msg->longitude);
    geodesy::UTMPoint utm_ego(geo_ego);
    utm_ego_ = utm_ego;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    heading_ = M_PI/2 + tf::getYaw(msg->orientation);
  }

  void bsmCallback(const apsrc_msgs::BasicSafetyMessage::ConstPtr& msg) {
    int node_idx = -1;
    if (BSM_node_list_.size()>0){
      for (size_t idx = 0; idx < BSM_node_list_.size(); ++idx){
        if (BSM_node_list_[idx].id == msg->BSMCore.ID){
          node_idx = idx;
          break;
        }
      }
    }

    if (node_idx == -1) {
      BSM_node node;
      node.id = msg->BSMCore.ID;
      BSM_node_list_.push_back(node);
      node_idx = BSM_node_list_.size() - 1;
    }

    geographic_msgs::GeoPoint geo_target = {};
    geo_target.altitude = static_cast<double>(msg->BSMCore.Elevation);
    geo_target.latitude = static_cast<double>(msg->BSMCore.Latitude);
    geo_target.longitude = static_cast<double>(msg->BSMCore.Longitude);

    geodesy::UTMPoint utm_target(geo_target);

    double deltaN = utm_target.northing - utm_ego_.northing;
    double deltaE = utm_target.easting - utm_ego_.easting;

    double x = cos(heading_) * deltaE + sin(heading_) * deltaN; 
    double y = -sin(heading_) * deltaE + cos(heading_) * deltaN;

    double new_yaw = atan2((utm_target.northing-BSM_node_list_[node_idx].y),
                            (utm_target.easting-BSM_node_list_[node_idx].x)) - heading_;

    BSM_node_list_[node_idx].yaw = BSM_node_list_[node_idx].yaw * (1-MA_gain_) + new_yaw * MA_gain_;
    BSM_node_list_[node_idx].x = utm_target.easting;
    BSM_node_list_[node_idx].y = utm_target.northing;

    visualization_msgs::Marker marker = {};
    marker.header.frame_id = "base_link";
    marker.header.stamp = msg->header.stamp;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, BSM_node_list_[node_idx].yaw);
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = .5;
    marker.color.r = 100;
    marker.color.b = 100;
    marker.color.g = 100;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://detected_objects_visualizer/models/car.dae";
    marker_pub_.publish(marker);


    visualization_msgs::Marker text; 
    text.header.frame_id = "base_link";
    text.id = 1;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x = x;
    text.pose.position.y = y;
    text.pose.position.z = 3; 
    text.pose.orientation.x = 0.0;
    text.pose.orientation.y = 0.0;
    text.pose.orientation.z = 0.0;
    text.pose.orientation.w = 1.0;
    text.scale.x = 1.0;
    text.scale.y = 1.0;
    text.scale.z = 1.0;
    text.color.r = 0.0;
    text.color.g = 1.0;
    text.color.b = 0.0;
    text.color.a = 1.0;
    text.text = "ID: " + msg->BSMCore.ID + "\n" +
    "heading:" + std::to_string(BSM_node_list_[node_idx].yaw) + "\n" +
    "dN: "  + std::to_string(deltaN) + "m\n" + 
    "dE: " + std::to_string(deltaE) + "m\n";
    marker_pub_.publish(text);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber bsm_sub_, gps_sub_, imu_sub_;
  ros::Publisher marker_pub_;

  std::string gps_topic_, imu_topic_, bsm_topic_;

  geodesy::UTMPoint utm_ego_;
  gps_common::GPSFix loc_;
  double heading_;

  std::vector<BSM_node> BSM_node_list_;
  double MA_gain_ = .1;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "v2x_viz_node");
    BsmSubscriber subscriber;
    ros::spin();
    return 0;
}
