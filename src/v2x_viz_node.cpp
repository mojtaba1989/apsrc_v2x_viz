#include "ros/ros.h"
#include <apsrc_msgs/BasicSafetyMessage.h>
#include <apsrc_msgs/BlindSpotChecker.h>
#include "geodesy/utm.h"
#include <visualization_msgs/Marker.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <derived_object_msgs/ObjectWithCovarianceArray.h>
#include <geometry_msgs/TwistStamped.h>


struct BSM_node
{
  ros::Time stamp;
  bool active;
  std::string id;
  double x;
  double y;
  double yaw;
  double abs_x;
  double abs_y;
  double velocity;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  bool within_blindspot = false;
};

class BsmSubscriber {
public:
  BsmSubscriber(){
    nh_ = ros::NodeHandle();
    pnh_ = ros::NodeHandle("~");
    loadParams();

    bsm_sub_ = nh_.subscribe(bsm_topic_, 10, &BsmSubscriber::bsmCallback, this);
    gps_sub_ = nh_.subscribe(gps_topic_, 10, &BsmSubscriber::gpsCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 10, &BsmSubscriber::imuCallback, this);

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("v2x/viz/markers", 1, true);
    blind_spot_check_pub_ = nh_.advertise<apsrc_msgs::BlindSpotChecker>("/v2x/BlindSpotChecker", 1, true);
    bs_timer_ = nh_.createTimer(ros::Duration(1/bs_freq_), std::bind(&BsmSubscriber::blindSpotPublisher, this));
    

    if (fake_lidar_){
      lidar_mod_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(lidar_topic_pub_, 1, true);
      lidar_sub_     = nh_.subscribe(lidar_topic_sub_, 10, &BsmSubscriber::lidarCallback, this);
      timer_ = nh_.createTimer(ros::Duration(1/cleanup_freq_), std::bind(&BsmSubscriber::cleanUpCallback, this));
    }

    if (fake_radar_){
      radar_pub_       = nh_.advertise<derived_object_msgs::ObjectWithCovarianceArray>("/fake_radar/fc", 1, true);
      current_speed_sub_  = nh_.subscribe("current_velocity", 1, &BsmSubscriber::velocityCallback, this);
      radar_timer_        = nh_.createTimer(ros::Duration(1/radar_freq_), std::bind(&BsmSubscriber::radarCallback, this));
    }
    
  }

  void cleanUpCallback()
  {
    if (BSM_node_list_.size() == 0){
      return;
    }
    std::vector<BSM_node> tmp;
    for (size_t idx = 0; idx < BSM_node_list_.size(); ++idx){
      if (ros::Time::now().toSec() - BSM_node_list_[idx].stamp.toSec() <= 1/cleanup_freq_){
        tmp.push_back(BSM_node_list_[idx]);
      }
    }
    BSM_node_list_ = tmp;
  }

  void loadParams()
  {
    pnh_.param<bool>("fake_lidar", fake_lidar_, false);
    pnh_.param<std::string>("pcd_file", pcd_FILENAME_, "null.pcd");
    pnh_.param<std::string>("lidar_topic_listener", lidar_topic_sub_, "/points_raw");
    pnh_.param<std::string>("lidar_topic_publisher", lidar_topic_pub_, "/points_mod");
    pnh_.param<std::string>("bsm_topic", bsm_topic_, "/v2x/BasicSafetyMessage");
    pnh_.param<std::string>("gps_topic", gps_topic_, "/gps/gps");
    pnh_.param<std::string>("imu_topic", imu_topic_, "/gps/imu");
    pnh_.param("cleanup_freq", cleanup_freq_, 1.0);
    pnh_.param("MA_gain", MA_gain_, 0.1);

    pnh_.param<std::string>("ignore_id", ignore_id_, "00000001");

    pnh_.param<bool>("fake_radar", fake_radar_, false);
    pnh_.param("radar_freq", radar_freq_, 1.0);
    pnh_.param("radar_to_baseline", radar_to_baseline_, 4.8);

    pnh_.param<bool>("enable_blindspot", enable_blindspot_, false);
    pnh_.param("blindspot_width", blindspot_width_, 3.3);
    pnh_.param("blindspot_length", blindspot_length_, 15.0);
    pnh_.param("blindspot_max_T", blindspot_max_T_, 4.0);
    pnh_.param("blindspot_step_T", blindspot_step_T_, 0.5);
    pnh_.param("blindspot_in_lane", blindspot_in_lane_, 1.0);
    pnh_.param("blindspot_publish_freq", bs_freq_, 1.0);

    ROS_INFO("Parameters Loaded");

    if (!fake_lidar_){
      ROS_INFO("%s", pcd_FILENAME_.c_str());
      return;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_FILENAME_, *dummy_pc_) == -1)
    {
      ROS_WARN("Couldn't read PCD file \n");
      fake_lidar_ = false;
    } else {
      ROS_INFO("PCD loaded \n");
    }
    return;
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
    if (msg->BSMCore.ID == ignore_id_){
      return;
    }
    
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
      if (fake_lidar_){
        pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_FILENAME_, *node.cloud);
      }
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
    BSM_node_list_[node_idx].abs_x = x;
    BSM_node_list_[node_idx].abs_y = y;
    BSM_node_list_[node_idx].stamp = ros::Time::now();
    BSM_node_list_[node_idx].velocity = msg->BSMCore.Speed;
    BSM_node_list_[node_idx].within_blindspot = checkBlindSpot(BSM_node_list_[node_idx].abs_x,
                                                               BSM_node_list_[node_idx].abs_y, 
                                                               BSM_node_list_[node_idx].velocity);

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
    if (BSM_node_list_[node_idx].within_blindspot){
      text.color.r = 1.0;
      text.color.g = 0.0;
      text.color.b = 0.0;
      text.color.a = 1.0;
    } else {
      text.color.r = 0.0;
      text.color.g = 1.0;
      text.color.b = 0.0;
      text.color.a = 1.0;
    }
    // text.text = "ID: " + msg->BSMCore.ID + "\n" +
    // "heading:" + std::to_string(BSM_node_list_[node_idx].yaw) + "\n" +
    // "dN: "  + std::to_string(deltaN) + "m\n" + 
    // "dE: " + std::to_string(deltaE) + "m\n";
    text.text = "ID: " + msg->BSMCore.ID + "\n";
    marker_pub_.publish(text);
  }

  bool checkBlindSpot(double x, double y, double velocity){
    if (!enable_blindspot_){ // Check if blindspot is enabled
      return false;
    }
    if (abs(y) > blindspot_width_ || abs(y) <= blindspot_in_lane_){ // Check if the vehicle is in the blindspot
      return false;
    }
    if (abs(x) > 5 * blindspot_length_){ // Check if the vehicle is too far away
      return false;
    }
    double T = 0;
    double delta_vel = current_speed_ - velocity;
    while (T <= blindspot_max_T_){
      if (abs(x-T*delta_vel)<= blindspot_length_){
        return true;
      }
      T += blindspot_step_T_;
    }
    return false;
  }

  void blindSpotPublisher(){
    apsrc_msgs::BlindSpotChecker msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "lidar";
    msg.left = false;
    msg.right = false;
    for (size_t bsm_idx = 0; bsm_idx<BSM_node_list_.size();bsm_idx++){
      if (BSM_node_list_[bsm_idx].within_blindspot){
        if (BSM_node_list_[bsm_idx].y > 0){
          msg.left = true;
        } else {
          msg.right = true;
        }
      }
    }
    blind_spot_check_pub_.publish(msg);
  }

  void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *raw);
    pcl::PointCloud<pcl::PointXYZI> combined_cloud = *raw;

    if (BSM_node_list_.size()> 0){
      for (size_t bsm_idx = 0; bsm_idx<BSM_node_list_.size();bsm_idx++){
        float z = -1.98;
        transform_.translation() << BSM_node_list_[bsm_idx].abs_x, BSM_node_list_[bsm_idx].abs_y, z;

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*BSM_node_list_[bsm_idx].cloud, *transformed_cloud, transform_);

        combined_cloud += *(transformed_cloud);
      }
    }
    pcl::toROSMsg(combined_cloud, pcd_msg_);
    pcd_msg_.header.frame_id = "lidar";
    lidar_mod_pub_.publish(pcd_msg_);
    return;
  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_speed_ = msg->twist.linear.x;
  }

  void radarCallback(){
    derived_object_msgs::ObjectWithCovarianceArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "radar_fc";
    if (BSM_node_list_.size() == 0){
      radar_pub_.publish(msg);
      return;
    }
    for (size_t bsm_idx = 0; bsm_idx<BSM_node_list_.size();bsm_idx++){
      derived_object_msgs::ObjectWithCovariance obj;
      obj.id = static_cast<uint32_t>(std::stoi(BSM_node_list_[bsm_idx].id));
      obj.pose.pose.position.x = BSM_node_list_[bsm_idx].abs_x - radar_to_baseline_;
      obj.pose.pose.position.y = BSM_node_list_[bsm_idx].abs_y;
      obj.pose.pose.position.z = 0.0;
      obj.pose.pose.orientation.x = 0.0;
      obj.pose.pose.orientation.y = 0.0;
      obj.pose.pose.orientation.z = 0.0;
      obj.pose.pose.orientation.w = 1.0;
      obj.twist.twist.linear.x = BSM_node_list_[bsm_idx].velocity - current_speed_;
      msg.objects.push_back(obj);
    }
    radar_pub_.publish(msg);
    return;
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber bsm_sub_, gps_sub_, imu_sub_, lidar_sub_;
  ros::Publisher marker_pub_, lidar_mod_pub_, blind_spot_check_pub_;

  std::string gps_topic_, imu_topic_, bsm_topic_;

  geodesy::UTMPoint utm_ego_;
  gps_common::GPSFix loc_;
  double heading_;

  std::vector<BSM_node> BSM_node_list_;
  double MA_gain_ = .1;

  bool fake_lidar_ = false;
  std::string pcd_FILENAME_, lidar_topic_sub_, lidar_topic_pub_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr dummy_pc_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 pcd_msg_;
  Eigen::Affine3f transform_ = Eigen::Affine3f::Identity();
  std::string ignore_id_ = "00000001";

  bool fake_radar_ = false;
  ros::Publisher radar_pub_;
  ros::Subscriber current_speed_sub_;
  double current_speed_ = 0.0;
  ros::Timer radar_timer_;
  double radar_freq_ = 10.0;
  double radar_to_baseline_ = 3.0;


  double cleanup_freq_ = 1.0;
  ros::Timer timer_;

  
  bool enable_blindspot_ = true;
  double blindspot_width_ = 3.3;
  double blindspot_in_lane_ = 1.0;
  double blindspot_length_ = 15.0;
  double blindspot_max_T_ = 5.0;
  double blindspot_step_T_ = 0.5;
  ros::Timer bs_timer_;
  double bs_freq_ = 1.0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "v2x_viz_node");
    BsmSubscriber subscriber;
    ros::spin();
    return 0;
}
