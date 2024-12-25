#include <string>
#include <vector>
#include "ros/ros.h"
#include "geodesy/utm.h"
#include <visualization_msgs/Marker.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/Imu.h>
#include <autoware_msgs/Lane.h>
#include <std_msgs/Int32.h>
#include <mutex>

#include <cmath>

#include "apsrc_msgs/MapData.h"
#include "apsrc_msgs/SignalPhaseAndTiming.h"
#include "apsrc_msgs/SPaTnMAP.h"
#include "apsrc_msgs/GenericLane.h"
#include "apsrc_msgs/IntersectionGeometry.h"

struct laneInfo{
  uint8_t id;
  geometry_msgs::Point P;
  float dist_to_ref;
  float dist_to_wp;
  int16_t closest_wp = -1;
};

class SPaTnMAP 
{
public:
  SPaTnMAP(){
    nh_ = ros::NodeHandle();
    loadParams();

    // Subscribers
    spat_sub_ = nh_.subscribe(spat_topic_, 10, &SPaTnMAP::spatCallback, this);
    map_sub_ = nh_.subscribe(map_topic_, 1, &SPaTnMAP::mapCallback, this);
    base_waypoints_sub_   = nh_.subscribe("base_waypoints", 1, &SPaTnMAP::baseWaypointCallback, this);
    closest_waypoint_sub  = nh_.subscribe("closest_waypoint", 10, &SPaTnMAP::closestWaypointCallback, this);
    gps_sub_ = nh_.subscribe(gps_topic_, 10, &SPaTnMAP::gpsCallback, this);
    // Publisher(s)
    spatnmap_pub_ = nh_.advertise<apsrc_msgs::SPaTnMAP>("/v2x/SPaTnMAP", 1, true);
  }

  void loadParams(){
    nh_.param<std::string>("spat_topic", spat_topic_, "/v2x/SPaT");
    nh_.param<std::string>("map_topic", map_topic_, "/v2x/MapData");
    nh_.param<float>("search_radius", r_search_, 2.0);
    nh_.param<float>("min_acceptable_distance", r_min_, 0.5);
    nh_.param<std::string>("gps_topic", gps_topic_, "/gps/gps");

    return;
  }

  void spatCallback(const apsrc_msgs::SignalPhaseAndTiming::ConstPtr& msg){
    if (signal_group_==-1){
      return;
    }
    if (msg->intersections.intersectionState.size()>0){
      std::unique_lock<std::mutex> map_lock(map_mtx_);
      for (auto SigG : msg->intersections.intersectionState[0].states.movementState){
        if (SigG.signalGroup==signal_group_){
          if (SigG.stateTimeSpeed.size()>0){
            {// map mutex scope
              std::unique_lock<std::mutex> wps_lock(wps_mtx_);
              pub_msg_.phase = SigG.stateTimeSpeed[0].eventState.state;
              pub_msg_.stop_waypoint =lane_.closest_wp;
              pub_msg_.distance_to_stop = distanceBetweenWaypoints(closest_waypoint_, lane_.closest_wp);
            }
            //Claculate sec of hour in Machine based on GPS time
            {//gps mutex scope
              std::unique_lock<std::mutex> gps_lock(gps_mtx_);
              time_t whole_seconds = static_cast<time_t>(std::floor(gps_.time));
              double fractional_part = gps_.time - whole_seconds;
              
              std::tm* time_info = std::gmtime(&whole_seconds);
              int seconds_since_hour = time_info->tm_min * 60 + time_info->tm_sec;
              double total_seconds_of_hour = seconds_since_hour + fractional_part;
              float time_to_change = static_cast<float>(SigG.stateTimeSpeed[0].minEndTime)/10 - total_seconds_of_hour;
              if (time_to_change < 0){
                time_to_change += 3600;
              }
              pub_msg_.time_to_stop = time_to_change;
            }
            spatnmap_pub_.publish(pub_msg_);
          }
        }
      }
    }
    return;
  }
  
  void mapCallback(const apsrc_msgs::MapData::ConstPtr& msg){
    // Check if waypoints and closest wp topics are published 
    if (!(recv_base_waypoints_ & recv_closet_waypoint_)){
      return;
    }

    // Check if received map is already processed using intersection id
    if (!recv_map_){
      intersection_id_ = msg->Intersections.IntersectionGeometry.id;
    } else {
      if (msg->Intersections.IntersectionGeometry.id == intersection_id_ && !remap_){
        return;
      } else {
        intersection_id_ = msg->Intersections.IntersectionGeometry.id;
      }
    }

    if (remap_ && target_wps_id_.size()>0 && 
        base_waypoints_.waypoints.size()==ref_waypoints_.waypoints.size()){
      // Check if new waypoints have exact same Pose
      // Ignoring modifications of speed
      // Save unnecessary process for velocity commands
      bool change_detected = false;
      for (auto id : target_wps_id_){
        if (base_waypoints_.waypoints[id].pose.pose.position.x!=
            ref_waypoints_.waypoints[id].pose.pose.position.x ||
            base_waypoints_.waypoints[id].pose.pose.position.y!=
            ref_waypoints_.waypoints[id].pose.pose.position.y){
          change_detected = true;
          break;
        }
      }
      if (!change_detected){
        return;
      }
    }

    geographic_msgs::GeoPoint geo_target = {};
    geo_target.altitude = static_cast<double>(msg->Intersections.IntersectionGeometry.refPoint.elevation);
    geo_target.latitude = static_cast<double>(msg->Intersections.IntersectionGeometry.refPoint.latitude)/10000000;
    geo_target.longitude = static_cast<double>(msg->Intersections.IntersectionGeometry.refPoint.longitude)/10000000;
    geodesy::UTMPoint utm_target(geo_target);
    

    std:size_t num_of_lanes = msg->Intersections.IntersectionGeometry.laneSet.size();
    if (num_of_lanes==0){
      return;
    }

    // collect lanes
    std::vector<laneInfo> lanes(num_of_lanes);
    for (std::size_t i; i < num_of_lanes; i++){
      apsrc_msgs::GenericLane generic_lane = msg->Intersections.IntersectionGeometry.laneSet[i];
      if (generic_lane.nodeList.NodeSetXY.size() > 0){
        lanes[i].id = generic_lane.laneID;
        lanes[i].P.x = generic_lane.nodeList.NodeSetXY[0].delta.X + utm_target.easting;
        lanes[i].P.y = generic_lane.nodeList.NodeSetXY[0].delta.y + utm_target.northing;
        lanes[i].P.z = 0;
      }
    }

    // find intersection middle point
    geometry_msgs::Point ref;
    ref.x = 0;
    ref.y = 0;
    ref.z = 0;
    for (size_t i = 0; i < lanes.size(); i++){
      ref.x += lanes[i].P.x;
      ref.y += lanes[i].P.y;
    }
    ref.x = ref.x/lanes.size();
    ref.y = ref.y/lanes.size();

    // update dist info for lanes
    for (size_t i = 0; i < lanes.size(); i++){
      lanes[i].dist_to_ref = distanceBetweenPoints(ref, lanes[i].P);
    }

    // find search radius
    float max_dist = 0;
    for (std::size_t i; i < num_of_lanes; i++){
      if (lanes[i].dist_to_ref > max_dist){
        max_dist = lanes[i].dist_to_ref;
      }
    }
    max_dist += r_search_; // add search radius to roi

    // narrow down wps
    target_wps_id_.clear();
    {//base waypints scope
      std::unique_lock<std::mutex> wps_lock(wps_mtx_);
      ref_waypoints_ = base_waypoints_; // store for detecting changes in next call
      for (size_t i = closest_waypoint_; i < base_waypoints_.waypoints.size(); i++){
        if (distanceBetweenPoints(ref, base_waypoints_.waypoints[i].pose.pose.position) <= max_dist){
          target_wps_id_.push_back(i);
        }
      }

      // ignore irrelevent map msg
      if (target_wps_id_.size()==0){
        wps_mtx_.unlock();
        return;
      }

      //match best wp to each lane
      for (laneInfo lane : lanes){
        float min_dist_tmp = 0;
        size_t id_tmp = 0;
        for (size_t id : target_wps_id_){
          float dist = distanceBetweenPoints(lane.P, base_waypoints_.waypoints[id].pose.pose.position);
          if (dist <= min_dist_tmp){
            min_dist_tmp = dist;
            id_tmp = id;
            if (min_dist_tmp <= r_min_){
              break;
            }
          }
        }
        if (id_tmp > 0 & min_dist_tmp <= r_search_){
          lane.closest_wp = id_tmp;
          lane.dist_to_wp = min_dist_tmp;
        } else {
          lane.closest_wp = -1;
          lane.dist_to_wp = -1;
        }
      }
    }

    // clean up lanes
    auto it = std::remove_if(lanes.begin(), lanes.end(), [](const laneInfo& entry) {
        return entry.closest_wp == -1; 
    });

    lanes.erase(it, lanes.end()); 

    // Sort lanes by dist_to_wp
    std::sort(lanes.begin(), lanes.end(), [](const laneInfo& a, const laneInfo& b) {
        return a.dist_to_wp < b.dist_to_wp;
    });

    // select approach and departing lanes
    laneInfo d_lane, a_lane;
    {// closest waypoint scope
      std::unique_lock<std::mutex> wp_lock(wp_mtx_);
      if (lanes.size()>= 2){
        if (lanes[0].closest_wp < lanes[1].closest_wp){
          a_lane = lanes[0];
          d_lane = lanes[1];
        } else {
          a_lane = lanes[1];
          d_lane = lanes[0];
        }
      } else {
        return;
      }
    }
    

    {// map scope
      std::unique_lock<std::mutex> map_lock(map_mtx_);
      apsrc_msgs::IntersectionGeometry intersection = msg->Intersections.IntersectionGeometry;
      for (auto lane : intersection.laneSet){
        bool exitLoops = false;
        if (lane.laneID==a_lane.id){
          for (auto connect_to : lane.connectsTo.ConnectsToList){
            if (connect_to.ConnectingLane.lane == d_lane.id){
              signal_group_ = connect_to.SignalGroup.SignalGroupID;
              exitLoops = true;
              break;
            }
          }
        }
        if (exitLoops) break;
      }
      lane_ = a_lane;
    }
    recv_map_ = true;
    remap_ = false;
    return;
  }

  void baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& msg){
    std::unique_lock<std::mutex> wps_lock(wps_mtx_);
    base_waypoints_ = *msg;
    recv_base_waypoints_ = true;
    remap_ = true;
    return;
  }

  void closestWaypointCallback(const std_msgs::Int32::ConstPtr& msg){
    std::unique_lock<std::mutex> wp_lock(wp_mtx_);
    closest_waypoint_ = msg->data;
    recv_closet_waypoint_ = true;
    return;
  }

   void gpsCallback(const gps_common::GPSFix::ConstPtr& msg){
    std::unique_lock<std::mutex> gps_lock(gps_mtx_);
    gps_ = *msg;
  }

  // Utils
  double distanceBetweenWaypoints(const int& begin, const int& end) const
  {
    // Check index
    if (begin < 0 || end < 0 || end >= base_waypoints_.waypoints.size())
    {
      ROS_WARN_THROTTLE(1, "Invalid input index range");
      return 0.0;
    }
    double dist_sum = 0.0;
    if (begin <= end){ // departing distance presented as negative number
      for (int i = begin; i < end; i++)
      {
        dist_sum += distanceBetweenPoints(
          base_waypoints_.waypoints[i].pose.pose.position,
          base_waypoints_.waypoints[i + 1].pose.pose.position);
      }
    } else {
      for (int i = end; i < begin; i++)
      {
        dist_sum -= distanceBetweenPoints(
          base_waypoints_.waypoints[i].pose.pose.position,
          base_waypoints_.waypoints[i + 1].pose.pose.position);
      }
    }
    return dist_sum;
  }

  double distanceBetweenPoints(const geometry_msgs::Point& begin, const geometry_msgs::Point& end) const
  {
    // Calculate the distance between the waypoints
    tf::Vector3 v1(begin.x, begin.y, 0);
    tf::Vector3 v2(end.x, end.y, 0);

    return tf::tfDistance(v1, v2);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber spat_sub_, map_sub_, gps_sub_;
  ros::Subscriber base_waypoints_sub_, closest_waypoint_sub;
  ros::Publisher spatnmap_pub_;

  // launch config
  std::string spat_topic_, map_topic_;
  float r_search_ = 2;
  float r_min_ = 0.5;

  // Mutex
  std::mutex wp_mtx_, wps_mtx_, gps_mtx_, map_mtx_;

  // Waypoint_params
  int32_t closest_waypoint_;
  autoware_msgs::Lane base_waypoints_;
  bool recv_closet_waypoint_ = false;
  bool recv_base_waypoints_ = false;
  bool remap_ = true;

  // GPS for timing
  gps_common::GPSFix gps_;
  std::string gps_topic_;

  // Map pars:
  bool recv_map_ = false;
  uint16_t intersection_id_;
  int64_t signal_group_ = -1;
  laneInfo lane_;
  std::vector<size_t> target_wps_id_;
  autoware_msgs::Lane ref_waypoints_;


  // Publishing msg
  apsrc_msgs::SPaTnMAP pub_msg_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "v2x_spatnmap_node");
    SPaTnMAP spatnmap;
    ros::spin();
    return 0;
}
