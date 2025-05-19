#include <string>
#include <vector>
#include "ros/ros.h"
#include <mutex>
#include <geometry_msgs/Point.h>
#include <autoware_msgs/Lane.h>

#include "apsrc_msgs/SignalPhaseAndTiming.h"
#include "apsrc_msgs/SPaTnMAP.h"
#include "std_msgs/Int32.h"

#include <yaml-cpp/yaml.h>


struct virtual_intersection
{
    uint16_t intersection_id;
    int64_t signal_group;
    int32_t waypoint_id;
    int32_t departure_id;
    float time_offset;
    float latitude;
    float longitude;
    float cycletime_red;
    float cycletime_yellow;
    float cycletime_green;
    float time_gap;
};

struct dist_point
{
    int32_t waypoint_id;
    int32_t target_id;
    float distance;
};

double distanceBetweenPoints(const geometry_msgs::Point& begin, const geometry_msgs::Point& end) const
{
  // Calculate the distance between the waypoints
  tf::Vector3 v1(begin.x, begin.y, begin.z);
  tf::Vector3 v2(end.x, end.y, end.z);

  return tf::tfDistance(v1, v2);
}


class V2xVirtualMap
{
private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber spat_sub_;
    ros::Subscriber closest_waypoint_sub_;
    ros::Subscriber waypoints_sub_;
    ros::Publisher spatnmap_pub_;

    std::mutex wp_mtx_;

    std::string virtual_map_file_;
    bool loop_ = false;
    
    autoware_msgs::Lane base_waypoints_;
    dist_point distance_ref_;

    int32_t closest_waypoint_;
    std::vector<virtual_intersection> virtual_intersections_;
    size_t closest_intersection_ = -1;

public:
    V2xVirtualMap(){
        nh_ = ros::NodeHandle();
        pnh_ = ros::NodeHandle("~");
        loadParams();
        loadVirtualMap();

        // Subscribers
        closest_waypoint_sub_  = nh_.subscribe("closest_waypoint", 10, &V2xVirtualMap::closestWaypointCallback, this);
        spat_sub_ = nh_.subscribe("/v2x/SPaT", 10, &V2xVirtualMap::spatCallback, this);
        waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &V2xVirtualMap::baseWaypointCallback, this);

        // Publisher(s)
        spatnmap_pub_ = nh_.advertise<apsrc_msgs::SPaTnMAP>("/v2x/SPaTnMAP", 1, true);
    }

    void loadParams(){
        pnh_.param<std::string>("virtual_map_file", virtual_map_file_, "./config/virtual_map.yaml");
        pnh_.param<bool>("enable_loop", loop_, false);
        ROS_INFO("Parameters Loaded");
        return;
    }

    void loadVirtualMap(){
        ROS_INFO("Loading Virtual Map: %s", virtual_map_file_.c_str());
        // Load the virtual map from the YAML file
        YAML::Node config = YAML::LoadFile(virtual_map_file_);
 
        // Check if the file was loaded successfully
        if (!config.IsDefined()) {
            ROS_ERROR("Failed to load virtual map file: %s", virtual_map_file_.c_str());
            exit(1);
        }

        virtual_intersections_.clear();
        YAML::Node virtual_map = config["virtual_map"];


        // Parse the virtual map data
        for (const auto& intersection : virtual_map) {
            uint16_t intersection_id = intersection["intersection_id"].as<uint16_t>();
            int64_t signal_group = intersection["signal_group"].as<int64_t>();
            int32_t waypoint_id = intersection["waypoint_id"].as<int32_t>();
            int32_t departure_id = intersection["departure_id"].as<int32_t>();
            float latitude = intersection["latitude"].as<float>();
            float longitude = intersection["longitude"].as<float>();
            float cycletime_red = intersection["cycletime_red"].as<float>();
            float cycletime_yellow = intersection["cycletime_yellow"].as<float>();
            float cycletime_green = intersection["cycletime_green"].as<float>();
            float time_offset = intersection["time_offset"].as<float>();

            // Store the data in a suitable data structure
            virtual_intersection vi;
            vi.intersection_id = intersection_id;
            vi.signal_group = signal_group;
            vi.waypoint_id = waypoint_id;
            vi.departure_id = departure_id;
            vi.latitude = latitude;
            vi.longitude = longitude;
            vi.time_offset = time_offset;
            vi.cycletime_red = cycletime_red;
            vi.cycletime_yellow = cycletime_yellow;
            vi.cycletime_green = cycletime_green;
            vi.time_gap = time_gap;
            virtual_intersections_.push_back(vi);
        }

        //sort the virtual intersections by waypoint_id
        std::sort(virtual_intersections_.begin(), virtual_intersections_.end(),
                  [](const virtual_intersection& a, const virtual_intersection& b) {
                      return a.waypoint_id < b.waypoint_id;
                  });
        ROS_INFO("Virtual Map Loaded with %zu intersections", virtual_intersections_.size());
        closest_intersection_ = 0;
        return;                  
    }

    void closestWaypointCallback(const std_msgs::Int32::ConstPtr& msg){
        std::unique_lock<std::mutex> wp_lock(wp_mtx_);
        closest_waypoint_ = msg->data;
        if (closest_waypoint_ == -1){
            ROS_WARN("No closest waypoint received");
            return;
        }
        if (virtual_intersections_[closest_intersection_].departure_id <= closest_waypoint_){
            if (closest_intersection_ +1 < virtual_intersections_.size()){
                closest_intersection_++;
                distance_ref_.target_id = virtual_intersections_[closest_intersection_].waypoint_id;
                distance_ref_.waypoint_id = closest_waypoint_;
                distance_ref_.distance = distanceBetweenWaypoints(closest_waypoint_, virtual_intersections_[closest_intersection_].target_id);
            } else {
                if (loop_){
                    closest_intersection_ = 0;
                    distance_ref_.target_id = virtual_intersections_[closest_intersection_].waypoint_id;
                    distance_ref_.waypoint_id = closest_waypoint_;
                    distance_ref_.distance = distanceBetweenWaypoints(closest_waypoint_, virtual_intersections_[closest_intersection_].target_id);
                } else {
                    return;
                }
            }
        }
        return;
    }

    void spatCallback(const apsrc_msgs::SignalPhaseAndTiming::ConstPtr& msg){
        if (msg->intersections.intersectionState[0].messageId == virtual_intersections_[closest_intersection_].intersection_id){
            apsrc_msgs::MovementList tmp = msg->intersections.intersectionState[0].states;
            for (auto& m : tmp.movementState){
                if (m.signalGroup == virtual_intersections_[closest_intersection_].signal_group){
                    apsrc_msgs::SPaTnMAP msg;
                    msg.header.stamp = ros::Time::now();
                    msg.intersection_id = virtual_intersections_[closest_intersection_].intersection_id;
                    msg.stop_waypoint = virtual_intersections_[closest_intersection_].waypoint_id;
                    msg.depart_waypoint = virtual_intersections_[closest_intersection_].departure_id;
                    // msg.distance_to_stop = msg.stop_waypoint - closest_waypoint_;
                    msg.distance_to_stop = update_distance();
                    msg.cycle_time_red = virtual_intersections_[closest_intersection_].cycletime_red;
                    msg.cycle_time_yellow = virtual_intersections_[closest_intersection_].cycletime_yellow;
                    msg.cycle_time_green = virtual_intersections_[closest_intersection_].cycletime_green;

                    // Calculate time to stop
                    time_t whole_seconds = static_cast<time_t>(std::floor(msg.header.stamp.toSec()));
                    double fractional_part = msg.header.stamp.toSec() - whole_seconds;
                    
                    std::tm* time_info = std::gmtime(&whole_seconds);
                    int seconds_since_hour = time_info->tm_min * 60 + time_info->tm_sec;
                    double total_seconds_of_hour = seconds_since_hour + fractional_part;
                    msg.time_to_stop = m.stateTimeSpeed[0].minEndTime - total_seconds_of_hour + virtual_intersections_[closest_intersection_].time_offset;
                    if (msg.time_to_stop < 0){
                        msg.time_to_stop += 3600;
                    }
                    msg.phase = m.stateTimeSpeed[0].eventState.state;
                    spatnmap_pub_.publish(msg);
                }
            }
        }
        return;
    }

    void baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& msg)
    {    
        std::unique_lock<std::mutex> wp_lock(wp_mtx_);
        base_waypoints_ = *msg;
        return;
    }

    double distanceBetweenWaypoints(const int& begin, const int& end) const
    {
        // Check index
        if (begin < 0 || begin >= base_waypooint.size() || end < 0 || end >= base_waypooint.size())
        {
            return -1.0;
        }

        int sign = 1;
        if (begin > end)
        {
            sign = -1;
            std::swap(begin, end);
        }

        // Calculate the distance between the waypoints
        double dist_sum = 0.0;
        for (int i = begin; i < end; i++)
        {
            dist_sum += distanceBetweenPoints(
            original_waypoints_.waypoints[i].pose.pose.position,
            original_waypoints_.waypoints[i + 1].pose.pose.position);
        }
        return dist_sum * sign;
    }

    float update_distance()
    {
        distance_ref_.distance += distanceBetweenWaypoints(closest_waypoint_, distance_ref_.waypoint_id);
        distance_ref_.waypoint_id = closest_waypoint_;
        return distance_ref_.distance;
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "v2x_virtual_map_node");
    V2xVirtualMap virtual_map_node;
    ros::spin();
    return 0;
}