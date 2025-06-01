#include <string>
#include <vector>
#include "ros/ros.h"
#include <mutex>
#include <tf/tf.h>
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

struct spatnmap_ref
{
    uint16_t intersection_id;
    float count_down;
    float end_time;
    uint8_t phase;
    double expiration_time;
};


double distanceBetweenPoints(const geometry_msgs::Point& begin, const geometry_msgs::Point& end)
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
    bool looped_ = false;
    
    autoware_msgs::Lane base_waypoints_;
    dist_point distance_ref_;

    int32_t closest_waypoint_;
    std::vector<virtual_intersection> virtual_intersections_;
    size_t closest_intersection_ = -1;

    double spatnmap_timer_freq_ = 50.0;
    spatnmap_ref phase_holder_;
    std::mutex spatnmap_mtx_;
    ros::Timer spatnmap_timer_;
    bool filler_activate_ = false;



public:
    V2xVirtualMap(){
        nh_ = ros::NodeHandle();
        pnh_ = ros::NodeHandle("~");
        loadParams();
        loadVirtualMap();

        // Subscribers
        closest_waypoint_sub_  = nh_.subscribe("closest_waypoint", 10, &V2xVirtualMap::closestWaypointCallback, this);
        waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &V2xVirtualMap::baseWaypointCallback, this);

        if (filler_activate_){
            spat_sub_ = nh_.subscribe("/v2x/SPaT", 10, &V2xVirtualMap::spatCallback_sup, this);
        } else {
            spat_sub_ = nh_.subscribe("/v2x/SPaT", 10, &V2xVirtualMap::spatCallback, this);
        }

        // Publisher(s)
        spatnmap_pub_ = nh_.advertise<apsrc_msgs::SPaTnMAP>("/v2x/SPaTnMAP", 1, true);

        // Timer
        if (filler_activate_){
            spatnmap_timer_ = nh_.createTimer(ros::Duration(1.0/spatnmap_timer_freq_), std::bind(&V2xVirtualMap::spatnmapTimerCallback, this));
        }
    }

    void loadParams(){
        pnh_.param<std::string>("virtual_map_file", virtual_map_file_, "./config/virtual_map.yaml");
        pnh_.param<bool>("enable_loop", loop_, false);
        pnh_.param("timer_freq", spatnmap_timer_freq_, 50.0);
        pnh_.param<bool>("filler_activate", filler_activate_, false);

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
        looped_ = closest_waypoint_ >= msg->data + 100 ? true : false;
        closest_waypoint_ = msg->data;
        if (closest_waypoint_ == -1){
            ROS_WARN("No closest waypoint received");
            return;
        }
        if (closest_waypoint_ > virtual_intersections_[virtual_intersections_.size()-1].departure_id){
            closest_intersection_ = 0;
            get_distance(true);
            return;
        }
        if (closest_waypoint_ > virtual_intersections_[closest_intersection_].departure_id){
            closest_intersection_ = (closest_intersection_ + 1) % virtual_intersections_.size();
            get_distance(false);
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

    void spatCallback_sup(const apsrc_msgs::SignalPhaseAndTiming::ConstPtr& msg){
        if (msg->intersections.intersectionState[0].messageId == virtual_intersections_[closest_intersection_].intersection_id){
            apsrc_msgs::MovementList tmp = msg->intersections.intersectionState[0].states;
            for (auto& m : tmp.movementState){
                if (m.signalGroup == virtual_intersections_[closest_intersection_].signal_group){
                    std::unique_lock<std::mutex> spat_lock(spatnmap_mtx_);
                    if (phase_holder_.phase == m.stateTimeSpeed[0].eventState.state 
                        && phase_holder_.end_time == m.stateTimeSpeed[0].minEndTime
                        && phase_holder_.intersection_id == msg->intersections.intersectionState[0].messageId){
                        return;
                    }
                    
                    phase_holder_.intersection_id = msg->intersections.intersectionState[0].messageId;
                    phase_holder_.end_time = m.stateTimeSpeed[0].minEndTime;
                    phase_holder_.phase = m.stateTimeSpeed[0].eventState.state;
                    
                    // Calculate time to stop
                    ros::Time current_time = ros::Time::now();
                    time_t whole_seconds = static_cast<time_t>(std::floor(current_time.toSec()));
                    double fractional_part = current_time.toSec() - whole_seconds;
                    
                    std::tm* time_info = std::gmtime(&whole_seconds);
                    int seconds_since_hour = time_info->tm_min * 60 + time_info->tm_sec;
                    double total_seconds_of_hour = seconds_since_hour + fractional_part;
                    phase_holder_.count_down = m.stateTimeSpeed[0].minEndTime - total_seconds_of_hour + virtual_intersections_[closest_intersection_].time_offset;
                    if (phase_holder_.count_down < 0){
                        phase_holder_.count_down += 3600;
                    }
                    phase_holder_.expiration_time = current_time.toSec() + phase_holder_.count_down;
                  }
            }
        }
        return;
    }

    void spatnmapTimerCallback(){
        std::unique_lock<std::mutex> spat_lock(spatnmap_mtx_);
        if (ros::Time::now().toSec() < phase_holder_.expiration_time){
            apsrc_msgs::SPaTnMAP msg;
            msg.header.stamp = ros::Time::now();
            msg.intersection_id = virtual_intersections_[closest_intersection_].intersection_id;
            msg.stop_waypoint = virtual_intersections_[closest_intersection_].waypoint_id;
            msg.depart_waypoint = virtual_intersections_[closest_intersection_].departure_id;
            msg.distance_to_stop = update_distance();
            msg.cycle_time_red = virtual_intersections_[closest_intersection_].cycletime_red;
            msg.cycle_time_yellow = virtual_intersections_[closest_intersection_].cycletime_yellow;
            msg.cycle_time_green = virtual_intersections_[closest_intersection_].cycletime_green;
            msg.time_to_stop = phase_holder_.expiration_time - msg.header.stamp.toSec();
            msg.phase = phase_holder_.phase;
            spatnmap_pub_.publish(msg);
        }
        return;
    }

    void baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& msg)
    {    
        std::unique_lock<std::mutex> wp_lock(wp_mtx_);
        base_waypoints_ = *msg;
        return;
    }

    double distanceBetweenWaypoints(const int& begin, const int& end, bool los)
    {
        if (begin < 0 || begin >= base_waypoints_.waypoints.size() || end < 0 || end >= base_waypoints_.waypoints.size())
        {
            return -1.0;
        }
        int begin_ = begin;
        int end_ = end;

        int sign = 1;
        if (begin_ > end_)
        {
            sign = -1;
            int tmp = begin_;
            begin_ = end_;
            end_ = tmp;
        }
        double dist_sum = 0.0;
        if (los)
        {
            // Calculate the distance between the waypoints
            dist_sum = distanceBetweenPoints(
            base_waypoints_.waypoints[begin_].pose.pose.position,
            base_waypoints_.waypoints[end_].pose.pose.position);
            return dist_sum;
        }
        for (int i = begin_; i < end_; i++)
        {
            dist_sum += distanceBetweenPoints(
            base_waypoints_.waypoints[i].pose.pose.position,
            base_waypoints_.waypoints[i + 1].pose.pose.position);
        }
        return dist_sum * sign;
    }

    float update_distance()
    {
        if (looped_){
            get_distance(false);
            return distance_ref_.distance;
        }
        distance_ref_.distance += distanceBetweenWaypoints(closest_waypoint_, distance_ref_.waypoint_id, false);
        distance_ref_.waypoint_id = closest_waypoint_;
        return distance_ref_.distance;
    }

    float get_distance(bool los)
    {   
        distance_ref_.waypoint_id = closest_waypoint_;
        distance_ref_.target_id = virtual_intersections_[closest_intersection_].waypoint_id;
        distance_ref_.distance = distanceBetweenWaypoints(closest_waypoint_, virtual_intersections_[closest_intersection_].waypoint_id, los);
        return distance_ref_.distance;
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "v2x_virtual_map_node");
    V2xVirtualMap virtual_map_node;
    ros::spin();
    return 0;
}