#include <string>
#include <vector>
#include "ros/ros.h"
#include <mutex>
#include "apsrc_msgs/SignalPhaseAndTiming.h"
#include "apsrc_msgs/SPaTnMAP.h"
#include "std_msgs/Int32.h"

#include <yaml-cpp/yaml.h>


struct virtual_intersection
{
    uint16_t intersection_id;
    int64_t signal_group;
    int32_t waypoint_id;
    float latitude;
    float longitude;
};


class V2xVirtualMap
{
private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber spat_sub_;
    ros::Subscriber closest_waypoint_sub_;
    ros::Publisher spatnmap_pub_;

    std::mutex wp_mtx_;

    std::string virtual_map_file_;
    bool loop_ = false;

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
        // Load the virtual map from the YAML file
        YAML::Node virtual_map = YAML::LoadFile(virtual_map_file_);

        // Check if the file was loaded successfully
        if (!virtual_map.IsDefined()) {
            ROS_ERROR("Failed to load virtual map file: %s", virtual_map_file_.c_str());
            exit(1);
        }

        virtual_intersections_.clear();


        // Parse the virtual map data
        for (const auto& intersection : virtual_map) {
            uint16_t intersection_id = intersection["intersection_id"].as<uint16_t>();
            int64_t signal_group = intersection["signal_group"].as<int64_t>();
            int32_t waypoint_id = intersection["waypoint_id"].as<int32_t>();
            float latitude = intersection["latitude"].as<float>();
            float longitude = intersection["longitude"].as<float>();

            // Store the data in a suitable data structure
            virtual_intersection vi;
            vi.intersection_id = intersection_id;
            vi.signal_group = signal_group;
            vi.waypoint_id = waypoint_id;
            vi.latitude = latitude;
            vi.longitude = longitude;
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
        if (virtual_intersections_[closest_intersection_].waypoint_id <= closest_waypoint_){
            if (closest_intersection_ +1 < virtual_intersections_.size()){
                closest_intersection_++;
            } else {
                if (loop_){
                    closest_intersection_ = 0;
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
                    msg.stop_waypoint = virtual_intersections_[closest_intersection_].waypoint_id;
                    msg.distance_to_stop = msg.stop_waypoint - closest_waypoint_;

                    // Calculate time to stop
                    time_t whole_seconds = static_cast<time_t>(std::floor(msg.header.stamp.toSec()));
                    double fractional_part = msg.header.stamp.toSec() - whole_seconds;
                    
                    std::tm* time_info = std::gmtime(&whole_seconds);
                    int seconds_since_hour = time_info->tm_min * 60 + time_info->tm_sec;
                    double total_seconds_of_hour = seconds_since_hour + fractional_part;
                    msg.time_to_stop = m.stateTimeSpeed[0].minEndTime - total_seconds_of_hour;
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
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "v2x_virtual_map_node");
    VirtualMap virtual_map_node;
    ros::spin();
    return 0;
}