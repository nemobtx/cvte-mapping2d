/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

#include <boost/thread.hpp>

class SlamGMapping
{
  public:
    SlamGMapping();
    SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    SlamGMapping(unsigned long int seed, unsigned long int max_duration_buffer);
    ~SlamGMapping();

    void init();
    void startLiveSlam();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
   
 

  private:
    ros::NodeHandle node_;
    ros::Publisher sst_;
    ros::Publisher sstm_;
    tf::TransformListener tf_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformBroadcaster* tfB_;
    GMapping::RangeSensor* gsp_laser_;
    // The angles in the laser, going from -x to x (adjustment is made to get the laser between
    // symmetrical bounds as that's what gmapping expects)
    std::vector<double> laser_angles_;
    // The pose, in the original laser frame, of the corresponding centered laser with z facing up
    // T_laser_centered-laser,实际上为Identity
    tf::Stamped<tf::Pose> centered_laser_pose_;
    // Depending on the order of the elements in the scan and the orientation of the scan frame,
    // We might need to change the order of the scan
    bool do_reverse_range_;
    unsigned int gsp_laser_beam_count_;
    GMapping::OdometrySensor* gsp_odom_;

    bool got_first_scan_;

    bool got_map_;
    nav_msgs::GetMap::Response map_;

    ros::Duration map_update_interval_;
    
    // T_map_odom
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex map_mutex_;

    int laser_count_;
    int throttle_scans_;

    boost::thread* transform_thread_;

    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    void updateMap(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& robot_pose);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
    
    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    // size of one pixel [m]
    double delta_;
    double occ_thresh_;
  
    
    ros::NodeHandle private_nh_;
    
    unsigned long int seed_;
    
    double transform_publish_period_;
    double tf_delay_;
    
    tf::TransformListener m_tf;// 订阅tf
    tf::StampedTransform T_map_base;
    GMapping::ScanMatcherMap* m_smap;// 地图 
    GMapping::OrientedPoint m_laser_pose;//T_base_laser
};
