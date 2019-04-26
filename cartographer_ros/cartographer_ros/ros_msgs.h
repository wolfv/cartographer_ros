#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MSGS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MSGS_H

#include "cartographer_ros_msgs/LandmarkEntry.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "cartographer_ros_msgs/SensorTopics.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapTexture.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"

#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/WriteState.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/PointCloud2.h"

#include "std_msgs/ColorRGBA.h"

#include "tf2_msgs/TFMessage.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {
namespace ros_msgs {

// cartographer_ros_msgs
namespace cartographer_ros_msgs {
using LandmarkEntry = ::cartographer_ros_msgs::LandmarkEntry;
using LandmarkList = ::cartographer_ros_msgs::LandmarkList;
using SensorTopics = ::cartographer_ros_msgs::SensorTopics;
using StatusCode = ::cartographer_ros_msgs::StatusCode;
using StatusResponse = ::cartographer_ros_msgs::StatusResponse;
using SubmapEntry = ::cartographer_ros_msgs::SubmapEntry;
using SubmapList = ::cartographer_ros_msgs::SubmapList;
using SubmapTexture = ::cartographer_ros_msgs::SubmapTexture;
using TrajectoryOptions = ::cartographer_ros_msgs::TrajectoryOptions;

using FinishTrajectory = ::cartographer_ros_msgs::FinishTrajectory;
using StartTrajectory = ::cartographer_ros_msgs::StartTrajectory;
using SubmapQuery = ::cartographer_ros_msgs::SubmapQuery;
using WriteState = ::cartographer_ros_msgs::WriteState;
}

// geometry_msgs
namespace geometry_msgs {
using Point = ::geometry_msgs::Point;
using Pose = ::geometry_msgs::Pose;
using Transform = ::geometry_msgs::Transform;
using TransformStamped = ::geometry_msgs::TransformStamped;
using Vector3 = ::geometry_msgs::Vector3;
using Quaternion = ::geometry_msgs::Quaternion;
}

// nav_msgs
namespace nav_msgs {
using OccupancyGrid = ::nav_msgs::OccupancyGrid;
using Odometry = ::nav_msgs::Odometry;
}

// sensor_msgs
namespace sensor_msgs {
using Imu = ::sensor_msgs::Imu;
using LaserScan = ::sensor_msgs::LaserScan;
using LaserEcho = ::sensor_msgs::LaserEcho;
using MultiEchoLaserScan = ::sensor_msgs::MultiEchoLaserScan;
using NavSatFix = ::sensor_msgs::NavSatFix;
using NavSatStatus= ::sensor_msgs::NavSatStatus;
using PointCloud2 = ::sensor_msgs::PointCloud2;
using PointField= ::sensor_msgs::PointField;
}

// std_msgs
namespace std_msgs {
using ColorRGBA = ::std_msgs::ColorRGBA;
}

// tf2_msgs
namespace tf2_msgs {
using TFMessage = ::tf2_msgs::TFMessage;
}

// visualization_msgs
namespace visualization_msgs {
using Marker = ::visualization_msgs::Marker;
using MarkerArray = ::visualization_msgs::MarkerArray;
}

}  // namespace ros_msgs
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MSGS_H
