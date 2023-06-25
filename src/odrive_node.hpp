#ifndef ODRIVE_NODE_HPP_
#define ODRIVE_NODE_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <unordered_set>
#include <can_msgs/Frame.h>

#define DEFAULT_CAN_INTERFACE       "can10"
#define DEFAULT_CAN_BITRATE         250000
#define DEFAULT_ENGAGE_ON_STARTUP   false
#define DEFAULT_DISENGAGE_ON_SHUTDOWN   true

#endif // ODRIVE_NODE_HPP_
