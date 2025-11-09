#ifndef ROS_MONITORS__VISIBILITY_CONTROL_H_
#define ROS_MONITORS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROS_MONITORS_EXPORT __attribute__ ((dllexport))
    #define ROS_MONITORS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROS_MONITORS_EXPORT __declspec(dllexport)
    #define ROS_MONITORS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROS_MONITORS_BUILDING_LIBRARY
    #define ROS_MONITORS_PUBLIC ROS_MONITORS_EXPORT
  #else
    #define ROS_MONITORS_PUBLIC ROS_MONITORS_IMPORT
  #endif
  #define ROS_MONITORS_PUBLIC_TYPE ROS_MONITORS_PUBLIC
  #define ROS_MONITORS_LOCAL
#else
  #define ROS_MONITORS_EXPORT __attribute__ ((visibility("default")))
  #define ROS_MONITORS_IMPORT
  #if __GNUC__ >= 4
    #define ROS_MONITORS_PUBLIC __attribute__ ((visibility("default")))
    #define ROS_MONITORS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROS_MONITORS_PUBLIC
    #define ROS_MONITORS_LOCAL
  #endif
  #define ROS_MONITORS_PUBLIC_TYPE
#endif

#endif  // ROS_MONITORS__VISIBILITY_CONTROL_H_
