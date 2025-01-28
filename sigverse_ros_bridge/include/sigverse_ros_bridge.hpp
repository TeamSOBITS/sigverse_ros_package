#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <map>

#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <bsoncxx/array/view.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>

#include <boost/array.hpp>

#define TYPE_TWIST        "geometry_msgs/Twist"
#define TYPE_CAMERA_INFO  "sensor_msgs/CameraInfo"
#define TYPE_IMAGE        "sensor_msgs/Image"
#define TYPE_LASER_SCAN   "sensor_msgs/LaserScan"
#define TYPE_TIME_SYNC    "sigverse/TimeSync"
#define TYPE_TF_LIST      "sigverse/TfList"

#define BUFFER_SIZE 25*1024*1024 //100MB

#define DEFAULT_PORT 50001
#define DEFAULT_SYNC_TIME_MAX_NUM 1

class SIGVerseROSBridge : public rclcpp::Node
{
private:
    static pid_t gettid(void);

    static void rosSigintHandler(int sig);
    static bool checkReceivable(int fd);

    static void setVectorDouble(std::vector<double> &destVec, const bsoncxx::array::view &arrayView);
    static void setVectorFloat(std::vector<float> &destVec, const bsoncxx::array::view &arrayView);

    template <size_t ArrayNum>
    static void setArrayDouble(std::array<double, ArrayNum> &vec, const bsoncxx::array::view &arrayView);
    static void *receivingThread(void *param);

    static bool isRunning;
    static int syncTimeCnt;
    static int syncTimeMaxNum;

    tf2_ros::TransformBroadcaster transformBroadcaster;

public:
    SIGVerseROSBridge();
    int run(int argc, char **argv);
};