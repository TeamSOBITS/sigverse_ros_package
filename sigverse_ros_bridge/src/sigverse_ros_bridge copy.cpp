#include "sigverse_ros_bridge.hpp"

bool SIGVerseROSBridge::isRunning = true;
int SIGVerseROSBridge::syncTimeCnt;
int SIGVerseROSBridge::syncTimeMaxNum;

pid_t SIGVerseROSBridge::gettid(void) {
    return syscall(SYS_gettid);
}

void SIGVerseROSBridge::rosSigintHandler(int sig) {
    isRunning = false;
    rclcpp::shutdown();
}

bool SIGVerseROSBridge::checkReceivable(int fd) {
    fd_set fdset;
    int ret;
    struct timeval timeout;
    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);

    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    ret = select(fd + 1, &fdset, NULL, NULL, &timeout);

    return ret == 1;
}

void SIGVerseROSBridge::setVectorDouble(std::vector<double> &destVec, const bsoncxx::array::view &arrayView) {
    int i = 0;
    for (auto itr = arrayView.cbegin(); itr != arrayView.cend(); ++itr) {
        destVec[i++] = (*itr).get_double();
    }
}

void SIGVerseROSBridge::setVectorFloat(std::vector<float> &destVec, const bsoncxx::array::view &arrayView) {
    int i = 0;
    for (auto itr = arrayView.cbegin(); itr != arrayView.cend(); ++itr) {
        destVec[i++] = (float)((*itr).get_double());
    }
}

template <size_t ArrayNum>
void SIGVerseROSBridge::setArrayDouble(std::array<double, ArrayNum> &destArray, const bsoncxx::array::view &arrayView) {
    int i = 0;
    for (auto itr = arrayView.cbegin(); itr != arrayView.cend(); ++itr) {
        destArray[i++] = (*itr).get_double();
    }
}

void *SIGVerseROSBridge::receivingThread(void *param) {
    int dstSocket = *((int *)param);

    int dummyArgc = 0;
    char **dummyArgv;

    char *buf;
    buf = new char[BUFFER_SIZE];

    if (buf == NULL) {
        std::cout << "Cannot malloc!" << std::endl;
        exit(EXIT_FAILURE);
    }

    long int totalReceivedSize;
    std::map<std::string, rclcpp::PublisherBase::SharedPtr> publisherMap;

    std::cout << "Socket open. tid=" << gettid() << std::endl;

    // Create and spin ROS node in each thread
    auto node = std::make_shared<SIGVerseROSBridge>();

    rclcpp::Rate loop_rate(100);

    while (rclcpp::ok()) {
        // Get total BSON data size
        totalReceivedSize = 0;

        if (!checkReceivable(dstSocket)) continue;

        char bufHeader[4];
        long int numRcv = read(dstSocket, bufHeader, sizeof(4));

        if (numRcv == 0) {
            close(dstSocket);
            std::cout << "Socket closed. tid=" << gettid() << std::endl;
            break;
        }
        if (numRcv == -1) {
            close(dstSocket);
            std::cout << "Socket error. tid=" << gettid() << std::endl;
            break;
        }
        if (numRcv < 4) {
            close(dstSocket);
            std::cout << "Can not get data size... tid=" << gettid() << std::endl;
            break;
        }
        totalReceivedSize += 4;

        int32_t msgSize;
        memcpy(&msgSize, &bufHeader, sizeof(int32_t));
        memcpy(&buf[0], &bufHeader, sizeof(int32_t));

        if (msgSize > BUFFER_SIZE) {
            close(dstSocket);
            std::cout << "Data size is too big. tid=" << gettid() << std::endl;
            break;
        }

        while (msgSize != totalReceivedSize) {
            size_t unreceivedSize = msgSize - (size_t)totalReceivedSize;

            if (!checkReceivable(dstSocket)) break;

            long int receivedSize = read(dstSocket, &(buf[totalReceivedSize]), unreceivedSize);

            totalReceivedSize += receivedSize;
        }

        if (msgSize != totalReceivedSize) {
            std::cout << "msgSize!=totalReceivedSize ?????? tid=" << gettid() << std::endl;
            continue;
        }

        bsoncxx::document::view bsonView((const uint8_t *)buf, (std::size_t)msgSize);

        std::string opValue = bsonView["op"].get_utf8().value.to_string();
        std::string topicValue = bsonView["topic"].get_utf8().value.to_string();
        std::string typeValue = bsonView["type"].get_utf8().value.to_string();

        // Advertise
        if (publisherMap.count(topicValue) == 0 && typeValue != TYPE_TIME_SYNC && typeValue != TYPE_TF_LIST) {
            if (typeValue == TYPE_TWIST) publisherMap[topicValue] = node->create_publisher<geometry_msgs::msg::Twist>(topicValue, 10);
            
            else if (typeValue == TYPE_CAMERA_INFO) publisherMap[topicValue] = node->create_publisher<sensor_msgs::msg::CameraInfo>(topicValue, 10);
            
            else if (typeValue == TYPE_IMAGE) publisherMap[topicValue] = node->create_publisher<sensor_msgs::msg::Image>(topicValue, 10);
            else if (typeValue == TYPE_LASER_SCAN) publisherMap[topicValue] = node->create_publisher<sensor_msgs::msg::LaserScan>(topicValue, 10);
            else {
                std::cout << "Not compatible message type! :" << typeValue << std::endl;
                continue;
            }

            std::cout << "Advertised " << topicValue << std::endl;
        }

        // Publish
        // Twist
        if (typeValue == TYPE_TWIST) {
            auto twist = geometry_msgs::msg::Twist();

            twist.linear.x = bsonView["msg"]["linear"]["x"].get_double();
            twist.linear.y = bsonView["msg"]["linear"]["y"].get_double();
            twist.linear.z = bsonView["msg"]["linear"]["z"].get_double();

            twist.angular.x = bsonView["msg"]["angular"]["x"].get_double();
            twist.angular.y = bsonView["msg"]["angular"]["y"].get_double();
            twist.angular.z = bsonView["msg"]["angular"]["z"].get_double();

            std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Twist>>(publisherMap[topicValue])->publish(twist);
        }

        // CameraInfo
        else if (typeValue == TYPE_CAMERA_INFO) {
            auto cameraInfo = sensor_msgs::msg::CameraInfo();

			// cameraInfo.header.seq        = (uint32_t)bsonView["msg"]["header"]["seq"]           .get_int32();
			cameraInfo.header.stamp.sec     = (uint32_t)bsonView["msg"]["header"]["stamp"]["secs"] .get_int32();
			cameraInfo.header.stamp.nanosec = (uint32_t)bsonView["msg"]["header"]["stamp"]["nsecs"].get_int32();
			cameraInfo.header.frame_id      =           bsonView["msg"]["header"]["frame_id"].get_utf8().value.to_string();

			cameraInfo.height            = (uint32_t)bsonView["msg"]["height"].get_int32();
			cameraInfo.width             = (uint32_t)bsonView["msg"]["width"] .get_int32();
			cameraInfo.distortion_model  =           bsonView["msg"]["distortion_model"].get_utf8().value.to_string();

			bsoncxx::array::view dView = bsonView["msg"]["D"].get_array().value;
			cameraInfo.d.resize((size_t)std::distance(dView.cbegin(), dView.cend()));
			setVectorDouble(cameraInfo.d, dView);

			setArrayDouble(cameraInfo.k, bsonView["msg"]["K"].get_array().value);
			setArrayDouble(cameraInfo.r, bsonView["msg"]["R"].get_array().value);
			setArrayDouble(cameraInfo.p, bsonView["msg"]["P"].get_array().value);

			cameraInfo.binning_x         = (uint32_t)bsonView["msg"]["binning_x"].get_int32();
			cameraInfo.binning_y         = (uint32_t)bsonView["msg"]["binning_y"].get_int32();
			cameraInfo.roi.x_offset      = (uint32_t)bsonView["msg"]["roi"]["x_offset"].get_int32();
			cameraInfo.roi.y_offset      = (uint32_t)bsonView["msg"]["roi"]["y_offset"].get_int32();
			cameraInfo.roi.height        = (uint32_t)bsonView["msg"]["roi"]["height"].get_int32();
			cameraInfo.roi.width         = (uint32_t)bsonView["msg"]["roi"]["width"].get_int32();
			cameraInfo.roi.do_rectify    = (uint8_t) bsonView["msg"]["roi"]["do_rectify"].get_bool();

            std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(publisherMap[topicValue])->publish(cameraInfo);
        }

        // Image
        else if (typeValue == TYPE_IMAGE) {
            auto image = sensor_msgs::msg::Image();

			// image.header.seq        = (uint32_t)bsonView["msg"]["header"]["seq"]           .get_int32();
			image.header.stamp.sec  = (uint32_t)bsonView["msg"]["header"]["stamp"]["secs"] .get_int32();
			image.header.stamp.nanosec = (uint32_t)bsonView["msg"]["header"]["stamp"]["nsecs"].get_int32();
			image.header.frame_id   =           bsonView["msg"]["header"]["frame_id"]      .get_utf8().value.to_string();
			image.height            = (uint32_t)bsonView["msg"]["height"]      .get_int32();
			image.width             = (uint32_t)bsonView["msg"]["width"]       .get_int32();
			image.encoding          =           bsonView["msg"]["encoding"]    .get_utf8().value.to_string();
			image.is_bigendian      = (uint8_t) bsonView["msg"]["is_bigendian"].get_int32(); //.raw()[0];
			image.step              = (uint32_t)bsonView["msg"]["step"]        .get_int32();

			size_t sizet = (image.step * image.height);
			image.data.resize(sizet);
			memcpy(&image.data[0], bsonView["msg"]["data"].get_binary().bytes, sizet);

            std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Image>>(publisherMap[topicValue])->publish(image);
        }

        // LaserScan
        else if (typeValue == TYPE_LASER_SCAN) {
            auto laserScan = sensor_msgs::msg::LaserScan();

			// laserScan.header.seq        = (uint32_t)bsonView["msg"]["header"]["seq"]           .get_int32();
			laserScan.header.stamp.sec  = (uint32_t)bsonView["msg"]["header"]["stamp"]["secs"] .get_int32();
			laserScan.header.stamp.nanosec = (uint32_t)bsonView["msg"]["header"]["stamp"]["nsecs"].get_int32();
			laserScan.header.frame_id   =           bsonView["msg"]["header"]["frame_id"]      .get_utf8().value.to_string();

			laserScan.angle_min       = (float)bsonView["msg"]["angle_min"]      .get_double();
			laserScan.angle_max       = (float)bsonView["msg"]["angle_max"]      .get_double();
			laserScan.angle_increment = (float)bsonView["msg"]["angle_increment"].get_double();
			laserScan.time_increment  = (float)bsonView["msg"]["time_increment"] .get_double();
			laserScan.scan_time       = (float)bsonView["msg"]["scan_time"]      .get_double();
			laserScan.range_min       = (float)bsonView["msg"]["range_min"]      .get_double();
			laserScan.range_max       = (float)bsonView["msg"]["range_max"]      .get_double();

			size_t sizet = (size_t)((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment + 1);

			laserScan.ranges.resize(sizet);
			laserScan.intensities.resize(sizet);

			bsoncxx::array::view dView_ranges = bsonView["msg"]["ranges"].get_array().value;
			laserScan.ranges.resize(std::distance(dView_ranges.cbegin(), dView_ranges.cend()));
			setVectorFloat(laserScan.ranges, dView_ranges);

			bsoncxx::array::view dView_intensities = bsonView["msg"]["intensities"].get_array().value;
			laserScan.intensities.resize(std::distance(dView_intensities.cbegin(), dView_intensities.cend()));
			setVectorFloat(laserScan.intensities, dView_intensities);

            std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(publisherMap[topicValue])->publish(laserScan);
        }

		// Time Synchronization (SIGVerse Original Type)
		else if(typeValue==TYPE_TIME_SYNC)
		{
			if(syncTimeCnt < syncTimeMaxNum)
			{
                uint32_t sec = (uint32_t)bsonView["msg"]["data"]["secs"].get_int32();
                uint32_t nsec = (uint32_t)bsonView["msg"]["data"]["nsecs"].get_int32();

                rclcpp::Time timestamp(sec, nsec);

                rclcpp::Clock clock;
                rclcpp::Time now = clock.now();

                int gapSec  = ((int)timestamp.seconds()  - (int)now.seconds());
                int gapMsec = ((int)timestamp.nanoseconds() - (int)now.nanoseconds()) / 1000 / 1000;

                std::string timeGap = "time_gap," + std::to_string(gapSec) + "," + std::to_string(gapMsec);

                ssize_t size = write(dstSocket, timeGap.c_str(), std::strlen(timeGap.c_str()));

                std::cout << "TYPE_TIME_SYNC " << timeGap.c_str() << std::endl;

                syncTimeCnt++;
			}
		}

		// Tf list data (SIGVerse Original Type)
		else if(typeValue==TYPE_TF_LIST)
		{
			// static tf::TransformBroadcaster transformBroadcaster;

			bsoncxx::array::view tfArrayView = bsonView["msg"].get_array().value;

			// std::vector<tf::StampedTransform> stampedTransformList;

			int i = 0;

            for(auto itr = tfArrayView.cbegin(); itr != tfArrayView.cend(); ++itr)
            {
                rclcpp::Time timestamp;

                std::string frameId      = (*itr)["header"]["frame_id"].get_utf8().value.to_string();
                int32_t sec              = (*itr)["header"]["stamp"]["secs"].get_int32();
                int32_t nsec             = (*itr)["header"]["stamp"]["nsecs"].get_int32();
                timestamp = rclcpp::Time(sec, nsec, RCL_ROS_TIME);
                std::string childFrameId = (*itr)["child_frame_id"].get_utf8().value.to_string();

                if(sec == 0 && nsec == 0)
                {
                    rclcpp::Clock clock;
                    timestamp = clock.now();
                }
				
                // tf::Vector3 position = tf::Vector3
				// (
				// 	(double)(*itr)["transform"]["translation"]["x"].get_double(),
				// 	(double)(*itr)["transform"]["translation"]["y"].get_double(),
				// 	(double)(*itr)["transform"]["translation"]["z"].get_double()
				// );

				// tf::Quaternion quaternion = tf::Quaternion
				// (
				// 	(double)(*itr)["transform"]["rotation"]["x"].get_double(),
				// 	(double)(*itr)["transform"]["rotation"]["y"].get_double(),
				// 	(double)(*itr)["transform"]["rotation"]["z"].get_double(),
				// 	(double)(*itr)["transform"]["rotation"]["w"].get_double()
				// );

				// tf::Transform transform;
				// transform.setOrigin(position);
				// transform.setRotation(quaternion);

				// stampedTransformList.push_back(tf::StampedTransform(transform, timestamp, frameId, childFrameId));
			}

			// transformBroadcaster.sendTransform(stampedTransformList);
		}
    }

    std::cout << "ROS bridge receiving thread exiting" << std::endl;
    return NULL;
}

SIGVerseROSBridge::SIGVerseROSBridge()
    : rclcpp::Node("sigverse_ros_bridge_node"),
        transformBroadcaster(this)
{
    signal(SIGINT, rosSigintHandler);
}

int SIGVerseROSBridge::run(int argc, char **argv) {
    int serverSocket;
    int clientSocket;
    struct sockaddr_in serverAddr;
    struct sockaddr_in clientAddr;
    socklen_t clientAddrSize = sizeof(clientAddr);

    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        std::cout << "Failed to create socket" << std::endl;
        return -1;
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(DEFAULT_PORT);

    if (bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cout << "Failed to bind socket" << std::endl;
        return -1;
    }

    if (listen(serverSocket, 5) == -1) {
        std::cout << "Failed to listen on socket" << std::endl;
        return -1;
    }

    while (isRunning) {
        clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &clientAddrSize);
        if (clientSocket == -1) {
            std::cout << "Failed to accept client connection" << std::endl;
            continue;
        }
        std::cout << "Failed to accept client connection" << std::endl;
        pthread_t thread;
        pthread_create(&thread, NULL, receivingThread, &clientSocket);
    }

    close(serverSocket);
    std::cout << "Server socket closed" << std::endl;

    return 0;
}

int main(int argc, char **argv) {
    // Call rclcpp::init only in main thread
    rclcpp::init(argc, argv);

    SIGVerseROSBridge bridge;
    return bridge.run(argc, argv);
}
