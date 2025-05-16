#ifndef __IMAGE_SERIALZATION_H__
#define __IMAGE_SERIALZATION_H__
#pragma once
#include <array>
#include <string>
#include <sensor_msgs/Image.h>
#include "common/base/communication/include/ros_serialization_bridge.h"

namespace acu {
namespace common {
typedef struct ImageBuffer
{
#define IMAGE_BUFFER_LEN 1024*1024*5
	char image1[IMAGE_BUFFER_LEN];
	char image2[IMAGE_BUFFER_LEN];
	char image3[IMAGE_BUFFER_LEN];
	char image4[IMAGE_BUFFER_LEN];
	ImageBuffer() {
		memset(image1, 0 , IMAGE_BUFFER_LEN);
		memset(image2, 0 , IMAGE_BUFFER_LEN);
		memset(image3, 0 , IMAGE_BUFFER_LEN);
		memset(image4, 0 , IMAGE_BUFFER_LEN);
	}

	void Serialization(const sensor_msgs::Image &ros_image1) {
		RosSerializationBridge::Serialization((uint8_t *)image1, IMAGE_BUFFER_LEN, ros_image1);
	}

	void Serialization(const sensor_msgs::Image &ros_image1, const sensor_msgs::Image &ros_image2) {
		RosSerializationBridge::Serialization((uint8_t *)image1, IMAGE_BUFFER_LEN, ros_image1);
		RosSerializationBridge::Serialization((uint8_t *)image2, IMAGE_BUFFER_LEN, ros_image2);
	}

	void Serialization(const sensor_msgs::Image &ros_image1, const sensor_msgs::Image &ros_image2,
	                   const sensor_msgs::Image &ros_image3) {
		RosSerializationBridge::Serialization((uint8_t *)image1, IMAGE_BUFFER_LEN, ros_image1);
		RosSerializationBridge::Serialization((uint8_t *)image2, IMAGE_BUFFER_LEN, ros_image2);
		RosSerializationBridge::Serialization((uint8_t *)image3, IMAGE_BUFFER_LEN, ros_image3);
	}

	void Serialization(const sensor_msgs::Image &ros_image1, const sensor_msgs::Image &ros_image2,
	                   const sensor_msgs::Image &ros_image3, const sensor_msgs::Image &ros_image4) {
		RosSerializationBridge::Serialization((uint8_t *)image1, IMAGE_BUFFER_LEN, ros_image1);
		RosSerializationBridge::Serialization((uint8_t *)image2, IMAGE_BUFFER_LEN, ros_image2);
		RosSerializationBridge::Serialization((uint8_t *)image3, IMAGE_BUFFER_LEN, ros_image3);
		RosSerializationBridge::Serialization((uint8_t *)image4, IMAGE_BUFFER_LEN, ros_image4);
	}

	void Deserialization(sensor_msgs::Image &ros_image1) {
		RosSerializationBridge::DeSerialization(ros_image1, (uint8_t *)image1, IMAGE_BUFFER_LEN);
	}

	void Deserialization(sensor_msgs::Image &ros_image1, sensor_msgs::Image &ros_image2) {
		RosSerializationBridge::DeSerialization(ros_image1, (uint8_t *)image1, IMAGE_BUFFER_LEN);
		RosSerializationBridge::DeSerialization(ros_image2, (uint8_t *)image2, IMAGE_BUFFER_LEN);
	}

	void Deserialization(sensor_msgs::Image &ros_image1, sensor_msgs::Image &ros_image2,
	                     sensor_msgs::Image &ros_image3) {
		RosSerializationBridge::DeSerialization(ros_image1, (uint8_t *)image1, IMAGE_BUFFER_LEN);
		RosSerializationBridge::DeSerialization(ros_image2, (uint8_t *)image2, IMAGE_BUFFER_LEN);
		RosSerializationBridge::DeSerialization(ros_image3, (uint8_t *)image3, IMAGE_BUFFER_LEN);
	}


	void Deserialization(sensor_msgs::Image &ros_image1, sensor_msgs::Image &ros_image2,
	                     sensor_msgs::Image &ros_image3, sensor_msgs::Image &ros_image4) {
		RosSerializationBridge::DeSerialization(ros_image1, (uint8_t *)image1, IMAGE_BUFFER_LEN);
		RosSerializationBridge::DeSerialization(ros_image2, (uint8_t *)image2, IMAGE_BUFFER_LEN);
		RosSerializationBridge::DeSerialization(ros_image3, (uint8_t *)image3, IMAGE_BUFFER_LEN);
		RosSerializationBridge::DeSerialization(ros_image4, (uint8_t *)image4, IMAGE_BUFFER_LEN);
	}

} ImageBuffer;

}
}


#endif
