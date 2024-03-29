/*
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#ifndef MAP_TO_IMAGE_HPP_
#define MAP_TO_IMAGE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <boost/beast/core/detail/base64.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;

namespace map_to_image {
/*!
  @class MapToImage
  @brief ROS2 MOVVO Map To Image Node
*/
class MapToImage {
public:
  /*!
    @brief Constructor of the MapToImage class
    @param[in] nh Injected rclcpp node dependency
  */
  MapToImage(rclcpp::Node::SharedPtr nh);

  /*!
    @brief It initializes the ROS2 publishers and subscribers of the class
    @return True if the class has been initialized correctly or false otherwise.
  */
  bool Initialize();

  /// @brief ROS2 parameters
  typedef struct
  {
    rclcpp::Parameter resolution_factor;
    rclcpp::Parameter map_topic;
    rclcpp::Parameter mono_img_topic;
    rclcpp::Parameter encoded_img_topic;
  } configuration;
  configuration parameters;

protected:

private:
  /*!
    @brief It transforms the map into an image and publishes it in jpeg and base64 formats
    @param[in] msg Received OccupancyGrid msg
  */
  void GetMapImageCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  /*!
    @brief It transforms the map into a cv Mat
  */
  cv::Mat MapToMonoImage(const nav_msgs::msg::MapMetaData map_info, std::vector<int8_t> map_data);
  /*!
    @brief Encodes the image in base 64 format
  */
  std::string base64_encode(unsigned char const*, size_t len, bool url = false);

  rclcpp::Node::SharedPtr nh_; /*!< RCLCPP Node */
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; /*!< Subscription to the OccupancyGrid map topic */

  image_transport::Publisher image_jpeg_pub_; /*!< Publisher of the image in jpeg */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr image_b64_pub_; /*!< Publisher of the image encoded in base 64 */
};
} //namespace map_to_image
#endif // MAP_TO_IMAGE_HPP_