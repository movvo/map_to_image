/*
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#include "map_to_image/map_to_image.hpp"

 //
 // Depending on the url parameter in base64_chars, one of
 // two sets of base64 characters needs to be chosen.
 // They differ in their last two characters.
 //
static const char* base64_chars[2] = {
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789"
             "+/",

             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789"
             "-_"};

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace map_to_image;

MapToImage::MapToImage(rclcpp::Node::SharedPtr nh) 
{
    nh_ = nh;

    // Declare
    nh_->declare_parameter("resolution_factor", 0.5);
    nh_->declare_parameter("map_topic", "map");
    nh_->declare_parameter("mono_img_topic", "image_jpeg");
    nh_->declare_parameter("endoded_img_topic", "image_encoded");
}

bool MapToImage::Initialize()
{
    try {
        // Get parameters
        parameters.resolution_factor = nh_->get_parameter("resolution_factor");
        parameters.map_topic = nh_->get_parameter("map_topic");
        parameters.mono_img_topic = nh_->get_parameter("mono_img_topic");
        parameters.encoded_img_topic = nh_->get_parameter("endoded_img_topic");
    }
    catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to get parameters, parameter not declared. Exception %s", ex.what());
        return false;
    } 
    catch (std::runtime_error & ex) {
        RCLCPP_ERROR(nh_->get_logger(), "Failed to get parameters, runtime error. Exception %s", ex.what());
        return false;
    }

    // Initialize OccupancyGrid subscriber
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
    map_sub_ = nh_->create_subscription<nav_msgs::msg::OccupancyGrid>(parameters.map_topic.as_string(), qos,
                                                                std::bind(&MapToImage::GetMapImageCallback, this, _1));

    rmw_qos_profile_t image_qos = qos.get_rmw_qos_profile();
    std::string mono_img_topic = parameters.mono_img_topic.as_string();
    image_jpeg_pub_ = image_transport::create_publisher(nh_.get(), nh_->get_name()+std::string("/")+mono_img_topic, image_qos);

    std::string encoded_img_topic = parameters.encoded_img_topic.as_string();
    image_b64_pub_ = nh_->create_publisher<std_msgs::msg::String>(nh_->get_name()+std::string("/")+encoded_img_topic, qos);

    return true;
}

void MapToImage::GetMapImageCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    nav_msgs::msg::OccupancyGrid map;
    map.info = msg->info;
    map.data = msg->data;

    cv::Mat mono_image = MapToMonoImage(map.info, map.data);
    image_jpeg_pub_.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mono_image).toImageMsg());

    // Downscale the image according to the defined factor
    int down_width = round(mono_image.cols*parameters.resolution_factor.as_double());
    int down_height = round(mono_image.rows*parameters.resolution_factor.as_double());
    cv::Mat resized_down;
    // Resize down
    cv::resize(mono_image, resized_down, cv::Size(down_width, down_height), cv::INTER_LINEAR);

    //Convert file into base 64
    std::vector<uchar> buffer;
    buffer.resize(static_cast<size_t>(resized_down.rows)*static_cast<size_t>(resized_down.cols));
    cv::imencode(".jpg", resized_down, buffer);
    std::string encoded_image = base64_encode(buffer.data(), buffer.size());

    // Publish encoded image
    std_msgs::msg::String encoded_msg;
    encoded_msg.data = encoded_image;
    image_b64_pub_->publish(encoded_msg);
}

cv::Mat MapToImage::MapToMonoImage(nav_msgs::msg::MapMetaData map_info, std::vector<int8_t> map_data)
{
    cv::Mat map_mat(cv::Size(map_info.width, map_info.height), CV_8UC1);

    for (size_t y = 0; y < map_info.height; y++) {
      for (size_t x = 0; x < map_info.width; x++) {
        int8_t map_cell_value = map_data[map_info.width*(map_info.height-y-1) + x];
        switch (map_cell_value) {
            case -1:
                map_mat.at<uchar>(y, x) = 127;
                break;
            case 0:
                map_mat.at<uchar>(y, x) = 255;
                break;
            case 100:
                map_mat.at<uchar>(y, x) = 0;
                break;
        }
      }
    }

    return map_mat;
}

std::string MapToImage::base64_encode(unsigned char const* bytes_to_encode, size_t in_len, bool url) {

    size_t len_encoded = (in_len +2) / 3 * 4;

    unsigned char trailing_char = url ? '.' : '=';

    //
    // Choose set of base64 characters. They differ
    // for the last two positions, depending on the url
    // parameter.
    // A bool (as is the parameter url) is guaranteed
    // to evaluate to either 0 or 1 in C++ therefore,
    // the correct character set is chosen by subscripting
    // base64_chars with url.
    //
    const char* base64_chars_ = base64_chars[url];

    std::string image_base64;
    image_base64.reserve(len_encoded);

    unsigned int pos = 0;

    while (pos < in_len) {
        image_base64.push_back(base64_chars_[(bytes_to_encode[pos + 0] & 0xfc) >> 2]);
        if (pos+1 < in_len) {
           image_base64.push_back(base64_chars_[((bytes_to_encode[pos + 0] & 0x03) << 4) + ((bytes_to_encode[pos + 1] & 0xf0) >> 4)]);
           if (pos+2 < in_len) {
              image_base64.push_back(base64_chars_[((bytes_to_encode[pos + 1] & 0x0f) << 2) + ((bytes_to_encode[pos + 2] & 0xc0) >> 6)]);
              image_base64.push_back(base64_chars_[  bytes_to_encode[pos + 2] & 0x3f]);
           }
           else {
              image_base64.push_back(base64_chars_[(bytes_to_encode[pos + 1] & 0x0f) << 2]);
              image_base64.push_back(trailing_char);
           }
        }
        else {
            image_base64.push_back(base64_chars_[(bytes_to_encode[pos + 0] & 0x03) << 4]);
            image_base64.push_back(trailing_char);
            image_base64.push_back(trailing_char);
        }
        pos += 3;
    }
    return image_base64;
}