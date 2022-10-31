/*
 *  Copyright 2022 @ MOVVO ROBOTICS
 *  ---------------------------------------------------------
 *  Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
 *  Contact: support.idi@movvo.eu
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "map_to_image/map_to_image.hpp"

int main(int argc, char * argv[]) {
    //init rclcpp
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("map_to_image");
    auto map_to_image_node = std::make_shared<map_to_image::MapToImage>(nh);
    if (map_to_image_node->Initialize()) {
        RCLCPP_INFO(nh->get_logger(), "MapToImage node successfully initialized");
    }
    //Spin
    executor.add_node(nh);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}