/*
 * main.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "RTOOdometryNode.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RTOOdometryNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}