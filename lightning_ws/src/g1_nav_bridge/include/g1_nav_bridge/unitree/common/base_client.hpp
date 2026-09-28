#pragma once
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request = unitree_api::msg::Request;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;

 public:
  BaseClient(rclcpp::Node* node, const std::string& topic_name_request,
             std::string /*topic_name_response*/)
      : req_puber_(
            node->create_publisher<Request>(topic_name_request, rclcpp::QoS(1))) {}

  int32_t Call(Request req, nlohmann::json& js) {
    (void)js;
    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    req_puber_->publish(req);
    return UT_ROBOT_SUCCESS;
  }

  int32_t Call(Request req) {
    nlohmann::json js;
    return Call(std::move(req), js);
  }
};