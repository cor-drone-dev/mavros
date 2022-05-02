/*
 * Copyright 2022 Michiel te Braake.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief RPM plugin
 * @file rpm.cpp
 * @author Michiel te Braake
 *
 * @addtogroup plugin
 * @{
 */

#include <tf2_eigen/tf2_eigen.h>

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/vibration.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief RPM plugin
 * @plugin RPM
 *
 * This plugin is intended to publish RAW_RPM message.
 */
class RPMPlugin : public plugin::Plugin
{
public:
  explicit RPMPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "rpm")
  {
    enable_node_watch_parameters();

    node_declate_and_watch_parameter(
      "frame_id", "base_link", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    rpm_pub = node->create_publisher<mavros_msgs::msg::RPM>("~/rpm", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&RPMPlugin::handle_rpm)
    };
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::RPM>::SharedPtr vibration_pub;

  std::string frame_id;

  void handle_rpm(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::RAW_RPM & rpm,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto rpm_msg = mavros_msgs::msg::RPM();

    rpm_msg.header = uas->synchronized_header(frame_id, rpm.time_usec);

    rpm_msg.index = rpm.index;
    rpm_msg.frequency = rpm.frequency;

    rpm_pub->publish(rpm_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::VibrationPlugin)
