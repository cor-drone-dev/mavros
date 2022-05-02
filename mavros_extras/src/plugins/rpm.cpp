/**
 * @brief RPM plugin
 * @file rpm.cpp
 * @author Michiel te Braake <m.j.m.tebraake@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2022 Michiel te Braake.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/RPM.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief RPM plugin
 *
 * This plugin is intended to publish MAV rotor RPM from FCU.
 */
class RPMPlugin : public plugin::PluginBase {
public:
	RPMPlugin() : PluginBase(),
		rpm_nh("~rpm")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		rpm_nh.param<std::string>("frame_id", frame_id, "base_link");

		rpm_pub = rpm_nh.advertise<mavros_msgs::RPM>("rpm", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&RPMPlugin::handle_rpm)
		};
	}

private:
	ros::NodeHandle rpm_nh;

	std::string frame_id;

	ros::Publisher rpm_pub;

	void handle_rpm(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RAW_RPM &raw_rpm)
	{
		auto rpm_msg = boost::make_shared<mavros_msgs::RPM>();

		rpm_msg->header.stamp = ros::Time::now();

    rpm_msg->index = raw_rpm.index;
    rpm_msg->frequency = raw_rpm.frequency;

		rpm_pub.publish(rpm_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RPMPlugin, mavros::plugin::PluginBase)
