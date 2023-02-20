#include <mavros/mavros_plugin.h>

#include <mavros_msgs/HoverThrustEstimate.h>

namespace mavros {
namespace std_plugins {
class HoverThrustEstimatePlugin : public plugin::PluginBase {
public:
	HoverThrustEstimatePlugin() : PluginBase(),
		lp_nh("~")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// header frame_id.
		// default to map (world-fixed,ENU as per REP-105).
		lp_nh.param<std::string>("frame_id", frame_id, "base_link");

		hover_thrust_estimate = lp_nh.advertise<mavros_msgs::HoverThrustEstimate>("hover_thrust_estimate", 10);
	}

	Subscriptions get_subscriptions() override {
		return {
			       make_handler(&HoverThrustEstimatePlugin::handle_hover_thrust_estimate)
		};
	}

private:
	ros::NodeHandle lp_nh;

	ros::Publisher hover_thrust_estimate;

	std::string frame_id;

	void handle_hover_thrust_estimate(const mavlink::mavlink_message_t *msg, mavlink::cor_drone_dev::msg::HOVER_THRUST_ESTIMATE &hte_mavlink)
	{
		// Create hover_thrust_estimate message
		auto hte_ros = boost::make_shared<mavros_msgs::HoverThrustEstimate>();
		
		// Fill hover_thrust_estimate message with received data
		hte_ros->header = m_uas->synchronized_header(frame_id, hte_mavlink.timestamp);
		hte_ros->timestamp = hte_mavlink.timestamp;
		hte_ros->timestamp_sample = hte_mavlink.timestamp_sample;
		hte_ros->hover_thrust = hte_mavlink.hover_thrust;
		hte_ros->hover_thrust_var = hte_mavlink.hover_thrust_var;
		hte_ros->accel_innov = hte_mavlink.accel_innov;
		hte_ros->accel_innov_var = hte_mavlink.accel_innov_var;
		hte_ros->accel_innov_test_ratio = hte_mavlink.accel_innov_test_ratio;
		hte_ros->accel_noise_var = hte_mavlink.accel_noise_var;
		hte_ros->valid = hte_mavlink.valid;

		// Publish message
		hover_thrust_estimate.publish(hte_ros);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HoverThrustEstimatePlugin, mavros::plugin::PluginBase)
