#ifndef HOVER_THRUST_ESTIMATE_HPP
#define HOVER_THRUST_ESTIMATE_HPP

#include <uORB/topics/hover_thrust_estimate.h>
#include <v2.0/cor_drone_dev/mavlink_msg_hover_thrust_estimate.h>

class MavlinkStreamHoverThrustEstimate : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHoverThrustEstimate(mavlink); }

	static constexpr const char *get_name_static() { return "HOVER_THRUST_ESTIMATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _hte_sub.advertised() ? MAVLINK_MSG_ID_HOVER_THRUST_ESTIMATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamHoverThrustEstimate(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _hte_sub{ORB_ID(hover_thrust_estimate)};

	bool send() override
	{
		hover_thrust_estimate_s hte;

		if (_hte_sub.update(&hte)) {
			mavlink_hover_thrust_estimate_t msg{};

			msg.timestamp = hte.timestamp;
            msg.timestamp_sample = hte.timestamp_sample;
			msg.hover_thrust = hte.hover_thrust;
            msg.hover_thrust_var = hte.hover_thrust_var;
            msg.accel_innov = hte.accel_innov;
            msg.accel_innov_var = hte.accel_innov_var;
            msg.accel_innov_test_ratio = hte.accel_innov_test_ratio;
            msg.accel_noise_var = hte.accel_noise_var;
            msg.valid = hte.valid;

			mavlink_msg_hover_thrust_estimate_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // HOVER_THRUST_ESTIMATE_HPP
