#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <random>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "Wind.pb.h"

namespace gazebo {

class AfcrWindProfile final : public WorldPlugin {
public:
	void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
	{
		_world = world;
		_kind = sdf->Get<std::string>("kind");
		_magnitude = sdf->Get<double>("magnitude");
		_start = sdf->Get<double>("start");
		_duration = sdf->Get<double>("duration");
		_period = sdf->Get<double>("period");
		_direction = sdf->Get<ignition::math::Vector3d>("direction");
		_direction.Normalize();
		_rng.seed(sdf->Get<unsigned int>("seed"));
		_node.reset(new transport::Node());
		_node->Init();
		_publisher = _node->Advertise<physics_msgs::msgs::Wind>("~/world_wind", 10);
		_connection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&AfcrWindProfile::Update, this));
	}

private:
	void Update()
	{
		const double now = _world->SimTime().Double();
		if (now - _last_publish < 0.05) { return; }
		_last_publish = now;
		double strength = 0.0;
		if (_kind == "periodic") {
			const double elapsed = now - _start;
			if (elapsed >= 0 && elapsed < 12.0 && std::fmod(elapsed, _period) < _duration) {
				strength = _magnitude;
			}
		} else if (_kind == "randomized_gust" && now >= _start && now < _start + _duration) {
			const auto step = static_cast<int64_t>((now - _start) * 2);
			if (step != _last_step) {
				_last_step = step;
				std::uniform_real_distribution<double> distribution(0.5, 1.5);
				_random_strength = _magnitude * distribution(_rng);
			}
			strength = _random_strength;
		}
		physics_msgs::msgs::Wind message;
		message.set_frame_id("base_link");
		message.set_time_usec(static_cast<uint64_t>(now * 1e6));
		auto *velocity = new gazebo::msgs::Vector3d();
		velocity->set_x(strength * _direction.X());
		velocity->set_y(strength * _direction.Y());
		velocity->set_z(0);
		message.set_allocated_velocity(velocity);
		_publisher->Publish(message);
	}

	physics::WorldPtr _world;
	transport::NodePtr _node;
	transport::PublisherPtr _publisher;
	event::ConnectionPtr _connection;
	ignition::math::Vector3d _direction{1, 0, 0};
	std::mt19937 _rng;
	std::string _kind;
	double _magnitude{0};
	double _start{46};
	double _duration{1};
	double _period{3};
	double _last_publish{-1};
	double _random_strength{0};
	int64_t _last_step{-1};
};

GZ_REGISTER_WORLD_PLUGIN(AfcrWindProfile)

} // namespace gazebo
