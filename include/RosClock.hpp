#include <slam3d/core/Clock.hpp>
#include <rclcpp/rclcpp.hpp>

namespace slam3d
{
	timeval fromRosTime(const rclcpp::Time& rt);
	rclcpp::Time fromTimeval(const timeval& tv);

	class RosClock : public Clock
	{
	public:
		RosClock(rclcpp::Clock::SharedPtr clock);
		
		timeval now();
		rclcpp::Time ros_now();
	
	private:
		rclcpp::Clock::SharedPtr mInternalClock;
	};
}
