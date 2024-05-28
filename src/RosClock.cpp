#include <RosClock.hpp>

using namespace slam3d;

timeval slam3d::fromRosTime(const rclcpp::Time& rt)
{
	timeval t;
	t.tv_sec = rt.seconds();

	rcl_time_point_value_t diff = t.tv_sec * 1000000000;
	rcl_time_point_value_t nsec = rt.nanoseconds() - diff;
	t.tv_usec = nsec / 1000;
	return t;
}

rclcpp::Time slam3d::fromTimeval(const timeval& tv)
{
	return rclcpp::Time(tv.tv_sec, tv.tv_usec * 1000, RCL_CLOCK_UNINITIALIZED);
}

RosClock::RosClock(rclcpp::Clock::SharedPtr clock) : mInternalClock(clock)
{
}

timeval RosClock::now()
{
	return fromRosTime(mInternalClock->now());
}

rclcpp::Time RosClock::ros_now()
{
	return mInternalClock->now();
}
