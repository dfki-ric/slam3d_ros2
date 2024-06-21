#pragma once

#include <slam3d/core/Logger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace slam3d
{
	class RosLogger : public Logger
	{
	public:
		RosLogger(const RosClock& c, const rclcpp::Logger& l) : Logger(c), mRclCppLogger(l) {}
		virtual void message(LOG_LEVEL lvl, const std::string& message)
		{
			switch(lvl)
			{
			case DEBUG:
				RCLCPP_DEBUG(mRclCppLogger, message.c_str());
				break;
			case INFO:
				RCLCPP_INFO(mRclCppLogger, message.c_str());
				break;
			case WARNING:
				RCLCPP_WARN(mRclCppLogger, message.c_str());
				break;
			case ERROR:
				RCLCPP_ERROR(mRclCppLogger, message.c_str());
				break;
			case FATAL:
				RCLCPP_FATAL(mRclCppLogger, message.c_str());
				break;
			default:
				break;
			}
		}
	private:
		rclcpp::Logger mRclCppLogger;
	};
}
