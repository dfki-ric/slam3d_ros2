#pragma once

#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/core/Graph.hpp>

#include <tf2_ros/buffer.h>

namespace slam3d
{
	class TfGravity : public PoseSensor
	{
	public:
		TfGravity(Graph* g, Logger* l, tf2_ros::Buffer* tf, tf2::Duration timeout,
			const std::string& robot_f, const std::string& reference_f, const Direction& reference);
		
		Transform getPose(timeval stamp);
		
		void handleNewVertex(IdType vertex);
		
	private:
		tf2_ros::Buffer* mTfBuffer;
		tf2::Duration mTimeout;
		Direction mGravityReference;
		std::string mRobotFrame;
		std::string mReferenceFrame;
	};
}
