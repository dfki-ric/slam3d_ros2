#pragma once

#include <slam3d/core/PoseSensor.hpp>
#include <slam3d/core/Graph.hpp>

#include <tf2_ros/buffer.h>

namespace slam3d
{
	class TfOdometry : public PoseSensor
	{
	public:
		TfOdometry(Graph* g, Logger* l, tf2_ros::Buffer* tf, const std::string& robot_f, const std::string& odom_f);
		
		Transform getPose(timeval stamp);
		
		void handleNewVertex(IdType vertex);
		
	private:
		tf2_ros::Buffer* mTfBuffer;
		Transform mLastOdometricPose;
		IdType mLastVertex;
		std::string mRobotFrame;
		std::string mOdometryFrame;
	};
}
