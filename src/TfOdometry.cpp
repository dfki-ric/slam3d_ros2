#include <TfOdometry.hpp>
#include <RosClock.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

using namespace slam3d;
using namespace std::chrono_literals;

#define TF_TIMEOUT 100ms

TfOdometry::TfOdometry(Graph* g, Logger* l, tf2_ros::Buffer* tf, const std::string& robot_f, const std::string& odom_f)
: PoseSensor("Odometry", g, l), mTfBuffer(tf), mRobotFrame(robot_f), mOdometryFrame(odom_f)
{
	mLastVertex = 0;
}

Transform TfOdometry::getPose(timeval stamp)
{
	try
	{
		return tf2::transformToEigen(mTfBuffer->lookupTransform(mOdometryFrame, mRobotFrame, fromTimeval(stamp), TF_TIMEOUT));
	}catch (const tf2::TransformException & ex)
	{
		throw InvalidPose(ex.what());
	}
}

void TfOdometry::handleNewVertex(IdType vertex)
{
	timeval stamp = mGraph->getVertex(vertex).timestamp;
	Transform currentPose = getPose(stamp);
	
	if(mLastVertex > 0)
	{
		Transform t = mLastOdometricPose.inverse() * currentPose;
		SE3Constraint::Ptr se3(new SE3Constraint(mName, t, Covariance<6>::Identity() * 0.01));
		try
		{
			mGraph->addConstraint(mLastVertex, vertex, se3);
		}catch(std::exception &e)
		{
			std::cout << "Failed to link vertex " << vertex << " to " << mLastVertex << std::endl;
			return;
		}
		mGraph->setCorrectedPose(vertex, mGraph->getVertex(mLastVertex).correctedPose * t);
	}
	
	mLastVertex = vertex;
	mLastOdometricPose = currentPose;
}
