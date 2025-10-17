#include <TfGravity.hpp>
#include <RosClock.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

using namespace slam3d;

TfGravity::TfGravity(Graph* g, Logger* l, tf2_ros::Buffer* tf, tf2::Duration timeout,
	const std::string& robot_f, const std::string& reference_f, const Direction& reference)
: PoseSensor("Gravity", g, l), mTfBuffer(tf), mTimeout(timeout)
{
	mRobotFrame = robot_f;
	mReferenceFrame = reference_f;
	mGravityReference = reference;
}

Transform TfGravity::getPose(timeval stamp)
{
	try
	{
		return tf2::transformToEigen(mTfBuffer->lookupTransform(mReferenceFrame, mRobotFrame, fromTimeval(stamp), mTimeout));
	}catch (const tf2::TransformException & ex)
	{
		throw InvalidPose(ex.what());
	}
}

void TfGravity::handleNewVertex(IdType vertex)
{
	timeval stamp = mGraph->getVertex(vertex).timestamp;
	Transform currentPose = getPose(stamp);
	
	Eigen::Quaterniond state(currentPose.rotation());
	Direction upVector = state.inverse() * Eigen::Vector3d::UnitZ();	
	GravityConstraint::Ptr grav(new GravityConstraint(mName, upVector, mGravityReference, Covariance<2>::Identity()));
	mGraph->addConstraint(vertex, 0, grav);
}
