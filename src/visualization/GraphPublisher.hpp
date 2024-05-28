#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>

namespace slam3d
{
	class Graph;
}

struct Color
{
	double r;
	double g;
	double b;
};

class GraphPublisher
{
public:
	GraphPublisher(rclcpp::Node* n, slam3d::Graph* g);
	~GraphPublisher();
	
	void addNodeSensor(const std::string& sensor, double r, double g, double b);
	void addEdgeSensor(const std::string& sensor);
	
	void publishNodes(const rclcpp::Time& stamp, const std::string& frame);
	void publishEdges(const std::string& sensor, const rclcpp::Time& stamp, const std::string& frame);
	void publishPoseEdges(const std::string& sensor, const rclcpp::Time& stamp, const std::string& frame);
	
private:
	rclcpp::Node* mNode;
	slam3d::Graph* mGraph;
	std::map<std::string, Color> mSensorMap;

	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mPosePublisher;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mLabelPublisher;
	std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> mEdgePublisherMap;
};
