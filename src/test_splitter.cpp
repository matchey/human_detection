
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <boost/thread.hpp>
#include "euclidean_cluster/indices.h"
#include "human_detection/split_cluster.h"

class ClusterPublisher
{
	public:
	using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
	using PointCloudPtr =  PointCloud::Ptr;
	using PointCloudConstPtr = PointCloud::ConstPtr;

	using SyncPolicy = message_filters::sync_policies::ExactTime
						<euclidean_cluster::IndicesClusters, sensor_msgs::PointCloud2>;

	ClusterPublisher();

	private:
	void callback(const euclidean_cluster::IndicesClusters::ConstPtr&,
				  const sensor_msgs::PointCloud2::ConstPtr&);
	void publish();

	std::string topic_sub_id;
	std::string topic_sub_pc;

	ros::NodeHandle n;
	message_filters::Subscriber<euclidean_cluster::IndicesClusters> id_sub;
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
	// ros::Publisher pc_pub;
	ros::Publisher pub;

	message_filters::Synchronizer<SyncPolicy> sync;

	human_detection::Splitter<pcl::PointXYZ> spliter;

	std::vector<pcl::PointIndices> cluster_indices;
	PointCloudPtr dspoints;
	PointCloudPtr cpoints;

	boost::mutex pt_mutex;
};

ClusterPublisher::ClusterPublisher()
	: topic_sub_id("/cluster/indices"), topic_sub_pc("/rm_ground/downsampled"),
	  id_sub(n, topic_sub_id, 1), pc_sub(n, topic_sub_pc, 1), 
	  sync(SyncPolicy(10), id_sub, pc_sub),
	  dspoints(new PointCloud), cpoints(new PointCloud)
{
	sync.registerCallback(boost::bind(&ClusterPublisher::callback, this, _1, _2));

	pub = n.advertise<euclidean_cluster::IndicesClusters>(topic_sub_id+"/split", 1);
}

void ClusterPublisher::callback(const euclidean_cluster::IndicesClusters::ConstPtr& ic,
							    const sensor_msgs::PointCloud2::ConstPtr& pc2)
{
	boost::mutex::scoped_lock(pt_mutex);
	cluster_indices.clear();
	euclidean_cluster::toPCL(*ic, cluster_indices);

	pcl::fromROSMsg(*pc2, *dspoints);

	publish();
}

void ClusterPublisher::publish()
{
	spliter.split(cluster_indices, dspoints);

	spliter.publish();

	euclidean_cluster::IndicesClusters ic;
	euclidean_cluster::fromPCL(cluster_indices, ic);
	pcl_conversions::fromPCL(dspoints->header, ic.header);
	pub.publish(ic);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_split_cluster");
	std::cout << "test split cluster" << std::endl;

	ClusterPublisher cp;

	ros::spin();

	return 0;
}

