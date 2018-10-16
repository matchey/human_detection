
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "euclidean_cluster/indices.h"

class HumanPosPublisher
{
	public:
	using PointT = pcl::PointNormal;
	using PointCloud = pcl::PointCloud<PointT>;
	using PointCloudPtr =  PointCloud::Ptr;
	using PointCloudConstPtr = PointCloud::ConstPtr;

	using SyncPolicy = message_filters::sync_policies::ExactTime
						<euclidean_cluster::IndicesClusters, sensor_msgs::PointCloud2>;

	HumanPosPublisher();

	private:
	void callback(const euclidean_cluster::IndicesClusters::ConstPtr&,
				  const sensor_msgs::PointCloud2::ConstPtr&);
	void publish();
	void setPosition();
	void calcCentroid(const PointCloudPtr&, PointT&) const;

	std::string topic_sub_id;
	std::string topic_sub_pc;

	ros::NodeHandle n;
	message_filters::Subscriber<euclidean_cluster::IndicesClusters> id_sub;
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
	message_filters::Synchronizer<SyncPolicy> sync;
	ros::Publisher pc_pub;
	ros::Publisher rm_pub;
	ros::Publisher pos_pub;

	std::vector<pcl::PointIndices> cluster_indices;
	PointCloudPtr dspoints;
	PointCloudPtr pc_human; // human pointcloud
	PointCloudPtr pc_removed; // human removed
	PointCloudPtr pc_pos; // human centroid
};

HumanPosPublisher::HumanPosPublisher()
	: topic_sub_id("/cluster/indices/split"), topic_sub_pc("/rm_ground/downsampled"),
	  id_sub(n, topic_sub_id, 1), pc_sub(n, topic_sub_pc, 1), 
	  sync(SyncPolicy(10), id_sub, pc_sub),
	  dspoints(new PointCloud), pc_human(new PointCloud),
	  pc_removed(new PointCloud), pc_pos(new PointCloud)
{
	sync.registerCallback(boost::bind(&HumanPosPublisher::callback, this, _1, _2));

	pos_pub = n.advertise<sensor_msgs::PointCloud2>("/cluster/human/position", 1);
	pc_pub = n.advertise<sensor_msgs::PointCloud2>("/cluster/human/points", 1);
	rm_pub = n.advertise<sensor_msgs::PointCloud2>("/cluster/human/removed", 1);
}

void HumanPosPublisher::callback(const euclidean_cluster::IndicesClusters::ConstPtr& ic,
							    const sensor_msgs::PointCloud2::ConstPtr& pc2)
{
	euclidean_cluster::toPCL(*ic, cluster_indices);
	pcl::fromROSMsg(*pc2, *dspoints);

	setPosition();

	publish();
}

void HumanPosPublisher::publish()
{
	sensor_msgs::PointCloud2 pc2;

	pcl::toROSMsg(*pc_human, pc2);
	pc_pub.publish(pc2);

	pcl::toROSMsg(*pc_removed, pc2);
	rm_pub.publish(pc2);

	pcl::toROSMsg(*pc_pos, pc2);
	pos_pub.publish(pc2);
}

void HumanPosPublisher::setPosition()
{
	PointT p;
	PointCloudPtr pc(new PointCloud);


	pc_human->points.clear();
	pc_removed->points.clear();
	pc_pos->points.clear();

	std::vector<int> id_human;
	std::vector<int> id_rm;

	for(const auto& it : cluster_indices){
		pc->points.clear();
		for(const auto& pit : it.indices){
			pc->points.push_back(dspoints->points[pit]);
			pc_human->push_back(dspoints->points[pit]);
			id_human.push_back(pit);
		}
		calcCentroid(pc, p);
		pc_pos->push_back(p);
	}

	std::sort(id_human.begin(), id_human.end());

	int npoints = dspoints->points.size();

	std::vector<int>::iterator it = id_human.begin();

	for(int i = 0; i < npoints; ++i){
		if(it == id_human.end() || i != *it){
			pc_removed->points.push_back(dspoints->points[i]);
			id_rm.push_back(i);
		}else{
			++it;
		}
	}

	pc_human->width = pc_human->points.size();
	pc_removed->width = pc_removed->points.size();
	pc_pos->width = pc_pos->points.size();

	pc_human->height = pc_removed->height = pc_pos->height = 1;
	pc_human->is_dense = pc_removed->is_dense =  pc_pos->is_dense = true;

	pc_human->header = pc_removed->header = pc_pos->header = dspoints->header;
}

void HumanPosPublisher::calcCentroid(const PointCloudPtr& pc, PointT& p) const
{
	int npoints = pc->points.size();

	if(!npoints) return;

	Eigen::Vector3d sum(0.0, 0.0, 0.0);

	for(int i = 0; i < npoints; ++i){
		sum.x() += pc->points[i].x;
		sum.y() += pc->points[i].y;
		sum.z() += pc->points[i].z;
	}

	p.x = sum.x() / npoints;
	p.y = sum.y() / npoints;
	p.z = sum.z() / npoints;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_human_pos_publisher");
	std::cout << "test human pos publisher" << std::endl;

	HumanPosPublisher hpp;

	ros::spin();

	return 0;
}

