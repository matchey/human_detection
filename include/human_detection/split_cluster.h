
#ifndef __SPLIT_CLUSTER_H
#define __SPLIT_CLUSTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
// #include "visualization_tools/bounding_box.h"
#include "visualization_tools/bounding_box_array.h"

namespace human_detection
{
	template<typename PointT>
	class Splitter
	{
		public:
		using PointCloud = pcl::PointCloud<PointT>;
		using PointCloudPtr = typename PointCloud::Ptr;
		using IndicesClusters = std::vector<pcl::PointIndices>;
		// typedef typename PointCloud::ConstPtr PointCloudConstPtr;

		Splitter();
		void split(IndicesClusters&, const PointCloudPtr&);
		void publish() const; // for debug

		private:
		int toDivide() const;
		void assign(const int&, IndicesClusters&);
		void constructGrid(const int&, IndicesClusters&);
		void divide(const int&, IndicesClusters&);
		double dist(const geometry_msgs::Pose&) const;

		PointCloudPtr pc;
		PointCloudPtr dspoints;
		visualization_tools::BoundingBoxArray bba;
		visualization_tools::BoundingBox bb;

		// for grid
		Eigen::Affine3d toOrigin;
		std::vector<int> vert_div;
		const int grid_dim;
		int dim_horz;
		int dim_vert;
		double size_horz;
		double size_vert;
		int npoints;
	};
	template class Splitter<pcl::PointXYZ>;
	template class Splitter<pcl::PointXYZI>;
	// template class Cluster<pcl::PointXYZRGBA>;
	template class Splitter<pcl::PointXYZRGB>;
	// template class Cluster<pcl::PointXY>;
	// template class Cluster<pcl::Normal>;
	template class Splitter<pcl::PointNormal>;
	// template class Cluster<pcl::PointXYZRGBNormal>;
	template class Splitter<pcl::PointXYZINormal>;
}

#endif

