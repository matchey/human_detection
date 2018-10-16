
// #include "visualization_tools/bounding_box_array.h"
#include "human_detection/split_cluster.h"

namespace human_detection
{
	template<typename PointT>
	Splitter<PointT>::Splitter()
	: pc(new PointCloud), dspoints(new PointCloud), grid_dim(7)
	{
	}

	template<typename PointT>
	void Splitter<PointT>::split
	     (IndicesClusters& ids, const PointCloudPtr& pc_in)
	{
		bba.clear();

		IndicesClusters ic_dived;

		dspoints = pc_in;

		pc->header = dspoints->header;

		for(const auto& it : ids){
			pc->points.clear();
			for(const auto& pit : it.indices){
				pc->points.push_back(dspoints->points[pit]);
			}
			bb.fromPCL(pc);
			IndicesClusters clusters;
			clusters.push_back(it);
			int nhumans = toDivide();
			assign(nhumans, clusters);
			for(const auto& dived : clusters){
				pc->points.clear();
				for(const auto& pit : dived.indices){
					pc->points.push_back(dspoints->points[pit]);
				}
				ic_dived.push_back(dived);
				bb.fromPCL(pc);
				if(0 < nhumans){
					bba.push_back(bb);
				}
			}
		}
		ids = ic_dived;
	}

	template<typename PointT>
	void Splitter<PointT>::publish() const
	{
		if(bba.size()){
			bba.publish();
		}
	}

	// private
	template<typename PointT>
	int Splitter<PointT>::toDivide() const
	{
		const double human_width = 0.7;
		const double height_velodyne = 1.3;

		int rtn = 0; 

		// depthとwidthが人のサイズ
		if(0.15 < bb.width() && bb.width() < 3.0 && 0.10 < bb.depth() && bb.depth() < 1.5){
			// BoundingBoxの最高点が人の頭の位置
			if(1.20 - height_velodyne < bb.head() && bb.head() < 1.9 - height_velodyne){
				// 遠くにいる時 BoundingBoxの高さ(最高点 - 最下点)が人の身長でない
				if( (2.0 < dist(bb.pose())) && !(1.20 < bb.height() && bb.height() < 1.9) ){
					rtn = 0;
				}else{
					rtn = std::max(int(bb.width() / human_width + 0.6), 1);
				}
			}
		}

		return rtn;
	}

	template<typename PointT>
	void Splitter<PointT>::assign(const int& nhumans, IndicesClusters& clusters)
	{
		if(nhumans == 1){
			return;
		}

		if(nhumans == 0){
			clusters.clear();
			return;
		}

		constructGrid(nhumans, clusters);
		divide(nhumans, clusters);
	}

	template<typename PointT>
	void Splitter<PointT>::constructGrid(const int& nhumans, IndicesClusters& clusters)
	{
		dim_horz = grid_dim * nhumans;
		dim_vert = grid_dim;
		size_horz = bb.width() / dim_horz;
		size_vert = bb.height() / dim_vert;

		int pcount[dim_horz][dim_vert];

		std::fill(pcount[0], pcount[dim_horz], 0);

		npoints = pc->points.size();

		Eigen::Quaterniond q(bb.pose().orientation.w, bb.pose().orientation.x,
				             bb.pose().orientation.y, bb.pose().orientation.z);
		Eigen::Translation<double, 3> trans(-bb.pose().position.x,
											-bb.pose().position.y, -bb.pose().position.z);
		toOrigin = q.inverse() * trans;

		for(int i = 0; i < npoints; ++i){
			Eigen::Vector3d p(pc->points[i].x, pc->points[i].y, pc->points[i].z);
			p = toOrigin * p;
			const int y = ((dim_horz/2.0) + p.y() / size_horz);
			const int z = p.z() / size_vert;

			if(0 < y && y < dim_horz && 0 < z && z < dim_vert){
				++pcount[y][z];
			}
		}

		vert_div.clear();
		for(int i = 1; i < nhumans; ++i){
			int begin = i*grid_dim - 2;
			int end = begin + 3;
			int max = 0;
			int div = begin;
			for(int horz = begin; horz < end; ++horz){
				int cnt = 0;
				for(int vert = 0; vert < grid_dim; ++vert){
					if(2 < pcount[horz][vert]){
						++cnt;
					}
				}
				if(max < cnt){
					max = cnt;
					div = horz;
				}
			}
			vert_div.push_back(div);
		}
	}

	template<typename PointT>
	void Splitter<PointT>::divide(const int& nhumans, IndicesClusters& clusters)
	{
		IndicesClusters dived;

		dived.resize(nhumans); // vert_div.size() = nhumans
		
		for(const auto& id : clusters[0].indices){
			Eigen::Vector3d p(dspoints->points[id].x,
					  	 	  dspoints->points[id].y, dspoints->points[id].z);
			p = toOrigin * p;
			const int y = ((dim_horz/2.0) + p.y() / size_horz);
			const int z = p.z() / size_vert;

			int cluster = 0;
			if(0 < y && y < dim_horz && 0 < z && z < dim_vert){
				for(const auto& div : vert_div){
					if(y <= div){
						break;
					}
					++cluster;
				}
				dived[cluster].header = dspoints->header;
				dived[cluster].indices.push_back(id);
			}
		}
		clusters = dived;
	}

	template<typename PointT>
	double Splitter<PointT>::dist(const geometry_msgs::Pose& pos) const
	{
		return sqrt(pow(pos.position.x, 2) + pow(pos.position.y, 2) + pow(pos.position.z, 2));
	}

} // namespace human_detection

