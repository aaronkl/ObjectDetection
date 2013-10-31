/*
 * object_recognition.h
 *
 *  Created on: Jul 16, 2013
 *      Author: Aaron Klein
 */

#ifndef OBJECT_RECOGNITION_H_
#define OBJECT_RECOGNITION_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/fpfh.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/keypoints/sift_keypoint.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class ObjectRecognition
{
public:
	/*
	 * Initialize all algorithm parameters and variables
	 */
	ObjectRecognition();

	/*
	 * Starts the recognition pipeline
	 */
	void recognize(pcl::PointCloud<PointT>::Ptr &scene);

	/*
	 * Loads the models and computes keypoints and descriptors. This function has to be called before recognize()
	 */
	void loadModel(std::vector<pcl::PointCloud<PointT>::Ptr> &models);

	/*
	 * Setter functions
	 */
	void setLeafSize(float leafSize) {  _leaf_size = leafSize; 	}
	void setNeRadius(float neRadius) { _ne_radius = neRadius; }
	void setCgSize(float cgSize) { _cg_size = cgSize;	}
	void setCgThresh(float cgThresh) {	_cg_thresh = cgThresh;	}
	void setDescrRad(float descrRad) {_descr_rad = descrRad;	}
	void setModelSs(float modelSs) {	_model_ss = modelSs;}
	void setRfRad(float rfRad) {	_rf_rad = rfRad;}
	void setSceneSs(float sceneSs) {	_scene_ss = sceneSs;}
	void setShowCorrespondences(bool showCorrespondences) {	_show_correspondences = showCorrespondences;}
	void setShowVisualization(bool showVisualization) {	_show_visualization = showVisualization; }
	void setUseCloudResolution(bool useCloudResolution) { _use_cloud_resolution = useCloudResolution; }


protected:
	double computeCloudResolution (const pcl::PointCloud<PointT>::ConstPtr &cloud);

	void computeKeypoints(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &keypoints, double resolution);

	void computeNormals(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<NormalType>::Ptr &normals);

	void computeDescriptors(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &keypoints, pcl::PointCloud<DescriptorType>::Ptr &descriptors);

	void extractPlanes(pcl::PointCloud<PointT>::Ptr &cloud);

	void downsample(pcl::PointCloud<PointT>::Ptr &cloud);

	void setupResolutionInvariance(const pcl::PointCloud<PointT>::Ptr &cloud);

	void findCorrespondences(pcl::PointCloud<DescriptorType>::Ptr &scene_descriptors,
			pcl::PointCloud<DescriptorType>::Ptr &model_descriptors);

	void groupCorrespondences(pcl::PointCloud<PointT>::Ptr &scene, pcl::PointCloud<PointT>::Ptr &scene_keypoints,
			pcl::PointCloud<PointT>::Ptr &model, pcl::PointCloud<PointT>::Ptr &model_keypoints);

	float refinement(const pcl::PointCloud<PointT>::Ptr &target, const pcl::PointCloud<PointT>::Ptr &source, Eigen::Matrix4f &final);

	void verify(pcl::PointCloud<PointT>::Ptr &scene);

	void clustering(pcl::PointCloud<PointT>::Ptr &scene, std::vector<pcl::PointCloud<PointT>::Ptr > &clusters);

	void ransac(pcl::PointCloud<PointT>::Ptr &scene, pcl::PointCloud<PointT>::Ptr &model, pcl::PointCloud<PointT>::Ptr &final);

private:

	/*
	 * Model information
	 */
	std::vector<pcl::PointCloud<PointT>::Ptr> _models;
	std::vector<pcl::PointCloud<PointT>::Ptr> _model_keypoints;
	std::vector<pcl::PointCloud<DescriptorType>::Ptr> _model_descriptors;

	pcl::CorrespondencesPtr _model_scene_corrs;
	std::vector<pcl::Correspondences> _clustered_corrs;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > _rototranslations;

	std::vector<pcl::PointCloud<PointT>::ConstPtr> _rotated_models;
	std::vector<pcl::PointCloud<PointT>::ConstPtr> _final_models;

	/*
	 * Algorithm parameters
	 */
	bool _show_visualization ;
	bool _show_correspondences ;
	bool _use_cloud_resolution ;
	float _model_ss ;
	float _scene_ss ;
	float _rf_rad;
	float _descr_rad;
	float _cg_size;
	float _cg_thresh;
	float _ne_radius;
	float _leaf_size;
};

#endif /* OBJECT_RECOGNITION_H_ */
