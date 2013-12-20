/*
 * object_recognition.cpp
 *
 *  Created on: Jul 16, 2013
 *      Author: Aaron Klein
 */

#include "object_recognition.h"

ObjectRecognition::ObjectRecognition()
:_leaf_size(0.001f),
 _ne_radius(0.02f),
 _cg_size(0.01),
 _cg_thresh(5),
 _descr_rad(0.02),
 _model_ss(0.01),
 _rf_rad(0.015),
 _scene_ss(0.03),
 _knn(10),
 _cluster_tolerance(0.5),
 _ransac_distance(0.05),
 _use_cloud_resolution(false),
 _show_correspondences(false),
 _show_visualization(false),
 _model_scene_corrs (new pcl::Correspondences ())
{}


void ObjectRecognition::loadModel(std::vector<pcl::PointCloud<PointT>::Ptr> &models)
{
	//_models = models;

	pcl::VoxelGrid<PointT> sor;
	sor.setLeafSize (0.001,0.001,0.001);

	for(std::vector<pcl::PointCloud<PointT>::Ptr>::iterator it = models.begin(); it != models.end(); it++)
	{
		pcl::PointCloud<PointT>::Ptr model(new pcl::PointCloud<PointT> ());

		std::cout << "Normalizing model point cloud" << std::endl;

		sor.setInputCloud (*it);
		sor.filter (*model);
		_models.push_back(model);

		std::cout << "Extracting model keypoints" << std::endl;

		pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT>);
		computeKeypoints(*it, keypoints, _model_ss);
		_model_keypoints.push_back(keypoints);

		std::cout << "Compute model descriptors" << std::endl;

		pcl::PointCloud<DescriptorType>::Ptr descriptors (new pcl::PointCloud<DescriptorType>);
		computeDescriptors(*it,keypoints, descriptors);
		_model_descriptors.push_back(descriptors);
	}
}

void ObjectRecognition::setupResolutionInvariance(const pcl::PointCloud<PointT>::Ptr &cloud)
{
	float resolution = static_cast<float> (computeCloudResolution (cloud));
	if (resolution != 0.0f)
	{
		_model_ss *= resolution;
		_scene_ss *= resolution;
		_rf_rad *= resolution;
		_descr_rad *= resolution;
		_cg_size *= resolution;
		_ne_radius *= resolution;
	}

	std::cout << "Model resolution: " << resolution << std::endl;
	std::cout << "Model sampling size: " << _model_ss << std::endl;
	std::cout << "Scene sampling size: " << _scene_ss << std::endl;
	std::cout << "LRF support radius: " << _rf_rad << std::endl;
	std::cout << "SHOT descriptor radius: " << _descr_rad << std::endl;
	std::cout << "Clustering bin size: " << _cg_size << std::endl << std::endl;
	std::cout << "Normal estimation radius: " << _ne_radius << std::endl << std::endl;
}

double ObjectRecognition::computeCloudResolution(const pcl::PointCloud<PointT>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return (res);
}

void ObjectRecognition::findCorrespondences(pcl::PointCloud<DescriptorType>::Ptr &scene_descriptors, pcl::PointCloud<DescriptorType>::Ptr &model_descriptors)
{
	pcl::KdTreeFLANN<DescriptorType> kd_tree;
	kd_tree.setInputCloud(model_descriptors);

	for (size_t i = 0; i < scene_descriptors->size (); ++i)
	{
		std::vector<int> neigh_indices (_knn);
		std::vector<float> neigh_sqr_dists (_knn);
		if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
		{
			continue;
		}

		int found_neighs = kd_tree.nearestKSearch (scene_descriptors->at (i), _knn, neigh_indices, neigh_sqr_dists);
		for(int k = 0; k < found_neighs; k++)
		{
			if(found_neighs > 0 && neigh_sqr_dists[k] < 0.25) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			{
				pcl::Correspondence corr (neigh_indices[k], static_cast<int> (i), neigh_sqr_dists[k]);
				_model_scene_corrs->push_back (corr);
			}
		}
	}
}

void ObjectRecognition::computeKeypoints(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &keypoints, double resolution)
{
	pcl::PointCloud<int> sampled_indices;

	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud (cloud);
	uniform_sampling.setRadiusSearch (resolution);
	uniform_sampling.compute (sampled_indices);

	pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);


	//	const float min_scale = 0.1f;
	//	const int n_octaves = 6;
	//	const int n_scales_per_octave = 10;
	//	const float min_contrast = 0.5f;
	////    const int k_sift = 5;
	////    const double r_sift = 0.005;
	//
	//	pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift;
	//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
	//	pcl::PointCloud<pcl::PointWithScale>::Ptr sifts (new pcl::PointCloud<pcl::PointWithScale>);
	//	sift.setInputCloud(cloud);
	//	sift.setSearchMethod (tree);
	////    sift.setKSearch (k_sift);
	////    sift.setRadiusSearch(r_sift);
	//	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	//	sift.setMinimumContrast(min_contrast);
	//	sift.compute (*sifts);
	//
	//	pcl::copyPointCloud(*sifts, *keypoints);

	//	double model_resolution = 0.005f;
	//	pcl::search::KdTree<PointT>::Ptr scene_tree (new pcl::search::KdTree<PointT> ());
	//
	//	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
	//	iss_detector.setSearchMethod (scene_tree);
	//	iss_detector.setSalientRadius (model_resolution);
	//	iss_detector.setNonMaxRadius (model_resolution);
	////	iss_detector.setThreshold21 (0.975);
	////	iss_detector.setThreshold32 (0.975);
	////	iss_detector.setMinNeighbors (5);
	////	iss_detector.setNumberOfThreads (4);
	//	iss_detector.setInputCloud (cloud);
	//	iss_detector.compute (*keypoints);

}

void ObjectRecognition::verify(pcl::PointCloud<PointT>::Ptr &scene)
{
	//	std::vector<bool> mask_hv;
	//	pcl::GlobalHypothesesVerification<PointT, PointT> hv;
	//
	//	//	std::cout << "Size of rotated models: " << _rotated_models.size() << std::endl;
	//
	//	// too slow and doesn't find anything ...
	//	hv.setResolution (0.003f);
	//	hv.setInlierThreshold (0.3f);
	//	hv.setRadiusClutter (0.3f);
	//	hv.setOcclusionThreshold (0.3f);
	//	hv.setSceneCloud(scene);
	//	hv.addModels (_rotated_models);
	//	hv.addCompleteModels(_rotated_models);
	//
	//	hv.verify ();
	//
	//	hv.getMask (mask_hv);
	//	for (size_t i = 0; i < _rotated_models.size (); i++)
	//	{
	//		if (!mask_hv[i])
	//			continue;
	//
	//		_final_models.push_back (_rotated_models.at (i));
	//	}
	//
	//	std::cout << "try: ";
	//	for(unsigned i = 0; i < mask_hv.size(); i++)
	//		std::cout << mask_hv[i] << " ; ";
	//	std::cout << std::endl;
}

void ObjectRecognition::computeNormals(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<NormalType>::Ptr &normals)
{
	pcl::NormalEstimationOMP<PointT, NormalType> norm_est;
	norm_est.setRadiusSearch(_ne_radius);
	//	norm_est.setKSearch(10);
	norm_est.setInputCloud (cloud);
	norm_est.compute (*normals);
}

void ObjectRecognition::groupCorrespondences(pcl::PointCloud<PointT>::Ptr &scene, pcl::PointCloud<PointT>::Ptr &scene_keypoints,
		pcl::PointCloud<PointT>::Ptr &model, pcl::PointCloud<PointT>::Ptr &model_keypoints)
{
	pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
	pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

	//TODO: Why are the normals computed again? Are they computed from the full point cloud or just from the keypoints?????
	std::cout << "Compute normals ... " << std::flush;
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal> ());
	computeNormals(scene, scene_normals);

	pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());
	computeNormals(model, model_normals);

	std::cout << "done" << std::endl;

	std::cout << "RF estimation ... " << std::flush;
	pcl::BOARDLocalReferenceFrameEstimation<PointT, NormalType, RFType> rf_est;
	rf_est.setFindHoles (true);
	rf_est.setRadiusSearch (_rf_rad);

	rf_est.setInputCloud (model_keypoints);
	rf_est.setInputNormals (model_normals);
	rf_est.setSearchSurface (model);
	rf_est.compute (*model_rf);

	rf_est.setInputCloud (scene_keypoints);
	rf_est.setInputNormals (scene_normals);
	rf_est.setSearchSurface (scene);
	rf_est.compute (*scene_rf);

	std::cout << "done" << std::endl;

	std::cout << "Correspondence grouping ... " << std::flush;
	pcl::Hough3DGrouping<PointT, PointT, RFType, RFType> clusterer;
	clusterer.setHoughBinSize (_cg_size);
	clusterer.setHoughThreshold (_cg_thresh);
	clusterer.setUseInterpolation (true);
	clusterer.setUseDistanceWeight (false);
	clusterer.setInputCloud (model_keypoints);
	clusterer.setInputRf (model_rf);
	clusterer.setSceneCloud (scene_keypoints);
	clusterer.setSceneRf (scene_rf);
	clusterer.setModelSceneCorrespondences (_model_scene_corrs);

	_clustered_corrs.clear();
	//	clusterer.cluster (_clustered_corrs);
	clusterer.recognize (_rototranslations, _clustered_corrs);
	//	pcl::GeometricConsistencyGrouping<PointT, PointT> gc_clusterer;
	//	gc_clusterer.setGCSize (_cg_size);
	//	gc_clusterer.setGCThreshold (_cg_thresh);
	//
	//	gc_clusterer.setInputCloud (model_keypoints);
	//	gc_clusterer.setSceneCloud (scene_keypoints);
	//	gc_clusterer.setModelSceneCorrespondences (_model_scene_corrs);
	//
	//	//gc_clusterer.cluster (clustered_corrs);
	//	gc_clusterer.recognize (_rototranslations, _clustered_corrs);
	std::cout << "done" << std::endl;

}

void ObjectRecognition::extractPlanes(pcl::PointCloud<PointT>::Ptr &cloud)
{
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (_ransac_distance);
	int nr_points = (int) cloud->points.size ();
	while (cloud->points.size () > 0.5 * nr_points)
	{
		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud = *cloud_f;
	}
}

void ObjectRecognition::clustering(pcl::PointCloud<PointT>::Ptr &scene, std::vector<pcl::PointCloud<PointT>::Ptr > &clusters)
{
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (scene);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (_cluster_tolerance);
	ec.setMinClusterSize (1000);
	ec.setMaxClusterSize (300000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (scene);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (scene->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		clusters.push_back(cloud_cluster);
	}
}

void ObjectRecognition::ransac(pcl::PointCloud<PointT>::Ptr &scene, pcl::PointCloud<PointT>::Ptr &model, pcl::PointCloud<PointT>::Ptr &final)
{
	std::vector<int> inliers;
	pcl::SampleConsensusModelRegistration<PointT>::Ptr consensusmodel (new pcl::SampleConsensusModelRegistration<PointT>(model));
	consensusmodel->setInputCloud(scene);
	consensusmodel->setInputTarget(model);

	pcl::RandomSampleConsensus<PointT> ransac (consensusmodel);
	ransac.setDistanceThreshold (0.01f);
	ransac.setMaxIterations(1000);
	ransac.computeModel();
	ransac.getInliers(inliers);

	pcl::copyPointCloud<PointT>(*scene, inliers, *final);
}

void ObjectRecognition::downsample(pcl::PointCloud<PointT>::Ptr &cloud)
{
	pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
	*tmp = *cloud;
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (tmp);
	sor.setLeafSize (_leaf_size, _leaf_size, _leaf_size);
	sor.filter (*cloud);
}

void ObjectRecognition::computeDescriptors(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &keypoints, pcl::PointCloud<DescriptorType>::Ptr &descriptors)
{
	//TODO: Compute normals from cloud or keypoints??????
	pcl::PointCloud<NormalType>::Ptr normals (new pcl::PointCloud<NormalType> ());
	computeNormals(cloud, normals);
//	computeNormals(keypoints, normals);
	pcl::SHOTColorEstimationOMP<PointT, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch (_descr_rad);
	descr_est.setInputCloud (keypoints);
	descr_est.setInputNormals (normals);
	descr_est.setSearchSurface (cloud);
	descr_est.compute (*descriptors);

//		pcl::SHOTEstimationOMP<PointT, NormalType, DescriptorType> descr_est;
//		descr_est.setRadiusSearch (_descr_rad);
//		descr_est.setInputCloud (keypoints);
//		descr_est.setInputNormals (normals);
//		descr_est.setSearchSurface (cloud);
//		descr_est.compute (*descriptors);

//		pcl::PFHEstimation<PointT, NormalType, DescriptorType> pfh;
//		pfh.setInputCloud (keypoints);
//		pfh.setInputNormals (normals);
//		pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
//		pfh.setSearchMethod (tree);
//		// Use all neighbors in a sphere of radius 5cm
//		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//		pfh.setRadiusSearch (_descr_rad);
//		pfh.compute (*descriptors);

	//		pcl::PFHRGBEstimation<PointT, NormalType, DescriptorType> pfhrgb;
	//		pfhrgb.setInputCloud (cloud);
	//		pfhrgb.setInputNormals (normals);
	//		pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	//		pfhrgb.setSearchMethod (tree);
	//		pfhrgb.setRadiusSearch (_descr_rad);
	//		pfhrgb.compute (*descriptors);

	//	pcl::PointCloud<pcl::IntensityGradient>::Ptr gradient(new pcl::PointCloud<pcl::IntensityGradient>());
	//	pcl::search::KdTree<PointT>::Ptr tree_gradient(new pcl::search::KdTree<PointT>());
	//
	//	pcl::IntensityGradientEstimation<PointT, pcl::Normal, pcl::IntensityGradient>grad;
	//
	//	grad.setSearchMethod (tree_gradient);
	//	grad.setRadiusSearch (0.10);
	//	grad.setInputCloud(keypoints);
	//	grad.setInputNormals(normals);
	//	grad.compute(*gradient);
	//
	//    pcl::RIFTEstimation<PointT, pcl::IntensityGradient, DescriptorType> rift_est;
	//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> (false));
	//    rift_est.setSearchMethod (tree);
	//    rift_est.setRadiusSearch (_descr_rad);
	//    rift_est.setNrDistanceBins (4);
	//    rift_est.setNrGradientBins (8);
	//
	//    rift_est.setInputCloud (keypoints);
	//    rift_est.setInputGradient (gradient);
	//
	//    rift_est.compute (*descriptors);
}

bool ObjectRecognition::refinement(const pcl::PointCloud<PointT>::Ptr &target,
		const pcl::PointCloud<PointT>::Ptr &source, Eigen::Matrix4f &final)
{
	pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;
	pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

	icp.setMaxCorrespondenceDistance (0.01f);
	icp.setMaximumIterations (2000);
	icp.setTransformationEpsilon (1e-6);
	icp.setEuclideanFitnessEpsilon(0);
	icp.setInputCloud (source);
	icp.setInputTarget (target);
	icp.setRANSACOutlierRejectionThreshold(0.01f);
	icp.align (*out);
	std::cout << "ICP has converged: " << icp.hasConverged() <<  " ICP Fitness Score: " << icp.getFitnessScore () << std::endl;

	final = icp.getFinalTransformation();

	//return (icp.getFitnessScore());
	return (icp.hasConverged());
}


void ObjectRecognition::recognize(pcl::PointCloud<PointT>::Ptr &scene_unfiltered)
{
//	pcl::visualization::PCLVisualizer viewer ("Object Recognition");

	std::cout << "Scene points before normalization: " << scene_unfiltered->points.size() << std::endl;
	pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT> ());

	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (scene_unfiltered);
	sor.setLeafSize (0.001,0.001,0.001);
	sor.filter (*scene);

	std::cout << "Scene points after normalization: " << scene->points.size() << std::endl;

	pcl::PointCloud<PointT>::Ptr scene_keypoints (new pcl::PointCloud<PointT> ());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
	std::vector<pcl::PointCloud<PointT>::Ptr > clusters;

	int index_clustered = 0;
	int id = 0;
	int number_off_scene_model_clustered = 0;

	std::fstream output_file;
	output_file.open("/home/kleinaa/devel/workspace/ObjectRecognition/object_recognition_out.dat", ios::out);

	extractPlanes(scene);

	//clustering(scene, clusters);

	std::cout << "Extracting scene keypoints" << std::endl;
	computeKeypoints(scene, scene_keypoints,_scene_ss);

	std::cout << "Compute scene descriptors" << std::endl;
	computeDescriptors(scene, scene_keypoints, scene_descriptors);

	std::fstream debug_file;
	debug_file.open("/home/kleinaa/devel/workspace/ObjectRecognition/debug_obj_rec.txt", ios::out);
	debug_file << "Scene total points: " << scene->size () << std::endl;
	debug_file << "Keypoints: " << scene_keypoints->size () << std::endl;
	debug_file << "Descriptors: " << scene_descriptors->points.size() <<  std::endl;

	try{
		for(unsigned i = 0; i < _models.size(); i++)
		{

			//setupResolutionInvariance(_models.at(i));


			//			pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT>);
			//			computeKeypoints(model, keypoints, _model_ss);
			//			_model_keypoints.push_back(keypoints);
			//
			//			pcl::PointCloud<DescriptorType>::Ptr descriptors (new pcl::PointCloud<DescriptorType>);
			//			computeDescriptors(model,keypoints, descriptors);
			//			_model_descriptors.push_back(descriptors);



			//			for(int i=0; i < scene_descriptors->size(); i++)
			//			{
			//				std::stringstream ss_sphere;
			//				ss_sphere << "sphere_" << i;
			//
			//				PointT& center = scene_keypoints->at(i);
			//				viewer.addSphere(center, _descr_rad, ss_sphere.str());
			//			}
			//			//			std::cout << "DEBUG: Visualize keypoints: " << scene_keypoints->points.size() << std::endl;
			//			//			pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_handler (scene, 255, 255, 255);
			//			//			viewer.addPointCloud (scene, scene_handler, "scene");
			//			//			pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_keypoints_handler (scene_keypoints, 0, 0, 255);
			//			//			viewer.addPointCloud (scene_keypoints, scene_keypoints_handler, "scene_keypoints");
			//			//
			//			while (!viewer.wasStopped ())
			//			{
			//				viewer.spinOnce ();
			//			}

			std::stringstream ss;
			ss << "scene_cluster_" << id++;

			std::cout << "Start search for model: " << i << std::endl;

			std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << "; Descriptors: " << scene_descriptors->points.size() <<  std::endl;

			std::cout << "Model total points: " << _models.at(i)->size() << "; Selected Keypoints: " << _model_keypoints.at(i)->size () << "; Descriptors: " << _model_descriptors.at(i)->points.size() << std::endl;

			std::cout << "Matching ... " << std::endl;

			findCorrespondences(scene_descriptors, _model_descriptors.at(i));

			//visualize correspondences
//			pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_keypoints_handler (scene_keypoints, 0, 0, 255);
//			viewer.addPointCloud (scene_keypoints, scene_keypoints_handler, "scene_keypoints");
//			pcl::PointCloud<PointT>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointT> ());
//			pcl::transformPointCloud (*_model_keypoints.at(i), *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
//			off_scene_model_keypoints->sensor_orientation_.x() = 0;
//			off_scene_model_keypoints->sensor_orientation_ = scene_keypoints->sensor_orientation_;
//			off_scene_model_keypoints->sensor_origin_ = scene_keypoints->sensor_origin_;
//			std::stringstream ss_line;
//			ss_line << "off_scene_model" << i;
//			pcl::visualization::PointCloudColorHandlerCustom<PointT> off_scene_model_color_handler (off_scene_model_keypoints, 255, 255, 128);
//			viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_color_handler, ss_line.str());
//			for (size_t i = 0; i < _model_scene_corrs->size (); ++i)
//			{
//				std::stringstream ss_line;
//				ss_line << "correspondence_line" << i;
//				PointT& model_point = off_scene_model_keypoints->at (_model_scene_corrs->at(i).index_query);
//				PointT& scene_point = scene_keypoints->at (_model_scene_corrs->at(i).index_match);
//				viewer.addLine<PointT, PointT> (model_point, scene_point, 0, 255, 0, ss_line.str ());
//			}
//			while (!viewer.wasStopped ())
//			{
//				viewer.spinOnce ();
//			}

			std::cout << "Model scene correspondences: " << _model_scene_corrs->size() << std::endl;

			std::cout << "Correspondence grouping ... " << std::endl;

			groupCorrespondences(scene, scene_keypoints, _models.at(i), _model_keypoints.at(i));

			std::cout << "Clustered_corrs: " << _clustered_corrs.size() << std::endl;

			std::cout << "Model instances found: " << _rototranslations.size () << std::endl;

			for (size_t j = 0; j < _rototranslations.size (); ++j)
			{

				pcl::PointCloud<PointT>::Ptr rotated_model (new pcl::PointCloud<PointT> ());
				pcl::transformPointCloud (*_models.at(i), *rotated_model, _rototranslations[j]);

				std::cout << "ICP refinement ... " << std::endl;
				Eigen::Matrix4f transformation;
				if(refinement(rotated_model, scene_keypoints, transformation))
				{
					pcl::PointCloud<PointT>::Ptr aligned_model (new pcl::PointCloud<PointT> ());
					pcl::transformPointCloud (*rotated_model, *aligned_model, transformation);

					Eigen::Matrix3f rotation = transformation.block<3,3>(0, 0);
					Eigen::Vector3f translation = transformation.block<3,1>(0, 3);

					printf ("\n");
					printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
					printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
					printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
					printf ("\n");
					printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

					PointT min_point, max_point;
					pcl::getMinMax3D(*aligned_model,min_point,max_point);
					output_file << min_point.x << " " << min_point.y << " " << min_point.z << std::endl;
					output_file << max_point.x << " " << max_point.y << " " << max_point.z << std::endl;

					pcl::PointCloud<PointT>::ConstPtr tmp = aligned_model;
					_rotated_models.push_back(tmp);


					if(_show_visualization)
					{
						pcl::visualization::PCLVisualizer viewer ("Object Recognition");

						pcl::visualization::PointCloudColorHandlerCustom<PointT> color_scene (scene, 255, 255, 255);
						scene_keypoints->sensor_orientation_.x() = 0; //to visualize the lines between corresponding points correctly

						viewer.addPointCloud (scene, color_scene, "scene");

						// Drawing a line for each clustered correspondence
						pcl::PointCloud<PointT>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointT> ());

						pcl::transformPointCloud (*_models.at(i), *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
						off_scene_model_keypoints->sensor_orientation_.x() = 0;

						off_scene_model_keypoints->sensor_orientation_ = scene_keypoints->sensor_orientation_;
						off_scene_model_keypoints->sensor_origin_ = scene_keypoints->sensor_origin_;

						std::stringstream ss_line;
						ss_line << "off_scene_model_clustered" << number_off_scene_model_clustered++;

						pcl::visualization::PointCloudColorHandlerCustom<PointT> off_scene_model_color_handler (off_scene_model_keypoints, 255, 255, 128);
						viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_color_handler, ss_line.str());

						pcl::PointCloud<PointT>::Ptr model_in_scene_cloud(new pcl::PointCloud<PointT>);
						for (size_t k = 0; k < _clustered_corrs[j].size (); ++k)
						{
							std::stringstream ss_line;
							ss_line << "clustered_correspondence_line" << index_clustered++;
							PointT& model_point = off_scene_model_keypoints->at (_clustered_corrs[j][k].index_query);
							PointT& scene_point = scene_keypoints->at (_clustered_corrs[j][k].index_match);

							//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
							viewer.addLine<PointT, PointT> (model_point, scene_point, 255, 0, 0, ss_line.str ());
							model_in_scene_cloud->points.push_back(scene_point);
						}

						//						pcl::visualization::PointCloudColorHandlerCustom<PointT> rotated_model_color_handler (model_in_scene_cloud, 0, 0, 255);
						//						viewer.addPointCloud (model_in_scene_cloud, rotated_model_color_handler, "rotated_model");
						//Visualize correspondences by lines between model and scene

						while (!viewer.wasStopped ())
						{
							viewer.spinOnce ();
						}
					}
				}
			}

			debug_file << "Search of model: " << i << std::endl;
			debug_file << "Number of model points after normalization = " << _models.at(i)->size() << std::endl;
			debug_file << "Number of model points keypoints = " << _model_keypoints.at(i)->size () << std::endl;
			debug_file << "Number of model points descriptors = " << _model_descriptors.at(i)->points.size() << std::endl;
			debug_file << "Model scene correspondences: " << _model_scene_corrs->size() << std::endl;
			debug_file << "Clustered_corrs: " << _clustered_corrs.size() << std::endl;
			debug_file<< "Model instances found: " << _rototranslations.size () << std::endl;


		}

		_model_scene_corrs->erase(_model_scene_corrs->begin(), _model_scene_corrs->end());
	}
	catch (...)
	{
		std::cerr << "Exception occurred!" << std::endl;
		output_file << 0 << std::endl;
		output_file.close();
	}


	//TODO: Add the verification step
	//	verify(scene);


	_final_models = _rotated_models;


	if(_final_models.size() == 0)
	{
		output_file << 0 << std::endl;
		output_file.close();
	}
	output_file.close();

}
