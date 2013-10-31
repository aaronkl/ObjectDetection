/*
 * main.cpp
 *
 *  Created on: Jul 16, 2013
 *      Author: Aaron Klein
 */


#include "object_recognition.h"

void loadModelsFromFile(std::string file, std::vector<pcl::PointCloud<PointT>::Ptr > &models)
{
	std::ifstream input(file.c_str());
	std::string filename;
	while(input.good())
	{
		std::getline (input, filename);
		std::cout << "Load model: " << filename << std::endl;
		if (filename.empty () || filename.at (0) == '#') // Skip blank lines or comments
			continue;

		pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile(filename, *tmp);
		std::cout << tmp->size() << std::endl;
		models.push_back(tmp);
	}
	input.close ();
}

void showHelp (char *filename)
{
	std::cout << std::endl;
	std::cout << "Usage: " << filename << " data_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -h:                     Show this help." << std::endl;
	std::cout << "     -v:                     Show the visualization." << std::endl;
	std::cout << "     -c:                     Show used correspondences." << std::endl;
	std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
	std::cout << "                             each radius given by that value." << std::endl;
	std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
	std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
	std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
	std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
	std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
	std::cout << "      val:        Clustering threshold (default 5)" << std::endl;
	std::cout << "     --ne_radius val:        Normal estimation radius (default 0.02)" << std::endl;
	std::cout << "     --leaf_size val:        Leaf size of the voxel grid (default 0.001)" << std::endl << std::endl;

}


int main (int argc, char *argv[])
{
	ObjectRecognition obj_rec;

	float leaf_size(0.001f);
	float ne_radius(0.05f);
	float cg_size(0.002);
	float cg_thresh(5);
	float descr_rad(0.008);
	float model_ss(0.0015);
	float rf_rad(0.07);
	float scene_ss(0.0015);


	if (pcl::console::find_switch (argc, argv, "-h"))
	{
		showHelp (argv[0]);
		exit (0);
	}
	if (pcl::console::find_switch (argc, argv, "-v"))
	{
		obj_rec.setShowVisualization(true);
	}
	if (pcl::console::find_switch (argc, argv, "-c"))
	{
		obj_rec.setShowCorrespondences(true);
	}
	if (pcl::console::find_switch (argc, argv, "-r"))
	{
		obj_rec.setUseCloudResolution(true);
	}

	pcl::console::parse_argument (argc, argv, "--model_ss", model_ss);
	obj_rec.setModelSs(model_ss);

	pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss);
	obj_rec.setSceneSs(scene_ss);

	pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad);
	obj_rec.setRfRad(rf_rad);

	pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad);
	obj_rec.setDescrRad(descr_rad);

	pcl::console::parse_argument (argc, argv, "--cg_size", cg_size);
	obj_rec.setCgSize(cg_size);

	pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh);
	obj_rec.setCgThresh(cg_thresh);

	pcl::console::parse_argument (argc, argv, "--leaf_size", leaf_size);
	obj_rec.setLeafSize(leaf_size);

	pcl::console::parse_argument (argc, argv, "--ne_radius", ne_radius);
	obj_rec.setNeRadius(ne_radius);

//	std::vector<int> filenames;
//	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
//	if (filenames.size () != 1)
//	{
//		std::cout << "Filenames missing.\n";
//		showHelp (argv[0]);
//		exit (-1);
//	}
//
//	std::string model_filename;
//	model_filename = argv[filenames[0]];
//	std::string scene_filename;
//	scene_filename = argv[filenames[1]];

	std::string model_filename;
	model_filename = argv[1];
	std::string scene_filename;
	scene_filename = argv[2];


	pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT> ());
	if (pcl::io::loadPCDFile (scene_filename, *scene) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		showHelp (argv[0]);
		return (-1);
	}
//	pcl::PointCloud<PointT>::Ptr model (new pcl::PointCloud<PointT> ());
//	if (pcl::io::loadPCDFile (model_filename, *model) < 0)
//	{
//		std::cout << "Error loading model cloud." << std::endl;
//		showHelp (argv[0]);
//		return (-1);
//	}

	// Debug information
	//std::fstream output("output.txt",  ios::out|ios::app);
	//output << "scene: " << scene_filename << std::endl;
	//output << "model: " << model_filename << std::endl;

	std::vector<pcl::PointCloud<PointT>::Ptr > models;
	loadModelsFromFile(model_filename, models);
	std::cout << models.size() << std::endl;
	obj_rec.loadModel(models);
	obj_rec.recognize(scene);
}


