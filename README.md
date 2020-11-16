# Pointcloud Registration

This package implements registration of point clouds. It also contains an additional feature called Spin Image Local. Based on original code written by Radu Rusu (rusu@willowgarage.com)

> This code was listed as part of a [ROS package](http://wiki.ros.org/pointcloud_registration); however the source code was unavailable from its original repository: [code.in.tum.de/git/mapping.git](http://code.in.tum.de/git/mapping.git). Thankfully, the source was available as part of the [Doxygen documentation](http://docs.ros.org/en/groovy/api/pointcloud_registration/html/index.html). Some changes were necessary to get the code to compile for ROS melodic.

Original Authors:

1. Hozefa Indorewala: h.indorewala@jacobs-university.de
2. Dejan Pangercic: dejan.pangercic@cs.tum.edu
3. Zoltan-Csaba Marton: marton@cs.tum.edu
4. Nico Blodow: blodow@cs.tum.edu

## Overview

The `pointcloud_registration` package primarily subscribes to a `sensor_msgs::PointCloud` topic to receive point clouds and then registers them in a global frame, one cloud at a time. The `pointcloud_registration` package implements the ICP algorithm but with a few modifications as explained below.

## ICP

Traditional ICP looks for correspondences between the source and target point clouds using the nearest neighbour search, estimates transformation and applies this to the correspondences and iterates this procedure until a satisfactory transformation is achieved.

In our case, we modified the ICP a bit to improve our correspondences which eventually leads to better transformation.

Firstly, the overlapping regions between the source and target point clouds are extracted by examining the nearest neighbors of each point in the source with the points in the target. This is done using a `KdTree` to improve efficiency. The overlapping regions were then used as the source and target point clouds for the ICP. This was done in order to improve the correspondence search in the ICP algorithm.

Using a radius of 10cm (in our case) for the fixed radius search, the first set of correspondences were selected out of these source and target point clouds. The correspondences were then filtered based on the following two criterion:

## ROS Parameters

`~<name>/publish_merged_pointcloud_topic` (string, default: "/merged_pointcloud"):
Topic to publish the final registered point cloud

`~<name>/subscribe_pointcloud_topic` (string, default: "/shoulder_cloud"):
Topic to subscribe to receive point clouds

`~<name>/max_number_of_iterations_icp` (int, default: 100):
Maximum number of iterations for the icp algorithm

`~<name>/max_nn_icp` (int, default: 10):
Maximum number of nearest neigbors to look for in the radius search for the icp algorithm

`~<name>/max_nn_overlap` (int, default: 20):
Maximum number of nearest neigbors to look for in the radius search to get the overlapping region of two point clouds

`~<name>/radius_icp` (double, default: 0.1):
Radius for the radius search for the icp algorithm

`~<name>/radius_overlap` (double, default: 0.1):
Radius for the radius search to get the overlapping region of two point clouds

`~<name>/filter_outliers` (bool, default: true):
Set true if wish to filter outliers using the Statistical Outlier Removal in pcl

`~<name>/downsample_pointcloud_before` (bool, default: false):
Set true if wish to downsample the point cloud before registration (Speeds up processing significantly)

`~<name>/downsample_pointcloud_after` (bool, default: true):
Set true if wish to downsample the point cloud after registration

`~<name>/downsample_leafsize` (double, default: 0.035):
Downsample leaf size

`~<name>/epsilon_z` (double, default: 0.01):
Maximum z-coordinate difference between two correspondences which is allowed

`~<name>/curvature_check` (bool, default: true):
Set true if wish to filter correspondences based on curvature difference

`~<name>/epsilon_curvature` (double, default: 0.01):
Maximum curvature value difference between two correspondences which is allowed

`~<name>/epsilon_transformation` (double, default: 1e-8):
Transformation epsilon value to be reached for convergence for the ICP algorithm
