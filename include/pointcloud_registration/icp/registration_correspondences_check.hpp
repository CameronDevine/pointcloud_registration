/*
 * Copyright (c) 2010, Hozefa Indorewala <indorewala@ias.in.tum.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Based on original code written by Radu Rusu
 * Modified by Hozefa Indorewala
 */


template <typename PointSource, typename PointTarget> inline void
  pcl::RegistrationCorrespondencesCheck<PointSource, PointTarget>::setInputTarget (const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    ROS_ERROR ("[pcl::%s::setInputTarget] Invalid or empty point cloud dataset given!", getClassName ().c_str ());
    return;
  }
  target_ = cloud;
  tree_->setInputCloud (target_);
}


template <typename PointSource, typename PointTarget> inline double
  pcl::RegistrationCorrespondencesCheck<PointSource, PointTarget>::getFitnessScore (double max_range)
{
  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  transformPointCloud (*input_, input_transformed, final_transformation_);

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    Eigen::Vector4f p1 = Eigen::Vector4f (input_transformed.points[i].x,
                                          input_transformed.points[i].y,
                                          input_transformed.points[i].z, 0);
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range)
      continue;

    Eigen::Vector4f p2 = Eigen::Vector4f (target_->points[nn_indices[0]].x,
                                          target_->points[nn_indices[0]].y,
                                          target_->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm ());
    nr++;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}


template <typename PointSource, typename PointTarget> inline void
  pcl::RegistrationCorrespondencesCheck<PointSource, PointTarget>::align (PointCloudSource &output)
{
  if (!initCompute ()) return;

  if (!target_)
  {
    ROS_WARN ("[pcl::%s::compute] No input target dataset was given!", getClassName ().c_str ());
    return;
  }

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Copy the header
  output.header   = input_->header;
  // Check if the output is dense or not
  if (indices_->size () != input_->points.size ())
  {
    output.width    = indices_->size ();
    output.height   = 1;
    output.is_dense = false;
  }
  else
  {
    output.width    = input_->width;
    output.height   = input_->height;
    output.is_dense = input_->is_dense;
  }

  // Copy the point data to output
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i] = input_->points[(*indices_)[i]];

  // Perform the actual transformation computation
  converged_ = false;
  final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix4f::Identity ();
  ROS_INFO("[RegistrationCorrespondencesCheck:] computeTransformation");
  computeTransformation (output);

  deinitCompute ();
}
