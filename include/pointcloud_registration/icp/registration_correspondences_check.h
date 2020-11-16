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
 * Based on code written by Radu Rusu
 * Modified by Hozefa Indorewala
 */
#ifndef _REGISTRATION_CORRESPONDENCES_CHECK_H_
#define _REGISTRATION_CORRESPONDENCES_CHECK_H_

#include <boost/function.hpp>
#include <boost/bind.hpp>

// PCL includes
#include <pcl/pcl_base.h>

#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

#include "pcl/registration/transforms.h"

namespace pcl
{

  template <typename PointSource, typename PointTarget>
  class RegistrationCorrespondencesCheck: public PCLBase<PointSource>
  {
    using PCLBase<PointSource>::initCompute;
    using PCLBase<PointSource>::deinitCompute;

    public:
      using PCLBase<PointSource>::input_;
      using PCLBase<PointSource>::indices_;

      typedef typename pcl::KdTree<PointTarget> KdTree;
      typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;


      RegistrationCorrespondencesCheck () : target_ (),
                        final_transformation_ (Eigen::Matrix4f::Identity ()),
                        transformation_ (Eigen::Matrix4f::Identity ()),
                        previous_transformation_ (Eigen::Matrix4f::Identity ()),
                        transformation_epsilon_ (0.0), corr_dist_threshold_ (std::numeric_limits<double>::max ()),
                        converged_ (false), min_number_correspondences_ (3), k_ (1)
      {
        tree_ = boost::make_shared<pcl::KdTreeFLANN<PointTarget> > ();     // ANN tree for nearest neighbor search
      }


      virtual ~RegistrationCorrespondencesCheck () {};


      virtual inline void setInputTarget (const PointCloudTargetConstPtr &cloud);


      inline PointCloudTargetConstPtr const getInputTarget () { return (target_ ); }


      inline Eigen::Matrix4f getFinalTransformation () { return (final_transformation_); }


      inline Eigen::Matrix4f getLastIncrementalTransformation () { return (transformation_); }


      inline void setMaximumIterations (int nr_iterations) { max_iterations_ = nr_iterations; }


      inline int getMaximumIterations () { return (max_iterations_); }


      inline void setMaxCorrespondenceDistance (double distance_threshold) { corr_dist_threshold_ = distance_threshold; }


      inline double getMaxCorrespondenceDistance () { return (corr_dist_threshold_); }


      inline void setTransformationEpsilon (double epsilon) { transformation_epsilon_ = epsilon; }


      inline double getTransformationEpsilon () { return (transformation_epsilon_); }



      inline void
        setPointRepresentation (const typename KdTree::PointRepresentationConstPtr &point_representation)
      {
        tree_->setPointRepresentation (point_representation);
      }


      inline double getFitnessScore (double max_range = std::numeric_limits<double>::max ());


      inline void align (PointCloudSource &output);

    protected:
      std::string reg_name_;

      KdTreePtr tree_;

      int nr_iterations_;

      int max_iterations_;

      PointCloudTargetConstPtr target_;

      Eigen::Matrix4f final_transformation_;

      Eigen::Matrix4f transformation_;

      Eigen::Matrix4f previous_transformation_;

      double transformation_epsilon_;

      double corr_dist_threshold_;

      bool converged_;

      int min_number_correspondences_;


      inline bool
        searchForNeighbors (const PointCloudSource &cloud, int index, double radius, int max_nn, std::vector<int> &indices, std::vector<float> &distances)
      {
        return (tree_->radiusSearch (cloud, index, radius, indices, distances, max_nn));
      }


      inline const std::string& getClassName () const { return (reg_name_); }

    private:

      virtual void computeTransformation (PointCloudSource &output) = 0;

      int k_;
  };
}

#include "pointcloud_registration/icp/registration_correspondences_check.hpp"

#endif  //#ifndef _REGISTRATION_CORRESPONDENCES_CHECK_H_
