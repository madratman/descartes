/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTESIAN_INTERPOLATOR_H_
#define CARTESIAN_INTERPOLATOR_H_

#include <descartes_trajectory/cart_trajectory_pt.h>
#include <tf/transform_datatypes.h>

namespace descartes_trajectory
{

class TrajectorySegment
{
public:

  virtual ~TrajectorySegment()
  {

  }

  virtual bool interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points) = 0;

protected:

  TrajectorySegment(){}

};

class BlendSegment: public TrajectorySegment
{
public:

  BlendSegment(const descartes_trajectory::CartTrajectoryPt& start,
               const descartes_trajectory::CartTrajectoryPt& end,
               double duration,
               double time_step);
  virtual ~BlendSegment();
  virtual bool interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points);

protected:



};

class LinearSegment: public TrajectorySegment
{
public:

  LinearSegment(const tf::Transform& start_pose,
                const tf::Transform& end_pose,
                double total_time,
                double interpolation_step);

  virtual ~LinearSegment();
  virtual bool interpolate(std::vector<descartes_core::TrajectoryPtPtr>& segment_points);

protected:

  tf::Transform start_pose_;
  tf::Transform end_pose_;
  double total_time_;
  double interpolation_step_;

};

class CartesianInterpolator
{
public:
  CartesianInterpolator();
  virtual ~CartesianInterpolator();

  bool initialize(descartes_core::RobotModelConstPtr robot_model, double tool_speed,double interpolation_interval,
                  double zone_radius);
  bool interpolate(const std::vector<descartes_trajectory::CartTrajectoryPt>& coarse_traj,
                   std::vector<descartes_core::TrajectoryPtPtr>& interpolated_traj);
protected:

  bool computeFullSegmentTimes(const std::vector<descartes_trajectory::CartTrajectoryPt>& coarse_traj,
                               std::vector<double>& ft);
  bool computeBlendSegmentTimes(const std::vector<descartes_trajectory::CartTrajectoryPt>& coarse_traj,
                                std::vector<double>& bt);

protected:

  descartes_core::RobotModelConstPtr robot_model_;
  double tool_speed_; // magnitude of the tool's linear displacement
  double interpolation_interval_ ; // time step between interpolated points
  double zone_radius_; // radius around each point where parabolic blending starts
};

} /* namespace descartes_trajectory */
#endif /* CARTESIAN_INTERPOLATOR_H_ */