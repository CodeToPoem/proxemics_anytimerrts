/*******************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015 Charly Huang
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Charly Huang
 ******************************************************************************/

#include <proxemics_anytimerrts/anytimerrts_planner.h>

namespace proxemics_anytimerrts {
	
typedef std::unordered_map < DiscreteState, State*, Hasher > state_map;
typedef std::pair < DiscreteState, State* > state_pair;

int Hasher::num_gridcells_ = 0;
int Hasher::num_orientations_ = 0;
int Hasher::num_vels_w_ = 0;
int Hasher::num_vels_x_ = 0;

AnytimeDynamicRRTs::AnytimeDynamicRRTs(std::string name, costmap_2d::Costmap2DROS *costmap) {
	
  //init a private ros node handle
  ros::NodeHandle private_nh("~/" + name);

  //init ros publishers
  vel_path_pub_ =
      private_nh.advertise<proxemics_anytimerrts::Path>("plan", 1);
  expanded_paths_pub_ =
      private_nh.advertise<nav_msgs::Path>("expanded_paths", 1);

  //init debug publishers
  current_node_pub_ =
      private_nh.advertise<geometry_msgs::PoseStamped>("current_node", 1);
  new_node_pub_ =
      private_nh.advertise<geometry_msgs::PoseStamped>("new_node", 1);
  all_expanded_pub_ =
      private_nh.advertise<geometry_msgs::PoseArray>("all_expanded", 1);
  debug_no_interrupt_ = false;

  //get planner behavior params from ros param server

  std::string dynamic_layers_plugin_name;

  //costs
  CostManager::CostFactors cost_factors;
  private_nh.param("lethal_cost", cost_factors.lethal_cost,
                   (int)costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  private_nh.param("time_cost_factor", cost_factors.time_cost, 200.0);
  private_nh.param("step_cost_factor", cost_factors.step_cost, 20.0);
  private_nh.param("rotation_cost_factor", cost_factors.rotation_cost, 5.0);
  private_nh.param("environment_cost_factor", cost_factors.environment_cost, 1.0);
  private_nh.param("dynamic_layers_plugin", dynamic_layers_plugin_name, std::string("undefined"));

  //planner preferences
  double collision_check_time_res;
  private_nh.param("allow_unknown", allow_unknown_, false);
  private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.15);
  private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.2);
  private_nh.param("time_resolution", time_resolution_, 0.5);
  private_nh.param("collision_check_time_resolution", collision_check_time_res, 0.1);
  private_nh.param("time_steps_lookahead", max_timesteps_, 15);
  private_nh.param("planning_timeout", planning_timeout_, 0.48);
  private_nh.param("passive_navigation", passive_navigation_, false);
  private_nh.param("publish_expanded", publish_expanded_, false);

  //motion constraints
  MotionConstraints motion_constraints;
  private_nh.param("min_vel_x", motion_constraints.min_vel_x, 0.0);
  private_nh.param("max_vel_x", motion_constraints.max_vel_x, 0.4);
  private_nh.param("acceleration_x", motion_constraints.acc_x, 0.8);
  private_nh.param("min_vel_phi", motion_constraints.min_vel_phi, -0.8);
  private_nh.param("max_vel_phi", motion_constraints.max_vel_phi, 0.8);
  private_nh.param("acceleration_phi", motion_constraints.acc_phi, 1.6);

  //flags for easier path finding
  private_nh.param("easy_deceleration", easy_deceleration_, false);
  private_nh.param("easy_turn_at_start", easy_turn_at_start_, false);
  private_nh.param("easy_turn_at_goal", easy_turn_at_goal_, false);

  //create a hash map for every time step
  for(int i=0; i<max_timesteps_; i++)
  {
    state_map* time_map = new state_map();
    time_map->rehash(10000);
    time_map->reserve(10000);
    expanded_states_.push_back(time_map);
  }

  //dynamic costmap for costs associated with static and dynamic obstacles
  dynamic_costmap_ = new DynamicCostmap(costmap, max_timesteps_,
                                        ros::Duration(time_resolution_),
                                        dynamic_layers_plugin_name);

  double resolution = dynamic_costmap_->getStaticROSCostmap()->getCostmap()->getResolution();
  map_index_check_time_inkr_ = 0.5 * resolution / motion_constraints.max_vel_x;

  //cost manager for cost calculation
  cost_calc_ = new CostManager(dynamic_costmap_, motion_constraints, cost_factors);

  //state discretizer for discretizing the configuration space
  discretizer_ = new StateDiscretizer(dynamic_costmap_, motion_constraints);

  //hasher for generating unique hashes for the hash map
  int num_gridcells, num_orientations, num_vels_x, num_vels_w;
  discretizer_->getLimits(num_gridcells, num_orientations, num_vels_x, num_vels_w);
  Hasher::setLimits(num_gridcells, num_orientations, num_vels_x, num_vels_w);

  //trajectory rollout for robot motion calculation
  trajectory_rollout_ = new TrajectoryRollout(motion_constraints);

  //heuristics calculator for estimating the remaining cost to reach the goal
  heuristic_calc_ = new Heuristics(dynamic_costmap_, cost_factors, motion_constraints);
  heuristic_calc_->estimateAdditional(easy_deceleration_,easy_turn_at_goal_,
                                      xy_goal_tolerance_);
  heuristic_calc_->setHasUnknown(allow_unknown_);
  heuristic_calc_->initPublisher("lattice_planner/heuristics", 100);
	
}

AnytimeDynamicRRTs::~AnytimeDynamicRRTs() {
	// TODO
}

// clean up everything
void AnytimeDynamicRRTs::reset() {
}

bool AnytimeDynamicRRTs::findReplanningWaypoint(ros::Time plan_release_time,
                              geometry_msgs::PoseStamped& replanning_start_pose,
                              geometry_msgs::Twist& replanning_start_vel){
}

void AnytimeDynamicRRTs::expandCircleAtStart(State* start) {
}

bool AnytimeDynamicRRTs::addState(State *state) {
}

bool AnytimeDynamicRRTs::getPath(geometry_msgs::PoseStamped start,
                           geometry_msgs::PoseStamped goal, bool replanning,
                           std::vector<geometry_msgs::PoseStamped>& path) {
}



}  // namespace proxemics_anytimerrts
