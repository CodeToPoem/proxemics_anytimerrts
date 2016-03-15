/*******************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Charly Huang
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

#ifndef _ANYTIMERRTSPLANNER_H_
#define _ANYTIMERRTSPLANNER_H_

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <proxemics_anytimerrts/pose.h>
#include <proxemics_anytimerrts/velocity.h>
#include <proxemics_anytimerrts/discrete_state.h>
#include <proxemics_anytimerrts/state.h>

#include <proxemics_anytimerrts/heuristics.h>
#include <proxemics_anytimerrts/state_discretizer.h>
#include <proxemics_anytimerrts/trajectory_rollout.h>
#include <proxemics_anytimerrts/hasher.h>
#include <proxemics_anytimerrts/Path.h>
#include <proxemics_anytimerrts/cost_manager.h>
#include <proxemics_anytimerrts/dynamic_costmap.h>

// sampling-based planner stuff
#include <proxemics_anytimerrts/srl_trajectory.h>
// Grid planner
#include <proxemics_anytimerrts/thetastar_leading_rrt.h>

#include <tf/transform_listener.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>

#include <proxemics_anytimerrts/world_model.h>
#include <proxemics_anytimerrts/costmap_model.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

/// SMP HEADERS
// #include <smp/components/samplers/uniform.hpp>
//#include <smp/components/samplers/uniformc.hpp>
// #include <smp/components/samplers/gauss.hpp>
#include <smp/components/samplers/theta_star_in_regions.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/pos.hpp>

#include <smp/components/collision_checkers/collisionCostMap.hpp>

#include <smp/components/multipurpose/minimum_time_reachability_thetastar.hpp>

#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>

#include <smp/utils/branch_and_bound_euclidean.hpp>

#include <smp/components/samplers/trajectory_bias.hpp>

#define HUMANMAX 20

#define SQRT2 1.414213562
//#define DEBUG

// PARAMETERS TO THE PROBLEM ***********************************************************************************
// *
//#define NUM_DIMENSIONS 15    // Change the number of dimensions from here. Scale it up to 30 - 50 
                             //   dimensions to see the convergence of RRT* towards an optimal solution
                             //   in very high dimensional configuration spaces without employing any heuristics. 

#define EXTENSION_LENGTH  RHO   // Maximum length of an extension. This parameter should ideally 
                                 //   be equal longest straight line from the initial state to
                                 //   anywhere in the state space. In other words, this parameter 
                                 //   should be "sqrt(d) L", where d is the dimensionality of space 
                                 //   and L is the side length of a box containing the obstacle free space.
                                 //   __NOTE__: Smaller values of this parameter will lead to a good feasible 
                                 //   solution very quickly, whilenot affecting the asymptotic optimality
                                 //   property of the RRT* algorithm.
// *
// *************************************************************************************************************


namespace proxemics_anytimerrts {
/**
 * @brief This class implements an Anytime Dynamic RRT* planner to plan on a layered dynamic costmap.
 * ==================================================================================
 * Rrt_planner
 * This node solves a motion planning problem given the current robot_state and the
 * goal. It is based on the Sampling-based Motion Planning Library from MIT, where you
 * can use three RRT variants.
 * ==================================================================================
 */ 
typedef state_pos state_t;
typedef input_pos input_t;

typedef minimum_time_reachability_vertex_data vertex_data_t;
typedef minimum_time_reachability_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
    typedef state_t state;
    typedef input_t input;
    typedef vertex_data_t vertex_data;
    typedef edge_data_t edge_data;
} typeparams;

//// Define the trajectory type
typedef trajectory<typeparams> trajectory_t;





// Define all planner component types
//typedef sampler_opra<typeparams,3> sampler_t;
// typedef sampler_uniform<typeparams,3> sampler_t;
typedef theta_star_in_regions<typeparams,3> sampler_t;
//typedef sampler_uniformc<typeparams,3> sampler_t;
//typedef sampler_gauss<typeparams,3> sampler_t;




typedef distance_evaluator_kdtree<typeparams,3> distance_evaluator_t;

typedef extender_pos<typeparams,3> extender_t;



//typedef collision_checker_standard<typeparams,2> collision_checker_t;
// typedef collision_checker_circle<typeparams,2> collision_checker_t;
typedef collision_checker_costmap<typeparams,2> collision_checker_t;

// typedef collision_checker_rectangle <typeparams,2> collision_checker_t;

typedef minimum_time_reachability<typeparams,2> min_time_reachability_t;
// typedef smoothness_cost<typeparams,2>  smoothness_cost_t;

// Define all algorithm types
typedef rrtstar<typeparams>  rrtstar_t;

// Types to save the tree information
typedef vertex<typeparams> vertex_t;
typedef edge<typeparams> edge_t;


/// Define Branch and Bound Type
typedef branch_and_bound_euclidean<typeparams,2> branch_and_bound_t;




// is this for trajectory updating?
void *pointer_to_sampler ;

int wrapper_to_sampler_update_trajectory (trajectory_t *trajectory_in) {
	((sampler_t*) (pointer_to_sampler)) -> update_trajectory (trajectory_in) ;
	
	return 1 ;
}

 
class AnytimeDynamicRRTs{

public:	
	/**
	 * @brief constructor
	 * @param name planner name
	 * @param costmap 2D static cost map as provided by the navigation stack
	 */ 
	 AnytimeDynamicRRTs(std::string name, costmap_2d::Costmap2DROS *costmap) ;
	 
	 /**
	  * @brief destructor
	  */ 
	  ~AnytimeDynamicRRTs() ;

  /**
   * @brief generate a path from a start pose to a goal pose.
   *
   * This method is called from the planner core once a goal is send to move_base.
   * If the certain navigation goal is set for the first time, a plan has to be
   * planned from the current robot position and zero velocity. During the frequent
   * replanning, the plan has to be generated from a waypoint in the future, since
   * the planning takes a certain amount of time. The 'replanning' parameter
   * specifies if the planning request is meant to generate an initial, new plan
   * or if it is a replanning request. If it is a replanning request, the start
   * waypoint for planning is calculated according to the specified planning timeout
   * (ros parameter move_base/TBPlanner/planning_timeout)
   *
   * @param start start pose of the robot
   * @param goal desired goal pose
   * @param replanning whether an initial, new plan is requested or replanning
   * @param[out] path the planned navigation path
   * @return whether a plan to the goal position could be found
   */
  bool getPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal,
               bool replanning, std::vector<geometry_msgs::PoseStamped>& path);
	  
private:

  /**
   * @brief reset the planning procedure: delete pointers and empty containers
   *        needed for one planning procedure.
   */
  void reset();

  /**
   * @brief find the waypoint from which to start planning if a plan to the
   *        desired goal was already planned right before this request.
   *
   * Since planning takes some time, the start position for planning should
   * not be the current position of the robot, but the position he will be in
   * once the plan is released. If the robot is already following a plan to a
   * given goal and a replanning request comes in, this method obtaines the
   * starting pose for planning according to the previously planned path and the
   * time at which the new plan is released.
   *
   * @param plan_release_time the time at which the new plan is released
   * @param[out] replanning_start_pose the robot pose from which to start planning
   * @param[out] replanning_start_vel the robot velocity from which to start planning
   * @return whether a valid waypoint for the plan_release_time could be found
   */
  bool findReplanningWaypoint(ros::Time plan_release_time,
                              geometry_msgs::PoseStamped& replanning_start_pose,
                              geometry_msgs::Twist& replanning_start_vel);
    
  //planner preferences
  bool allow_unknown_; ///< whether the planner is allowed to expand into unknown map regions
  double xy_goal_tolerance_; ///< the Euclidean goal tolerance distance
  double yaw_goal_tolerance_; ///< the rotational goal tolerance in rad
///  double time_resolution_; ///< time resolution of the discretized configuration space
///  double map_index_check_time_inkr_; ///< time increment to slice trajectories for collision checking
  int max_timesteps_; ///< maximum number of time steps of the discretized configuration space
  double planning_timeout_; ///< timeout for the planning after which the best solution found so far is returned
  bool passive_navigation_; ///< flag for planning dynamically for the max number of time steps, not only until the goal is found
///  bool publish_expanded_; ///< flag for publishing the expanded search tree for visualization

  //flags for easing path finding - violate optimality but necessary for quick
  //calculation
  bool easy_deceleration_; ///< flag for simplified planning of deceleration at the goal
  bool easy_turn_at_start_; ///< flag for simplified planning of turn in place at the start
  bool easy_turn_at_goal_; ///< flag for simplified planning of turn in place at the goal

  //properties of current path finding run
  Pose start_pose_; ///< start pose for planning
  Pose goal_pose_; ///< desired goal pose
  ros::Time path_time_; ///< time at which the path is released (after planning_timeout)
  ///lattice_planner::Path current_plan_; ///< last planned path
  proxemics_anytimerrts::Path current_plan_ ;  ///< last planned path
  nav_msgs::Path expanded_paths_; ///< paths expanded during search
  bool planning_timed_out_; ///< whether the planning is timed out

  //auxiliary classes
  DynamicCostmap* dynamic_costmap_; ///< dynamic costmap instance
  StateDiscretizer* discretizer_; ///< state discretizer instance
  TrajectoryRollout* trajectory_rollout_; ///< trajectory rollout instance
  Heuristics* heuristic_calc_; ///< heuristics calculator
  CostManager* cost_calc_; ///< cost calculater

  //containers for expanded states
  std::vector<State*> queue_; ///<priority queue for expanded states
  std::vector< std::unordered_map<DiscreteState, State*,Hasher>* > expanded_states_; ///< hash map for expanded states

  //ros stuff
  ros::NodeHandle nh_; ///< ROS node handle
  ros::Publisher expanded_paths_pub_; ///< publisher for the search tree
  ros::Publisher vel_path_pub_; ///< publisher for the planned path with planned velocities

  //debug things
  ros::Publisher current_node_pub_; ///< debug publisher for the expanding parent state
  ros::Publisher new_node_pub_; ///< debug publisher for the expanded child state
  ros::Publisher all_expanded_pub_; ///< debug publisher for all already expanded states
  bool debug_no_interrupt_; ///< flag to stop debugging step by step
	 
} ;
 
} // namespace proxemics_anytimerrts
#endif
