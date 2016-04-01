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

   /**
   * @brief plan a kinodynamic path using RRT
   * @param, traj Trajectory generated
   * @param, type of motion planner to use
   * @param, start pose
   * @return true, if the plan was found
   */
   int plan(Trajectory *traj, int type, geometry_msgs::PoseStamped& start);  

/// *****************************************************************************
/// RRT Planner's parameters
/// *****************************************************************************
    double cellwidth_;  ///<  @brief Cell width

    double cellheight_; ///<  @brief Cell height

    base_local_planner::CostmapModel* world_model_; ///<  @brief World Model associated to the costmap

    std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief FootPrint list of points of the robot

    costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use

    costmap_2d::Costmap2D* costmap_; ///< @brief The ROS wrapper for the costmap the controller will use

    std::string node_name_;  ///<  @brief Node name

    Grid_planner *grid_planner_; ///<  @brief Grid Planner that searches a discrete path

    double goal_x_; ///<  @brief x coordinate of the goal pose

    double goal_y_; ///<  @brief y coordinate of the goal pose

    double goal_theta_; ///<  @brief yaw angle of the goal pose

    double toll_goal_; ///<  @brief toll the goal region


    double rx,ry,rz; ///<  @brief robot coordinates

    int Nit_; ///<  @brief  Number of max iterations


    Trajectory *trajectory_; ///<  @brief Keep the path locally

    /// Size of the scene
    double xscene_; ///<  @brief Width of the scene in meters

    double yscene_; ///<  @brief Height of the scene in meters

    int n_hum_; ///<  @brief number of the humans in the scene (to use when the social cost need to be added)

    double hposes_[HUMANMAX][3]; ///<  @brief  humans in the scene


    std::vector<Tpoint> tree_; ///<  @brief tree and expansions occured

    std::vector<size_t> nexpasions_; ///<  @brief expansions done in the tree

    float numVertices_; ///<  @brief Number of iterations

    float timeIter_; ///<  @brief Time per iteration

    float timeSolution_; ///<  @brief Time to find a solution iteration

    double nrew_;  ///<  @brief Number of rewiring per iteration

    double xsupp_; ///<  @brief Width of the support

    double ysupp_;///<  @brief Width of the support

    std::vector<Tobstacle> obstacle_positions; ///<  @brief Obstacles


    int TIMECOUNTER ; ///<  @brief Flag to terminate the iterations after a certain amount of seconds

    int FIRSTSOLUTION ;  ///<  @brief Flag to stop the planner once you found the first solution

    int DEB_RRT; ///<  @brief If you want to show all the debug info

    int BOX; ///<  @brief  One if you want to select the neighbours in a Box which respects the diff-drive kinematics

    double RADIUS; ///<  @brief Radius where to select the nearest vertex in plane RRT

    bool goal_init_; ///<  @brief Flag that indicates if the goal is initialized

    double curr_cost_; ///<  @brief Current cost

    int no_fails;  ///<  @brief Number of fails

    double MAXTIME; ///<  @brief Max time of iterations for planning

    int NUMBER_UPDATE_TRAJ; ///<  @brief Number of times the planner improves the trajectory

    int NOANIM; ///<  @brief Activate Publishing markers

    int DISPLAY_METRICS; ///<  @brief Displaying metrics currently not used

    int first_sol; ///<  @brief First solution flag, not used currently

    int SELECT_FUNC; ///<  @brief To select different cost function in min_time_reachability, currently not used

    int WHATTOSHOW; ///<  @brief Selects what kind of info to display in Rviz

    int BRANCHBOUND; ///<  @brief Selects Branching and Bounding of the tree

    double THRS_BRANCHBOUND;  ///<  @brief Thrs for Branching and Bounding of the tree

    int BRANCHBOUND_RATIO; ///<  @brief Ratio for Branching and Bounding of the tree

    int GOAL_BIASING; ///<  @brief Flag to activate Goal Biasing

    double GOAL_BIASING_THS; ///<  @brief Goal Biasing THRS

    int COST_DISPLAY; ///<  @brief To display cost not used currently

    double RHO; ///<  @brief Stopping distance of the POSQ extend function

    double DT; ///<  @brief Integration time stamp for the POSQ extender

    double L_AXIS; ///<  @brief Axis length of the robot

    int READ_AGENTS; ///<  @brief If consider agents in the planner

    double SCALING_IRL; ///<  @brief To scale IRL currently not used

    geometry_msgs::Pose start_pose_; ///<  @brief Start pose

    geometry_msgs::Pose goal_pose_; ///<  @brief Goal pose

    //
    int PARALLEL; ///<  @brief Parallelize collision checker (only rectangular colllision checker)

    int K; ///<  @brief K nearest obstacles to consider in Collision checkin  (only rectangular colllision checker)

    double RAD_OBST; ///<  @brief Radius of the nearest neighbors to consider in Collision checkin  (only rectangular colllision checker)

    double robot_width_; ///<  @brief robot width

    double robot_length_; ///<  @brief robot length

    double collision_boundary; ///<  @brief enlarge the size of the robot

    std::string cost_file_name; ///<  @brief file where to save te costs

    //
    ros::Time begin; ///<  @brief fMap loading only during the first Nsecs

    double initialization_time; ///<  @brief initialization time for map

    double map_loading_time; ///<  @brief map loading time

    double max_map_loading_time; ///<  @brief max map loading time

    int cnt_map_readings; ///<  @brief counter of map readings

    double agents_size_; ///<  @brief Number of Agents

    double xmin_; ///<  @brief xmin for the grid

    double xmax_; ///<  @brief xmax for the grid

    double ymin_; ///<  @brief ymin for the grid

    double ymax_; ///<  @brief ymax for the grid

    int typepar; ///<  @brief type of the planner to use

    double inscribed_radius_; ///<  @brief inscribed radius for the robot shape

    double circumscribed_radius_ ; ///<  @brief circumscribe radius for the robot shape

    ///
    trajectory_t *support_; ///<  @brief Path to be used to generate the sampler support

    trajectory_t *support_bias_; ///<  @brief Path to be used to generate the sampler support

    int TYPE_SAMPLING;  ///<  @brief choose type of sampling

    int EXT_SIGMAS; ///<  @brief In case of Gaussian sampling load external sigmas

    int NUMBER_RUNS; ///<  @brief How many times to run the planner

    double sigmaxi_; ///<  @brief Sigma along the x axis

    double sigmayi_; ///<  @brief Sigma along the y axis


    double computed_sigma_x_; ///<  @brief Computed Sigma along the x axis

    double computed_sigma_y_; ///<  @brief Computed Sigma along the y axis

    double width_strip_; ///<  @brief Width of the Theta* sampling support


    int LEARNED; ///<  @brief Learned distance metric

    int FINDNEAREST; ///<  @brief find nearest vertex in RRT

    int NOTLEARNED;  ///<  @brief Not Learned distance metric

    int ONLYTHETACOST; ///<  @brief Consider only the Theta* cost

    double Kd; ///<  @brief Gain for Theta* cost

    int ADD_COST_THETASTAR; ///<  @brief Add theta* cost

    int AVERAGING; ///<  @brief Averaging among multiple segments

    double OR_RANGE; ///<  @brief Define orientation ranges for theta* sampling

    int MODEL_COST; ///<  @brief Type of cost to consider in the distance metrics

    double Kround; ///<  @brief Gain in Theta* Distance Metric

    double Kdist; ///<  @brief Gain in Theta* Distance Metric

    double Kth; ///<  @brief Gain in Theta* Distance Metric

    double Kor; ///<  @brief Gain in Theta* Distance Metric

    double Kangle; ///<  @brief Gain in Theta* Distance Metric

    double BIAS_PROB; ///<  @brief Probability of Classic Path Biasing technique

    double DISPERSION; ///<  @brief Dispersion in the Classic Path Biasing technique

    int n_dis_traj; ///<  @brief Number of points where to compute the Distance Metric (Not used at the moment)

    double LMAX; /// Thresold for the trapezoid function in the linear combination for the weights in the  Theta* Sampling

    int SRCPARENT;  ///<  @brief if to check for short cut in RRT connection (not used)

    bool initialized_; ///<  @brief check if the global planner is initialized

    int cnt_make_plan_; ///<  @brief counter of planning calls

    int TYPEPLANNER; ///<  @brief type of the planner to use

    std::string costmap_frame_;  ///<  @brief costmap frame

    std::string planner_frame_; ///<  @brief planner frame

    bool ADD_COST_FROM_COSTMAP; ///<  @brief Choose if to add cost from costmap

    int ADD_COST_PATHLENGTH; ///<  @brief Choose if to add cost associated to path length and changes of heading

    int LEVEL_OBSTACLE_; ///<  @brief Minimum cell cost to have to accept a cell as obstacle

    double GB_ORIENT_; ///<  @brief Size of the Orientation range during goal biasing

    double width_map_;  ///<  @brief Width of the 2D space where to sample

    double height_map_; ///<  @brief Height of the 2D space where to sample

    double center_map_x_; ///<  @brief x coordinate of the center of 2D space where to sample

    double center_map_y_; ///<  @brief y coordinate of the center of 2D space where to sample

    int cnt_no_plan_; ///<  @brief counter of no planning sol

/// ************************RRT Planner's parameters ****************************


/************************ ANYTIMERRT EXCLUSIVE ZONE !! *************************/

    double selCost ;
    
    double cost_bound ;
    double dist_bias ;   // distance bias, known as T.db in the paper
    double cost_bias ;   // cost bias, known as T.cb in the paper
    double delta_d, delta_c ; // decreasing dist_bias by a δd each iteration and increasing cost by δc
    double epsilon_f ;   // each succesive solution was guaranteed to be epsilon_f less costly than the previous solution.
/************************ END OF ANYTIMERRT DEFINITIONS *************************/  
	  
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
///  Pose start_pose_; ///< start pose for planning
///  Pose goal_pose_; ///< desired goal pose
  ros::Time path_time_; ///< time at which the path is released (after planning_timeout)
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
	 
/// ************************************************************************************************
/// RRT planner's parameters
/// ************************************************************************************************
    // Publishers
    ros::Publisher pub_path_;    // WARNING: TO PUBLISH A PATH
    ros::Publisher pub_goal_;
    ros::Publisher pub_tree_;
    ros::Publisher pub_tree_dedicated_;
    ros::Publisher pub_path_dedicated_;
    ros::Publisher pub_samples_;
    ros::Publisher pub_graph_;
    ros::Publisher pub_no_plan_;
    ros::Publisher pub_obstacle_markers_;

    // ros::Publisher pub_sensor_range_;
    // subscribers
    ros::Subscriber sub_obstacles_;    // to read the obstacles' positions
    ros::Subscriber sub_all_agents_;   // to read the agents' poses AND   to read the current robot pose
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_daryl_odom_;
    
    
/// ************************************************************************************************	 
} ;
 
} // namespace proxemics_anytimerrts
#endif
