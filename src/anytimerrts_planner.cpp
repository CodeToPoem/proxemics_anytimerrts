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
	
   if (!initialized_){
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

/// ****************** RRT Planner's settings below ******************************
            // number of RRT* iterations

        // ** RRT* Section **
        // 1 Create the planner algorithm -- Note that the min_time_reachability variable acts both
        //                                       as a model checker and a cost evaluator.
        // cout<<"Debug: Initiliaze planner"<<endl;
        /// Need to be set
        this->node_name_ = name;
        this->initialized_ = false;
        this->goal_theta_=0;
        this->goal_x_=0;
        this->goal_y_=0;
        this->cellheight_=1.5;
        this->cellwidth_=1.5;
        this->xsupp_=0;
        this->ysupp_=0;
        this->timeIter_=0;
        this->nrew_=0;
        this->goal_init_=false;
        this->DISPLAY_METRICS=0;
        this->ADD_COST_PATHLENGTH=1;
        this->no_fails=0;
//        this->curr_cost_=0;
        this->MAXTIME=0;
        this->NUMBER_UPDATE_TRAJ=0;
        this->NOANIM=0;
        this->first_sol=0;
        this->SELECT_FUNC=0;
        this->WHATTOSHOW=0;
        this->RHO=0.15;
        this->DT=0.1;
        this->inscribed_radius_ = 1;
        this->circumscribed_radius_ =1;
        this->max_map_loading_time=20;
        this->support_ = new trajectory_t();
        this->support_bias_ = new trajectory_t();
        trajectory_ = new Trajectory();
        this->cnt_make_plan_ = 0;
        this->cnt_no_plan_ = 0;
        
        // setting anytimeRRT parameters
        // Following the value given by the work of Luo YuanFu
        this->cost_bound = 100000 ;  //infinity
        this->curr_cost_= 100000 ; // infinity
        this->dist_bias = 1.0;
        this->cost_bias = 0.0;
        this->delta_d = 0.1;
        this->delta_c = 0.1;
        this->epsilon_f = 0.04;
        
        
        ros::NodeHandle node("~/anytimeRRTs");
        nh_ =  node;

        ROS_INFO("anytimeRRTs initializing");



        listener = new tf::TransformListener();

        costmap_ros_ = costmap_ros;

        costmap_ = costmap_ros_->getCostmap();


        try{

              footprint_spec_ = costmap_ros_->getRobotFootprint();

              if( (int)(footprint_spec_.size()) > 0)
                  ROS_INFO("footprint_spec_ loaded with %d elements", (int)footprint_spec_.size());

              world_model_ = new CostmapModel(*costmap_);

              grid_planner_ = new Grid_planner(node, new CostmapModel(*costmap_) , footprint_spec_, costmap_ros_);

              grid_planner_->initialize();

        }
        catch (exception& e)
        {
            ROS_ERROR("An exception occurred. Exception Nr. %s", e.what() );
        }



        /// all the ROS DATA
        // setup publishers

        pub_path_ = nh_.advertise<nav_msgs::Path>("rrt_planner_path", 5);

        pub_goal_ = nh_.advertise<visualization_msgs::Marker>("rrt_planner_goal",1000);

        pub_tree_=nh_.advertise<visualization_msgs::Marker>("rrt_planner_tree",1000);

        pub_tree_dedicated_=nh_.advertise<visualization_msgs::Marker>("rrt_planner_tree_dedicated",1000);

        pub_path_dedicated_=nh_.advertise<visualization_msgs::Marker>("rrt_planner_path_dedicated",1000);

        pub_samples_ = nh_.advertise<visualization_msgs::Marker>("rrt_planner_samples",1000);

        // pub_graph_=nh_.advertise<pedsim_msgs::Tree>("/rrt_planner/graph",1);
        pub_no_plan_ = nh_.advertise<std_msgs::Bool>("rrt_planner/no_plan",1);


        sub_obstacles_=nh_.subscribe("/move_base_node/global_costmap/costmap",1, &Srl_global_planner::callbackObstacles,this);


        // subscriberss TODO!!!!
        sub_goal_=nh_.subscribe("/move_base_simple/goal",1,&Srl_global_planner::callbackSetGoal,this);


        sub_daryl_odom_=nh_.subscribe("/odom",5,&Srl_global_planner::callbackSetRobotPose,this);


        ROS_INFO("ROS publishers and subscribers initialized");


        /// Initializing number of read maps
        cnt_map_readings=0;

        loop_rate = new ros::Rate(2/DT);



        /// ==================
        /// READING PARAMETERS
        /// ==================
        /// define dim of scene
        double x1,x2, y1,y2, csx,csy;

        nh_.getParam("/move_base_node/x1", this->xmin_);
        nh_.getParam("/move_base_node/x2", this->xmax_);
        nh_.getParam("/move_base_node/y1", this->ymin_);
        nh_.getParam("/move_base_node/y2", this->ymax_);
        nh_.getParam("/move_base_node/cell_size_x", csx);
        nh_.getParam("/move_base_node/cell_size_y", csy);

        int tcount,firstsol,deburrt;
        nh_.getParam("/move_base_node/TIMECOUNTER", this->TIMECOUNTER);
        nh_.getParam("/move_base_node/FIRSTSOLUTION", this->FIRSTSOLUTION);
        nh_.getParam("/move_base_node/DEB_RRT", this->DEB_RRT);
        nh_.getParam("/move_base_node/BOX",this->BOX);
        nh_.getParam("/move_base_node/WHATTOSHOW",this->WHATTOSHOW);
        nh_.getParam("/move_base_node/RADIUS",this->RADIUS);
        nh_.getParam("/move_base_node/RHO",this->RHO);
        nh_.getParam("/move_base_node/DT",this->DT);
        nh_.getParam("/move_base_node/L_AXIS",this->L_AXIS);
        nh_.getParam("/move_base_node/THRS_BRANCHBOUND",this->THRS_BRANCHBOUND);
        nh_.getParam("/move_base_node/BRANCHBOUND",this->BRANCHBOUND);
        nh_.getParam("/move_base_node/BRANCHBOUND_RATIO",this->BRANCHBOUND_RATIO);
        nh_.getParam("/move_base_node/GOAL_BIASING",this->GOAL_BIASING);
        nh_.getParam("/move_base_node/GOAL_BIASING_THS",this->GOAL_BIASING_THS);
        nh_.getParam("/move_base_node/SCALING_IRL",this->SCALING_IRL);
        nh_.getParam("/move_base_node/PARALLEL",this->PARALLEL);
        nh_.getParam("/move_base_node/K",this->K);
        nh_.getParam("/move_base_node/RAD_OBST",this->RAD_OBST);
        nh_.getParam("/move_base_node/ROBOT_LENGTH",this->robot_length_);
        nh_.getParam("/move_base_node/ROBOT_WIDTH",this->robot_width_);
        nh_.getParam("/move_base_node/READ_AGENTS",this->READ_AGENTS);
        nh_.getParam("/move_base_node/file_name",this->cost_file_name);
        nh_.getParam("/move_base_node/front_rear_error_distance",this->collision_boundary);
        nh_.getParam("/move_base_node/max_map_loading_time",this->max_map_loading_time);
        nh_.getParam("/move_base_node/agents_size",this->agents_size_);
        nh_.getParam("/move_base_node/MAXTIME",this->MAXTIME);
        nh_.getParam("/move_base_node/NUMBER_UPDATE_TRAJ",this->NUMBER_UPDATE_TRAJ);
        nh_.getParam("/move_base_node/NOANIM",this->NOANIM);
        nh_.getParam("/move_base_node/SELECT_FUNC",this->SELECT_FUNC);
        nh_.getParam("/move_base_node/DISPLAY_METRICS",this->DISPLAY_METRICS);
        nh_.getParam("/move_base_node/COST_DISPLAY",this->COST_DISPLAY);
        nh_.getParam("/move_base_node/max_iterations", this->Nit_);
        nh_.getParam("/move_base_node/inscribed_radius",this->inscribed_radius_);
        nh_.getParam("/move_base_node/circumscribed_radius",this->circumscribed_radius_);
        nh_.getParam("/move_base_node/TYPE_SAMPLING",this->TYPE_SAMPLING);
        nh_.getParam("/move_base_node/LEARNED", this->LEARNED);
        nh_.getParam("/move_base_node/FINDNEAREST", this->FINDNEAREST);
        nh_.getParam("/move_base_node/NOTLEARNED", this->NOTLEARNED);
        nh_.getParam("/move_base_node/ONLYTHETACOST", this->ONLYTHETACOST);
        nh_.getParam("/move_base_node/OR_RANGE",this->OR_RANGE);
        nh_.getParam("/move_base_node/AVERAGING",this->AVERAGING);
        nh_.getParam("/move_base_node/Kd",this->Kd);
        nh_.getParam("/move_base_node/Kangle",this->Kangle);
        nh_.getParam("/move_base_node/Kdist",this->Kdist);
        nh_.getParam("/move_base_node/Kor",this->Kor);
        nh_.getParam("/move_base_node/WIDTH_STRIP",this->width_strip_);
        nh_.getParam("/move_base_node/LMAX",this->LMAX);
        nh_.getParam("/move_base_node/Kth",this->Kth);
        nh_.getParam("/move_base_node/n_dis_traj",this->n_dis_traj);
        nh_.getParam("/move_base_node/ADD_COST_THETASTAR",this->ADD_COST_THETASTAR);;
        nh_.getParam("/move_base_node/Kround",this->Kround);
        nh_.getParam("/move_base_node/MODEL_COST",this->MODEL_COST);
        nh_.getParam("/move_base_node/BIAS_PROB",this->BIAS_PROB);
        nh_.getParam("/move_base_node/DISPERSION",this->DISPERSION);
        nh_.getParam("/move_base_node/type_planner", this->TYPEPLANNER);
        nh_.getParam("/move_base_node/planner_frame",this->planner_frame_);
        nh_.getParam("/move_base_node/ADD_COST_FROM_COSTMAP", this->ADD_COST_FROM_COSTMAP);
        nh_.getParam("/move_base_node/LEVEL_COLLCHECKER_OBSTACLE_", this->LEVEL_OBSTACLE_);
        nh_.getParam("/move_base_node/GOAL_BIASING_ORIENTATION_RANGE", this->GB_ORIENT_);
        nh_.getParam("/move_base_node/ADD_COST_PATHLENGTH", this->ADD_COST_PATHLENGTH);

        /// store dim of scene
        this->xscene_=x2-x1;
        this->yscene_=y2-y1;
        /// store sizes
        this->cellwidth_=csx;
        this->cellheight_=csy;

        ROS_INFO("RRT_planner initialized");

        int size_footprint = (int)footprint_spec_.size();

        ROS_INFO("Current size of the footprint %d", size_footprint);

        initialized_ = true;
/// ****************** RRT Planner's settings ends *******************************


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
   else	{
	  ROS_WARN("RRT planner not initialized");
   }
}

AnytimeDynamicRRTs::~AnytimeDynamicRRTs() {
  reset();

  std::vector< state_map* >::const_iterator it;
  for(it = expanded_states_.begin(); it != expanded_states_.end(); ++it)
  {
    delete (*it);
  }

  delete discretizer_;
  delete trajectory_rollout_;
  delete heuristic_calc_;
  delete cost_calc_;
}

// clean up everything
void AnytimeDynamicRRTs::reset() {
	//TODO
}

bool AnytimeDynamicRRTs::findReplanningWaypoint(ros::Time plan_release_time,
                              geometry_msgs::PoseStamped& replanning_start_pose,
                              geometry_msgs::Twist& replanning_start_vel){

  if(current_plan_.poses.empty())
  {
    ROS_WARN("AStarLattice: error while trying to find the next waypoint for "
             "continuous planning: plan is empty");
    return false;
  }

  conti_pose = current_plan_.poses.back();
  replanning_start_vel = current_plan_.velocities.back();

  int i=0;

  //check array for the first element that is at conti_time or later
  while(i < current_plan_.poses.size() &&
        fabs((conti_pose.header.stamp - conti_time).toSec()) > time_resolution_ / 2)
  {
    conti_pose = current_plan_.poses.at(i);
    replanning_start_vel = current_plan_.velocities.at(i);
    ++i;
  }

  //correct planning position for turning in place:
  //when turning in place, amcl's estimated robot pose can deviate a lot so that
  //after turning the robot's estimated position does not match the one from the
  //current plan any more. To overcome this problem, it is checked here whether the
  //robot is only turning and if so, the robot position for the next planning is
  //updated with the currently estimated position from amcl instead of the
  //waypoint from the current plan.
  try
  {
    geometry_msgs::Twist prev_vel = current_plan_.velocities.at(i - 2);
    if(prev_vel.linear.x == 0 && replanning_start_vel.linear.x == 0)
    {
      ROS_DEBUG("turn in place. update pose with amcl");
      tf::Stamped<tf::Pose> robot_pose;
      if(dynamic_costmap_->getStaticROSCostmap()->getRobotPose(robot_pose))
      {
        conti_pose.pose.position.x = robot_pose.getOrigin().getX();
        conti_pose.pose.position.y = robot_pose.getOrigin().getY();
      }
    }
  }
  catch(std::out_of_range ex)
  {
    ROS_ERROR("could not adjust next waypoint for turning in place, out of range");
  }

  //check if element is the closest possible to conti time
  if(fabs((conti_pose.header.stamp - conti_time).toSec()) < time_resolution_ / 2)
    return true;

  else
  {
    ROS_ERROR_STREAM("AStarLattice: could not find next waypoint for continuous "
                     "planning for time " << conti_time << std::endl <<
                     current_plan_);
    return false;
  }
  								  
}

/// ==================================================================================
/// plan(Trajectory *traj,int type)
/// method to solve a planning probleme.
/// ==================================================================================
int AnytimeDynamicRRTs::plan(Trajectory *traj, int type, geometry_msgs::PoseStamped& start){

    if(DEB_RRT>0)
      ROS_INFO("Startin to plan!!!");


    // RRT* section
    // for each movement clear the obstacle list and refill it with the static ones and the agents
    // 1. CREATE PLANNING OBJECTS
    //cout<<"Debug: Initiliaze components";
    // 1.a Create the components
    sampler_t sampler;
    // State, input, vertex_data, and edge_data definitions
    distance_evaluator_t distance_evaluator;

    extender_t extender;
    collision_checker_t collision_checker;

    collision_checker.size_robot=robot_length_+collision_boundary*2;
    collision_checker.LEVEL_OBSTACLE_ = this->LEVEL_OBSTACLE_;
    /// IF RECTANGULAR COLLISION CHECKER
    // collision_checker.setParam(K,RAD_OBST,PARALLEL);
    // collision_checker.setRobotDim(robot_width_,robot_length_,collision_boundary);

    extender.L_axis=this->L_AXIS;
    // smoothness_cost_t min_smoothness;
    min_time_reachability_t min_time_reachability;

    min_time_reachability.file_name=cost_file_name;

    branch_and_bound_t branch_and_bound;



    rrtstar_t planner (sampler,distance_evaluator,extender,collision_checker,
                          min_time_reachability,min_time_reachability);

    collision_checker.initialize(world_model_, footprint_spec_, inscribed_radius_, circumscribed_radius_, costmap_ros_, planner_frame_);

    min_time_reachability.initWorldModel(world_model_, footprint_spec_, inscribed_radius_, circumscribed_radius_, costmap_ros_, planner_frame_);

    distance_evaluator.set_list_vertices(&(planner.list_vertices));

    if(DEB_RRT>0)
      ROS_INFO("Planner created");
        //added for single_integrator_extender
    //    extender.set_max_length(2);
    double side;
    int multiplier_side=sqrt(5);
    //considering a square inscribed in a circle, let's calculate a radius and then multiplie it for 2 (conservative way)
    side=(sqrt((goal_x_-rx)*(goal_x_-rx)+(goal_y_-ry)*(goal_y_-ry)))*multiplier_side;

    if(DEB_RRT>0)
      ROS_INFO("Planner setting parameters");

    planner.parameters.set_phase (type);   // The phase parameter can be used to run the algorithm as an RRT,
    // See the documentation of the RRG algorithm for more information.
    if(type==0){

            planner.parameters.set_fixed_radius(RADIUS);

    }
    planner.BOX=0;

    planner.parameters.set_gamma (side);    // Set this parameter should be set at least to the side length of
    //   the (bounded) state space. E.g., if the state space is a box
    //   with side length L, then this parameter should be set to at
    //   least L for rapid and efficient convergence in trajectory space.
    planner.parameters.set_dimension (3);
    planner.parameters.set_max_radius (2*side);  // This parameter should be set to a high enough value. In practice,
    //   one can use smaller values of this parameter to get a good
    //   solution quickly, while preserving the asymptotic optimality.

    if(RADIUS>0)
        planner.parameters.set_fixed_radius(RADIUS);


    /// setting RHO end condition
    planner.RHO=this->RHO;
    planner.DT=this->DT;

    if(DEB_RRT>0)
      ROS_INFO("Planner setting learning and support");

    planner.LEARNED=this->LEARNED;
    planner.FINDNEAREST=this->FINDNEAREST;
    planner.SRCPARENT=this->SRCPARENT;
    // planner.updatepathsupport(support_);

    if(DEB_RRT>0)
      ROS_INFO("setting cost evaluator");

    /// setting cost evaluator
    min_time_reachability.NOTLEARNED=this->NOTLEARNED;
    min_time_reachability.MODEL_COST=this->MODEL_COST;
    min_time_reachability.ONLYTHETACOST=this->ONLYTHETACOST;

    if(DEB_RRT>0)
      ROS_INFO("setting cost evaluator - update_trajectory");

    // by Charly: defining region_goal
    region<2> region_goal;
    
    min_time_reachability.update_trajectory(support_);
    min_time_reachability.Kd=this->Kd;
    min_time_reachability.Kangle=this->Kangle;
    min_time_reachability.set_goal_region (region_goal);    
    min_time_reachability.Kdist = this->Kdist;
    min_time_reachability.Kth = this->Kth;
    min_time_reachability.Kor = this->Kor;
    min_time_reachability.ADD_COST_THETASTAR = this->ADD_COST_THETASTAR;
    min_time_reachability.ADD_COST_FROM_COSTMAP = this->ADD_COST_FROM_COSTMAP;
    min_time_reachability.ADD_COST_PATHLENGTH = this->ADD_COST_PATHLENGTH;

    min_time_reachability.n_dis_traj = this->n_dis_traj;


    /// Reading current robot pose from cost map
    if(DEB_RRT>0)
      ROS_INFO("Reading current robot pose from cost map");


    rx = start.pose.position.x;
    ry = start.pose.position.y;
    rz = tf::getYaw(start.pose.orientation);
    rz = set_angle_to_range(rz,0);

    //! 2 Set the Sampler Support
    if(DEB_RRT>0)
      ROS_INFO("Setting Sampler");

    /// For a Rectangle as Sampling Support
    region<3> sampler_support;
    sampler_support.center[0] = rx;
    sampler_support.center[1] = ry;
    sampler_support.center[2] = goal_theta_;
    sampler_support.size[0] = width_map_ ;
    sampler_support.size[1] = height_map_ ;
    sampler_support.size[2] = 2*M_PI;



    /// Not Uniform
    sampler.set_support(sampler_support);
    sampler.AVERAGING=this->AVERAGING;
    sampler.OR_RANGE=this->OR_RANGE;
    sampler.LMAX=this->LMAX;

    if(DEB_RRT>0)
      ROS_INFO("Local Planner -->  SupportCenter, SupportSize: (%f ,%f, %f) - (%f ,%f, %f)",sampler_support.center[0],sampler_support.center[1],sampler_support.center[2],sampler_support.size[0],sampler_support.size[1],sampler_support.size[2]);


    if(DEB_RRT>0)
      ROS_INFO("Define type of sampling unit");

    sampler.use_type(TYPE_SAMPLING);

    // In case of biasing

    region<3> region_goal_sup;

    region_goal_sup.center[0] = goal_x_ ;
    region_goal_sup.center[1] = goal_y_ ;
    region_goal_sup.center[2] = goal_theta_ ;

    region_goal_sup.size[0] = toll_goal_;
    region_goal_sup.size[1] = toll_goal_;
    region_goal_sup.size[2] = this->GB_ORIENT_;
    sampler.set_goal(region_goal_sup);
    sampler.set_goal_biasing(GOAL_BIASING);
    sampler.set_goal_biasing_ths(GOAL_BIASING_THS);


    if(DEB_RRT>0)
      ROS_INFO("Set Type Sampling");

    if(TYPE_SAMPLING==1){
        sampler.setsigmas(sigmaxi_,sigmayi_);
        sampler.use_extsigmas(EXT_SIGMAS);
            /// Insert the path obtained by ThetaStar
       if(DEB_RRT>0)
          ROS_INFO("Importing Theta* path into the sampling unit");

        sampler.update_trajectory(support_);
        sampler.Kround=this->Kround;
    }
    else if(TYPE_SAMPLING==0 || TYPE_SAMPLING==3 ) {

        sampler.set_width_strip(width_strip_);
            /// Insert the path obtained by ThetaStar
        if(DEB_RRT>0)
          ROS_INFO("Importing Theta* path into the sampling unit");

        sampler.update_trajectory(support_);
        sampler.Kround=this->Kround;

    }else if(TYPE_SAMPLING==5){

    sampler.update_trajectory_bias(support_bias_);
    sampler.set_bias_probability(BIAS_PROB);
    sampler.set_sample_dispersion(DISPERSION);

    }










if(DEB_RRT>0){
    ROS_DEBUG_STREAM("Sampler support center...."<< sampler_support.center[0]<< " "<<sampler_support.center[1]<< " "<< sampler_support.center[2]<< " ");
    ROS_DEBUG_STREAM("Sampler Sides :"<< sampler_support.size[0]<<" " <<sampler_support.size[1]<<" " <<sampler_support.size[2]);

}





if(DEB_RRT>0)
    ROS_DEBUG_STREAM("Debug: collision checker");


if( DEB_RRT>0)
    ROS_DEBUG_STREAM("Debug: Add Pedestrians");





    double obx,oby;
    //    double rubber=-0.10;
    double rubber=0.0;

    //add all the static obstacles
if(DEB_RRT>0)
    ROS_DEBUG_STREAM("Debug: Adding static Obstacles");


    if(DEB_RRT>0)
        ROS_INFO("Loading Obstacles");

    for (size_t i = 0; i < obstacle_positions.size(); i++) {
        Tobstacle l = obstacle_positions[i];
        region<2> obstaclei;


        obstaclei.center[0]=l.x;
        obstaclei.center[1]=l.y;

        obstaclei.size[0]=l.cell_width+rubber;
        obstaclei.size[1]=l.cell_height+rubber;
        collision_checker.add_obstacle(obstaclei);

    }



    // 3.b Initialize the model checker and the cost evaluator
if (DEB_RRT>0)
    ROS_DEBUG_STREAM("Debug: goal region");

    //region<2> region_goal;
    region_goal.center[0] =goal_x_ ;
    region_goal.center[1] =goal_y_ ;
    region_goal.size[0] = toll_goal_/2;
    region_goal.size[1] = toll_goal_/2;

    min_time_reachability.set_goal_region (region_goal);





if(DEB_RRT>0)
    ROS_DEBUG_STREAM("Goal Region: x:"<<region_goal.center[0]<<" y:"<<region_goal.center[1]);

if(DEB_RRT>0)
      ROS_INFO("Goal Region: x: %f y: %f",region_goal.center[0],region_goal.center[1]);


    // 2.g. Initialize the branch and bound utility   NUM_DIMENSION = 3
    double bnb_goal_center[3] ;
    for (int i = 0 ; i < 2 ; i++){
		bnb_goal_center[i] = 8.0 ;
	}
	for (int i = 2 ; i < 3 ; i++){
		bnb_goal_center[i] = 0.0 ;
	}
    
    // Set branch and bound util
    branch_and_bound.set_planner (&planner);
    branch_and_bound.set_goal_region (region_goal);
    branch_and_bound.set_root_vertex (planner.get_root_vertex());


    planner.WHATTOSHOW=WHATTOSHOW;


    if(DEB_RRT>0)
      ROS_INFO("Loading Robot Pose");

    // 3.c Initialize the planner to commanded Agent position ** TODO ** We can get the initial position
    state_t *state_initial = new state_t;

    state_initial->state_vars[0] = rx;

    state_initial->state_vars[1] = ry;

    state_initial->state_vars[2] = rz;

    if( DEB_RRT>0)
      ROS_INFO_STREAM("Robot position-> x:"<<state_initial->state_vars[0]<<" y:"<<state_initial->state_vars[1]<<" z:"<<state_initial->state_vars[2]);


    // ROS_INFO("Robot position-> %f %f %f:",state_initial->state_vars[0],state_initial->state_vars[1],state_initial->state_vars[2]);

    planner.initialize (state_initial);




    if( DEB_RRT>0)
      ROS_DEBUG_STREAM("Debug: RRT* iterations");


    min_time_reachability.begin_time=clock();
    float timep=0;
    double rw=0;

    int iterations=1;
    int it=0;

    double begin_time_ext_ros;
    begin_time_ext_ros=ros::WallTime::now().toSec();

    double begin_time_solution,end_time_solution;
    begin_time_solution=ros::WallTime::now().toSec();

    double curr_MAXTIME;
    double curr_NUM_UPDATES;
    /// if no plan was found give much more time to the next trial
    if(cnt_no_plan_>0)
    {

        curr_MAXTIME = (cnt_no_plan_+1)*(cnt_no_plan_+1)*MAXTIME;
        curr_NUM_UPDATES = 1;

        if( DEB_RRT>0)
          ROS_WARN("Giving more time, now %f sec", MAXTIME);

    }else{

        curr_MAXTIME = MAXTIME;
        curr_NUM_UPDATES = NUMBER_UPDATE_TRAJ;
    }

// Charly wants the time to find the first solution
// and have its exported
ofstream myfile ;
myfile.open("/home/charly/anytimeRRTs_time_register.txt") ;

myfile << "time\t\tcost" << std::endl 
       << "----------------------------" << std::endl ;

if(DEB_RRT>0)
  ROS_INFO_STREAM ("Openning document to register execution time !!") ;


if(TIMECOUNTER){

    
    while(timep<curr_MAXTIME && min_time_reachability.cntUpdates<curr_NUM_UPDATES ){


    

        if(timep>curr_MAXTIME)
            break;

        begin_time_ext_ros=ros::WallTime::now().toSec();
        
        // chrono timer
        auto startTime = std::chrono::high_resolution_clock::now() ;
        
        planner.iteration ();
        timep+=  ros::WallTime::now().toSec() - begin_time_ext_ros;
        // chrono timer
        auto endTime = std::chrono::high_resolution_clock::now() ;
        
        // chrono timer duration
        myfile << "\t" << std::chrono::duration_cast<std::chrono::microseconds> (endTime - startTime).count() << std::endl 
               << "or, timep+: " << timep ;
        
        
        
        it++;

        publishSample(planner.statex,planner.statey,planner.statetheta,it);
        iterations=it;

        rw+=planner.nrewiring;
        if (it%100000 == 0) {

            if(DEB_RRT>0)
              ROS_INFO("Seconds: %f", timep);

            iterations=it;

        }


        if(FIRSTSOLUTION>0){

          if(min_time_reachability.foundTraj==true){

            curr_cost_=min_time_reachability.cost;
            if(first_sol==0){
                end_time_solution=ros::WallTime::now().toSec() -begin_time_solution;
                
                // Charly wants this time info 
                myfile << "First solution after seconds: " << end_time_solution << std::endl ;

                if( DEB_RRT>0)
                  ROS_INFO("First Solution after Seconds: %f", end_time_solution);


                  
                first_sol++;
             }



            }
        }
        else
        {

             if(min_time_reachability.foundTraj==true){

                curr_cost_=min_time_reachability.cost;
                }

        }


        
    }
}

else {
    for (int i = 0; i < Nit_; i++){
        begin_time_ext_ros=ros::WallTime::now().toSec();
        planner.iteration ();
        timep+=  ros::WallTime::now().toSec() - begin_time_ext_ros;
        it++;

        publishSample(planner.statex,planner.statey,planner.statetheta,it);
        rw+=planner.nrewiring;

            iterations=i;

        if (i%100 == 0) {

          if(DEB_RRT>0)
            ROS_INFO("Iteration: %d" , i);
            iterations=i;

        }
       //double min_cost = min_time_reachability.get_best_cost ();

	
       // Here is the implementation of AnytimeRRT
       selCost = dist_bias * min_time_reachability.get_best_cost () + cost_bias * min_time_reachability.cost ;
	          
       double min_cost = selCost ;
       
       int min_cost_index ;
       // typeparam = 2
       for (int i = 1 ; i < 2; i++)  {
		   if (selCost < min_cost)  {
			   min_cost = selCost ;
			   min_cost_index = i ;
		   }
	   }

       if(BRANCHBOUND){

           if (min_cost>THRS_BRANCHBOUND && (i%BRANCHBOUND_RATIO == 0)) {

             if(DEB_RRT>0)
                  ROS_INFO("Min Cost %f, Running branch_and_bound",min_cost );

                  branch_and_bound.set_upper_bound_cost (min_cost);
                  branch_and_bound.run_branch_and_bound ();
           }

       }

        if(FIRSTSOLUTION>0){
        /// If solution is found then stop to iterate
            if(min_time_reachability.foundTraj==true){
                curr_cost_=min_time_reachability.cost;
                end_time_solution=ros::WallTime::now().toSec() -begin_time_solution;


                // AnytimeRRT is here !!
                cost_bound = (1 - epsilon_f) * min_time_reachability.cost ;
                dist_bias = dist_bias - delta_d ;
                cost_bias = cost_bias - delta_c ;
    	        if(dist_bias<0) dist_bias=0;
    	        if(cost_bias>1) cost_bias=1.0;                

                
                
        // if(min_smoothness.foundTraj==true){
            break;
            }
        }

    }
}





   if(DEB_RRT>0)
      ROS_INFO ("Execution tkime registering is done. The document is closed.") ;


    if(FIRSTSOLUTION==0){
        end_time_solution=ros::WallTime::now().toSec()-begin_time_solution;
        


        if( DEB_RRT>0)
          ROS_INFO("%d Solution after Seconds: %f",NUMBER_UPDATE_TRAJ, end_time_solution);
          
    }

// Charly closes the document
myfile.close() ;

    nrew_=rw/iterations;
    timeIter_=timep/iterations;

    trajectory_t trajectory_final;

    if(DEB_RRT>0)
      ROS_INFO( "Try to Get Solution " );


    min_time_reachability.get_solution (trajectory_final);            // GET THE TRAJECTORY SOLUTION




    /// IF NO TRAJECTORY SAVE THE TREE
    if( trajectory_final.list_states.size()==0){
        typedef rrtstar_t planner_t;

    if(!NOANIM)
        publishTree();


        planner_t *planner_int;
        planner_int =&planner;
        list <vertex_t *> *list_vertices = &(planner_int->list_vertices);
        /// Metrics (timeIter_ saved before)
        numVertices_=list_vertices->size();
        timeSolution_=end_time_solution;
        return 0;

    }


    float x,y,theta;
    traj->reset();


    for (typename list<state_t*>::iterator it_state = trajectory_final.list_states.begin();
         it_state != trajectory_final.list_states.end(); it_state++){
        state_t *state_curr = *it_state;
        x = (state_curr)->state_vars[0];
        y = (state_curr)->state_vars[1];
        theta = (state_curr)->state_vars[2];
        traj->addPointEnd(Tpoint(x,y,theta));

    }


    /// Publish the obtained path
    if(!NOANIM)
        publishPath(traj);



    double v,w ;




    for (typename list<input_t*>::iterator it_input = trajectory_final.list_inputs.begin();
         it_input != trajectory_final.list_inputs.end(); it_input++){
        input_t *input_curr = *it_input;
        v=(input_curr)->input_vars[0];
        w=(input_curr)->input_vars[1];
        traj->addVelocities(v,w);

    }




    typedef rrtstar_t planner_t;
    planner_t *planner_int;
    planner_int =&planner;


    list <vertex_t *> *list_vertices = &(planner_int->list_vertices);
    numVertices_=list_vertices->size();
    timeSolution_=end_time_solution;



    if(!NOANIM)
        publishTree();




    return 1;
}

/// ==================================================================================
/// getPath() which is equivalent to makePlan(). getPath() calls in plan() to do the planning.
/// ==================================================================================
bool AnytimeDynamicRRTs::getPath(geometry_msgs::PoseStamped start,
                           geometry_msgs::PoseStamped goal, bool replanning,
                           std::vector<geometry_msgs::PoseStamped>& path) {
  reset() ;
   
  ros::Time begin = start.header.stamp;
  path_time_ = begin + ros::Duration(planning_timeout_);

  planning_timed_out_ = false;

  geometry_msgs::Twist start_vel;
  if(replanning && !current_plan_.poses.empty())
  {
    geometry_msgs::PoseStamped conti_pose;
    geometry_msgs::Twist conti_vel;

    //if the next conti waypoint for continuous path generation was found,
    //plan from there. If not, plan from the static start position
    if(findReplanningWaypoint(path_time_, conti_pose, conti_vel))
    {
      //start pose and start vel for continuous path generation
      start = conti_pose;
      start_vel = conti_vel;
      //time we want to release the new path
      path_time_ = start.header.stamp;
    }
    else
      ROS_ERROR("conti waypoint not found");
  }

#ifndef DEBUG
  ros::Timer planning_timer = nh_.createTimer(path_time_ - ros::Time::now(),
                                             &AStarLattice::plannerTimeoutCallback,
                                             this, true);
#endif

  dynamic_costmap_->update();

  // fll start and end pose 
  // TODO
  
  /// ******************* RRT Planning goes here ********************************
    if(this->initialized_){

        this->setGoal((double)goal.pose.position.x, (double)goal.pose.position.y, (double)tf::getYaw(goal.pose.orientation), toll_goal_ , goal.header.frame_id );
        /// Grid Planning
        /// To check the frame.. Reading in Odom Frame ...
        grid_planner_->setStart(start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation), start.header.frame_id );

        grid_planner_->setGoal(goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation), goal.header.frame_id  );

        std::vector< geometry_msgs::PoseStamped > grid_plan;

        if(TYPE_SAMPLING!=4){

            if(!grid_planner_->run_grid_planner(grid_plan)){

                    ROS_WARN("NO PATH FOUND FROM THE GRID PLANNER");
                    return false;
            }

            if((int)grid_plan.size()==1){

            		ROS_WARN("DISCRETE PATH OF SINGLE CELL, STAY HERE");
            		return false;
            }

            setGlobalPathSupport(grid_plan);

          }


        geometry_msgs::PoseStamped s;
        s = transformPose(start);



        if(this->plan(trajectory_, this->TYPEPLANNER, s)){


                        std::vector<Tpoint> path = trajectory_->getPath();

                        plan.clear();
                        cnt_no_plan_= 0;
                        cnt_make_plan_++ ;

                        for (size_t i = 0; i < path.size(); i++) {

                            geometry_msgs::PoseStamped posei;
                            posei.header.seq = cnt_make_plan_;
                            posei.header.stamp = ros::Time::now();
                            posei.header.frame_id = planner_frame_; /// Check in which frame to publish
                            posei.pose.position.x = path[i].x;
                            posei.pose.position.y = path[i].y;
                            posei.pose.position.z = 0;
                            posei.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,path[i].z);
                            plan.push_back(posei);

                        }


                        return true;



        }else{
                        cnt_no_plan_++;

                        return false;

        }


    }else{

        return false;
    }  
  /// ******************* RRT Planning ends here ******************************** 
  
    if(!path.empty())
    return true;

  else
    return false;        							   
}



}  // namespace proxemics_anytimerrts
