#include <moveit/robot_trajectory/robot_trajectory.h>

#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

#include <tf_conversions/tf_eigen.h>

#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>

#include "cartesian_path_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>

const static int SAMPLE_ITERATIONS = 10;

//NOT COMPLETE. 
// Anonymous namespace

//TOCHECK the world and tool frames should be loaded from ROS_parameters?

// implement in https://github.com/ros-industrial-consortium/descartes/blob/issue_%2352_moveit_cartesian_path_service/descartes_moveit/src/cartesian_path_service.cpp
namespace
{
  
	// Start a service client
	ros::NodeHandle node_handle;
	ros::ServiceServer cartPathService = node_handle.advertiseService("get_cartesian_path", cartPathService_callback);

	// std_msgs/Header header
	// moveit_msgs/RobotState start_state
	// string group_name
	// string link_name
	// geometry_msgs/Pose[] waypoints
	// float64 max_step
	// float64 jump_threshold
	// bool avoid_collisions
	// moveit_msgs/Constraints path_constraints
	// ---
	// moveit_msgs/RobotState start_state
	// moveit_msgs/RobotTrajectory solution
	// float64 fraction
	// moveit_msgs/MoveItErrorCodes error_code

	bool cartPathService_callback(moveit_msgs::GetCartesianPath::Request &req, moveit_msgs::GetCartesianPath::Response &res)
	{
	    std::string group_name = req.group_name;
	
	    robot_state::RobotState start_state;
	    robot_state::robotStateMsgToRobotState(req.start_state, start_state);
	    moveit::core::RobotModelConstPtr robot_model = robot_state->getRobotModel();
	    const moveit::core::JointModelGroup* joint_model_group_ptr = robot_state->getJointModelGroup(group_name);
	    const std::vector<std::string>& link_names = joint_model_group_ptr->getLinkModelNames();
	    tool_frame = link_names.back();
	    world_frame = robot_state->getRobotModel()->getModelFrame();


	// how to get the tool_frame and world_from the GetCartesianPath srv, from waypoints?! and same for iterations?
	//note to self: the above is done by descartes_moveit/moveit_state_adapter! Recall Shaun's email. 
	//it takes inverse, root_to_world, etc
	// but does it solve the problem

	 //# Optional name of IK link for which waypoints are specified.
	// # If not specified, the tip of the group (which is assumed to be a chain)
	// # is assumed to be the link
	// string link_name



  	descartes_core::RobotModelConstPtr robot_model = createRobotModel(start_state,
                                                            group_name, tool_frame, 
                                                            world_frame, SAMPLE_ITERATIONS);

 	if (joint_model_group_ptr)
  	{
	    // std::string link_name = req.link_name;
	    // if (link_name.empty() && !jmg->getLinkModelNames().empty())
	    //   link_name = jmg->getLinkModelNames().back();

	    bool ok = true;
	    EigenSTL::vector_Affine3d waypoints(req.waypoints.size());
	    // const std::string &default_frame = context_->planning_scene_monitor_->getRobotModel()->getModelFrame();
	    // bool no_transform = req.header.frame_id.empty() || robot_state::Transforms::sameFrame(req.header.frame_id, default_frame) ||
	    //   robot_state::Transforms::sameFrame(req.header.frame_id, link_name); 

	    bool no_transform = req.header.frame_id.empty() || robot_state::Transforms::sameFrame(req.header.frame_id, world_frame) ||
	      robot_state::Transforms::sameFrame(req.header.frame_id, link_names);
	    

	    //the following is not needed. Might come handy for testing, though. (TOCHECK)
	    for (std::size_t i = 0 ; i < req.waypoints.size() ; ++i)
	    {
	      if (no_transform)
	        tf::poseMsgToEigen(req.waypoints[i], waypoints[i]);
	      else
	      {
	        geometry_msgs::PoseStamped p;
	        p.header = req.header;
	        p.pose = req.waypoints[i];
	        // if (performTransform(p, default_frame))
	        if (performTransform(p, world_frame))
	          tf::poseMsgToEigen(p.pose, waypoints[i]);
	        else
	        {
	          ROS_ERROR("Error encountered transforming waypoints to frame '%s'", defaultdefault_frame.c_str());
	          ok = false;
	          break;
	        }
	      }
	    }

	    //EXPERIMENT_BEGIN
	    if(ok)
   		{
     	  // Create a vector of axial trajectory pts from given waypoints. The following is a method!
    	  // TODO : take this method out from here, and the return type isn't a vector.

   		  // TODO / TOCHECK is the following method required?

		  // descartes_core::TrajectoryPtPtr waypointToAxialTrajectoryPt(const EigenSTL::vector_Affine3d& waypoints, double discretization)
		  // {
		  //   using namespace descartes_core;
		  //   using namespace descartes_trajectory;

		    double* rx;
		    double* ry;
		    double* rz;
		    double* x;
		    double* y;
		    double* z;
		    rx = new double[waypoints.size];
		    ry = new double[waypoints.size];
		    rz = new double[waypoints.size];
		    x = new double[waypoints.size];
		    y = new double[waypoints.size];
		    z = new double[waypoints.size];
		    boost::shared_ptr<vector<descartes_core::TrajectoryPt> > vec(new vector<descartes_trajectory::CartTrajectoryPt>);

		    for (std::size_t i = 0 ; i < waypoints.size() ; ++i)
		    {
		    	Eigen::Vector3d rpy =  waypoints[i].rotation().eulerAngles(0,1,2);
		    	//TOCHECK the rotation().eulerAngles works for Eigen::Affine3d
		    	//Do these methods exist for EigenSTL::vector_Affine3d waypoints as well?

			    rx[i] = rpy(0);
			    ry[i] = rpy(1);
			    rz[i] = rpy(2);
			    x[i] = waypoints[i].translation()(0);
			    y[i] = waypoints[i].translation()(1);
			    z[i] = waypoints[i].translation()(2);

			    // TOCHECK could be easier by using std::vector, like done in godel/trajectory_planning
			    boost::shared_ptr<TrajectoryPt> pointer;
		   		pointer_pb = pointer(new CartTrajectoryPt(TolerancedFrame(utils::toFrame(x[i],y[i], z[i], rx[i], ry[i], rz[i], descartes_core::utils::EulerConventions::XYZ),
		             ToleranceBase::zeroTolerance<PositionTolerance>(x[i], y[i], z[i]),
		             ToleranceBase::createSymmetric<OrientationTolerance>(rx[i], ry[i], 0, 0, 0, 2.0 * M_PI)),
		            0.0, discretization));	
		   		vec.push_back(pointer_pb);
			 }   

		    // return vec;
			
		  }
	    }
	    //TODO add suitable error

	    return true;
	}
	

	// moveit_msgs::GetCartesianPath::Request service_request;
	// moveit_msgs::GetCartesianPath::Response service_response;
	// service_request.


	// call the service
	// service_client.call(ser)
