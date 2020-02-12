#include <mpc_local_planner_mb/mpc_local_planner_mb.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <string>
#include <vector>
#include <algorithm>
#include <angles/angles.h>
#include <std_msgs/Float32.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>
#include <algorithm>
#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mpc_local_planner_mb::MPCLocalPlannerMBROS, nav_core::BaseLocalPlanner)
using namespace std;
using namespace Eigen;
namespace mpc_local_planner_mb {

    MPCLocalPlannerMBROS::MPCLocalPlannerMBROS():initialized_(false),
        odom_helper_("odom"){

    }
    MPCLocalPlannerMBROS::~MPCLocalPlannerMBROS() {}


    void MPCLocalPlannerMBROS::initialize(std::string name,
                                           tf2_ros::Buffer* tfBuffer,
                                           costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!isInitialized()) {

            ros::NodeHandle private_nh("~");
            ros::NodeHandle nh;
            private_nh.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal" );

            _pub_ref_path_odom = private_nh.advertise<nav_msgs::Path>("/move_base_MPCLocalPlanner/mpc_reference_path_odom", 1);
            _pub_ref_path_baselink = private_nh.advertise<nav_msgs::Path>("/move_base_MPCLocalPlanner/mpc_reference_path_baselink", 1);
            _pub_mpc_traj   = private_nh.advertise<nav_msgs::Path>("/move_base_MPCLocalPlanner/local_plan", 1);// MPC trajectory output
            _pub_twist = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)
            _sub_goal   = private_nh.subscribe( _goal_topic, 1, &MPCLocalPlannerMBROS::goalCB, this);


            //Parameters for control loop
            // private_nh.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
            private_nh.param("debug_info", _debug_info, false);
            private_nh.param("delay_mode", _delay_mode,false);
            private_nh.param("max_speed", _max_speed, 6.0); // unit: m/s
            private_nh.param("min_speed", _min_speed, 0.1); // unit: m/s
            private_nh.param("goal_radius", _goalRadius, 0.2); // unit: m
            private_nh.param("controller_freq", _controller_freq, 10);
            private_nh.param("vehicle_Lf", _Lf, 0.25); // distance between the front of the vehicle and its center of gravity

            _dt = double(1.0/_controller_freq); // time step duration dt in s

            //Parameter for MPC solver
            private_nh.param("mpc_steps", _mpc_steps, 20.0);
            private_nh.param("mpc_ref_cte", _ref_cte, 0.0);
            private_nh.param("mpc_ref_epsi", _ref_epsi, 0.0);
            private_nh.param("mpc_ref_vel", _ref_vel, 1.5);
            private_nh.param("mpc_w_cte", _w_cte, 100.0);
            private_nh.param("mpc_w_epsi", _w_epsi, 100.0);
            private_nh.param("mpc_w_vel", _w_vel, 100.0);
            private_nh.param("mpc_w_delta", _w_delta, 100.0);
            private_nh.param("mpc_w_accel", _w_accel, 50.0);
            private_nh.param("mpc_w_delta_d", _w_delta_d, 0.0);
            private_nh.param("mpc_w_accel_d", _w_accel_d, 0.0);
            private_nh.param("mpc_max_steering", _max_steering, 0.523); // Maximal steering radian (~30 deg)
            private_nh.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel
            private_nh.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

            //Init parameters for MPC object
            _mpc_params["DT"] = _dt;
            _mpc_params["LF"] = _Lf;
            _mpc_params["STEPS"]    = _mpc_steps;
            _mpc_params["REF_CTE"]  = _ref_cte;
            _mpc_params["REF_EPSI"] = _ref_epsi;
            _mpc_params["REF_V"]    = _ref_vel;
            _mpc_params["W_CTE"]    = _w_cte;
            _mpc_params["W_EPSI"]   = _w_epsi;
            _mpc_params["W_V"]      = _w_vel;
            _mpc_params["W_DELTA"]  = _w_delta;
            _mpc_params["W_A"]      = _w_accel;
            _mpc_params["W_DDELTA"] = _w_delta_d;
            _mpc_params["W_DA"]     = _w_accel_d;
            _mpc_params["MAXSTR"]   = _max_steering;
            _mpc_params["MAXTHR"]   = _max_throttle;
            _mpc_params["BOUND"]    = _bound_value;
            mpc_solver.LoadParams(_mpc_params);

            if(_debug_info)
            {
                //Display the parameters
                cout << "\n===== Parameters =====" << endl;
                cout << "max_speed: "  << _max_speed << endl;
                cout << "min_speed: "  << _min_speed << endl;
                cout << "debug_info: "  << _debug_info << endl;
                cout << "delay_mode: "  << _delay_mode << endl;
                cout << "vehicle_Lf: "  << _Lf << endl;
                cout << "frequency: "   << _dt << endl;
                cout << "mpc_steps: "   << _mpc_steps << endl;
                cout << "mpc_ref_vel: " << _ref_vel << endl;
                cout << "mpc_w_cte: "   << _w_cte << endl;
                cout << "mpc_w_epsi: "  << _w_epsi << endl;
                cout << "mpc_max_steering: "  << _max_steering << endl;
            }

            //Parameter for topics & Frame name
            tfBuffer_ = tfBuffer;
            tfListener_ = new tf2_ros::TransformListener(*tfBuffer);
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);

            // make sure to update the costmap we'll use for this cycle
            costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
            planner_util_.initialize(tfBuffer_, costmap, costmap_ros_->getGlobalFrameID());

            if( private_nh.getParam( "odom_topic", odom_topic_ ))
            {
                odom_helper_.setOdomTopic( odom_topic_ );
            }
            // odom_helper_.setOdomTopic( odom_topic_ );

            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool MPCLocalPlannerMBROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel){

        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
            ROS_ERROR("MPC Could not get local plan");
            return false;
        }

        //if the global plan passed in is empty... we won't do anything
        if(transformed_plan.empty()) {
            ROS_WARN_NAMED("mpc_local_planner", "Received an empty transformed plan.");
            return false;
        }
        // ROS_FATAL_NAMED("mpc_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size()) ;

        if (isGoalReached())
        {
            //publish an empty plan because we've reached our goal position
            publishZeroVelocity();
            transformed_plan.clear();

        }
        else
        {
            if(!transformed_plan.empty() && _goal_received){
                bool isOk = mpcComputeVelocityCommands(transformed_plan, cmd_vel);
                if (isOk)
                {
                    //publishGlobalPlan(transformed_plan);
                }
                else
                {
                    ROS_WARN_NAMED("mpc_local_planner", "mpc planner failed to produce path.");
                    std::vector<geometry_msgs::PoseStamped> empty_plan;
                    //publishGlobalPlan(empty_plan);
                }
                return isOk;

            }else{
                if(transformed_plan.empty()){
                    ROS_WARN_NAMED("mpc_local_planner", "MPC tries to compute vel without a reference global plan.");
                }
                if(!_goal_received){
                    ROS_WARN_NAMED("mpc_local_planner", "MPC tries to compute vel without a received goal.");
                }

            }

        }
    }

    bool MPCLocalPlannerMBROS::mpcComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped> &path, geometry_msgs::Twist &cmd_vel){

        // Get robot pose
        geometry_msgs::PoseStamped robot_pose;
        costmap_ros_->getRobotPose(robot_pose);
        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);
        // Eigen::Vector3f vel(robot_vel.pose.position.x,
        //                     robot_vel.pose.position.y, tf::getYaw(robot_vel.getRotation()));
        const double v = robot_vel.pose.position.x;


        // Display the MPC reference trajectory in odom coordinate
        nav_msgs::Path _mpc_ref_traj;
        _mpc_ref_traj.header.frame_id = "odom";
        _mpc_ref_traj.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped tempPose;
        tempPose.header = _mpc_ref_traj.header;
        for(int i = 0; i < path.size(); i++)
        {
            tempPose.pose = path[i].pose;
            _mpc_ref_traj.poses.push_back(tempPose);
        }
        _pub_ref_path_odom.publish(_mpc_ref_traj);

        nav_msgs::Odometry odom;
        odom_helper_.getOdom(odom);
        double px = robot_pose.pose.position.x; //pose: odom frame
        double py = robot_pose.pose.position.y;

        // cout << "px: " << px << endl;
        // cout << "py: " << py << endl;
        // tf::Pose pose;
        // tf::poseMsgToTF(odom.pose.pose, pose);
        // double psi = tf::getYaw(pose.getRotation());
        // double psi = tf2::getYaw(odom.pose.pose.orientation);
        double psi = tf2::getYaw(robot_pose.pose.orientation);

        const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        // Waypoints related parameters
        double cospsi = cos(psi);
        double sinpsi = sin(psi);
        // Convert to the vehicle coordinate system
        std::vector<double> waypoints_x;
        std::vector<double> waypoints_y;
        waypoints_x.clear();
        waypoints_y.clear();

        // Display the MPC reference trajectory in odom coordinate
        nav_msgs::Path _vehicle_ref_traj;
        _vehicle_ref_traj.header.frame_id = "base_link"; // points in car coordinate
        _vehicle_ref_traj.header.stamp = ros::Time::now();
        tempPose.header = _vehicle_ref_traj.header;
        for(int i = 0; i < path.size(); i++)
        {
            double dx = path[i].pose.position.x - px;
            double dy = path[i].pose.position.y - py;
            waypoints_x.push_back( dx * cospsi + dy * sinpsi);
            waypoints_y.push_back( dy * cospsi - dx * sinpsi);
            tempPose.pose.position.x = dx * cospsi + dy * sinpsi;
            tempPose.pose.position.y = dy * cospsi - dx * sinpsi;
            _vehicle_ref_traj.poses.push_back(tempPose);
        }
        _pub_ref_path_baselink.publish(_vehicle_ref_traj);
        int size_of_path = waypoints_x.size();
        if(isGoalReached()){
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            return true;

        }
        double* ptrx = &waypoints_x[0];
        double* ptry = &waypoints_y[0];
        Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, size_of_path);
        Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, size_of_path);
        // calculate cte and epsi
        auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
        double cte = polyeval(coeffs, 0);
        double epsi = atan(coeffs[1]);
        if (_debug_info){
            cout<<"psi is"<<endl;
            cout<<"path size is"<<path.size()<<endl;
            cout<<"waypoints x size is"<<waypoints_x.size()<<endl;
            cout<<"coeffs is "<<coeffs<<endl;
            cout<<"cte is"<<cte<<endl;
            cout<<"epsi is"<<epsi<<endl;
        }
        Eigen::VectorXd state(6);
        if(_delay_mode)
        {
            // Kinematic model is used to predict vehicle state at the actual
            // moment of control (current time + delay dt)
            const double px_act = v * _dt;
            const double py_act = 0;
            const double psi_act = v * steering * _dt / _Lf;
            const double v_act = v + throttle * _dt;
            const double cte_act = cte + v * sin(epsi) * _dt;
            const double epsi_act = -epsi + psi_act;
            state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, epsi;
        }
        // state << 0, 0, 0, v, cte, epsi;

        // Solve MPC Problem
        vector<double> mpc_results = mpc_solver.Solve(state, coeffs);

        // MPC result (all described in car frame)
        _steering = mpc_results[0]; // radian
        _throttle = mpc_results[1]; // acceleration
        _speed = v + _throttle*_dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if (_speed < _min_speed)
            _speed = _min_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        if(_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "psi: " << psi << endl;
            cout << "V: " << v << endl;
            //cout << "odom_path: \n" << odom_path << endl;
            //cout << "x_points: \n" << x_veh << endl;
            //cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_steering: \n" << _steering << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
        }

        // Display the MPC predicted trajectory
        nav_msgs::Path _mpc_predi_traj;
        _mpc_predi_traj.header.frame_id = "base_link"; // points in car coordinate
        _mpc_predi_traj.header.stamp = ros::Time::now();
        for(int i=0; i<mpc_solver.mpc_x.size(); i++)
        {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_predi_traj.header;
            tempPose.pose.position.x = mpc_solver.mpc_x[i];
            tempPose.pose.position.y = mpc_solver.mpc_y[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_predi_traj.poses.push_back(tempPose);
        }
        // publish the mpc trajectory
        _pub_mpc_traj.publish(_mpc_predi_traj);

        cmd_vel.linear.x = _speed;
        cmd_vel.angular.z = _steering;
        // _pub_twist.publish(cmd_vel);
        return true;
    }

    bool MPCLocalPlannerMBROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if(!initialized_){
            ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
            return false;
        }
        return planner_util_.setPlan(orig_global_plan);
    }


    // CallBack: Update goal status
    void MPCLocalPlannerMBROS::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
    {
        _goal_pos = goalMsg->pose.position;
        _goal_received = true;
        _goal_reached = false;
        ROS_INFO_NAMED("mpc_local_planner", "MPC received a Goal.");

    }

    bool MPCLocalPlannerMBROS::isGoalReached(){
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        bool ret = isInGoalArea();
        if(ret) {
            publishZeroVelocity();
            ROS_INFO_NAMED("mpc_local_planner", "MPC reached the Goal.");
            return true;
        } else {
            return false;
        }

    }

    void MPCLocalPlannerMBROS::publishZeroVelocity(){
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      _pub_twist.publish(cmd_vel);
    }

    // Public: return _thread_numbers
    int MPCLocalPlannerMBROS::get_thread_numbers()
    {
        return _thread_numbers;
    }


    bool MPCLocalPlannerMBROS::isInGoalArea()
    {
       costmap_ros_->getRobotPose(current_pose_);

        if(_goal_received)
        {
            double car2goal_x = _goal_pos.x - current_pose_.pose.position.x;
            double car2goal_y = _goal_pos.y - current_pose_.pose.position.y;
            double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
            if(dist2goal < _goalRadius)
            {
                _goal_reached = true;
                _goal_received = false;
                // ROS_INFO("Goal Reached !");
                publishZeroVelocity();
                return true;
            }
        }
        return false;
    }


    double MPCLocalPlannerMBROS::polyeval(Eigen::VectorXd coeffs, double x){
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    Eigen::VectorXd MPCLocalPlannerMBROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                                                   int order){
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++) {
            A(i, 0) = 1.0;
        }

        for (int j = 0; j < xvals.size(); j++) {
            for (int i = 0; i < order; i++) {
                A(j, i + 1) = A(j, i) * xvals(j);
            }
        }
        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    void MPCLocalPlannerMBROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, l_plan_pub_);
    }

    void MPCLocalPlannerMBROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "mpc_local_planner_node");


 ROS_INFO("Waiting for global path msgs ~");
 ros::AsyncSpinner spinner(4); // Use multi threads
 spinner.start();
 ros::waitForShutdown();
 return 0;
}
