#ifndef MCP_LOCAL_PLANNER_MB_MPC_LOCAL_PLANNER_H
#define MCP_LOCAL_PLANNER_MB_MPC_LOCAL_PLANNER_H
#include <vector>
#include <math.h>
#include <mpc_local_planner_mb/mpc_local_planner_mb.h>
#include <mpc_local_planner_mb/MPC.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/local_planner_util.h>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/QR>

using namespace std;
using namespace Eigen;
namespace mpc_local_planner_mb {
    class MPCLocalPlannerMBROS : public nav_core::BaseLocalPlanner{
    public:
        /**
         * @brief  Constructor for mpc path follower wrapper
         */
        MPCLocalPlannerMBROS();
        /**
         * @brief  Destructor for the wrapper
         */
        ~MPCLocalPlannerMBROS();
        /**
         * @brief  Constructs the ros wrapper
         * @param name The name to give this instance of the trajectory planner
         * @param tf A pointer to a transform listener
         * @param costmap The cost map to use for assigning costs to trajectories
         */
        void initialize(std::string name, tf2_ros::Buffer* tfBuffer,
                        costmap_2d::Costmap2DROS* costmap_ros);
        /**
         * @brief  Given the current position, orientation, and velocity of the robot,
         * compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid trajectory was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Set the plan that the controller is following
         * @param orig_global_plan The plan to pass to the controller
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        /**
         * @brief  Check if the goal pose has been achieved
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();
        int get_thread_numbers();

        bool isInitialized() {
            return initialized_;
        }

    private:

        void publishZeroVelocity();
        /**
         * @brief evaluate a polynominal
         * @param coefficients and input
         * @return output of the polynominal
         */
        int ClosestWaypoint(double x, double y, nav_msgs::Path global_path)
        {}
        /**
         * @brief evaluate a polynominal
         * @param coefficients and input
         * @return output of the polynominal
         */
        double polyeval(Eigen::VectorXd coeffs, double x);

        /**
         * @brief fit a polynominal
         * @param vector x, vector y and order
         * @return output coefficients
         */
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order);

        bool mpcComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::Twist& cmd_vel );

        void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        bool isInGoalArea();

        // tf::TransformListener* tf_; ///< @brief Used for transforming point clouds
        // tf::TransformListener _tf_listener;

        // for visualisation, publishers of global and local plan
        ros::Publisher g_plan_pub_, l_plan_pub_;
        // ros::Subscriber odom_sub;
        // ros::Subscriber _sub_odom, _sub_path, _sub_goal, _sub_amcl;


        ros::Publisher _pub_ref_path_odom, _pub_mpc_traj, _pub_ref_path_baselink, _pub_twist;
        ros::Subscriber _sub_goal;

        // costmap_2d::Costmap2DROS* costmap_ros_;

        geometry_msgs::Point _goal_pos;
        std::string odom_topic_ = "/odom";

        // bool setup_;
        // string _globalPath_topic, _goal_topic;
        // string _map_frame, _odom_frame, _car_frame;
        string _goal_topic;


        // tf::Stamped<tf::Pose> current_pose_;
        geometry_msgs::PoseStamped current_pose_;

        tf2_ros::Buffer* tfBuffer_;
        tf2_ros::TransformListener* tfListener_;
        //! costmap ROS wrapper ptr
        costmap_2d::Costmap2DROS* costmap_ros_;
        // std::string global_frame;

        bool initialized_;
        // bool debug_;

        base_local_planner::OdometryHelperRos odom_helper_;

        base_local_planner::LatchedStopRotateController latchedStopRotateController_;

        std::vector<geometry_msgs::PoseStamped> global_plan_;

        base_local_planner::LocalPlannerUtil planner_util_;
        // ros::Publisher _pub_odompath, _pub_twist, _pub_ackermann, _pub_mpctraj;


        // Eigen::Vector3f vel;
        // float  DT;
        // float pathLength_;
        MPC mpc_solver;
        map<string, double> _mpc_params;
        double _mpc_steps, _ref_cte, _ref_epsi, _ref_vel, _w_cte, _w_epsi, _w_vel,
        _w_delta, _w_accel, _w_delta_d, _w_accel_d, _max_steering, _max_throttle, _bound_value;
        double _Lf, _dt, _steering, _throttle, _speed, _max_speed, _goalRadius;
        // double _pathLength, _goalRadius, _waypointsDist;
        // int _controller_freq, _downSampling, _thread_numbers;
        int _controller_freq, _thread_numbers;
        bool _goal_received, _goal_reached, _debug_info, _delay_mode;

        // bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;




    };
}; //MCP_LOCAL_PLANNER_MB_MPC_LOCAL_PLANNER_H
#endif
