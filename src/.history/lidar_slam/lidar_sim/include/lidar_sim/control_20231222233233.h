#ifndef CONTROL
#define CONTROL


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>

#include "/home/lcx/Fast-Drone-250/devel/include/lidar_sim/shot.h"
#include "/home/lcx/Fast-Drone-250/devel/include/lidar_sim/yolo_msg.h"
#include "/home/lcx/Fast-Drone-250/devel/include/lidar_sim/yolo_msg_array.h"

#include "/home/lcx/Fast-Drone-250/devel/include/lidar_sim/goal.h"
#include "sensor_msgs/Image.h"
#include <numeric>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <tf/tf.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Path {
public:
    ros::Time prev_time;

    int flag_enemy;

    double distance_to_target;
    double A_distance_to_target;

    // double roll, pitch, yaw;

    double dx, dy;
    double target_x, target_y;
    double diss;
    double diss_A;
    double diss_B;

    double current_x;
    double current_y;
    double current_z;

    double _diss_x,_diss_y,_diss_z;
    double vs_current_x;
    double vs_current_y;
    double vs_current_z;
    double vs_current_yaw;


    double diss_x;
    double diss_y;
    double diss_z;

    double current_time;
    double elapsed_time;
    double start_time;
    double Grew1_z;
    double Grew2_z;
    double Tweak1_z;
    double Tweak2_z;
    double Flying;

    double Z_1;
    double Z_2;

    double _A_x;
    double _A_y;

    double _B_x;
    double _B_y;

    std_msgs::Float64 altitude;
    std_msgs::Float64 cx;
    std_msgs::Float64 cy;

    std::string label;
    std::string enemy;
    std::string enemy_true;

    std::vector<std::vector<double>> depthes;
    int minIndex;
    std::vector<int32_t> desire_label;
    int32_t sum;
    std::vector<std::vector<int32_t>> circle_coordinates;
    std::vector<std::vector<int32_t>> label_coordinates;
    std::vector<int32_t> center_points;

    // Eigen::MatrixXd line_strip;
    std::vector<geometry_msgs::Point> line_strip;
    double yaw_true;
    double picth_true;
    std::string yolo_label;
    // std::vector<double> depth;
    lidar_sim::goal depth;

    // double depth;
    // double yaw;
    int32_t center_x;
    int32_t center_y;

    const float pixel_w=640.0;
    const float pixel_h=480.0;
    const float FOV_w=56.0*M_PI/180.0;
    const float FOV_h=60.0*M_PI/180.0;  

    float j,k;
    float g,h;

    uint32_t id;
    int32_t task;
    bool is_distance_updated; //flase

    double threshold; //0.15

    uint8_t data[4];
    char buf[8];

    struct ARM{
        int8_t ContractAngle;
        int8_t Openangle;
        int8_t Descendangle;
    }Arm_Angle;

    int16_t Point;

    //遍历除法减小速度
    std::vector<int> divisors;

    enum flyState{
        Fly,
        Pub_A,
        Move_A,
        Fixed_A,
        Detect,
        Move_B,
        Land,
        A2D,
        D2B,
        B2D,
        D2C,
        C2H,
        Circle_1,
        Circle_2,
        Circle_2_3,
        Circle_2_2_3,

        Circle_3,
        Circle_4,
        Circle_5,
        Circle_6,
        Circle_7,
        Circle_8,
        Circle_9,
        Circle_10,
        Circle_11,
        Circle_12,
        Circle_13        
    };

    enum grewState{
        Finding,
        Prepare,
        Grew,
    };

    flyState FlyState;
    grewState GrewState;
    // Path() : mbc_("move_base", true){
    // }
    Path(): mbc_("move_base", true){
        int32_t task = 0;
        int flag_enemy = 0;
        threshold = 0.2;
        bool is_distance_updated = false;

        Z_1 = 1.5;
        Z_2 = 3;

        _A_x = 3.2;
        _A_y = 3.2;

        _B_x = 0;
        _B_y = 0;

        g = 0;

        yaw_true = 0;

        minIndex = -1;

        ros::NodeHandle nh;

        // 创建发布者对象
        path_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 20);
        shoot_pub= nh.advertise<lidar_sim::shot>("/data/shot", 10);
        depth_goal_pub= nh.advertise<lidar_sim::goal>("/depth_goal_pose", 10);

        // path_pub = nh.advertise<nav_msgs::Path>("path", 10);
        // flag_pub = nh.advertise<std_msgs::Int32>("/flag", 10);

        altitude_sub = nh.subscribe("/mavros/altitude", 10, &Path::altitudeCallback, this);
        cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &Path::CmdVelCallback, this);
        // plan_sub = nh.subscribe<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 10, &Path::PlanCallback, this);

        state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &Path::stateCallback, this);
        local_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &Path::localCallback, this);

        yolo_sub = nh.subscribe<lidar_sim::yolo_msg_array>("/CvAnswer", 10, &Path::YoloCallback, this);

        depth_sub = nh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 10, &Path::DepthCallback, this);

        plan_odom_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 1, &Path::plan_odomCallback, this);

        // plan_odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/odometry", 1, &Path::plan_odomCallback, this);

        a_star_list_sub = nh.subscribe<visualization_msgs::Marker>("/drone_0_ego_planner_node/a_star_list", 10, &Path::a_star_listCallback, this);

        // yolo_sub = nh.subscribe("/CvAnswer", 10, boost::bind(&Path::YoloCallback, this, _1));

        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        FlyState = Fly;
        
        GrewState = Finding;

        nh.param("Flying", Flying, 0.5); 
        nh.param("Grew1_z", Grew1_z, 0.3); 
        nh.param("Grew2_z", Grew2_z, 0.2);
        nh.param("Tweak1_z", Tweak1_z, 0.05); 
        nh.param("Tweak2_z", Tweak2_z, 0.01); 
    }
    void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void DepthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void SetGoal(double x, double y,double z);
    void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

    void a_star_listCallback(const visualization_msgs::Marker::ConstPtr& msg);
    void YoloCallback(const lidar_sim::yolo_msg_array::ConstPtr& msg);

    void localCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void plan_odomCallback(const nav_msgs::Odometry::ConstPtr& msg);


    // void PlanCallback(const nav_msgs::Path::ConstPtr& msg);
    void arm();
    void disarm();
    void setMode(std::string mode);
    void DetectAndShoot();


    std::pair<double, double> TaskDetection(int target_id);

     //起飞
    void _Fly();
    void _Fly1();
    void _Fly2();


    void _H2D();
    void _grew_D();

    void action2();
    void action3();
    void fixed(double yawValue);

    void run();

private:
    // 创建消息对象
    mavros_msgs::PositionTarget PositionTarget;
    mavros_msgs::PositionTarget PathTarget;
    mavros_msgs::PositionTarget PlanTarget;

    mavros_msgs::SetMode land_cmd;

    lidar_sim::shot desir_shot;

    mavros_msgs::State current_state;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Publisher path_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher shoot_pub;
    ros::Publisher depth_goal_pub;

    ros::Subscriber state_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber altitude_sub;
    ros::Subscriber detection_sub;
    ros::Subscriber task_sub;
    ros::Subscriber local_sub;
    ros::Subscriber plan_sub;
    ros::Subscriber yolo_sub;
    ros::Subscriber depth_sub;
    ros::Subscriber plan_odom_sub;
    ros::Subscriber a_star_list_sub;

    move_base_msgs::MoveBaseGoal goal;

    // 节点文件

    std::map<int, std::pair<double, double>> object_map;
    MoveBaseClient mbc_;

};
#endif





