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

// #include <tf/tf.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Path {
public:
    ros::Time prev_time;
    double distance_to_target;
    double roll, pitch, yaw;

    double dx, dy;
    double target_x, target_y;
    double diss;
    double diss_A;
    double diss_B;
    double current_x;
    double current_y;
    double current_z;
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

    double Z_target;


    std_msgs::Float64 altitude;
    std_msgs::Float64 cx;
    std_msgs::Float64 cy;

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
        Move_B,
        Land,
        A2D,
        D2B,
        B2D,
        D2C,
        C2H,
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
        
        threshold = 0.2;
        bool is_distance_updated = false;

        Z_target = 1;
        ros::NodeHandle nh;

        // 创建发布者对象
        path_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 20);
        // path_pub = nh.advertise<nav_msgs::Path>("path", 10);
        // flag_pub = nh.advertise<std_msgs::Int32>("/flag", 10);


        altitude_sub = nh.subscribe("/mavros/altitude", 10, &Path::altitudeCallback, this);
        cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &Path::CmdVelCallback, this);
        plan_sub = nh.subscribe<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 10, &Path::PlanCallback, this);

        state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &Path::stateCallback, this);
        local_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &Path::localCallback, this);

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

    void SetGoal(double x, double y,double z);
    void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

    void localCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void PlanCallback(const nav_msgs::Path::ConstPtr& msg);
    void arm();
    void disarm();
    void setMode(std::string mode);


    std::pair<double, double> TaskDetection(int target_id);

     //起飞
    void _Fly();
    void _H2D();
    void _grew_D();

    void action2();
    void action3();
    void fixed();
    void run();

private:
    // 创建消息对象
    mavros_msgs::PositionTarget PositionTarget;
    mavros_msgs::PositionTarget PathTarget;
    mavros_msgs::PositionTarget PlanTarget;

    mavros_msgs::SetMode land_cmd;


    mavros_msgs::State current_state;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Publisher path_pub;
    ros::Publisher trajectory_pub;

    ros::Subscriber state_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber altitude_sub;
    ros::Subscriber detection_sub;
    ros::Subscriber task_sub;
    ros::Subscriber local_sub;
    ros::Subscriber plan_sub;

    move_base_msgs::MoveBaseGoal goal;

    // 节点文件

    std::map<int, std::pair<double, double>> object_map;
    MoveBaseClient mbc_;

};
#endif





