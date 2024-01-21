#include "../include/lidar_sim/control.h"

#include <std_msgs/Int32.h>


void Path::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    double linear_x = msg->linear.x;
    double linear_y = msg->linear.y;
    double linear_z = msg->linear.z;
    double angular_z = msg->angular.z;

    PathTarget.position.z = Z_1;
    
    PathTarget.velocity.x = linear_x;
    PathTarget.velocity.y = linear_y;
    PathTarget.velocity.z = linear_z;
    PathTarget.yaw_rate = angular_z;

    PathTarget.coordinate_frame = PathTarget.FRAME_LOCAL_NED;
    // PathTarget.coordinate_frame = 1;

    PathTarget.type_mask = PathTarget.IGNORE_PX |
                           PathTarget.IGNORE_PY |
                        //    PathTarget.IGNORE_PZ |
                        //    PathTarget.IGNORE_VZ |
                           PathTarget.IGNORE_AFX|
                           PathTarget.IGNORE_AFY|
                           PathTarget.IGNORE_AFZ|
                           PathTarget.IGNORE_YAW;
    // std::cout << "2222"  << std::endl;
}

// void Path::DepthCallback(const sensor_msgs::Image::ConstPtr& msg){
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try {
//         cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
//     } catch (cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     // 访问深度值
//     uint16_t depth_value = cv_ptr->image.at<uint16_t>(center_x, center_y);
//     std::cout << depth_value << std::endl;
// }

void Path::DepthCallback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // for (int i = 0; i < circle_coordinates.size(); i++){
        if (minIndex > -1){
            std::cout<< minIndex << std::endl;
            std::vector<int32_t> coordinate = circle_coordinates[minIndex];
            // std::vector<int32_t> coordinate = circle_coordinates[i];

            // std::cout << minIndex << coordinate[0] <<  "  "  << coordinate[1] <<  "  "  <<coordinate[2] <<  "  "  << coordinate[3] <<  "  "  <<std::endl;
            // std::cout << coordinate[0] <<  "  "  << coordinate[1] <<  "  "  <<coordinate[2] <<  "  "  << coordinate[3] <<  "  "  <<std::endl;

        // for (int j = 0; j < coordinate.size(); ++j) {
            // circle_coordinates.clear();
            int radius = (std::min((coordinate[2] - coordinate[0]), (coordinate[3] - coordinate[1])) / 2);
            if (radius < 320 && radius > 0){
                    
                    std::cout << "radius " << radius << std::endl;
                    // 分析圆环边缘的深度值
                    std::vector<uint16_t> depth_values;
                    center_x = (coordinate[2] + coordinate[0])/2;
                    center_y = (coordinate[3] + coordinate[1])/2;
                    std::cout << "center" << center_x << "  " << center_y << std::endl;
                    for (double angle = 0; angle < 360; angle += 1.0) {
                        int x = center_x + radius * cos(angle * M_PI / 180.0);
                        int y = center_y + radius * sin(angle * M_PI / 180.0);
                        depth_values.push_back(cv_ptr->image.at<uint16_t>(y, x));
                    }

                    std::vector<uint16_t> filtered_depth_values;
                    for (auto depth : depth_values) {
                        if (depth < 7000) {
                            filtered_depth_values.push_back(depth);
                        }
                    }
                    if (filtered_depth_values.empty()) {
                            std::cout << "No depth values less than 7." << std::endl;
                        } else {
                        // 计算平均或中值深度
                        uint16_t average_depth = std::accumulate(filtered_depth_values.begin(), filtered_depth_values.end(), 0) / filtered_depth_values.size();
                        std::nth_element(filtered_depth_values.begin(), filtered_depth_values.begin() + filtered_depth_values.size() / 2, filtered_depth_values.end());
                        uint16_t median_depth = filtered_depth_values[filtered_depth_values.size() / 2];

    // std::nth_element(filtered_depth_values.begin(), filtered_depth_values.begin() + filtered_depth_values.size() / 2, filtered_depth_values.end());
    // uint16_t median_depth_less_than_7 = filtered_depth_values[filtered_depth_values.size() / 2];
                        // std::cout << "Average Depth: " << average_depth << std::endl;
                        // std::cout << "Median Depth: " << median_depth << std::endl;

                        double f_x = 383.10565185546875; // X轴的焦距
                        double f_y = 383.10565185546875; // Y轴的焦距
                        double c_x = 314.6989440917969; // 光学中心的X坐标
                        double c_y = 239.00680541992188; // 光学中心的Y坐标

                        double X = median_depth * 0.001; // 假设深度值是以毫米为单位

                        double Y = (c_x - center_x) * X / f_x ;
                        double Z = (c_y - center_y) * X / f_y;

                        X = X + vs_current_x;
                        Y = Y + vs_current_y;
                        Z = Z + vs_current_z;

                        std::cout << "Object Position: X = " << X << ", Y = " << Y << ", Z = " << Z << std::endl;

                        // depth.push_back(X);
                        // depth.push_back(Y);
                        // depth.push_back(Z);
                        depth.x = X;
                        depth.y = Y;
                        depth.z = Z;
                        minIndex = -1;
                        // depthes.push_back(depth);
                        }
        }
        }
}

void Path::YoloCallback(const lidar_sim::yolo_msg_array::ConstPtr& msg){
    if (msg->yolo_array.size() > 0) {
        int size = msg->yolo_array.size();

        for (int i = 0; i < size; i++) {
            if (yolo_label == msg->yolo_array[i].label ) {          
                desire_label.push_back(msg->yolo_array[i].x1);
                desire_label.push_back(msg->yolo_array[i].y1);
                desire_label.push_back(msg->yolo_array[i].x2);
                desire_label.push_back(msg->yolo_array[i].y2);
                // yolo_label = "";
                std::cout << "desire_label  " << desire_label[0] << 
                "  " << desire_label[1] <<
                "  " << desire_label[2] <<
                "  " << desire_label[3] << std::endl;
                };


                // label_coordinates.push_back(new_coordinate);

                // center_points.push_back(msg->yolo_array[i].center_x);
                // center_points.push_back(msg->yolo_array[i].center_y);
            if (msg->yolo_array[i].label == "quan" || msg->yolo_array[i].label == "kuang"){
                std::vector<int32_t> new_coordinate{
                    msg->yolo_array[i].x1,
                    msg->yolo_array[i].y1,
                    msg->yolo_array[i].x2,
                    msg->yolo_array[i].y2
                };
                circle_coordinates.push_back(new_coordinate);
            }    
        }

        // double minDistance = std::numeric_limits<double>::max(); // 初始化为一个较大的值，确保第一次比较时肯定小于
        // // minIndex = 0; // 用于保存最小值对应的索引

        // for (int i = 0; i < circle_coordinates.size(); ++i)
        // {
        //     double sum = 0.0;
        //     std::cout << "22222" << circle_coordinates.size() <<std::endl;
        //     // 计算平方和
        //     for (int j = 0; j < circle_coordinates[i].size(); ++j)
        //     {
        //         sum += std::pow(circle_coordinates[i][j] - desire_label[j], 2);
        //     }

        //     // 更新最小值和对应的索引
        //     if (sum < minDistance)
        //     {
        //         minDistance = sum;
        //         minIndex = i;
        //     }
        // }

        double minDistance = std::numeric_limits<double>::max();
        // int minIndex = 0; // Ensure this is initialized

        if (!circle_coordinates.empty() && !desire_label.empty()) {
            for (int i = 0; i < circle_coordinates.size(); ++i) {
                double sum = 0.0;
                std::cout << "Circle " << i << " size: " << circle_coordinates[i].size() << ", Desire label size: " << desire_label.size() << std::endl;

                // Check for size mismatch
                if (circle_coordinates[i].size() != desire_label.size()) {
                    std::cerr << "Size mismatch at index " << i << std::endl;
                    continue; // Skip this iteration
                }

                // Calculate the sum of squares
                for (int j = 0; j < circle_coordinates[i].size(); ++j) {
                    sum += std::pow(circle_coordinates[i][j] - desire_label[j], 2);
                }

                // Update the minimum distance and index
                if (sum < minDistance) {
                    minDistance = sum;
                    minIndex = i;
                }
            }
            desire_label.clear();
        } else {
            // std::cerr << "circle_coordinates or desire_label is empty" << std::endl;
        }

    }
    }

void Path::DetectAndShoot(){
    // std::cout << " label" << label << std::endl;

    // std::cout << " _coordinate0  " << float(coordinate[0]) << std::endl;
    // std::cout << " _coordinate1  " << float(coordinate[1]) << std::endl;
    // std::cout << " _coordinate2  " << float(coordinate[2])<< std::endl;
    // std::cout << " _coordinate3  " << float(coordinate[3]) << std::endl;
    // std::cout << "enemy_true " << enemy_true << std::endl;
    // if (label == enemy_true && !label.empty()){
    // if (label == "A" && !label.empty()){

    // // if (label == "C"){

    //     j=(coordinate[0] + coordinate[2]) * 0.5 - 0.5 * pixel_w;
    //     // k=coordinate[3] * 1.5 - coordinate[0] * 0.5 - 0.5 * pixel_h;          //目标点坐标（j，k）

    //     g=atan(j/(pixel_w*0.5)*tan(FOV_w*0.5));
    //     // h=atan(k/(pixel_h*0.5)*tan(FOV_h*0.5));

    //     yaw_true = -g * 180 / 3.14;
    //     // picth_true = h;
    //     std::cout << " yaw_true " << yaw_true << std::endl;
    // }
}

// void Path::PlanCallback(const nav_msgs::Path::ConstPtr& msg){
//     if (!msg->poses.empty()){
//         if (msg->poses.size() > 20){
//             geometry_msgs::PoseStamped first_pose = msg->poses[20];
//             // PlanTarget.yaw = yaw;
//             PlanTarget.position.z = Z_1;
//             PlanTarget.position.x = first_pose.pose.position.x;
//             PlanTarget.position.y = first_pose.pose.position.y;

//             // double yaw = tf::getYaw(first_pose.pose.orientation);
//             // PlanTarget.yaw = yaw;
//             // PlanTarget.position.z = first_pose.pose.position.z;
//             PlanTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
//             PlanTarget.type_mask=PositionTarget.IGNORE_VX|
//                                 PositionTarget.IGNORE_VY|
//                                 PositionTarget.IGNORE_VZ|
//                                 PositionTarget.IGNORE_AFX|
//                                 PositionTarget.IGNORE_AFY|
//                                 PositionTarget.IGNORE_AFZ|
//                                 PositionTarget.IGNORE_YAW|
//                                 PositionTarget.IGNORE_YAW_RATE;
//         }
//         else{
//             if (msg->poses.size() > 5){
//                 geometry_msgs::PoseStamped first_pose = msg->poses[msg->poses.size()/2];
//                 // PlanTarget.yaw = yaw;
//                 PlanTarget.position.z = Z_1;
//                 PlanTarget.position.x = first_pose.pose.position.x;
//                 PlanTarget.position.y = first_pose.pose.position.y;

//                 // double yaw = tf::getYaw(first_pose.pose.orientation);
//                 // PlanTarget.yaw = yaw;
//                 // PlanTarget.position.z = first_pose.pose.position.z;
//                 PlanTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
//                 PlanTarget.type_mask=PositionTarget.IGNORE_VX|
//                                     PositionTarget.IGNORE_VY|
//                                     PositionTarget.IGNORE_VZ|
//                                     PositionTarget.IGNORE_AFX|
//                                     PositionTarget.IGNORE_AFY|
//                                     PositionTarget.IGNORE_AFZ|
//                                     PositionTarget.IGNORE_YAW|
//                                     PositionTarget.IGNORE_YAW_RATE;
//             }
//             else{
//                 geometry_msgs::PoseStamped first_pose = msg->poses[msg->poses.size() - 1];
//                 // PlanTarget.yaw = yaw;
//                 PlanTarget.position.z = Z_1;
//                 PlanTarget.position.x = first_pose.pose.position.x;
//                 PlanTarget.position.y = first_pose.pose.position.y;

//                 // double yaw = tf::getYaw(first_pose.pose.orientation);
//                 // PlanTarget.yaw = yaw;
//                 // PlanTarget.position.z = first_pose.pose.position.z;
//                 PlanTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
//                 PlanTarget.type_mask=PositionTarget.IGNORE_VX|
//                                     PositionTarget.IGNORE_VY|
//                                     PositionTarget.IGNORE_VZ|
//                                     PositionTarget.IGNORE_AFX|
//                                     PositionTarget.IGNORE_AFY|
//                                     PositionTarget.IGNORE_AFZ|
//                                     PositionTarget.IGNORE_YAW|
//                                     PositionTarget.IGNORE_YAW_RATE;
//             }
//         }
//     }
//     else{
//         std::cout << "no more massages" << diss_A << std::endl;

//     }
  
// };
void Path::SetGoal(double x, double y,double z){
    goal.target_pose.header.frame_id = "map";  // 坐标系（frame_id）根据实际情况设置
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;    // 设置目标点的X坐标
    goal.target_pose.pose.position.y = y;    // 设置目标点的Y坐标
    goal.target_pose.pose.orientation.x = 0.0; // 设置目标点的姿态四元数
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1; // 设置目标点的姿态，这里是单位四元数表示

    ROS_INFO("发布导航目标点...");
    mbc_.sendGoal(goal);
}
//获取高度数据
void Path::altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg) {
    altitude.data = msg->local;
    // ROS_INFO("Current altitude: %.2f meters", altitude.data);
}

void Path::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void Path::plan_odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // 获取机器人的姿态信息
    geometry_msgs::Pose pose = msg->pose.pose;

    // 提取机器人的位置信息
    geometry_msgs::Point position = pose.position;

    // 提取机器人的四元数表示的姿态信息
    geometry_msgs::Quaternion orientation = pose.orientation;

    // 将四元数转换为欧拉角（roll、pitch、yaw）
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    vs_current_x = position.x;
    vs_current_y = position.y;
    vs_current_z = position.z;
    vs_current_yaw = yaw;

    
    _diss_x = PlanTarget.position.x - vs_current_x;
    _diss_y = PlanTarget.position.y - vs_current_y;
    _diss_z = PlanTarget.position.z - vs_current_z;

    // yaw = msg->yaw;

    // 计算与目标位置的距离
    A_distance_to_target = std::sqrt(std::pow(PlanTarget.position.x - vs_current_x, 2) + 
    std::pow(PlanTarget.position.y - vs_current_y, 2) + 
    std::pow(PlanTarget.position.z - vs_current_z, 2));

    // diss_A = std::sqrt(std::pow(_A_x - current_x, 2) + std::pow(_A_y - current_y, 2));
    // diss_B = std::sqrt(std::pow(current_x - _B_x, 2) + std::pow(current_y - _B_y, 2));

    // std::cout << "dissA:" << diss_A << std::endl;
    // std::cout << "dissB:" << diss_A << std::endl;

    // 判断是否到达目标位置
    if (distance_to_target > (threshold + 0.1)) {
        // ROS_INFO("Arrived at target position!");
        // task+=1;
        // std::cout << "task:" << current_x << std::endl;
        is_distance_updated = true;
    }
    // 输出机器人的位置和 yaw 角度
    ROS_INFO("Robot's Current Position: x = %f, y = %f, z = %f", position.x, position.y, position.z);
    ROS_INFO("Robot's Yaw Angle: %f (in radians)", yaw);
}

void Path::localCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // 获取当前无人机的位置
    current_x = msg->pose.position.x;
    current_y = msg->pose.position.y;
    current_z = msg->pose.position.z;

    diss_x = PositionTarget.position.x - current_x;
    diss_y = PositionTarget.position.y - current_y;
    diss_z = PositionTarget.position.z - current_z;

    // yaw = msg->yaw;

    // 计算与目标位置的距离
    distance_to_target = std::sqrt(std::pow(PositionTarget.position.x - current_x, 2) + std::pow(PositionTarget.position.y - current_y, 2) + std::pow(PositionTarget.position.z - current_z, 2));

    diss_A = std::sqrt(std::pow(_A_x - current_x, 2) + std::pow(_A_y - current_y, 2));
    diss_B = std::sqrt(std::pow(current_x - _B_x, 2) + std::pow(current_y - _B_y, 2));

    std::cout << "dissA:" << distance_to_target << std::endl;
    std::cout << "dissB:" << distance_to_target << std::endl;

    // 判断是否到达目标位置
    if (distance_to_target > (threshold + 0.1)) {
        // ROS_INFO("Arrived at target position!");
        // task+=1;
        // std::cout << "task:" << current_x << std::endl;
        is_distance_updated = true;
    }
    // is_distance_updated = true;
}

void Path::arm() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) 
    {
        ROS_INFO("Vehicle D, success=%d", arm_cmd.response.success);
    }
    else
    {
        ROS_ERROR("Failed to arm vehicle");
        ROS_ERROR("Failed to arm vehicle, success=%d", arm_cmd.response.success);
    }
}

void Path::disarm() {
    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
    if (arming_client.call(disarm_cmd) && disarm_cmd.response.success) {
        ROS_INFO("Vehicle disarmed");
    }
    else {
        ROS_ERROR("Failed to disarm vehicle");
        }
}

void Path::setMode(std::string mode) {
    mavros_msgs::SetMode set_mode_cmd;
    set_mode_cmd.request.custom_mode = mode;
    if (set_mode_client.call(set_mode_cmd) && set_mode_cmd.response.mode_sent) {
        // ROS_INFO_STREAM("Mode changed to " << mode);
    }
    else {
        ROS_ERROR("Failed to set mode");
    }
}

std::pair<double, double> Path::TaskDetection(int target_id)
{
    for (const auto& object : object_map)
    {
        int id = object.first;
        double cx = object.second.first;
        double cy = object.second.second;

        if (id == target_id)
        {
            return std::make_pair(cx, cy);
        }
    }

    return std::make_pair(640.0, 360.0);
}


void Path::a_star_listCallback(const visualization_msgs::Marker::ConstPtr& msg){
    line_strip = msg->points;
    std::cout << "getpoint" << "    "<< line_strip.size() << std::endl;
    if (!msg->points.empty()){
        if (msg->points.size() > 5){
            // geometry_msgs::PoseStamped first_pose = msg->points[5];
            geometry_msgs::PoseStamped first_pose;
            first_pose.pose.position.x = msg->points[5].x;
            first_pose.pose.position.y = msg->points[5].y;
            first_pose.pose.position.z = msg->points[5].z;

            PlanTarget.position.z = first_pose.pose.position.z;
            PlanTarget.position.x = first_pose.pose.position.x;
            PlanTarget.position.y = first_pose.pose.position.y;
            // double yaw = tf::getYaw(first_pose.pose.orientation);
            // PlanTarget.yaw = yaw;
            // PlanTarget.position.z = first_pose.pose.position.z;
            PlanTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
            PlanTarget.type_mask=PositionTarget.IGNORE_VX|
                                PositionTarget.IGNORE_VY|
                                PositionTarget.IGNORE_VZ|
                                PositionTarget.IGNORE_AFX|
                                PositionTarget.IGNORE_AFY|
                                PositionTarget.IGNORE_AFZ|
                                PositionTarget.IGNORE_YAW|
                                PositionTarget.IGNORE_YAW_RATE;
        }
        else{
            // geometry_msgs::PoseStamped first_pose = msg->points[msg->points.size() - 1];
            // PlanTarget.yaw = yaw;
            geometry_msgs::PoseStamped first_pose;
            first_pose.pose.position.x = msg->points[msg->points.size() - 1].x;
            first_pose.pose.position.y = msg->points[msg->points.size() - 1].y;
            first_pose.pose.position.z = msg->points[msg->points.size() - 1].z;

            PlanTarget.position.z = first_pose.pose.position.z;
        
            PlanTarget.position.x = first_pose.pose.position.x;
            PlanTarget.position.y = first_pose.pose.position.y;

            // double yaw = tf::getYaw(first_pose.pose.orientation);
            // PlanTarget.yaw = yaw;
            // PlanTarget.position.z = first_pose.pose.position.z;
            PlanTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
            PlanTarget.type_mask=PositionTarget.IGNORE_VX|
                                PositionTarget.IGNORE_VY|
                                PositionTarget.IGNORE_VZ|
                                PositionTarget.IGNORE_AFX|
                                PositionTarget.IGNORE_AFY|
                                PositionTarget.IGNORE_AFZ|
                                PositionTarget.IGNORE_YAW|
                                PositionTarget.IGNORE_YAW_RATE;
        }      
    }
    else{
        std::cout << "no more massages" << diss_A << std::endl;
    }
    std::cout << "PlanTarget" << PlanTarget.position.x << "  " << PlanTarget.position.y << "  " << PlanTarget.position.z <<std::endl;
} 


//起飞
void Path::_Fly(){
    PositionTarget.position.x=0;
    PositionTarget.position.y=0;
    PositionTarget.position.z=Z_1;
    PositionTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
    PositionTarget.type_mask=PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
}

void Path::_Fly1(){
    PositionTarget.position.x=3;
    PositionTarget.position.y=1;
    PositionTarget.position.z=Z_1;
    PositionTarget.coordinate_frame=PositionTarget.FRAME_LOCAL_NED;
    PositionTarget.type_mask=PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE;
}

void Path::fixed(double yawValue)
{
    PositionTarget.position.x=_A_x;
    PositionTarget.position.y=_A_y;
    PositionTarget.position.z = Z_1;

    PositionTarget.yaw = yawValue;  // 设置偏航角度为π弧度，即180度
    // PositionTarget.yaw_rate =  M_PI / 3;
    PositionTarget.coordinate_frame = PositionTarget.FRAME_LOCAL_NED;
    PositionTarget.type_mask=PositionTarget.IGNORE_VX|
    PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|
    PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|
    PositionTarget.IGNORE_AFZ|
    // PositionTarget.IGNORE_YAW;
    PositionTarget.IGNORE_YAW_RATE;
}

void Path::run()
{ 
    // 设置控制频率
    ros::Rate rate(30.0);
    
    // // 等待飞控连接
    while (ros::ok() && !current_state.connected) {
        std::cout << "WAITING FOR connected" << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    // 发送空速度控制指令
    _Fly();

    // 切换到offboard模式
    while (ros::ok() && (current_state.mode != "OFFBOARD")) {
        std::cout << "WAITING FOR OFFBOARD" << std::endl;
        path_pub.publish(PositionTarget);
        setMode("OFFBOARD");
        ros::spinOnce();
        rate.sleep();
    }

    // 等待解锁完成
    while (ros::ok() && !current_state.armed) {
        arm();
        ros::Duration(2).sleep();
        std::cout << "WAITING FOR ARMED" << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    // start_time = ros::Time::now().toSec();

    while(ros::ok()){
        // if (!is_distance_updated) {
        //     ros::spinOnce();
        //     continue;
        // }
        switch(FlyState)
        {
            case Fly:
                _Fly();
                FlyState = Fly;
                // FlyState = Circle_1;

                // if (distance_to_target < threshold)
                // if (distance_to_target < 0.16 && diss_y < 0.03 && diss_z < 0.02) 

                path_pub.publish(PositionTarget);

                if (distance_to_target < 0.16 && diss_z < 0.05)         // 测试起飞
                {
                    FlyState = Circle_1;
                //     // setMode("AUTO.LAND");
                }
                std::cout << "Flying2222" << std::endl;
                break;

            case Circle_1:
                // _Fly1();
                // SetGoal(1, 0, 1.5);

                FlyState = Circle_1;

                // yolo_label = "1";
                // if (depth.x > 2){
                //     std::cout << "depth" <<depth.x << "  "  << depth.y<< "  "<< depth.z<<std::endl;
                depth.x = 3;
                depth.y = 0.5;
                depth.z = 1.5;

                depth_goal_pub.publish(depth);
                
                // }
                std::cout << "circle 1" << std::endl;
                if (line_strip.size() < 2)         
                {
                    // setMode("AUTO.LAND");
                    FlyState = Circle_2;
                    // std::cout << "LAND" << std::endl;
                }
                path_pub.publish(PlanTarget);

                break;

            case Circle_2:
                FlyState = Circle_2;
                // yolo_label = "2";
                // if (depth.x > 2){
                // std::cout << "depth" <<depth.x << "  "  << depth.y<< "  "<< depth.z<<std::endl;
                depth.x = 3;
                depth.y = 0.5;
                depth.z = 1.5;
                depth_goal_pub.publish(depth);
                // }

                // 修改
                if (A_distance_to_target < 1 && _diss_z < 0.05)         // 测试起飞
                {
                    setMode("AUTO.LAND");
                    std::cout << "LAND" << std::endl;
                    // FlyState = Circle_2;
                }

                path_pub.publish(PlanTarget);
                // path_pub.publish(PlanTarget);
                // if (diss_A < 0.2)
                // {
                //     fixed(0);
                //     path_pub.publish(PositionTarget);
                //     FlyState = Fixed_A;
                // }
                std::cout << "Move_A" << std::endl;
                start_time = ros::Time::now().toSec();
                break;

            // todo
            case Fixed_A:
                path_pub.publish(PositionTarget);
                FlyState = Fixed_A;
                std::cout << "Fixed_A" << std::endl;
                current_time = ros::Time::now().toSec();
                elapsed_time = current_time - start_time;
                // DetectAndShoot();
                fixed(yaw_true);
                // if(yaw_true < 2 && yaw_true != 0 && yaw_true > -2){
                if((yaw_true < 1 && yaw_true != 0 && yaw_true > -1) || (elapsed_time > 10)){

                    desir_shot.shot = 1;
                    shoot_pub.publish(desir_shot);
                    std::cout << "shot" << std::endl;
                    FlyState = Move_B;
                    SetGoal(0, 0, 1.0);                                 // 测试激光
                    path_pub.publish(PlanTarget);
                }
                // elapsed_time = current_time - start_time;
                // if (elapsed_time >= 2.0){

                //     SetGoal(0, 0, 1.0);
                //     FlyState = Move_B;
                //     path_pub.publish(PlanTarget);
                // }
                break;
            // case Detect:
            //     DetectAndShoot();
            //     fixed(yaw_true);
            //     FlyState = Detect;
            //     break;

            case Move_B:
                FlyState = Move_B;
                path_pub.publish(PlanTarget);
                std::cout << "Move_B" << std::endl;
                
                if (diss_B < 0.1 && current_x < 0.04){
                    FlyState = Land;
                    std::cout << "Land!" << std::endl;
                }
                break;
            case Land:
                // while (ros::ok() && (current_state.mode != "OFFBOARD"))
                setMode("AUTO.LAND");
                FlyState = Land;
                std::cout << "Land!" << std::endl;
                break;
        }
        // path_pub.publish(PathTarget);
            ros::spinOnce();
            rate.sleep();
        }
        // spinThread.join();
        // 关闭飞行模式和解锁
        setMode("MANUAL");
        disarm();
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "path_publisher");

    Path path;
    path.run();

    return 0;
}
