#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    publishing_cmd_vel = false;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(10);
    pubCmdVel    = n->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pubGoToXYA   = n->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    pubTorso     = n->advertise<std_msgs::Float64>("/torso_controller/command", 10);
    pubLaAngle1  = n->advertise<std_msgs::Float64>("/la_1_controller/command" , 10);
    pubLaAngle2  = n->advertise<std_msgs::Float64>("/la_2_controller/command" , 10);
    pubLaAngle3  = n->advertise<std_msgs::Float64>("/la_3_controller/command" , 10);
    pubLaAngle4  = n->advertise<std_msgs::Float64>("/la_4_controller/command" , 10);
    pubLaAngle5  = n->advertise<std_msgs::Float64>("/la_5_controller/command" , 10);
    pubLaAngle6  = n->advertise<std_msgs::Float64>("/la_6_controller/command" , 10);
    pubLaAngle7  = n->advertise<std_msgs::Float64>("/la_7_controller/command" , 10);
    pubRaAngle1  = n->advertise<std_msgs::Float64>("/ra_1_controller/command" , 10);
    pubRaAngle2  = n->advertise<std_msgs::Float64>("/ra_2_controller/command" , 10);
    pubRaAngle3  = n->advertise<std_msgs::Float64>("/ra_3_controller/command" , 10);
    pubRaAngle4  = n->advertise<std_msgs::Float64>("/ra_4_controller/command" , 10);
    pubRaAngle5  = n->advertise<std_msgs::Float64>("/ra_5_controller/command" , 10);
    pubRaAngle6  = n->advertise<std_msgs::Float64>("/ra_6_controller/command" , 10);
    pubRaAngle7  = n->advertise<std_msgs::Float64>("/ra_7_controller/command" , 10);
    pubLaAngleGl = n->advertise<std_msgs::Float64>("/la_grip_left_controller/command" , 10);
    pubLaAngleGr = n->advertise<std_msgs::Float64>("/la_grip_right_controller/command", 10);
    pubRaAngleGl = n->advertise<std_msgs::Float64>("/ra_grip_left_controller/command" , 10);
    pubRaAngleGr = n->advertise<std_msgs::Float64>("/ra_grip_right_controller/command", 10);
    pubHdPan     = n->advertise<std_msgs::Float64>("/head_pan_controller/command",  10, true);
    pubHdTilt    = n->advertise<std_msgs::Float64>("/head_tilt_controller/command", 10, true);
    pubSay       = n->advertise<sound_play::SoundRequest>("/robotsound", 10);
    subRecognizedSpeech    = n->subscribe("/recognized", 1, &QtRosNode::callback_recognized_speech, this);
    cltAStar               = n->serviceClient<nav_msgs::GetPlan>("/path_planning/a_star_search");
    cltSmoothPath          = n->serviceClient<custom_msgs::SmoothPath>("/path_planning/smooth_path");
    cltLaInverseKinematics = n->serviceClient<custom_msgs::InverseKinematics>("/manipulation/la_inverse_kinematics");
    cltRaInverseKinematics = n->serviceClient<custom_msgs::InverseKinematics>("/manipulation/ra_inverse_kinematics");
    cltFindObject          = n->serviceClient<custom_msgs::FindObject>("/vision/find_object");
    int pub_zero_counter = 5;
    while(ros::ok() && !this->gui_closed)
    {
        if(publishing_cmd_vel)
        {
            pubCmdVel.publish(cmd_vel);
            pub_zero_counter = 5;
        }
        else if(--pub_zero_counter > 0)
        {
            if(pub_zero_counter <= 0)
                pub_zero_counter = 0;
            pubCmdVel.publish(cmd_vel);
        }
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publish_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    pubCmdVel.publish(cmd_vel);
}

void QtRosNode::start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    publishing_cmd_vel = true;
}

void QtRosNode::stop_publishing_cmd_vel()
{
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    publishing_cmd_vel = false;
}

bool QtRosNode::call_a_star_search(float start_x, float start_y, float goal_x, float goal_y, nav_msgs::Path& path)
{
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    bool success = cltAStar.call(srv);
    path = srv.response.plan;
    return success;
}

bool QtRosNode::call_smooth_path(nav_msgs::Path& path, nav_msgs::Path& smooth_path)
{
    custom_msgs::SmoothPath srv;
    srv.request.path = path;
    bool success = cltSmoothPath.call(srv);
    smooth_path = srv.response.smooth_path;
    return success;
}

void QtRosNode::publish_goto_xya(float goal_x, float goal_y, float goal_a)
{
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = goal_x;
    msg.pose.position.y = goal_y;
    msg.pose.orientation.w = cos(goal_a/2);
    msg.pose.orientation.z = sin(goal_a/2);
    pubGoToXYA.publish(msg);
}

void QtRosNode::set_param_inflation_radius(float inflation_radius)
{
    n->setParam("/path_planning/inflation_radius", inflation_radius);
}

void QtRosNode::set_param_cost_radius(float cost_radius)
{
    n->setParam("/path_planning/cost_radius",  cost_radius);
}

void QtRosNode::set_param_smoothing_alpha(float smoothing_alpha)
{
    n->setParam("/path_planning/smoothing_alpha",  smoothing_alpha);
}
  
void QtRosNode::set_param_smoothing_beta(float  smoothing_beta)
{
    n->setParam("/path_planning/smoothing_beta" ,  smoothing_beta);
}

void QtRosNode::get_robot_pose(float& robot_x, float& robot_y, float& robot_a)
{
    tf::StampedTransform t;
    tf::Quaternion q;
    tf_listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(100.0));
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
}

void QtRosNode::publish_torso_position(float tr)
{
    std_msgs::Float64 msg;
    msg.data = tr;
    pubTorso.publish(msg);
}

void QtRosNode::publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
{
    std_msgs::Float64 msg;
    msg.data = a1;
    pubLaAngle1.publish(msg);
    msg.data = a2;
    pubLaAngle2.publish(msg);
    msg.data = a3;
    pubLaAngle3.publish(msg);
    msg.data = a4;
    pubLaAngle4.publish(msg);
    msg.data = a5;
    pubLaAngle5.publish(msg);
    msg.data = a6;
    pubLaAngle6.publish(msg);
    msg.data = a7;
    pubLaAngle7.publish(msg);
}

void QtRosNode::publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
{
    std_msgs::Float64 msg;
    msg.data = a1;
    pubRaAngle1.publish(msg);
    msg.data = a2;
    pubRaAngle2.publish(msg);
    msg.data = a3;
    pubRaAngle3.publish(msg);
    msg.data = a4;
    pubRaAngle4.publish(msg);
    msg.data = a5;
    pubRaAngle5.publish(msg);
    msg.data = a6;
    pubRaAngle6.publish(msg);
    msg.data = a7;
    pubRaAngle7.publish(msg);
}

void QtRosNode::publish_la_grip_angles(float al, float ar)
{
    std_msgs::Float64 msg;
    msg.data = al;
    pubLaAngleGl.publish(msg);
    msg.data = ar;
    pubLaAngleGr.publish(msg);
}

void QtRosNode::publish_ra_grip_angles(float al, float ar)
{
    std_msgs::Float64 msg;
    msg.data = al;
    pubRaAngleGl.publish(msg);
    msg.data = ar;
    pubRaAngleGr.publish(msg);
}

void QtRosNode::publish_head_angles(float pan, float tilt)
{
    std_msgs::Float64 msg;
    msg.data = pan;
    pubHdPan.publish(msg);
    msg.data = tilt;
    pubHdTilt.publish(msg);
}

bool QtRosNode::call_la_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    custom_msgs::InverseKinematics srv;
    srv.request.x = cartesian[0];
    srv.request.y = cartesian[1];
    srv.request.z = cartesian[2];
    srv.request.roll  = cartesian[3];
    srv.request.pitch = cartesian[4];
    srv.request.yaw   = cartesian[5];
    if(!cltLaInverseKinematics.call(srv))
        return false;
    articular.clear();
    articular.push_back(srv.response.q1);
    articular.push_back(srv.response.q2);
    articular.push_back(srv.response.q3);
    articular.push_back(srv.response.q4);
    articular.push_back(srv.response.q5);
    articular.push_back(srv.response.q6);
    articular.push_back(srv.response.q7);
    return true;
}

bool QtRosNode::call_ra_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    custom_msgs::InverseKinematics srv;
    srv.request.x = cartesian[0];
    srv.request.y = cartesian[1];
    srv.request.z = cartesian[2];
    srv.request.roll  = cartesian[3];
    srv.request.pitch = cartesian[4];
    srv.request.yaw   = cartesian[5];
    if(!cltRaInverseKinematics.call(srv))
        return false;
    articular.clear();
    articular.push_back(srv.response.q1);
    articular.push_back(srv.response.q2);
    articular.push_back(srv.response.q3);
    articular.push_back(srv.response.q4);
    articular.push_back(srv.response.q5);
    articular.push_back(srv.response.q6);
    articular.push_back(srv.response.q7);
    return true;
}


bool QtRosNode::call_find_object(std::string obj_name)
{
    custom_msgs::FindObject srv;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/points", ros::Duration(1.0));
    srv.request.cloud = *ptr;
    srv.request.name = obj_name;
    cltFindObject.call(srv);
}

void QtRosNode::publish_text_to_say(std::string txt)
{
    sound_play::SoundRequest msg;
    msg.sound   = -3;
    msg.command = 1;
    msg.volume  = 1.0;
    msg.arg2    = "voice_kal_diphone";
    msg.arg     = txt;
    pubSay.publish(msg);
}

void QtRosNode::callback_recognized_speech(const std_msgs::String::ConstPtr& msg)
{
    str_recognized_speech = msg->data;
}
