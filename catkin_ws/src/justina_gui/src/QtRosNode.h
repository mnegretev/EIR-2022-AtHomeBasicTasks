#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "custom_msgs/InverseKinematics.h"
#include "custom_msgs/SmoothPath.h"
#include "custom_msgs/FindObject.h"
#include "sensor_msgs/PointCloud2.h"
#include "sound_play/SoundRequest.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher pubCmdVel;
    ros::Publisher pubGoToXYA;
    ros::Publisher pubTorso;
    ros::Publisher pubLaAngle1;
    ros::Publisher pubLaAngle2;
    ros::Publisher pubLaAngle3;
    ros::Publisher pubLaAngle4;
    ros::Publisher pubLaAngle5;
    ros::Publisher pubLaAngle6;
    ros::Publisher pubLaAngle7;
    ros::Publisher pubLaAngleGl;
    ros::Publisher pubLaAngleGr;
    ros::Publisher pubRaAngle1;
    ros::Publisher pubRaAngle2;
    ros::Publisher pubRaAngle3;
    ros::Publisher pubRaAngle4;
    ros::Publisher pubRaAngle5;
    ros::Publisher pubRaAngle6;
    ros::Publisher pubRaAngle7;
    ros::Publisher pubRaAngleGl;
    ros::Publisher pubRaAngleGr;
    ros::Publisher pubHdPan;
    ros::Publisher pubHdTilt;
    ros::Publisher pubSay;
    ros::Subscriber subRecognizedSpeech;
    ros::ServiceClient cltLaInverseKinematics;
    ros::ServiceClient cltRaInverseKinematics;
    ros::ServiceClient cltAStar;
    ros::ServiceClient cltSmoothPath;
    ros::ServiceClient cltFindObject;
    tf::TransformListener tf_listener;
    
    geometry_msgs::Twist cmd_vel;
    bool publishing_cmd_vel;
    bool gui_closed;
    std::string str_recognized_speech;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void stop_publishing_cmd_vel();
    void get_robot_pose(float& robot_x, float& robot_y, float& robot_a);
    void set_param_inflation_radius(float inflation_radius);
    void set_param_cost_radius(float cost_radius);
    void set_param_smoothing_alpha(float smoothing_alpha);
    void set_param_smoothing_beta(float  smoothing_beta);
    bool call_a_star_search(float start_x, float start_y, float goal_x, float goal_y, nav_msgs::Path& path);
    bool call_smooth_path(nav_msgs::Path& path, nav_msgs::Path& smooth_path);
    void publish_goto_xya(float goal_x, float goal_y, float goal_a);

    void publish_torso_position(float tr);
    void publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_la_grip_angles(float a1, float a2);
    void publish_ra_grip_angles(float a1, float a2);
    void publish_head_angles(float pan, float tilt);
    bool call_la_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    bool call_ra_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular);

    bool call_find_object(std::string obj_name);

    void publish_text_to_say(std::string txt);
    void callback_recognized_speech(const std_msgs::String::ConstPtr& msg);
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
