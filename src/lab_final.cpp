#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"  // KEY!!
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64.h"

/*
    Publish:
        /cmd_vel
            geometry_msgs/Twist.msg
            - Vector3 linear (x, y, z)
            - Vector3 angular

    Subscribe:
        /status
            std_msgs::Float64MultiArray

            -data[ ] (a float array)

        /start
            std_msgs::Int64
            -data
*/


ros::Publisher cmd_vel_pub;
ros::Subscriber robot_pose_sub;
ros::Subscriber goal_pose_sub;


const float pi = 3.14159265358979323846;
//            0       1       2
enum State{MOVING, TURNING, IDLE};
State state = MOVING; //Initial turtle state/pose.

float rho, alpha, beta;
float goal_x = 0;
float goal_y = 0;
float goal_z = 0;
float goal_theta = 0;
float robot_x = 0;
float robot_y = 0;
float robot_theta = 0;

float error_x;
float error_y;
float error_theta;


// goal = [[10,9],[8.7,11.7],[6,13],[3.3,11.7],[2,9],[3,7],[2,5],[3.3,2.3],[6,1],[8.7,2.3],[10,5],[9,7]]
// Goal pose for robot. {x, y, theta}
// std::vector<std::vector<double>> GOALS = {
//     {16, 15.95 , atan2(-1, 17-15.95)},
//     {15, 17 , atan2(-0.6, 0)},
//     {14.4, 17, pi},
//     {13, 17, atan2(-0.5, -1)},
//     {12.5, 16, atan2(0.5, 14.75-16)},
//     {13, 14.75, -pi/2},
//     {12.5, 16, atan2(1.5, -4)},
//     {13, 12, atan2(1.4, 0) },
//     {14.4, 12, 0, atan2(0.6, 0)},
//     {15, 12,  atan2(1, 15.95-12)   },
//     {16, 15.95, atan2((15.5+15.95)/2 - 16, 14.75-15.95)},
//     {(15.5+15.95)/2, 14.75,  pi/2}  
// };



std::vector<std::vector<double>> GOALS={
{15.95,	15.555}, {15.69325,16.08825},
{15.4365,	16.6215}, {14.90325,16.87825},
{14.37,	17.135}, {13.83675,16.87825},
{13.3035,	16.6215},{13.04675,16.08825},
{12.79,	15.555},{12.9875,	15.16},
{13.185,	14.765},{12.9875,	14.37},
{12.79,	13.975},{13.04675,13.44175},
{13.3035,	12.9085},{13.83675,12.65175},
{14.37,	12.395},{14.90325,	12.65175},
{15.4365,	12.9085}, {15.69325,	13.44175},
{15.95,	13.975},{15.7525,	14.37},
{15.555,	14.765},{15.7525,15.16}
};

bool start = false;

void Start_Callback(const std_msgs::Int64::ConstPtr& msg){
    start = true; // start the controller node!
}

void Robot_Pose_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    
    // robot current pose feedback (from sensors measurement)
    robot_x = msg->data[0];
    robot_y = msg->data[1];
    robot_theta = msg->data[2];


    if(robot_theta > pi){
        while(robot_theta > pi)
            robot_theta -= 2*pi;
    }
    if(robot_theta <= -pi){
        while(robot_theta <= 2*pi)
            robot_theta += 2*pi;
    }

    float delta_x = goal_x - robot_x;
    float delta_y = goal_y - robot_y;
    float delta_theta = goal_theta - robot_theta;

    // Convert to polar coordinate.
    rho = sqrt(pow(delta_x,2)+pow(delta_y,2));


        // % xc : robot_x.
        // % xi : initial_x.


        // % 1st - quardrum
        float x_c = goal_x;
        float y_c = goal_y;
        float angle_c = goal_theta;

        float x_i = robot_x;
        float y_i = robot_y;
        float angle = robot_theta;

        
        float x_err = error_x;
        float y_err = error_y;


        if (x_c - x_i >= 0 && y_c - y_i >= 0){
            alpha = (atan2(y_err,x_err) - angle);
            beta = angle_c - atan2(y_err,x_err);
        }
        // % 2nd - quardrum
        if (x_c - x_i <= 0 && y_c - y_i >= 0){
            alpha = (atan2(y_err,x_err) - angle);
            beta = angle_c + (2*pi - atan2(y_err,x_err));
        }
        // % 3rd quardrum
        if (x_c - x_i <=0 && y_c - y_i <=0){
            alpha = pi + atan2(y_err,x_err) - angle + pi;
            beta = -(pi + atan2(y_err,x_err) - angle_c) + pi;
        }

        // % 4th quardrum
        if (x_c - x_i >=0 && y_c - y_i <=0){
            alpha = pi + atan2(y_err,x_err) - angle + pi;
            beta = -(pi + atan2(y_err,x_err) - angle_c) + pi;
        }

    // alpha = atan2(delta_y, delta_x) - (robot_theta);

    /* ------ TODO: 4 Quadrant ------ */

    // alpha constraint: -pi ~ pi. 
    if (alpha <= -pi){
        while(alpha <= -pi){
            alpha += 2*pi;
        }
    }
    if (alpha > pi){
        while(alpha > pi)
            alpha -= 2*pi;
    }


    if (beta <= -pi){
        while(beta <= -pi){
            beta += 2*pi;
        }
    }
    if (beta > pi){
        while(beta > pi)
            beta -= 2*pi;
    }

    // beta = delta_theta - alpha;

}


/*
void Goal_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Once receive a command, change state!
    state = MOVING;
    float qx, qy, qz, qw;
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;    
    qw = msg->pose.orientation.w;

    goal_theta = atan2(2*(qw*qz+qx*qy), 1-2*(qz*qz+qy*qy));
    
    // if(goal_theta <0){
    //     while(goal_theta <0)
    //         goal_theta += 2*pi;
    // }
    // if(goal_theta > 2*pi){
    //     while(goal_theta>2*pi)
    //         goal_theta -= 2*pi;
    // }

}
*/




/* -------- Kinematics ---------- */
const float wheel_r = 0.06; //cm
const float wheel_separaton = 0.24; //cm

const float Max_Wheel_Speed = 2*pi;
// const float max_v = 2*wheel_r*Max_Wheel_Speed;
// const float max_w = 2*max_v/wheel_separaton; 

const float max_v = 0.4;
const float max_w = 0.5;


/* -------- Gain ---------- */
// const float Krho = 0.5;
// const float Ka = 2.2; 
// const float Kb = -0.6;

// Claude
// const float Krho = 2;
// const float Ka = 8;
// const float Kb = -1.5;
// const float Kp = 0.5;

const float Kp = 0.3; // 0.5


const float Krho = 0.5;   // 2
const float Ka = 2.5;     // 8
const float Kb = -1.12 ;    // -1.5



int counter = 0; // index for which goal we are trying to reach.

float delta_x, delta_y, delta_theta, theta_r;
float thetaConstraint(float thi)
{
	// while(abs(thi)>pi)
	// {
	// 	thi = (thi > 0) ?  thi - 2*pi : thi + 2*pi;
	// }
	// thi = (thi == -1*pi) ? -1*thi : thi;
	// return thi;
    if(thi > pi ){
        while(thi > pi) thi -= 2*pi; 
    }
    else if(thi <= -pi){
        while(thi <= -pi) thi += 2*pi;
    }
    return thi;

}

float deg2rad(float degree){
    return degree*pi/180;
}


int main(int argc, char **argv)
{
    // Initialize the node here
	ros::init(argc, argv, "controller");
    ros::NodeHandle node;

	ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber robot_pose_sub = node.subscribe("/status", 10, Robot_Pose_Callback);
	// ros::Subscriber goal_pose_sub = node.subscribe("/move_base_simple/goal", 10, Goal_Pose_Callback);

    ros::Subscriber Start_sub = node.subscribe("/start", 10, Start_Callback);
    // Set the publish rate here
	ros::Rate rate(100);

    float v, w;
	geometry_msgs::Twist twist;

    float temp_goal_x, temp_goal_y, next_goal_x, next_goal_y, dx, dy;
    // Main Control Loop.

    // After receive a new goal: keep localization task.
	while (ros::ok()) 
	{
        if(start == true){
        
                goal_x = GOALS[counter%GOALS.size()][0];
                goal_y = GOALS[counter%GOALS.size()][1];
                // goal_theta = GOALS[counter%GOALS.size()][2];
                dx = GOALS[(counter+1)%GOALS.size()][0] - goal_x;
                dy = GOALS[(counter+1)%GOALS.size()][1] - goal_y;
                goal_theta = atan2(dy,dx);


            thetaConstraint(goal_theta);

            error_x = goal_x - robot_x;
            error_y = goal_y - robot_y;
            error_theta = goal_theta - robot_theta;

            // ROS_INFO("State: %d, Goal theta: %.2f, Robot Theta: %.2f", state, goal_theta, robot_theta);
            // ROS_INFO("Error: %.3f %.3f %.3f", error_x, error_y, error_theta);
            printf("Robot at: %.2f %.2f %.2f, Goal %.2f %.2f %.2f\n", robot_x, robot_y, robot_theta, goal_x, goal_y, goal_theta);
            // ROS_INFO("Error: %.3f %.3f %.3f", error_x, error_y, error_theta);


            switch (state)
            {
            case MOVING:
                ////////// Main Control Loop! /////////////
                if (abs(error_x) < 0.2 && abs(error_y) < 0.2 && abs(error_theta) < 0.2)
                {
                // if (abs(error_x) < 0.01 && abs(error_y) < 0.01)
                // if (abs(error_x) < 0.3 && abs(error_y) < 0.3)
                // {
                    state = IDLE;
                }
                else if (abs(error_x) < 0.2 && abs(error_y) < 0.2)
                {
                    state = TURNING;
                }
                else
                {
                    /* ----- For Consistency ----- */
                    delta_x = error_x;
                    delta_y = error_y;
                    delta_theta = -error_theta;
                    theta_r = goal_theta;

                    // cout<<"Moving now!!\n";
                    rho = sqrt(pow(delta_x,2) + pow(delta_y,2));
                    alpha = atan2(delta_y,delta_x) - theta_r - delta_theta;
                    alpha = thetaConstraint(alpha);
                    
                    beta = -1*(alpha + delta_theta);
                    beta = thetaConstraint(beta);
                    
                    v = Krho*rho;
                    w = Ka*alpha + Kb*beta;

                    /* --- Control Law --- */
                    // v = Krho * rho;
                    // w = Ka * alpha + Kb * beta;
                    // ROS_INFO("Robot is MOVING !!!");
                }
                break;
            case TURNING:

                if (abs(error_theta) < 0.2 || abs(robot_theta + 2 * pi) < 0.2 || abs(error_theta - 2 * pi) < 0.2)
                {
                    state = IDLE;
                }
                else
                {
                    /* --- Control Law --- */
                    v = 0;
                    w = Kp * error_theta;
                }

                break;
            case IDLE:
                // Stop and listen to new goal.
                // reset error signals.
                v = 0;
                w = 0;
                // ROS_INFO("Robot IDLE and break!");
                state = MOVING;
                counter++;
        #ifdef DEBUG_LAB2
                ROS_INFO("Robot reached the goal and is now IDLE.");
        #endif

                break;
            }
               


                
            /* Command Inputs Constraints */
            if ( v > max_v ){
                v = max_v;
            }
            if ( v < -max_v ){
                v = -max_v;
            }        
            if ( w > max_w ){
                w = max_w;
            }
            if ( w < -max_w ){
                w = -max_w;
            }
            // Publish!
            twist.linear.x =  v;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = w;
            cmd_vel_pub.publish(twist); 
        
        
        } // end if start == true.
        // else: controller IDLE (Sleep and wait for user command)

        ros::spinOnce();    // Allow processing of incoming messages
        rate.sleep();

    }
}

