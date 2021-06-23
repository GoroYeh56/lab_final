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
std::vector<std::vector<double>> GOALS12={
{15.95,	15.555},
{15.4365,	16.6215},
{14.37,	17.135},
{13.3035,	16.6215},
{12.79,	15.555},
{13.185,	14.765},
{12.79,	13.975},
{13.3035,	12.9085},
{14.37,	12.395},
{15.4365,	12.9085},
{15.95,	13.975},
{15.555,	14.765}
};



std::vector<std::vector<double>> GOALS24={
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

float thetaConstraint(float thi);

void Robot_Pose_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    
    // robot current pose feedback (from sensors measurement)
    robot_x = msg->data[0];
    robot_y = msg->data[1];
    robot_theta = msg->data[2];


    // if(robot_theta > pi){
    //     while(robot_theta > pi)
    //         robot_theta -= 2*pi;
    // }
    // if(robot_theta <= -pi){
    //     while(robot_theta <= 2*pi)
    //         robot_theta += 2*pi;
    // }

    robot_theta = thetaConstraint(robot_theta);

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

// const float max_v = 0.4;
// const float max_v = 1.0;
// const float max_w = 1.2;
float max_v, max_w;


/* -------- Gain ---------- */
// const float Krho = 0.5;
// const float Ka = 2.2; 
// const float Kb = -0.6;

// Claude
// const float Krho = 2;
// const float Ka = 8;
// const float Kb = -1.5;
// const float Kp = 0.5;
float Kp, Ka, Kb, Krho;

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

    // if(thi >= 2*pi ){
    //     while(thi >= 2*pi) thi -= 2*pi; 
    // }
    // else if(thi < 0){
    //     while(thi < 0) thi += 2*pi;
    // }
    // return thi;

}

float deg2rad(float degree){
    return degree*pi/180;
}


float distance = 0;
int goals_mode;


#include <set>
std::set<int> neck_set;
std::set<int>::iterator it;

 
int main(int argc, char **argv)
{
    // Initialize the node here
	ros::init(argc, argv, "controller");
    ros::NodeHandle node;

    // Get parameters.
    float X_CRITERION, Y_CRITERION, THETA_CRITERION;
    double DISTANCE_THRESHOLD, NECK_THRESHOLD;

    ros::param::param<std::float_t>("/X_CRITERION", X_CRITERION, 0.1);
    ros::param::param<std::float_t>("/Y_CRITERION", Y_CRITERION, 0.1);    
    ros::param::param<std::float_t>("/THETA_CRITERION", THETA_CRITERION,0.35);
    ros::param::param<std::float_t>("/Kp", Kp, 0.8); 
    ros::param::param<std::float_t>("/Krho", Krho, 0.5);
    ros::param::param<std::float_t>("/Ka", Ka, 2.5); 
    ros::param::param<std::float_t>("/Kb", Kb, -1.12);
    ros::param::param<std::float_t>("/max_v", max_v, 0.5); 
    ros::param::param<std::float_t>("/max_w", max_w, 1.0); 
    ros::param::param<std::double_t>("DISTANCE_THRESHOLD", DISTANCE_THRESHOLD, 0.85); 
    ros::param::param<std::double_t>("NECK_THRESHOLD", NECK_THRESHOLD, 0.6); 
    ros::param::param<std::int32_t>("mode", goals_mode,  12 );



    ROS_INFO("goals_mode: %d",goals_mode);

    /* -- Initialize GOALS ---*/
    std::vector<std::vector<double>> GOALS;
    if( goals_mode==12) {
        for(int i=0; i<GOALS12.size(); i++){
                GOALS.push_back(GOALS12[i]);
        }

        neck_set = {0, 4, 5, 6, 9, 10,11};
    }
    else{
        for(int i=0; i<GOALS24.size(); i++){
                GOALS.push_back(GOALS24[i]);
        }
        neck_set = {0, 9,  10, 11, 21, 22, 23};
    }

    ROS_INFO("Done init GOALS.\n");
    for(auto &v: GOALS)
        ROS_INFO("{ %.2f, %.2f}",v[0],v[1]);

	ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber robot_pose_sub = node.subscribe("/status", 10, Robot_Pose_Callback);
	// ros::Subscriber goal_pose_sub = node.subscribe("/move_base_simple/goal", 10, Goal_Pose_Callback);

    ros::Subscriber Start_sub = node.subscribe("/start", 10, Start_Callback);
    // Set the publish rate here
	ros::Rate rate(1000);

    float v, w;
	geometry_msgs::Twist twist;

    float temp_goal_x, temp_goal_y, next_goal_x, next_goal_y, dx, dy;
    // Main Control Loop.

    // Measure Running Time
    ros::WallTime start_, end_, now;
    bool start_measure = false;
    // After receive a new goal: keep localization task.
	while (ros::ok()) 
	{
        if(start == true){
        
            if(!start_measure){
                start_ = ros::WallTime::now();
                start_measure = true;
            }

            if(counter >= 2*GOALS.size()+1){
                // Publish!
                break; // End program.
            }

            goal_x = GOALS[counter%GOALS.size()][0];
            goal_y = GOALS[counter%GOALS.size()][1];
            // goal_theta = GOALS[counter%GOALS.size()][2];
            dx = GOALS[(counter+1)%GOALS.size()][0] - goal_x;
            dy = GOALS[(counter+1)%GOALS.size()][1] - goal_y;
            goal_theta = atan2(dy,dx);

            // goal_x = robot_x + 1;
            // goal_y = robot_y + 1;
            // dx = goal_x - robot_x;
            // dy = goal_y - robot_y;
            // goal_theta = atan2(dy,dx);


            goal_theta = thetaConstraint(goal_theta);

            error_x = goal_x - robot_x;
            error_y = goal_y - robot_y;
            error_theta = goal_theta - robot_theta;
            error_theta = thetaConstraint(error_theta);

            
            distance = sqrt(pow(error_x, 2) + pow(error_y,2));
            // ONLY in MOVING state
            switch (state)
            {
            case MOVING:  
                ////////// Main Control Loop! /////////////
                
                 it= neck_set.find(counter%GOALS.size());

                if(  it != neck_set.end() ){ // Should be careful
                    if(distance < NECK_THRESHOLD ){ //&& counter>0){
                        now = ros::WallTime::now();
                        std::cout<<"Dis: "<<distance<<", Done running "<<counter<<", time executed: "<<(now - start_).toNSec()* 1e-9<<" sec"<<std::endl;
                        counter++;                    
                    }
                    else{
                        if (abs(error_x) < X_CRITERION && abs(error_y) < Y_CRITERION && abs(error_theta) < THETA_CRITERION)
                        {
                            now = ros::WallTime::now();
                            std::cout<<"Else - Dis: "<<distance<<", Done running "<<counter<<", time executed: "<<(now - start_).toNSec()* 1e-9<<" sec"<<std::endl;
                            counter++;  
                        }
                        else
                        {   
                            // std::cout<<"dis: "<<distance<<std::endl;
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
                    }
                }
                else{
                    if(distance < DISTANCE_THRESHOLD ){ //&& counter>0){
                        now = ros::WallTime::now();
                        std::cout<<"Dis: "<<distance<<", Done running "<<counter<<", time executed: "<<(now - start_).toNSec()* 1e-9<<" sec"<<std::endl;
                        counter++;                    
                    }
                    else{
                        if (abs(error_x) < X_CRITERION && abs(error_y) < Y_CRITERION && abs(error_theta) < THETA_CRITERION)
                        {
                            now = ros::WallTime::now();
                            std::cout<<"Else - Dis: "<<distance<<", Done running "<<counter<<", time executed: "<<(now - start_).toNSec()* 1e-9<<" sec"<<std::endl;
                            counter++;  
                        }
                        else
                        {   
                            // std::cout<<"dis: "<<distance<<std::endl;
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
                    }
                }
                    
                break;
        #ifdef DEBUG_LAB2
                ROS_INFO("Robot reached the goal and is now IDLE.");
        #endif
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

    // First STOP the robot
    twist.linear.x =  0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    cmd_vel_pub.publish(twist); 


    // End running two laps.
    end_ = ros::WallTime::now();

    // print results
    // double execution_time = (end_ - start_).toNSec() * 1e-6;
    double execution_time = (end_ - start_).toNSec()* 1e-9;
    // ROS_INFO_STREAM("Exectution time (ms): " << execution_time);  
    ROS_INFO_STREAM("Exectution time (sec): " << execution_time);  

    return 0;
}
