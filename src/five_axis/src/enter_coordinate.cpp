#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <vector>

#include <ros/ros.h> 
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "five_axis/msg_angle.h"

#define PI 3.1415926
#define to_deg 180/PI
#define to_rad PI/180

using namespace std;

double g_x=0, g_y=0, g_z=0;
double g_x2=0, g_y2=0, g_z2=0;

ros::Publisher angle_pub1;
ros::Publisher angle_pub2;
ros::Publisher angle_pub3;
ros::Publisher angle_pub4;
ros::Publisher gripper_pub;

class Arm{
    private:
    //arm length and chassis height
        double l1=0, l2=0, l3=0, l4=0, chassis=0;
    //goal coordinate
        double gx=0, gy=0;   
    //joints angle 
        double A_angle=0, B_angle=0, C_angle=0, D_angle=0;
    //joint coordinate
        vector<double> A, B, C, D, E;
    //gripper state
        bool gripper = false, vacuum = false;
        bool flag=true;
    //included in setGoal
        void getAngle();;        
        void getCoordinate();       
    // to make sure that the input goal is reachable    
        bool reachable(double gx, double gy, double gz);

    public:
    //initialize arms' length and chassis
        Arm(double l1, double l2, double l3, double l4, double chassis);
    // input goal coordinate and calcul0ate corresponding data
        void setGoal(double gx, double gy, double gz);
    // show goal coordinates and angles
        void showGoalInfo();
    // shwo gripper information
        void showGripper();
    // move to goal
        void moveToGoal();
    // move to goal 2
        void moveToGoal2();
    // main function
        void createArm();
};

int main(int argc, char** argv){
    ros::init(argc, argv, "FIVE_AXIS_ARM");
    ros::NodeHandle nh;
    angle_pub1 = nh.advertise<std_msgs::Float32>("ANGLE1", 1);
    angle_pub2 = nh.advertise<std_msgs::Float32>("ANGLE2", 1);
    angle_pub3 = nh.advertise<std_msgs::Float32>("ANGLE3", 1);
    angle_pub4 = nh.advertise<std_msgs::Float32>("ANGLE4", 1);
    gripper_pub = nh.advertise<std_msgs::Bool>("GRIPPER", 1);

    Arm pomelo(19,22,18,7,6);
    pomelo.createArm();
    return 0;
}

// declare and initialize arms' length 
Arm::Arm(double l1, double l2, double l3, double l4, double chassis):
    l1(l1), l2(l2), l3(l3), l4(l4), chassis(chassis){};

// calculate required angle
void Arm::getAngle(){        
    double bd = sqrt(gx*gx+(gy+l4-l1)*(gy+l4-l1));
    double ad = sqrt(gx*gx+(gy+l4)*(gy+l4));
    double be = sqrt(gx*gx+(l1-gy)*(l1-gy));

    this->B_angle = acos((l2*l2+bd*bd-l3*l3)/(2*l2*bd))*to_deg
                    +acos((l1*l1+bd*bd-ad*ad)/(2*l1*bd))*to_deg;
    this->C_angle = acos((l2*l2+l3*l3-bd*bd)/(2*l2*l3))*to_deg;
    this->D_angle = acos((l3*l3+bd*bd-l2*l2)/(2*l3*bd))*to_deg
                    +acos((l4*l4+bd*bd-be*be)/(2*l4*bd))*to_deg;
    return;
}
    
//calculate coordinates (included in setGoal)
void Arm::getCoordinate(){       
    A.push_back(0); A.push_back(0);
    B.push_back(0); B.push_back(l1);
    C.push_back(sin(B_angle*to_rad)*l2), C.push_back(-cos(B_angle*to_rad)*l2+l1);
    D.push_back(this->gx); D.push_back(this->gy + l4);
    E.push_back(this->gx); E.push_back(this->gy);
    return; 
}

// to make sure that the input goal is reachable    
bool Arm::reachable(double gx, double gy, double gz){
    if(l2+l3 <= sqrt(gx*gx + gy*gy)){
        printf("ERROR: overranging arguments (x,y)");
        this->flag = false;
        return 0;
    }
    if(gz<0 || gz > l1+l2+l3-l4){
        printf("ERROR: overranging arguments (z)");
        this->flag = false;
        return 0;
        
    } 
    return 1;
}

// input goal coordinate and calculate corresponding data
void Arm::setGoal(double gx, double gy, double gz){      
    if(reachable(gx,gy,gz)){
    // coordinate system transformation
        this->gx = sqrt(gx*gx + gy*gy); 
        this->gy = gz - this->chassis;
        this->A_angle = atan(gy/gx)*to_deg;
    // calculate terminal angle and coordinate 
        getAngle();
        getCoordinate();
    } else {
        printf("ERROR: Overranged Input");
        exit(0);
    }
}
        
// show both angle and coordinate
void Arm::showGoalInfo(){
    if (!flag) return; 

    // print info
    printf( "=== Angle Info(unit: degree) ===\n");
    printf( "angle A = %f\n" ,this->A_angle );
    printf( "angle B = %f\n" ,this->B_angle );
    printf( "angle C = %f\n" ,this->C_angle );
    printf( "angle D = %f\n\n" ,this->D_angle );

    printf( "=== Coordinate Info(unit: centimeter) ===\n" );
    printf( "A: (%f, %f)\n", this->A[0], this->A[1] );
    printf( "B: (%f, %f)\n", this->B[0], this->B[1] );
    printf( "C: (%f, %f)\n", this->C[0], this->C[1] );
    printf( "D: (%f, %f)\n", this->D[0], this->D[1] );
    printf( "E: (%f, %f)\n\n", this->E[0], this->E[1] );
    
    // publish angle
    std_msgs::Float32 angleA;
    angleA.data = this->A_angle;
    angle_pub1.publish(angleA);

    std_msgs::Float32 angleB;
    angleB.data = this->B_angle;
    angle_pub2.publish(angleB);

    std_msgs::Float32 angleC;
    angleC.data = this->C_angle;
    angle_pub3.publish(angleC);

    std_msgs::Float32 angleD;
    angleD.data = this->D_angle;
    angle_pub4.publish(angleD);
}

// show grippers state
void Arm::showGripper(){
    // print info
    printf("[ Gripper State ]\n");
    printf("gripper: %d\nvacuum: %d\n\n", gripper, vacuum);

    // publish info
    std_msgs::Bool grip;
    grip.data = gripper;
    gripper_pub.publish(grip);
}

void Arm::moveToGoal(){
//gripper state
    gripper = false;
    vacuum = false;
    this->showGripper();

// transition point
    Arm transition(this->l1, this->l2, this->l3, this->l4, this->chassis);
    transition.setGoal(g_x, g_y, g_z+10);
    printf( "[ Transition Goal Data ]\n" );
    transition.showGoalInfo();
    ros::Duration(3).sleep();   // delay: moving time

// goal point
    printf( "[ Goal data ]\n" );
    this->setGoal(g_x, g_y, g_z);
    this->showGoalInfo();
}

void Arm::moveToGoal2(){
//gripper state - gravbbing 
    gripper = true;
    vacuum = true;
    this->showGripper();
    ros::Duration(2).sleep();   // delay: make sure grip tightly

// transition point
    Arm transition2(this->l1, this->l2, this->l3, this->l4, this->chassis);
    transition2.setGoal(g_x2, g_y2, g_z2+10);
    printf( "[ Transition Goal Data ]\n" );
    transition2.showGoalInfo();
    ros::Duration(3).sleep();   // delay: moving time

// goal point
    printf( "[ Goal data ]\n" );
    this->setGoal(g_x2, g_y2, g_z2);
    this->showGoalInfo();

//gripper state - placing
    gripper = false;
    vacuum = false;
    this->showGripper();
}

void Arm::createArm(){
    cout << "enter goal coordinate: (x,y,z)" << endl;
    cin >> g_x >> g_y >> g_z ;
    this->moveToGoal();

    cout << "enter goal coordinate: (x,y,z)" << endl;
    cin >> g_x2 >> g_y2 >> g_z2;
    this->moveToGoal2();
}
