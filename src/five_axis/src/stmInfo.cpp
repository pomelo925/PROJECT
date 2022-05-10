#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

void callback(const std_msgs::Int32MultiArray::ConstPtr &p){
    printf("n: %d\n", p->data);
}

//利用 publish 以 5Hz 每次傳送一組 0和1組成的1X4 陣列，不斷循環，subscribe收到後列印出來。
//0111, 1011, 1101, 1110, 1101, 1011, 0111, 1011, ......
int main(int argc, char** argv){
    while(ros::ok()){
        ros::init(argc, argv, "stminfo");
        ros::NodeHandle nh;
        ros::Subscriber sub_test = nh.subscribe("TEST", 1, callback);
        ros::spinOnce();
    }
}