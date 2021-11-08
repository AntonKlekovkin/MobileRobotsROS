#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <vector>

#define PI 3.1415926

ros::Publisher pub;
ros::Publisher pubTheorTrajectory;
ros::Publisher pubTheorLinVel;
ros::Publisher pubTheorAngVel;
ros::Publisher pubRealLinVel;
ros::Publisher pubRealAngVel;

void mySigintHandler(int sig)
{
    geometry_msgs::Twist messageStop;

    messageStop.angular.z = 0.0;
    pub.publish(messageStop);
    ROS_INFO("I'm stoped sigint");
    
    ros::shutdown();
}

double linVelReal = 0;
double angVelReal = 0;

void msgCallbackOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    linVelReal = msg->twist.twist.linear.x;
    angVelReal = msg->twist.twist.angular.z;
}

int main(int argc, char* argv[])
{
    bool transmit = true;

    ros::init(argc, argv, "motion_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Subscriber subOdom = nh.subscribe("odom", 100, msgCallbackOdom);
    ros::spinOnce();

    signal(SIGINT, mySigintHandler);
    geometry_msgs::Vector3 points;
    geometry_msgs::Vector3 linVel;
    geometry_msgs::Vector3 angVel;
   
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pubTheorTrajectory = nh.advertise<geometry_msgs::Vector3>("theor_trajectory", 1000);
    pubTheorLinVel = nh.advertise<geometry_msgs::Vector3>("theor_lin_vel", 1000);
    pubTheorAngVel = nh.advertise<geometry_msgs::Vector3>("theor_ang_vel", 1000);
    pubRealLinVel = nh.advertise<geometry_msgs::Vector3>("real_lin_vel", 1000);
    pubRealAngVel = nh.advertise<geometry_msgs::Vector3>("real_ang_vel", 1000);
    
    geometry_msgs::Twist myMessage;

    // determing of trajectory points
    
    float dt = 0.01;
    float dtT = 0.01;

    const int numberPoints = 500;
    float x[numberPoints+1];
    float y[numberPoints+1];
    float s[numberPoints+1];
    float t[numberPoints+1];
    int lengthT;

    int i = 0;

    // float rightLimit = 2*PI;
    // for(float s = 0; s <= PI/2; s+=dtT)
    // {
    //     t[i] = sin(s)*sin(s)*rightLimit;
    //     //t[i] = s*4;
    //     ROS_INFO("i=%d, t=%f", i, t[i]);    
    //     i++;
    // }
    const float a1=23.55, b1=0.2186, c1=1.579, a2=23.25, b2=0.2487, c2=4.755, 
        a3=0.1469, b3=1.11, c3=4.841, a4=0.1867, b4=3.519, c4=-1.52, a5=0.301, b5=3.513, c5=4.745;
    for(i = 0; i < numberPoints; i++)
    {
        float s = ((float)i/(float)numberPoints)*8;
        //t[i] = sin(s)*sin(s);
        t[i] = a1*sin(b1*s+c1) + a2*sin(b2*s+c2) + a3*sin(b3*s+c3) + a4*sin(b4*s+c4);// + a5*sin(b5*s+c5);
        //t[i] = s*4;
        ROS_INFO("i=%d, t=%f", i, t[i]);    
    }

    // for(i = 0; i < numberPoints; i++)
    // {
    //     float s = (float)i/(float)numberPoints*PI/2;
    //     t[i] = sin(s)*sin(s)*rightLimit;
    //     //t[i] = s*4;
    //     ROS_INFO("i=%d, t=%f", i, t[i]);    
    // }

    // for(float s = 0; s < PI/2; s+=dtT)
    // {
    //     t[i] = sin(s)*sin(s)*rightLimit/3;
    //     ROS_INFO("i=%d, t=%f", i, t[i]);    
    //     i++;
    // }
    // for(float s = PI/2; s < PI; s+=dtT)
    // {
    //     t[i] = sin(s+PI/2)*sin(s+PI/2)*rightLimit/3 + rightLimit/3;
    //     ROS_INFO("i=%d, t=%f", i, t[i]);    
    //     i++;
    // }
    // for(float s = PI; s < PI*1.5; s+=dtT)
    // {
    //     t[i] = sin(s+PI)*sin(s+PI)*rightLimit/3 + 2*rightLimit/3;
    //     //t[i] = s*4;
    //     ROS_INFO("i=%d, t=%f", i, t[i]);    
    //     i++;
    // }

    lengthT = i-1;
    i = 0;
    float kTr = 1;    
    for(i=0; i < lengthT; i++)
    {
        x[i] = sin(t[i]+(PI/2))*kTr;
        y[i] = sin(2*t[i])*kTr;

        //x[i] = sin(t[i])*kTr;
        //y[i] = cos(t[i])*kTr;

        points.x = y[i];
        points.y = (x[i]-kTr) *(-1);
        pubTheorTrajectory.publish(points);
        ros::Duration(0.01).sleep();    
    }

    int massLength = lengthT;
    float dx, dy;
    float tetta[massLength];
    float dl[massLength];
    float dw[massLength];
    float v[massLength];
    float w[massLength];

    ROS_INFO("Length arr = %d", massLength);

    // determine of angular and linear velocities
    float tettaStart = PI/2;
    float maxLinVelocity = 0;
    float maxAngVelocity = 0;
    float maxLinVelocityReal = 1.5;
    float maxAngVelocityReal = 1.5;
    for(int i = 0; i < massLength-1; i++)
    {
        dx = x[i+1] - x[i];
        dy = y[i+1] - y[i];
        dl[i] = sqrt(dx*dx + dy*dy);
        tetta[i] = atan2(y[i+1] - y[i], x[i+1] - x[i]);
        if(tetta[i] < 0)
        {
            tetta[i] += 2*PI;
        }

        if(i==0)
        {
            dw[i] = tetta[i] - tettaStart;
        }
        else
        {
            dw[i] = tetta[i] - tetta[i-1];
        }

        if(dw[i] > PI)
        {
            dw[i] -= 2*PI;
        }

        if(dw[i] < -PI)
        {
            dw[i] += 2*PI;
        }

        v[i] = dl[i] / dt;
        w[i] = dw[i] / dt;

        if(v[i] > maxLinVelocity)
        {
            maxLinVelocity = v[i];
        }
        if(w[i] > maxAngVelocity)
        {
            maxAngVelocity = w[i];
        }        
    }

    ROS_INFO("maxV = %f, maxW = %f", maxLinVelocity, maxAngVelocity);

    ros::spinOnce();

    // transmit v and w to robot
    float k1,k2,k;
    float coeffSafety = 1;
    k1 = maxLinVelocityReal/maxLinVelocity * coeffSafety;
    k2 = maxAngVelocityReal/maxAngVelocity * coeffSafety;

    if(k1<k2){k=k1;}
    else {k=k2;}

    if(k<0.1) k=0.1;

    //k=k1;
    ROS_INFO("k=%f", k);

    for(int i = 0; i < massLength-1; i++)
    {
        linVel.x = i;
        linVel.y = v[i]*k;
        angVel.x = i;
        angVel.y = w[i]*k;

        pubTheorLinVel.publish(linVel);
        pubTheorAngVel.publish(angVel);
        ros::Duration(0.01).sleep();
    }

    for(int i = 0; i < massLength-1; i++)
    {
        ROS_INFO("i=%d, v=%f, w=%f", i, v[i], w[i]);
                
        myMessage.linear.x = v[i]*k;
        myMessage.angular.z = w[i]*k;
        if(transmit)
        {
            pub.publish(myMessage);

            linVel.x = i;
            linVel.y = linVelReal;
            pubRealLinVel.publish(linVel);
            angVel.x = i;
            angVel.y = angVelReal;
            pubRealAngVel.publish(angVel);

            ros::Duration(dt/k).sleep();
        }
        ros::spinOnce();
    }

    
    myMessage.linear.x = 0.0;
    myMessage.angular.z = 0.0;
    pub.publish(myMessage);
    ros::Duration(1).sleep();
    
    // while(ros::ok())
    // {
    //     // myMessage.angular.z = 1.0;
    //     // pub.publish(myMessage);
    //     // ros::Duration(1).sleep();

    //     // myMessage.angular.z = 0.0;
    //     // pub.publish(myMessage);
    //     // ros::Duration(1).sleep();

    //     //ros::spinOnce();
    // }

    mySigintHandler(1);
    return 0;
    
}