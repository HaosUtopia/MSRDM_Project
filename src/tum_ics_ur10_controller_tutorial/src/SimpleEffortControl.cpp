#include<tum_ics_ur10_controller_tutorial/SimpleEffortControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

SimpleEffortControl::SimpleEffortControl(double weight, const QString &name):
    ControlEffort(name,SPLINE_TYPE,JOINT_SPACE,weight),
    m_startFlag(false),
    m_Kp(Matrix6d::Zero()),
    m_Kd(Matrix6d::Zero()),
    m_Ki(Matrix6d::Zero()),
    m_goal(Vector6d::Zero()),
    m_DeltaQ(Vector6d::Zero()),
    m_DeltaQ_1(Vector6d::Zero()),
    m_DeltaQp(Vector6d::Zero()),
    m_DeltaQp_1(Vector6d::Zero()),
    m_iDeltaQ(Vector6d::Zero()),
    m_iDeltaQ_1(Vector6d::Zero())
{
    L1 = 0.1273;
    L2 = 0.6127;
    L3 = 0.5716;
    L4 = 0.1639;
    L5 = 0.1157;
    L6 = 0.0922;
    L7 = 0.1273;
    L8 = 0.1759939;
    L9 = 0.306;
    L10 = 0.047077;
    L11 = 0.28615;
    L12 = 0.162077;
    L13 = 0.1157;
    L14 = 0.0922;

    m1 = 7.778; m2 = 12.93; m3 = 3.87; m4 = 1.96; m5 = 1.96; m6 = 0.202;
    I111 = 0.03; I122 = 0.03; I133 = 0.02; I211 = 0.42; I222 = 0.42; I233 = 0.04;
    I311 = 0.11; I322 = 0.11; I333 = 0.01; I411 = 0.01; I422 = 0.01; I433 = 0.01;
    I511 = 0.01; I522 = 0.01; I533 = 0.01; I611 = 0.0; I622 = 0.0; I633 = 0.0;
    g = 9.81; gx = 0.0; gy = 0.0; gz = 1.0; 
    b1 = 0; b2 = 0; b3 = 0; b4 = 0; b5 = 0; b6 = 0; 
    
    m_GoalX.setZero();
    m_GoalXp.setZero();
    m_GoalXpp.setZero();


    Theta(0,0) = I122+(L4*L4)*m5+(L4*L4)*m6+(L8*L8)*m2+(L10*L10)*m3+(L12*L12)*m4;
    Theta(1,0) = I211;
    Theta(2,0) = I322;
    Theta(3,0) = I411;
    Theta(4,0) = b1;
    Theta(5,0) = I222;
    Theta(6,0) = I311;
    Theta(7,0) = I433;
    Theta(8,0) = (L3*L3)*m4+(L3*L3)*m5+(L3*L3)*m6+(L11*L11)*m3;
    Theta(9,0) = (L2*L2)*(m3+m4)+(L2*L2)*m5+(L2*L2)*m6+(L9*L9)*m2;
    Theta(10,0) = I511;
    Theta(11,0) = I522;
    Theta(12,0) = I533;
    Theta(13,0) = I611;
    Theta(14,0) = I622;
    Theta(15,0) = I633;
    Theta(16,0) = (L14*L14)*m6;
    Theta(17,0) = L2*L4*(m5+m6)+L2*L10*m3+L2*L12*m4;
    Theta(18,0) = L8*L9*m2;
    Theta(19,0) = g*gx*(L4*m5+L4*m6+L8*m2+L10*m3+L12*m4);
    Theta(20,0) = g*gy*(L4*m5+L4*m6+L8*m2+L10*m3+L12*m4);
    Theta(21,0) = (L5*L5)*m6+(L13*L13)*m5;
    Theta(22,0) = L3*(L5*m6+L13*m5);
    Theta(23,0) = L2*(L3*m4+L11*m3);
    Theta(24,0) = L2*L3*(m5+m6);
    Theta(25,0) = L5*L14*m6;
    Theta(26,0) = L4*L14*m6;
    Theta(27,0) = L3*L4*(m5+m6)+L3*L12*m4+L10*L11*m3;
    Theta(28,0) = L2*L14*m6;
    Theta(29,0) = L14*g*gx*m6;
    Theta(30,0) = L2*g*gy*(m3+m4+m5+m6);
    Theta(31,0) = L9*g*gy*m2;
    Theta(32,0) = L2*g*gx*(m3+m4+m5+m6);
    Theta(33,0) = L9*g*gx*m2;
    Theta(34,0) = L14*g*gy*m6;
    Theta(35,0) = L2*(L5*m6+L13*m5);
    Theta(36,0) = L3*L14*m6;
    Theta(37,0) = L4*(L5*m6+L13*m5);
    Theta(38,0) = L3*g*gy*(m4+m5+m6)+L11*g*gy*m3;
    Theta(39,0) = L3*g*gx*(m4+m5+m6)+L11*g*gx*m3;
    Theta(40,0) = g*gy*(L5*m6+L13*m5);
    Theta(41,0) = g*gx*(L5*m6+L13*m5);
    Theta(42,0) = I233;
    Theta(43,0) = I333;
    Theta(44,0) = I422;
    Theta(45,0) = b2;
    Theta(46,0) = L2*g*gz*(m3+m4+m5+m6);
    Theta(47,0) = L9*g*gz*m2;
    Theta(48,0) = L3*g*gz*(m4+m5+m6)+L11*g*gz*m3;
    Theta(49,0) = g*gz*(L5*m6+L13*m5);
    Theta(50,0) = L14*g*gz*m6;
    Theta(51,0) = b3;
    Theta(52,0) = b4;
    Theta(53,0) = b5;
    Theta(54,0) = b6;

    pubCtrlData = n.advertise<tum_ics_ur_robot_msgs::ControlData>("SimpleEffortCtrlData",100);
    subGoal = n.subscribe("/trajectory_generator/position",100,&SimpleEffortControl::setGoal,this);
    srvStart = n.serviceClient<std_srvs::Empty>("/trajectory_start");
    srvModLED = n.serviceClient<tum_ics_skin_msgs::setSkinCellLedColor>("/setSkinCellLedColor");
    pubPath = n.advertise<nav_msgs::Path>("drawing_path",100);

    J_.setZero();

    path.header.frame_id = "world";

    //m_controlPeriod=0.002; //set the control period to the standard 2 ms
    //m_controlPeriod_2=m_controlPeriod/2.0;

    //ROS_INFO_STREAM("SimpleEffortCtrl Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

    

}

SimpleEffortControl::~SimpleEffortControl()
{

}

void SimpleEffortControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void SimpleEffortControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void SimpleEffortControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}

void SimpleEffortControl::setGoal(const trajectory_generator::TrajectoryPosition::ConstPtr& msg){
    if(m_GoalLED != msg->led){
        m_GoalLED = msg->led;
        ROS_INFO_STREAM("Change LED from "<<!msg->led<<" to " << int(msg->led));
        tum_ics_skin_msgs::setSkinCellLedColor msgAlt;
        tum_ics_skin_msgs::SkinCellLedColor color;
        color.cellId = 43;
        color.r = msg->led ? 255 : 0;
        color.g = msg->led ? 255 : 0;
        color.b = msg->led ? 255 : 0;
        msgAlt.request.color.push_back(color);
        //color.cellId = 43;
        //msgAlt.request.color.push_back(color);
        //color.cellId = 44;
        //msgAlt.request.color.push_back(color);

        srvModLED.call(msgAlt);

    }
    if(msg->x>=100){
        path.poses.clear();
        return;
    }
    m_GoalX[1] = msg->x;
    m_GoalX[2] = msg->y;
    m_GoalXp[1] = msg->vx;
    m_GoalXp[2] = msg->vy;
    m_GoalXpp[1] = msg->ax;
    m_GoalXpp[2] = msg->ay;
    // ROS_INFO_STREAM("Tracking Goal: X Y LED"<< m_GoalX[1]<<", "<<m_GoalX[2]<<", "<<m_GoalLED);
}

Vector7d SimpleEffortControl::FK(Vector6d Q){
    using namespace std;

    Vector7d X;

    double q1,q2,q3,q4,q5,q6;

    q1 = Q[0];
    q2 = Q[1];
    q3 = Q[2];
    q4 = Q[3];
    q5 = Q[4];
    q6 = Q[5];

    X(0,0) = -L6*(cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5))+cos(q1)*(L3*cos(q2+q3)+L2*cos(q2))-L4*sin(q1)-L5*sin(q2+q3+q4)*cos(q1);
    X(1,0) = L6*(cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5))+sin(q1)*(L3*cos(q2+q3)+L2*cos(q2))+L4*cos(q1)-L5*sin(q2+q3+q4)*sin(q1);
    X(2,0) = L1-L3*sin(q2+q3)-L2*sin(q2)-L5*cos(q2+q3+q4)-L6*sin(q2+q3+q4)*sin(q5);
    
    X(3,0) = sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)/2.0;
    X(4,0) = (cos(q1)*cos(q5)+cos(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q5)-sin(q2+q3+q4)*cos(q5)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0);
    X(5,0) = ((-cos(q5)*sin(q1)+cos(q2+q3+q4)*sin(q6)+cos(q2+q3+q4)*cos(q1)*sin(q5)+sin(q2+q3+q4)*cos(q5)*cos(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0;
    X(6,0) = (cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0);

    // X(3,0) = atan2(-cos(q2+q3+q4)*cos(q6)+sin(q2+q3+q4)*cos(q5)*sin(q6),-sin(q2+q3+q4)*sin(q5));
    // X(4,0) = atan2(cos(q2+q3+q4)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6),sqrt(-pow(cos(q2+q3+q4)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6),2.0)+1.0));
    // X(5,0) = atan2(-cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*sin(q1)*sin(q6),cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*sin(q6));

    return X;
}

Eigen::Matrix<double,7,6> SimpleEffortControl::Jacobian(Vector6d Q){
    using namespace std;

    Eigen::Matrix<double,7,6> Jef;

    double q1,q2,q3,q4,q5,q6;

    q1 = Q[0];
    q2 = Q[1];
    q3 = Q[2];
    q4 = Q[3];
    q5 = Q[4];
    q6 = Q[5];

    Jef(0,0)=L5*sin(q2 + q3 + q4)*sin(q1) - sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2)) - L4*cos(q1) - L6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5));
    Jef(0,1)=-cos(q1)*(L3*sin(q2 + q3) + L2*sin(q2) + L5*cos(q2 + q3 + q4) + L6*sin(q2 + q3 + q4)*sin(q5));
    Jef(0,2)=-cos(q1)*(L3*sin(q2 + q3) + L5*cos(q2 + q3 + q4) + L6*sin(q2 + q3 + q4)*sin(q5));
    Jef(0,3)=-cos(q1)*(L5*cos(q2 + q3 + q4) + L6*sin(q2 + q3 + q4)*sin(q5));
    Jef(0,4)=L6*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5));
    Jef(0,5)=0;
    Jef(1,0)=cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2)) - L6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - L4*sin(q1) - L5*sin(q2 + q3 + q4)*cos(q1);
    Jef(1,1)=-sin(q1)*(L3*sin(q2 + q3) + L2*sin(q2) + L5*cos(q2 + q3 + q4) + L6*sin(q2 + q3 + q4)*sin(q5));
    Jef(1,2)=-sin(q1)*(L3*sin(q2 + q3) + L5*cos(q2 + q3 + q4) + L6*sin(q2 + q3 + q4)*sin(q5));
    Jef(1,3)=-sin(q1)*(L5*cos(q2 + q3 + q4) + L6*sin(q2 + q3 + q4)*sin(q5));
    Jef(1,4)=L6*cos(q2 + q3 + q4)*cos(q5)*sin(q1) - L6*cos(q1)*sin(q5);
    Jef(1,5)=0;
    Jef(2,0)=0;
    Jef(2,1)=(L6*sin(q2 + q3 + q4 - q5))/2 - L3*cos(q2 + q3) - L2*cos(q2) - (L6*sin(q2 + q3 + q4 + q5))/2 + L5*sin(q2 + q3 + q4);
    Jef(2,2)=(L6*sin(q2 + q3 + q4 - q5))/2 - L3*cos(q2 + q3) - (L6*sin(q2 + q3 + q4 + q5))/2 + L5*sin(q2 + q3 + q4);
    Jef(2,3)=L5*sin(q2 + q3 + q4) - L6*cos(q2 + q3 + q4)*sin(q5);
    Jef(2,4)=-L6*sin(q2 + q3 + q4)*cos(q5);
    Jef(2,5)=0;

    Jef(3,0)= ((cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/4.0;
    Jef(3,1)= (cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/4.0);
    Jef(3,2)= (cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/4.0);
    Jef(3,3)= (cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/4.0);
    Jef(3,4)= ((-sin(q2+q3+q4)*cos(q5)+cos(q6)*(cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5))+sin(q6)*(cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5)))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/4.0;
    Jef(3,5)= ((cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/4.0;
    Jef(4,0)= ((cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0+((cos(q1)*cos(q5)+cos(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q5)-sin(q2+q3+q4)*cos(q5)*sin(q6))*(cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(4,1)= ((sin(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*cos(q5)*sin(q6)+sin(q2+q3+q4)*sin(q1)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0-((cos(q1)*cos(q5)+cos(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q5)-sin(q2+q3+q4)*cos(q5)*sin(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(4,2)= ((sin(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*cos(q5)*sin(q6)+sin(q2+q3+q4)*sin(q1)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0-((cos(q1)*cos(q5)+cos(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q5)-sin(q2+q3+q4)*cos(q5)*sin(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(4,3)= ((sin(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*cos(q5)*sin(q6)+sin(q2+q3+q4)*sin(q1)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0-((cos(q1)*cos(q5)+cos(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q5)-sin(q2+q3+q4)*cos(q5)*sin(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(4,4)= (-cos(q1)*sin(q5)+cos(q2+q3+q4)*cos(q5)*sin(q1)+sin(q2+q3+q4)*sin(q5)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)+((-sin(q2+q3+q4)*cos(q5)+cos(q6)*(cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5))+sin(q6)*(cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5)))*(cos(q1)*cos(q5)+cos(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q5)-sin(q2+q3+q4)*cos(q5)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(4,5)= ((cos(q2+q3+q4)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0+((cos(q1)*cos(q5)+cos(q2+q3+q4)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q5)-sin(q2+q3+q4)*cos(q5)*sin(q6))*(cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(5,0)= (cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)-((-cos(q5)*sin(q1)+cos(q2+q3+q4)*sin(q6)+cos(q2+q3+q4)*cos(q1)*sin(q5)+sin(q2+q3+q4)*cos(q5)*cos(q6))*(cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(5,1)= (sin(q2+q3+q4)*sin(q6)-cos(q2+q3+q4)*cos(q5)*cos(q6)+sin(q2+q3+q4)*cos(q1)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)+((-cos(q5)*sin(q1)+cos(q2+q3+q4)*sin(q6)+cos(q2+q3+q4)*cos(q1)*sin(q5)+sin(q2+q3+q4)*cos(q5)*cos(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(5,2)= (sin(q2+q3+q4)*sin(q6)-cos(q2+q3+q4)*cos(q5)*cos(q6)+sin(q2+q3+q4)*cos(q1)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)+((-cos(q5)*sin(q1)+cos(q2+q3+q4)*sin(q6)+cos(q2+q3+q4)*cos(q1)*sin(q5)+sin(q2+q3+q4)*cos(q5)*cos(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(5,3)= (sin(q2+q3+q4)*sin(q6)-cos(q2+q3+q4)*cos(q5)*cos(q6)+sin(q2+q3+q4)*cos(q1)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)+((-cos(q5)*sin(q1)+cos(q2+q3+q4)*sin(q6)+cos(q2+q3+q4)*cos(q1)*sin(q5)+sin(q2+q3+q4)*cos(q5)*cos(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(5,4)= ((sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5)-sin(q2+q3+q4)*cos(q6)*sin(q5))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0-((-sin(q2+q3+q4)*cos(q5)+cos(q6)*(cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5))+sin(q6)*(cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5)))*(-cos(q5)*sin(q1)+cos(q2+q3+q4)*sin(q6)+cos(q2+q3+q4)*cos(q1)*sin(q5)+sin(q2+q3+q4)*cos(q5)*cos(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(5,5)= ((cos(q2+q3+q4)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q6))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0-((-cos(q5)*sin(q1)+cos(q2+q3+q4)*sin(q6)+cos(q2+q3+q4)*cos(q1)*sin(q5)+sin(q2+q3+q4)*cos(q5)*cos(q6))*(cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(6,0)= ((cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0+(pow(cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6),2.0)*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(6,1)= (-cos(q2+q3+q4)*cos(q1)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q6)+sin(q2+q3+q4)*cos(q1)*cos(q5)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6)*sin(q1))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)-((cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(6,2)= (-cos(q2+q3+q4)*cos(q1)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q6)+sin(q2+q3+q4)*cos(q1)*cos(q5)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6)*sin(q1))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)-((cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(6,3)= (-cos(q2+q3+q4)*cos(q1)*cos(q6)+cos(q2+q3+q4)*sin(q1)*sin(q6)+sin(q2+q3+q4)*cos(q1)*cos(q5)*sin(q6)+sin(q2+q3+q4)*cos(q5)*cos(q6)*sin(q1))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)-((cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*(cos(q2+q3+q4)*sin(q5)+cos(q2+q3+q4)*cos(q1)*sin(q6)+cos(q2+q3+q4)*cos(q6)*sin(q1)+sin(q2+q3+q4)*cos(q1)*cos(q5)*cos(q6)-sin(q2+q3+q4)*cos(q5)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(6,4)= (cos(q6)*(cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5))-sin(q6)*(cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5)))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0)*(-1.0/2.0)+((-sin(q2+q3+q4)*cos(q5)+cos(q6)*(cos(q5)*sin(q1)-cos(q2+q3+q4)*cos(q1)*sin(q5))+sin(q6)*(cos(q1)*cos(q5)+cos(q2+q3+q4)*sin(q1)*sin(q5)))*(cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6))*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;
    Jef(6,5)= ((cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1))*1.0/sqrt(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0))/2.0+(pow(cos(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))-sin(q2+q3+q4)*cos(q1)*cos(q6)+sin(q2+q3+q4)*sin(q1)*sin(q6),2.0)*1.0/pow(-sin(q2+q3+q4)*sin(q5)+cos(q6)*(sin(q1)*sin(q5)+cos(q2+q3+q4)*cos(q1)*cos(q5))+sin(q6)*(cos(q1)*sin(q5)-cos(q2+q3+q4)*cos(q5)*sin(q1))-sin(q2+q3+q4)*cos(q1)*sin(q6)-sin(q2+q3+q4)*cos(q6)*sin(q1)+1.0,3.0/2.0))/4.0;

    // Jef(3,0)=0;
    // Jef(3,1)=-sin(q1);
    // Jef(3,2)=-sin(q1);
    // Jef(3,3)=-sin(q1);
    // Jef(3,4)=-sin(q2 + q3 + q4)*cos(q1);
    // Jef(3,5)=cos(q2 + q3 + q4)*cos(q1)*sin(q5) - cos(q5)*sin(q1);
    // Jef(4,0)=0;
    // Jef(4,1)=cos(q1);
    // Jef(4,2)=cos(q1);
    // Jef(4,3)=cos(q1);
    // Jef(4,4)=-sin(q2 + q3 + q4)*sin(q1);
    // Jef(4,5)=cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5);
    // Jef(5,0)=1;
    // Jef(5,1)=0;
    // Jef(5,2)=0;
    // Jef(5,3)=0;
    // Jef(5,4)=-cos(q2 + q3 + q4);
    // Jef(5,5)=-sin(q2 + q3 + q4)*sin(q5);
    
    return Jef;
}

Eigen::Matrix<double,6,55> SimpleEffortControl::evalYr(Vector6d Q, Vector6d Qp, Vector6d Qrp, Vector6d Qrpp){
    double q1,q2,q3,q4,q5,q6;
    double qp1,qp2,qp3,qp4,qp5,qp6;
    double qrp1,qrp2,qrp3,qrp4,qrp5,qrp6;
    double qrpp1,qrpp2,qrpp3,qrpp4,qrpp5,qrpp6;

    q1 = Q[0]; q2 = Q[1]; q3 = Q[2]; q4 = Q[3]; q5 = Q[4]; q6 = Q[5];
    qp1 = Qp[0]; qp2 = Qp[1]; qp3 = Qp[2]; qp4 = Qp[3]; qp5 = Qp[4]; qp6 = Qp[5];
    qrp1 = Qrp[0]; qrp2 = Qrp[1]; qrp3 = Qrp[2]; qrp4 = Qrp[3]; qrp5 = Qrp[4]; qrp6 = Qrp[5];
    qrpp1 = Qrpp[0]; qrpp2 = Qrpp[1]; qrpp3 = Qrpp[2]; qrpp4 = Qrpp[3]; qrpp5 = Qrpp[4]; qrpp6 = Qrpp[5]; 

    Yr.setZero();
    Yr(0,0) = qrpp1;
    Yr(0,1) = qrpp1/2.0-(qrpp1*cos(q2*2.0))/2.0+(qp1*qrp2*sin(q2*2.0))/2.0+(qp2*qrp1*sin(q2*2.0))/2.0;
    Yr(0,2) = qrpp1/2.0+(qrpp1*cos(q2*2.0+q3*2.0))/2.0-(qp1*qrp2*sin(q2*2.0+q3*2.0))/2.0-(qp2*qrp1*sin(q2*2.0+q3*2.0))/2.0-(qp1*qrp3*sin(q2*2.0+q3*2.0))/2.0-(qp3*qrp1*sin(q2*2.0+q3*2.0))/2.0;
    Yr(0,3) = qrpp1/2.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(0,4) = qrp1;
    Yr(0,5) = qrpp1/2.0+(qrpp1*cos(q2*2.0))/2.0-(qp1*qrp2*sin(q2*2.0))/2.0-(qp2*qrp1*sin(q2*2.0))/2.0;
    Yr(0,6) = qrpp1/2.0-(qrpp1*cos(q2*2.0+q3*2.0))/2.0+(qp1*qrp2*sin(q2*2.0+q3*2.0))/2.0+(qp2*qrp1*sin(q2*2.0+q3*2.0))/2.0+(qp1*qrp3*sin(q2*2.0+q3*2.0))/2.0+(qp3*qrp1*sin(q2*2.0+q3*2.0))/2.0;
    Yr(0,7) = qrpp1/2.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(0,8) = qrpp1/2.0+(qrpp1*cos(q2*2.0+q3*2.0))/2.0-(qp1*qrp2*sin(q2*2.0+q3*2.0))/2.0-(qp2*qrp1*sin(q2*2.0+q3*2.0))/2.0-(qp1*qrp3*sin(q2*2.0+q3*2.0))/2.0-(qp3*qrp1*sin(q2*2.0+q3*2.0))/2.0;
    Yr(0,9) = qrpp1/2.0+(qrpp1*cos(q2*2.0))/2.0-(qp1*qrp2*sin(q2*2.0))/2.0-(qp2*qrp1*sin(q2*2.0))/2.0;
    Yr(0,10) = qrpp1/4.0+(qrpp1*cos(q5*2.0))/4.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/4.0+(qrpp2*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp2*cos(q2+q3+q4+q5*2.0))/4.0+(qrpp3*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp3*cos(q2+q3+q4+q5*2.0))/4.0+(qrpp4*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp4*cos(q2+q3+q4+q5*2.0))/4.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp2*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp2*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp5*sin(q5*2.0))/4.0-(qp5*qrp1*sin(q5*2.0))/4.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(0,11) = qrpp1/2.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/2.0-qrpp5*cos(q2+q3+q4)+(qp2*qrp5*sin(q2+q3+q4))/2.0+(qp5*qrp2*sin(q2+q3+q4))/2.0+(qp3*qrp5*sin(q2+q3+q4))/2.0+(qp5*qrp3*sin(q2+q3+q4))/2.0+(qp4*qrp5*sin(q2+q3+q4))/2.0+(qp5*qrp4*sin(q2+q3+q4))/2.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/2.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(0,12) = qrpp1/4.0-(qrpp1*cos(q5*2.0))/4.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/4.0-(qrpp2*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp2*cos(q2+q3+q4+q5*2.0))/4.0-(qrpp3*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp3*cos(q2+q3+q4+q5*2.0))/4.0-(qrpp4*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp4*cos(q2+q3+q4+q5*2.0))/4.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp2*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp2*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp5*sin(q5*2.0))/4.0+(qp5*qrp1*sin(q5*2.0))/4.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(0,13) = qrpp1*(3.0/8.0)+(qrpp1*cos(q5*2.0))/8.0-(qrpp1*cos(q6*2.0))/8.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/8.0+(qrpp2*cos(q2+q3+q4+q5-q6*2.0))/8.0-(qrpp2*cos(q2+q3+q4+q5+q6*2.0))/8.0+(qrpp3*cos(q2+q3+q4+q5-q6*2.0))/8.0-(qrpp3*cos(q2+q3+q4+q5+q6*2.0))/8.0+(qrpp4*cos(q2+q3+q4+q5-q6*2.0))/8.0-(qrpp4*cos(q2+q3+q4+q5+q6*2.0))/8.0-(qrpp5*cos(q2+q3+q4+q5-q6*2.0))/8.0+(qrpp5*cos(q2+q3+q4+q5+q6*2.0))/8.0+(qrpp1*cos(q5*2.0-q6*2.0))/1.6E+1+(qrpp1*cos(q5*2.0+q6*2.0))/1.6E+1+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0-(qrpp2*cos(q2+q3+q4-q5-q6*2.0))/8.0+(qrpp2*cos(q2+q3+q4-q5+q6*2.0))/8.0+(qrpp2*cos(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qrpp2*cos(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qrpp2*cos(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qrpp2*cos(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qrpp3*cos(q2+q3+q4-q5-q6*2.0))/8.0+(qrpp3*cos(q2+q3+q4-q5+q6*2.0))/8.0+(qrpp3*cos(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qrpp3*cos(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qrpp3*cos(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qrpp3*cos(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qrpp4*cos(q2+q3+q4-q5-q6*2.0))/8.0+(qrpp4*cos(q2+q3+q4-q5+q6*2.0))/8.0+(qrpp4*cos(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qrpp4*cos(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qrpp4*cos(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qrpp4*cos(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qrpp5*cos(q2+q3+q4-q5-q6*2.0))/8.0+(qrpp5*cos(q2+q3+q4-q5+q6*2.0))/8.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qrpp2*cos(q2+q3+q4-q5*2.0))/8.0-(qrpp2*cos(q2+q3+q4+q5*2.0))/8.0+(qrpp3*cos(q2+q3+q4-q5*2.0))/8.0-(qrpp3*cos(q2+q3+q4+q5*2.0))/8.0+(qrpp4*cos(q2+q3+q4-q5*2.0))/8.0-(qrpp4*cos(q2+q3+q4+q5*2.0))/8.0+(qrpp5*cos(q2+q3+q4-q6*2.0))/4.0+(qrpp5*cos(q2+q3+q4+q6*2.0))/4.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1-qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)-(qrpp5*cos(q2+q3+q4))/2.0+(qp2*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp2*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp2*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp2*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp3*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp3*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp2*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp2*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp3*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp3*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp4*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp4*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp2*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp3*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp4*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp4*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp2*qrp5*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp5*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp5*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp5*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp2*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp2*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp3*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp4*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp5*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp5*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp5*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp5*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp6*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp2*qrp6*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp6*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp2*qrp6*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp6*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp5*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp5*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp5*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp5*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp3*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp4*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp5*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp5*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp5*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp5*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp6*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp6*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp6*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp3*qrp6*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp6*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp6*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp6*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp5*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp5*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp5*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp5*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp4*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp5*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp5*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp5*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp5*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp5*qrp5*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp5*qrp5*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp6*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp6*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp6*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp6*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp4*qrp6*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp6*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp6*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp6*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp5*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp5*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp6*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp6*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp6*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp5*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp6*qrp5*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/1.6E+1-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/1.6E+1+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/1.6E+1-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/1.6E+1-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp2*qrp2*sin(q2+q3+q4-q5*2.0))/8.0+(qp2*qrp2*sin(q2+q3+q4+q5*2.0))/8.0-(qp2*qrp3*sin(q2+q3+q4-q5*2.0))/8.0+(qp2*qrp3*sin(q2+q3+q4+q5*2.0))/8.0-(qp3*qrp2*sin(q2+q3+q4-q5*2.0))/8.0+(qp3*qrp2*sin(q2+q3+q4+q5*2.0))/8.0-(qp2*qrp4*sin(q2+q3+q4-q5*2.0))/8.0+(qp2*qrp4*sin(q2+q3+q4+q5*2.0))/8.0-(qp3*qrp3*sin(q2+q3+q4-q5*2.0))/8.0+(qp3*qrp3*sin(q2+q3+q4+q5*2.0))/8.0-(qp4*qrp2*sin(q2+q3+q4-q5*2.0))/8.0+(qp4*qrp2*sin(q2+q3+q4+q5*2.0))/8.0+(qp2*qrp5*sin(q2+q3+q4-q5*2.0))/8.0+(qp2*qrp5*sin(q2+q3+q4+q5*2.0))/8.0-(qp3*qrp4*sin(q2+q3+q4-q5*2.0))/8.0+(qp3*qrp4*sin(q2+q3+q4+q5*2.0))/8.0-(qp4*qrp3*sin(q2+q3+q4-q5*2.0))/8.0+(qp4*qrp3*sin(q2+q3+q4+q5*2.0))/8.0+(qp5*qrp2*sin(q2+q3+q4-q5*2.0))/8.0+(qp5*qrp2*sin(q2+q3+q4+q5*2.0))/8.0-(qp2*qrp5*sin(q2+q3+q4-q6*2.0))/8.0-(qp2*qrp5*sin(q2+q3+q4+q6*2.0))/8.0+(qp3*qrp5*sin(q2+q3+q4-q5*2.0))/8.0+(qp3*qrp5*sin(q2+q3+q4+q5*2.0))/8.0-(qp4*qrp4*sin(q2+q3+q4-q5*2.0))/8.0+(qp4*qrp4*sin(q2+q3+q4+q5*2.0))/8.0-(qp5*qrp2*sin(q2+q3+q4-q6*2.0))/8.0-(qp5*qrp2*sin(q2+q3+q4+q6*2.0))/8.0+(qp5*qrp3*sin(q2+q3+q4-q5*2.0))/8.0+(qp5*qrp3*sin(q2+q3+q4+q5*2.0))/8.0-(qp3*qrp5*sin(q2+q3+q4-q6*2.0))/8.0-(qp3*qrp5*sin(q2+q3+q4+q6*2.0))/8.0+(qp4*qrp5*sin(q2+q3+q4-q5*2.0))/8.0+(qp4*qrp5*sin(q2+q3+q4+q5*2.0))/8.0-(qp5*qrp3*sin(q2+q3+q4-q6*2.0))/8.0-(qp5*qrp3*sin(q2+q3+q4+q6*2.0))/8.0+(qp5*qrp4*sin(q2+q3+q4-q5*2.0))/8.0+(qp5*qrp4*sin(q2+q3+q4+q5*2.0))/8.0-(qp4*qrp5*sin(q2+q3+q4-q6*2.0))/8.0-(qp4*qrp5*sin(q2+q3+q4+q6*2.0))/8.0-(qp5*qrp4*sin(q2+q3+q4-q6*2.0))/8.0-(qp5*qrp4*sin(q2+q3+q4+q6*2.0))/8.0+(qp5*qrp6*sin(q2+q3+q4-q6*2.0))/4.0-(qp5*qrp6*sin(q2+q3+q4+q6*2.0))/4.0+(qp6*qrp5*sin(q2+q3+q4-q6*2.0))/4.0-(qp6*qrp5*sin(q2+q3+q4+q6*2.0))/4.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1-qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)-qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp2*qrp5*sin(q2+q3+q4))/4.0+(qp5*qrp2*sin(q2+q3+q4))/4.0+(qp3*qrp5*sin(q2+q3+q4))/4.0+(qp5*qrp3*sin(q2+q3+q4))/4.0+(qp4*qrp5*sin(q2+q3+q4))/4.0+(qp5*qrp4*sin(q2+q3+q4))/4.0-(qp1*qrp5*sin(q5*2.0))/8.0-(qp5*qrp1*sin(q5*2.0))/8.0+(qp1*qrp6*sin(q6*2.0))/8.0+(qp6*qrp1*sin(q6*2.0))/8.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp2*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp2*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp2*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp2*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp3*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp3*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp2*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp2*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp3*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp3*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp4*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp4*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp3*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp3*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp4*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp4*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp2*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp2*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp4*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp4*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp6*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp6*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp3*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp3*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp6*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp6*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp4*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp4*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp5*qrp5*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp5*qrp5*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp6*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp6*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp5*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp5*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp6*qrp5*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp6*qrp5*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp1*qrp5*sin(q5*2.0-q6*2.0))/1.6E+1-(qp1*qrp5*sin(q5*2.0+q6*2.0))/1.6E+1-(qp5*qrp1*sin(q5*2.0-q6*2.0))/1.6E+1-(qp5*qrp1*sin(q5*2.0+q6*2.0))/1.6E+1+(qp1*qrp6*sin(q5*2.0-q6*2.0))/1.6E+1-(qp1*qrp6*sin(q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp1*sin(q5*2.0-q6*2.0))/1.6E+1-(qp6*qrp1*sin(q5*2.0+q6*2.0))/1.6E+1-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/1.6E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/1.6E+1-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/1.6E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/1.6E+1+(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0+(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0;
    Yr(0,14) = qrpp1*(3.0/8.0)+(qrpp1*cos(q5*2.0))/8.0+(qrpp1*cos(q6*2.0))/8.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/8.0-(qrpp2*cos(q2+q3+q4+q5-q6*2.0))/8.0+(qrpp2*cos(q2+q3+q4+q5+q6*2.0))/8.0-(qrpp3*cos(q2+q3+q4+q5-q6*2.0))/8.0+(qrpp3*cos(q2+q3+q4+q5+q6*2.0))/8.0-(qrpp4*cos(q2+q3+q4+q5-q6*2.0))/8.0+(qrpp4*cos(q2+q3+q4+q5+q6*2.0))/8.0+(qrpp5*cos(q2+q3+q4+q5-q6*2.0))/8.0-(qrpp5*cos(q2+q3+q4+q5+q6*2.0))/8.0-(qrpp1*cos(q5*2.0-q6*2.0))/1.6E+1-(qrpp1*cos(q5*2.0+q6*2.0))/1.6E+1-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0+(qrpp2*cos(q2+q3+q4-q5-q6*2.0))/8.0-(qrpp2*cos(q2+q3+q4-q5+q6*2.0))/8.0-(qrpp2*cos(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qrpp2*cos(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qrpp2*cos(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qrpp2*cos(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qrpp3*cos(q2+q3+q4-q5-q6*2.0))/8.0-(qrpp3*cos(q2+q3+q4-q5+q6*2.0))/8.0-(qrpp3*cos(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qrpp3*cos(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qrpp3*cos(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qrpp3*cos(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qrpp4*cos(q2+q3+q4-q5-q6*2.0))/8.0-(qrpp4*cos(q2+q3+q4-q5+q6*2.0))/8.0-(qrpp4*cos(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qrpp4*cos(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qrpp4*cos(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qrpp4*cos(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qrpp5*cos(q2+q3+q4-q5-q6*2.0))/8.0-(qrpp5*cos(q2+q3+q4-q5+q6*2.0))/8.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qrpp2*cos(q2+q3+q4-q5*2.0))/8.0-(qrpp2*cos(q2+q3+q4+q5*2.0))/8.0+(qrpp3*cos(q2+q3+q4-q5*2.0))/8.0-(qrpp3*cos(q2+q3+q4+q5*2.0))/8.0+(qrpp4*cos(q2+q3+q4-q5*2.0))/8.0-(qrpp4*cos(q2+q3+q4+q5*2.0))/8.0-(qrpp5*cos(q2+q3+q4-q6*2.0))/4.0-(qrpp5*cos(q2+q3+q4+q6*2.0))/4.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)+qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)-(qrpp5*cos(q2+q3+q4))/2.0-(qp2*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp2*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp2*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp2*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp2*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp2*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp3*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp3*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp2*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp2*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp2*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp2*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp3*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp3*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp4*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp4*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp2*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp2*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp3*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp4*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp4*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp2*qrp5*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp5*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp2*qrp5*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp5*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp2*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp3*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp4*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp5*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp5*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp5*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp5*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp6*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp2*qrp6*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp6*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp6*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp2*qrp6*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp5*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp5*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp5*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp5*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp3*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp4*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp5*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp5*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp5*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp5*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp6*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp6*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp6*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp3*qrp6*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp6*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp6*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp3*qrp6*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp5*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp5*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp5*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp5*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp4*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp5*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1-(qp5*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1-(qp5*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp5*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp5*qrp5*sin(q2+q3+q4-q5-q6*2.0))/8.0-(qp5*qrp5*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp6*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp6*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp6*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp6*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp4*qrp6*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp6*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp6*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp4*qrp6*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp5*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp5*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp6*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp6*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1-(qp6*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp6*qrp5*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp6*qrp5*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/1.6E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/1.6E+1-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/1.6E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/1.6E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/8.0-(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/8.0+(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1+(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1-(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp2*qrp2*sin(q2+q3+q4-q5*2.0))/8.0+(qp2*qrp2*sin(q2+q3+q4+q5*2.0))/8.0-(qp2*qrp3*sin(q2+q3+q4-q5*2.0))/8.0+(qp2*qrp3*sin(q2+q3+q4+q5*2.0))/8.0-(qp3*qrp2*sin(q2+q3+q4-q5*2.0))/8.0+(qp3*qrp2*sin(q2+q3+q4+q5*2.0))/8.0-(qp2*qrp4*sin(q2+q3+q4-q5*2.0))/8.0+(qp2*qrp4*sin(q2+q3+q4+q5*2.0))/8.0-(qp3*qrp3*sin(q2+q3+q4-q5*2.0))/8.0+(qp3*qrp3*sin(q2+q3+q4+q5*2.0))/8.0-(qp4*qrp2*sin(q2+q3+q4-q5*2.0))/8.0+(qp4*qrp2*sin(q2+q3+q4+q5*2.0))/8.0+(qp2*qrp5*sin(q2+q3+q4-q5*2.0))/8.0+(qp2*qrp5*sin(q2+q3+q4+q5*2.0))/8.0-(qp3*qrp4*sin(q2+q3+q4-q5*2.0))/8.0+(qp3*qrp4*sin(q2+q3+q4+q5*2.0))/8.0-(qp4*qrp3*sin(q2+q3+q4-q5*2.0))/8.0+(qp4*qrp3*sin(q2+q3+q4+q5*2.0))/8.0+(qp5*qrp2*sin(q2+q3+q4-q5*2.0))/8.0+(qp5*qrp2*sin(q2+q3+q4+q5*2.0))/8.0+(qp2*qrp5*sin(q2+q3+q4-q6*2.0))/8.0+(qp2*qrp5*sin(q2+q3+q4+q6*2.0))/8.0+(qp3*qrp5*sin(q2+q3+q4-q5*2.0))/8.0+(qp3*qrp5*sin(q2+q3+q4+q5*2.0))/8.0-(qp4*qrp4*sin(q2+q3+q4-q5*2.0))/8.0+(qp4*qrp4*sin(q2+q3+q4+q5*2.0))/8.0+(qp5*qrp2*sin(q2+q3+q4-q6*2.0))/8.0+(qp5*qrp2*sin(q2+q3+q4+q6*2.0))/8.0+(qp5*qrp3*sin(q2+q3+q4-q5*2.0))/8.0+(qp5*qrp3*sin(q2+q3+q4+q5*2.0))/8.0+(qp3*qrp5*sin(q2+q3+q4-q6*2.0))/8.0+(qp3*qrp5*sin(q2+q3+q4+q6*2.0))/8.0+(qp4*qrp5*sin(q2+q3+q4-q5*2.0))/8.0+(qp4*qrp5*sin(q2+q3+q4+q5*2.0))/8.0+(qp5*qrp3*sin(q2+q3+q4-q6*2.0))/8.0+(qp5*qrp3*sin(q2+q3+q4+q6*2.0))/8.0+(qp5*qrp4*sin(q2+q3+q4-q5*2.0))/8.0+(qp5*qrp4*sin(q2+q3+q4+q5*2.0))/8.0+(qp4*qrp5*sin(q2+q3+q4-q6*2.0))/8.0+(qp4*qrp5*sin(q2+q3+q4+q6*2.0))/8.0+(qp5*qrp4*sin(q2+q3+q4-q6*2.0))/8.0+(qp5*qrp4*sin(q2+q3+q4+q6*2.0))/8.0-(qp5*qrp6*sin(q2+q3+q4-q6*2.0))/4.0+(qp5*qrp6*sin(q2+q3+q4+q6*2.0))/4.0-(qp6*qrp5*sin(q2+q3+q4-q6*2.0))/4.0+(qp6*qrp5*sin(q2+q3+q4+q6*2.0))/4.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1-qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1-qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1-qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1-qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1-qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1-qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q6*2.0)*(3.0/1.6E+1)-qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q6*2.0)*(3.0/1.6E+1)+(qp2*qrp5*sin(q2+q3+q4))/4.0+(qp5*qrp2*sin(q2+q3+q4))/4.0+(qp3*qrp5*sin(q2+q3+q4))/4.0+(qp5*qrp3*sin(q2+q3+q4))/4.0+(qp4*qrp5*sin(q2+q3+q4))/4.0+(qp5*qrp4*sin(q2+q3+q4))/4.0-(qp1*qrp5*sin(q5*2.0))/8.0-(qp5*qrp1*sin(q5*2.0))/8.0-(qp1*qrp6*sin(q6*2.0))/8.0-(qp6*qrp1*sin(q6*2.0))/8.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/8.0+(qp2*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp2*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp2*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp2*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp3*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp3*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp2*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp2*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp3*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp3*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp4*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp4*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp3*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp3*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp4*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp4*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp2*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp2*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp4*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp4*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp6*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp6*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp3*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp3*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp6*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp6*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp4*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp4*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp5*qrp5*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp5*qrp5*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp6*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0-(qp6*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp5*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp5*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp6*qrp5*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp6*qrp5*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp1*qrp5*sin(q5*2.0-q6*2.0))/1.6E+1+(qp1*qrp5*sin(q5*2.0+q6*2.0))/1.6E+1+(qp5*qrp1*sin(q5*2.0-q6*2.0))/1.6E+1+(qp5*qrp1*sin(q5*2.0+q6*2.0))/1.6E+1-(qp1*qrp6*sin(q5*2.0-q6*2.0))/1.6E+1+(qp1*qrp6*sin(q5*2.0+q6*2.0))/1.6E+1-(qp6*qrp1*sin(q5*2.0-q6*2.0))/1.6E+1+(qp6*qrp1*sin(q5*2.0+q6*2.0))/1.6E+1+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/1.6E+1-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/1.6E+1+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/1.6E+1-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/1.6E+1-(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qp1*qrp6*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0-(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/8.0-(qp6*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/8.0;
    Yr(0,15) = qrpp1/4.0-(qrpp1*cos(q5*2.0))/4.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/4.0+(qrpp6*cos(q2+q3+q4+q5))/2.0-(qrpp2*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp2*cos(q2+q3+q4+q5*2.0))/4.0-(qrpp3*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp3*cos(q2+q3+q4+q5*2.0))/4.0-(qrpp4*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp4*cos(q2+q3+q4+q5*2.0))/4.0-(qrpp6*cos(q2+q3+q4-q5))/2.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp2*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp2*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp6*sin(q2+q3+q4-q5))/4.0+(qp3*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp6*qrp2*sin(q2+q3+q4-q5))/4.0-(qp3*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp6*sin(q2+q3+q4-q5))/4.0+(qp4*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp6*qrp3*sin(q2+q3+q4-q5))/4.0-(qp4*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp6*sin(q2+q3+q4-q5))/4.0-(qp5*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp6*qrp4*sin(q2+q3+q4-q5))/4.0-(qp5*qrp6*sin(q2+q3+q4-q5))/4.0-(qp6*qrp5*sin(q2+q3+q4-q5))/4.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp5*sin(q5*2.0))/4.0+(qp5*qrp1*sin(q5*2.0))/4.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0-(qp2*qrp6*sin(q2+q3+q4+q5))/4.0-(qp6*qrp2*sin(q2+q3+q4+q5))/4.0-(qp3*qrp6*sin(q2+q3+q4+q5))/4.0-(qp6*qrp3*sin(q2+q3+q4+q5))/4.0-(qp4*qrp6*sin(q2+q3+q4+q5))/4.0-(qp6*qrp4*sin(q2+q3+q4+q5))/4.0-(qp5*qrp6*sin(q2+q3+q4+q5))/4.0-(qp6*qrp5*sin(q2+q3+q4+q5))/4.0;
    Yr(0,16) = qrpp1*(3.0/4.0)+(qrpp1*cos(q5*2.0))/4.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/4.0+(qrpp2*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp2*cos(q2+q3+q4+q5*2.0))/4.0+(qrpp3*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp3*cos(q2+q3+q4+q5*2.0))/4.0+(qrpp4*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp4*cos(q2+q3+q4+q5*2.0))/4.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-qrpp5*cos(q2+q3+q4)-(qp2*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp2*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp2*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp2*qrp5*sin(q2+q3+q4))/2.0+(qp5*qrp2*sin(q2+q3+q4))/2.0+(qp3*qrp5*sin(q2+q3+q4))/2.0+(qp5*qrp3*sin(q2+q3+q4))/2.0+(qp4*qrp5*sin(q2+q3+q4))/2.0+(qp5*qrp4*sin(q2+q3+q4))/2.0-(qp1*qrp5*sin(q5*2.0))/4.0-(qp5*qrp1*sin(q5*2.0))/4.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/4.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/4.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/4.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(0,17) = qrpp2*sin(q2)+qp2*qrp2*cos(q2);
    Yr(0,18) = -qrpp2*sin(q2)-qp2*qrp2*cos(q2);
    Yr(0,19) = cos(q1);
    Yr(0,20) = sin(q1);
    Yr(0,21) = qrpp1/2.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0))/2.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(0,22) = -qrpp1*sin(q4)-qrpp1*sin(q2*2.0+q3*2.0+q4)-(qp1*qrp4*cos(q4))/2.0-(qp4*qrp1*cos(q4))/2.0-qp1*qrp2*cos(q2*2.0+q3*2.0+q4)-qp2*qrp1*cos(q2*2.0+q3*2.0+q4)-qp1*qrp3*cos(q2*2.0+q3*2.0+q4)-qp3*qrp1*cos(q2*2.0+q3*2.0+q4)-(qp1*qrp4*cos(q2*2.0+q3*2.0+q4))/2.0-(qp4*qrp1*cos(q2*2.0+q3*2.0+q4))/2.0;
    Yr(0,23) = qrpp1*cos(q3)+qrpp1*cos(q2*2.0+q3)-(qp1*qrp3*sin(q3))/2.0-(qp3*qrp1*sin(q3))/2.0-qp1*qrp2*sin(q2*2.0+q3)-qp2*qrp1*sin(q2*2.0+q3)-(qp1*qrp3*sin(q2*2.0+q3))/2.0-(qp3*qrp1*sin(q2*2.0+q3))/2.0;
    Yr(0,24) = qrpp1*cos(q3)+qrpp1*cos(q2*2.0+q3)-(qp1*qrp3*sin(q3))/2.0-(qp3*qrp1*sin(q3))/2.0-qp1*qrp2*sin(q2*2.0+q3)-qp2*qrp1*sin(q2*2.0+q3)-(qp1*qrp3*sin(q2*2.0+q3))/2.0-(qp3*qrp1*sin(q2*2.0+q3))/2.0;
    Yr(0,25) = (qrpp2*cos(q2+q3+q4+q5))/2.0+(qrpp3*cos(q2+q3+q4+q5))/2.0+(qrpp4*cos(q2+q3+q4+q5))/2.0-(qrpp5*cos(q2+q3+q4+q5))/2.0+(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0+q5))/2.0+(qrpp2*cos(q2+q3+q4-q5))/2.0+(qrpp3*cos(q2+q3+q4-q5))/2.0+(qrpp4*cos(q2+q3+q4-q5))/2.0+(qrpp5*cos(q2+q3+q4-q5))/2.0-(qrpp1*cos(q2*2.0+q3*2.0+q4*2.0-q5))/2.0-(qp2*qrp2*sin(q2+q3+q4-q5))/2.0-(qp2*qrp3*sin(q2+q3+q4-q5))/2.0-(qp3*qrp2*sin(q2+q3+q4-q5))/2.0-(qp2*qrp4*sin(q2+q3+q4-q5))/2.0-(qp3*qrp3*sin(q2+q3+q4-q5))/2.0-(qp4*qrp2*sin(q2+q3+q4-q5))/2.0-(qp3*qrp4*sin(q2+q3+q4-q5))/2.0-(qp4*qrp3*sin(q2+q3+q4-q5))/2.0-(qp4*qrp4*sin(q2+q3+q4-q5))/2.0+(qp5*qrp5*sin(q2+q3+q4-q5))/2.0+(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0+(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0+(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0+(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0+(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0+(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0-q5))/4.0-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5))/4.0-(qp2*qrp2*sin(q2+q3+q4+q5))/2.0-(qp2*qrp3*sin(q2+q3+q4+q5))/2.0-(qp3*qrp2*sin(q2+q3+q4+q5))/2.0-(qp2*qrp4*sin(q2+q3+q4+q5))/2.0-(qp3*qrp3*sin(q2+q3+q4+q5))/2.0-(qp4*qrp2*sin(q2+q3+q4+q5))/2.0-(qp3*qrp4*sin(q2+q3+q4+q5))/2.0-(qp4*qrp3*sin(q2+q3+q4+q5))/2.0-(qp4*qrp4*sin(q2+q3+q4+q5))/2.0+(qp5*qrp5*sin(q2+q3+q4+q5))/2.0-(qp1*qrp2*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0-(qp2*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0-(qp1*qrp3*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0-(qp3*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0-(qp1*qrp4*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0-(qp4*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0-(qp1*qrp5*sin(q2*2.0+q3*2.0+q4*2.0+q5))/4.0-(qp5*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5))/4.0;
    Yr(0,26) = qrpp2*cos(q2+q3+q4+q5)*(-1.0/2.0)-(qrpp3*cos(q2+q3+q4+q5))/2.0-(qrpp4*cos(q2+q3+q4+q5))/2.0-(qrpp5*cos(q2+q3+q4+q5))/2.0+qrpp1*cos(q5)*2.0+(qrpp2*cos(q2+q3+q4-q5))/2.0+(qrpp3*cos(q2+q3+q4-q5))/2.0+(qrpp4*cos(q2+q3+q4-q5))/2.0-(qrpp5*cos(q2+q3+q4-q5))/2.0-qp1*qrp5*sin(q5)-qp5*qrp1*sin(q5)-(qp2*qrp2*sin(q2+q3+q4-q5))/2.0-(qp2*qrp3*sin(q2+q3+q4-q5))/2.0-(qp3*qrp2*sin(q2+q3+q4-q5))/2.0-(qp2*qrp4*sin(q2+q3+q4-q5))/2.0-(qp3*qrp3*sin(q2+q3+q4-q5))/2.0-(qp4*qrp2*sin(q2+q3+q4-q5))/2.0+(qp2*qrp5*sin(q2+q3+q4-q5))/2.0-(qp3*qrp4*sin(q2+q3+q4-q5))/2.0-(qp4*qrp3*sin(q2+q3+q4-q5))/2.0+(qp5*qrp2*sin(q2+q3+q4-q5))/2.0+(qp3*qrp5*sin(q2+q3+q4-q5))/2.0-(qp4*qrp4*sin(q2+q3+q4-q5))/2.0+(qp5*qrp3*sin(q2+q3+q4-q5))/2.0+(qp4*qrp5*sin(q2+q3+q4-q5))/2.0+(qp5*qrp4*sin(q2+q3+q4-q5))/2.0-(qp5*qrp5*sin(q2+q3+q4-q5))/2.0+(qp2*qrp2*sin(q2+q3+q4+q5))/2.0+(qp2*qrp3*sin(q2+q3+q4+q5))/2.0+(qp3*qrp2*sin(q2+q3+q4+q5))/2.0+(qp2*qrp4*sin(q2+q3+q4+q5))/2.0+(qp3*qrp3*sin(q2+q3+q4+q5))/2.0+(qp4*qrp2*sin(q2+q3+q4+q5))/2.0+(qp2*qrp5*sin(q2+q3+q4+q5))/2.0+(qp3*qrp4*sin(q2+q3+q4+q5))/2.0+(qp4*qrp3*sin(q2+q3+q4+q5))/2.0+(qp5*qrp2*sin(q2+q3+q4+q5))/2.0+(qp3*qrp5*sin(q2+q3+q4+q5))/2.0+(qp4*qrp4*sin(q2+q3+q4+q5))/2.0+(qp5*qrp3*sin(q2+q3+q4+q5))/2.0+(qp4*qrp5*sin(q2+q3+q4+q5))/2.0+(qp5*qrp4*sin(q2+q3+q4+q5))/2.0+(qp5*qrp5*sin(q2+q3+q4+q5))/2.0;
    Yr(0,27) = qrpp2*sin(q2+q3)+qrpp3*sin(q2+q3)+qp2*qrp2*cos(q2+q3)+qp2*qrp3*cos(q2+q3)+qp3*qrp2*cos(q2+q3)+qp3*qrp3*cos(q2+q3);
    Yr(0,28) = qrpp1*sin(q2*2.0+q3+q4-q5)*(-1.0/2.0)-(qrpp1*sin(q3+q4-q5))/2.0+(qrpp2*sin(q2+q5))/2.0-(qrpp5*sin(q2+q5))/2.0+(qrpp1*sin(q2*2.0+q3+q4+q5))/2.0+(qrpp2*sin(q2-q5))/2.0+(qrpp5*sin(q2-q5))/2.0+(qrpp1*sin(q3+q4+q5))/2.0+(qp1*qrp2*cos(q2*2.0+q3+q4+q5))/2.0+(qp2*qrp1*cos(q2*2.0+q3+q4+q5))/2.0+(qp1*qrp3*cos(q2*2.0+q3+q4+q5))/4.0+(qp3*qrp1*cos(q2*2.0+q3+q4+q5))/4.0+(qp1*qrp4*cos(q2*2.0+q3+q4+q5))/4.0+(qp4*qrp1*cos(q2*2.0+q3+q4+q5))/4.0+(qp1*qrp5*cos(q2*2.0+q3+q4+q5))/4.0+(qp5*qrp1*cos(q2*2.0+q3+q4+q5))/4.0+(qp2*qrp2*cos(q2-q5))/2.0-(qp5*qrp5*cos(q2-q5))/2.0+(qp1*qrp3*cos(q3+q4+q5))/4.0+(qp3*qrp1*cos(q3+q4+q5))/4.0+(qp1*qrp4*cos(q3+q4+q5))/4.0+(qp4*qrp1*cos(q3+q4+q5))/4.0+(qp1*qrp5*cos(q3+q4+q5))/4.0+(qp5*qrp1*cos(q3+q4+q5))/4.0-(qp1*qrp2*cos(q2*2.0+q3+q4-q5))/2.0-(qp2*qrp1*cos(q2*2.0+q3+q4-q5))/2.0-(qp1*qrp3*cos(q2*2.0+q3+q4-q5))/4.0-(qp3*qrp1*cos(q2*2.0+q3+q4-q5))/4.0-(qp1*qrp4*cos(q2*2.0+q3+q4-q5))/4.0-(qp4*qrp1*cos(q2*2.0+q3+q4-q5))/4.0+(qp1*qrp5*cos(q2*2.0+q3+q4-q5))/4.0+(qp5*qrp1*cos(q2*2.0+q3+q4-q5))/4.0-(qp1*qrp3*cos(q3+q4-q5))/4.0-(qp3*qrp1*cos(q3+q4-q5))/4.0-(qp1*qrp4*cos(q3+q4-q5))/4.0-(qp4*qrp1*cos(q3+q4-q5))/4.0+(qp1*qrp5*cos(q3+q4-q5))/4.0+(qp5*qrp1*cos(q3+q4-q5))/4.0+(qp2*qrp2*cos(q2+q5))/2.0-(qp5*qrp5*cos(q2+q5))/2.0;
    Yr(0,29) = cos(q1)*cos(q5)+cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)-cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5)-cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5)-cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5);
    Yr(0,30) = -cos(q1)*cos(q2);
    Yr(0,31) = cos(q1)*cos(q2);
    Yr(0,32) = cos(q2)*sin(q1);
    Yr(0,33) = -cos(q2)*sin(q1);
    Yr(0,34) = cos(q5)*sin(q1)-cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)+cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5)+cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5)+cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5);
    Yr(0,35) = -qrpp1*sin(q2*2.0+q3+q4)-qrpp1*sin(q3+q4)-qp1*qrp2*cos(q2*2.0+q3+q4)-qp2*qrp1*cos(q2*2.0+q3+q4)-(qp1*qrp3*cos(q2*2.0+q3+q4))/2.0-(qp3*qrp1*cos(q2*2.0+q3+q4))/2.0-(qp1*qrp4*cos(q2*2.0+q3+q4))/2.0-(qp4*qrp1*cos(q2*2.0+q3+q4))/2.0-(qp1*qrp3*cos(q3+q4))/2.0-(qp3*qrp1*cos(q3+q4))/2.0-(qp1*qrp4*cos(q3+q4))/2.0-(qp4*qrp1*cos(q3+q4))/2.0;
    Yr(0,36) = (qrpp1*sin(q2*2.0+q3*2.0+q4+q5))/2.0+(qrpp2*sin(q2+q3-q5))/2.0+(qrpp3*sin(q2+q3-q5))/2.0+(qrpp5*sin(q2+q3-q5))/2.0+(qrpp1*sin(q4+q5))/2.0-(qrpp1*sin(q2*2.0+q3*2.0+q4-q5))/2.0-(qrpp1*sin(q4-q5))/2.0+(qrpp2*sin(q2+q3+q5))/2.0+(qrpp3*sin(q2+q3+q5))/2.0-(qrpp5*sin(q2+q3+q5))/2.0-(qp1*qrp4*cos(q4-q5))/4.0-(qp4*qrp1*cos(q4-q5))/4.0+(qp1*qrp5*cos(q4-q5))/4.0+(qp5*qrp1*cos(q4-q5))/4.0+(qp2*qrp2*cos(q2+q3+q5))/2.0+(qp2*qrp3*cos(q2+q3+q5))/2.0+(qp3*qrp2*cos(q2+q3+q5))/2.0+(qp3*qrp3*cos(q2+q3+q5))/2.0-(qp5*qrp5*cos(q2+q3+q5))/2.0+(qp1*qrp2*cos(q2*2.0+q3*2.0+q4+q5))/2.0+(qp2*qrp1*cos(q2*2.0+q3*2.0+q4+q5))/2.0+(qp1*qrp3*cos(q2*2.0+q3*2.0+q4+q5))/2.0+(qp3*qrp1*cos(q2*2.0+q3*2.0+q4+q5))/2.0+(qp1*qrp4*cos(q2*2.0+q3*2.0+q4+q5))/4.0+(qp4*qrp1*cos(q2*2.0+q3*2.0+q4+q5))/4.0+(qp1*qrp5*cos(q2*2.0+q3*2.0+q4+q5))/4.0+(qp5*qrp1*cos(q2*2.0+q3*2.0+q4+q5))/4.0+(qp2*qrp2*cos(q2+q3-q5))/2.0+(qp2*qrp3*cos(q2+q3-q5))/2.0+(qp3*qrp2*cos(q2+q3-q5))/2.0+(qp3*qrp3*cos(q2+q3-q5))/2.0-(qp5*qrp5*cos(q2+q3-q5))/2.0+(qp1*qrp4*cos(q4+q5))/4.0+(qp4*qrp1*cos(q4+q5))/4.0+(qp1*qrp5*cos(q4+q5))/4.0+(qp5*qrp1*cos(q4+q5))/4.0-(qp1*qrp2*cos(q2*2.0+q3*2.0+q4-q5))/2.0-(qp2*qrp1*cos(q2*2.0+q3*2.0+q4-q5))/2.0-(qp1*qrp3*cos(q2*2.0+q3*2.0+q4-q5))/2.0-(qp3*qrp1*cos(q2*2.0+q3*2.0+q4-q5))/2.0-(qp1*qrp4*cos(q2*2.0+q3*2.0+q4-q5))/4.0-(qp4*qrp1*cos(q2*2.0+q3*2.0+q4-q5))/4.0+(qp1*qrp5*cos(q2*2.0+q3*2.0+q4-q5))/4.0+(qp5*qrp1*cos(q2*2.0+q3*2.0+q4-q5))/4.0;
    Yr(0,37) = qrpp2*cos(q2+q3+q4)+qrpp3*cos(q2+q3+q4)+qrpp4*cos(q2+q3+q4)-qp2*qrp2*sin(q2+q3+q4)-qp2*qrp3*sin(q2+q3+q4)-qp3*qrp2*sin(q2+q3+q4)-qp2*qrp4*sin(q2+q3+q4)-qp3*qrp3*sin(q2+q3+q4)-qp4*qrp2*sin(q2+q3+q4)-qp3*qrp4*sin(q2+q3+q4)-qp4*qrp3*sin(q2+q3+q4)-qp4*qrp4*sin(q2+q3+q4);
    Yr(0,38) = -cos(q2+q3)*cos(q1);
    Yr(0,39) = cos(q2+q3)*sin(q1);
    Yr(0,40) = sin(q1+q2+q3+q4)/2.0+sin(-q1+q2+q3+q4)/2.0;
    Yr(0,41) = cos(q1+q2+q3+q4)/2.0-cos(-q1+q2+q3+q4)/2.0;
    Yr(1,1) = qp1*qrp1*sin(q2*2.0)*(-1.0/2.0);
    Yr(1,2) = (qp1*qrp1*sin(q2*2.0+q3*2.0))/2.0;
    Yr(1,3) = qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0)*(-1.0/2.0);
    Yr(1,5) = (qp1*qrp1*sin(q2*2.0))/2.0;
    Yr(1,6) = qp1*qrp1*sin(q2*2.0+q3*2.0)*(-1.0/2.0);
    Yr(1,7) = (qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(1,8) = qrpp2+qrpp3+(qp1*qrp1*sin(q2*2.0+q3*2.0))/2.0;
    Yr(1,9) = qrpp2+(qp1*qrp1*sin(q2*2.0))/2.0;
    Yr(1,10) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0-(qrpp2*cos(q5*2.0))/2.0-(qrpp3*cos(q5*2.0))/2.0-(qrpp4*cos(q5*2.0))/2.0+(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp2*qrp5*sin(q5*2.0))/2.0+(qp5*qrp2*sin(q5*2.0))/2.0+(qp3*qrp5*sin(q5*2.0))/2.0+(qp5*qrp3*sin(q5*2.0))/2.0+(qp4*qrp5*sin(q5*2.0))/2.0+(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(1,11) = qp1*qrp5*sin(q2+q3+q4)*(-1.0/2.0)-(qp5*qrp1*sin(q2+q3+q4))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(1,12) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0+(qrpp2*cos(q5*2.0))/2.0+(qrpp3*cos(q5*2.0))/2.0+(qrpp4*cos(q5*2.0))/2.0-(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp5*sin(q5*2.0))/2.0-(qp5*qrp2*sin(q5*2.0))/2.0-(qp3*qrp5*sin(q5*2.0))/2.0-(qp5*qrp3*sin(q5*2.0))/2.0-(qp4*qrp5*sin(q5*2.0))/2.0-(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(1,13) = qrpp2/4.0+qrpp3/4.0+qrpp4/4.0-(qrpp2*cos(q5*2.0))/4.0+(qrpp2*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0))/4.0+(qrpp3*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0))/4.0+(qrpp4*cos(q6*2.0))/4.0-(qrpp2*cos(q5*2.0)*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0)*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0)*cos(q6*2.0))/4.0+(qp2*qrp5*sin(q5*2.0))/4.0+(qp5*qrp2*sin(q5*2.0))/4.0+(qp3*qrp5*sin(q5*2.0))/4.0+(qp5*qrp3*sin(q5*2.0))/4.0-(qp2*qrp6*sin(q6*2.0))/4.0+(qp4*qrp5*sin(q5*2.0))/4.0+(qp5*qrp4*sin(q5*2.0))/4.0-(qp6*qrp2*sin(q6*2.0))/4.0-(qp3*qrp6*sin(q6*2.0))/4.0-(qp6*qrp3*sin(q6*2.0))/4.0-(qp4*qrp6*sin(q6*2.0))/4.0-(qp6*qrp4*sin(q6*2.0))/4.0-(qrpp5*sin(q6*2.0)*sin(q5))/2.0-(qp5*qrp5*sin(q6*2.0)*cos(q5))/2.0-(qp5*qrp6*cos(q6*2.0)*sin(q5))/2.0-(qp6*qrp5*cos(q6*2.0)*sin(q5))/2.0+(qp2*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp2*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp2*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp3*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp6*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp4*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp6*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp6*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp1*qrp1*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q2*2.0))/8.0-qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q6*2.0)*sin(q4*2.0)*(3.0/8.0)-qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q3*2.0)*(3.0/8.0)-qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q2*2.0)*(3.0/8.0)-(qp1*qrp5*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q5*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+qp1*qrp1*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*(3.0/8.0)+(qrpp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q4*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0))/8.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0;
    Yr(1,14) = qrpp2/4.0+qrpp3/4.0+qrpp4/4.0-(qrpp2*cos(q5*2.0))/4.0-(qrpp2*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0))/4.0-(qrpp3*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0))/4.0-(qrpp4*cos(q6*2.0))/4.0+(qrpp2*cos(q5*2.0)*cos(q6*2.0))/4.0+(qrpp3*cos(q5*2.0)*cos(q6*2.0))/4.0+(qrpp4*cos(q5*2.0)*cos(q6*2.0))/4.0+(qp2*qrp5*sin(q5*2.0))/4.0+(qp5*qrp2*sin(q5*2.0))/4.0+(qp3*qrp5*sin(q5*2.0))/4.0+(qp5*qrp3*sin(q5*2.0))/4.0+(qp2*qrp6*sin(q6*2.0))/4.0+(qp4*qrp5*sin(q5*2.0))/4.0+(qp5*qrp4*sin(q5*2.0))/4.0+(qp6*qrp2*sin(q6*2.0))/4.0+(qp3*qrp6*sin(q6*2.0))/4.0+(qp6*qrp3*sin(q6*2.0))/4.0+(qp4*qrp6*sin(q6*2.0))/4.0+(qp6*qrp4*sin(q6*2.0))/4.0+(qrpp5*sin(q6*2.0)*sin(q5))/2.0+(qp5*qrp5*sin(q6*2.0)*cos(q5))/2.0+(qp5*qrp6*cos(q6*2.0)*sin(q5))/2.0+(qp6*qrp5*cos(q6*2.0)*sin(q5))/2.0-(qp2*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp2*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp2*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp3*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp6*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp4*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp6*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp6*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp1*qrp1*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q2*2.0))/8.0+qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q6*2.0)*sin(q4*2.0)*(3.0/8.0)+qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q3*2.0)*(3.0/8.0)+qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q2*2.0)*(3.0/8.0)-(qp1*qrp5*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q5*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0-qp1*qrp1*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*(3.0/8.0)-(qrpp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q4*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-qp6*qrp1*pow(cos(q1),2.0)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6)-qp6*qrp1*cos(q5)*cos(q6)*pow(sin(q1),2.0)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6);
    Yr(1,15) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0+(qrpp2*cos(q5*2.0))/2.0+(qrpp3*cos(q5*2.0))/2.0+(qrpp4*cos(q5*2.0))/2.0+qrpp6*cos(q5)-(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp6*sin(q5))/2.0-(qp6*qrp5*sin(q5))/2.0-(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp6*sin(q2+q3+q4-q5))/4.0-(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp6*qrp1*sin(q2+q3+q4-q5))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp5*sin(q5*2.0))/2.0-(qp5*qrp2*sin(q5*2.0))/2.0-(qp3*qrp5*sin(q5*2.0))/2.0-(qp5*qrp3*sin(q5*2.0))/2.0-(qp4*qrp5*sin(q5*2.0))/2.0-(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp6*sin(q2+q3+q4+q5))/4.0+(qp6*qrp1*sin(q2+q3+q4+q5))/4.0;
    Yr(1,16) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0-(qrpp2*cos(q5*2.0))/2.0-(qrpp3*cos(q5*2.0))/2.0-(qrpp4*cos(q5*2.0))/2.0+(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp5*sin(q2+q3+q4))/2.0-(qp5*qrp1*sin(q2+q3+q4))/2.0+(qp2*qrp5*sin(q5*2.0))/2.0+(qp5*qrp2*sin(q5*2.0))/2.0+(qp3*qrp5*sin(q5*2.0))/2.0+(qp5*qrp3*sin(q5*2.0))/2.0+(qp4*qrp5*sin(q5*2.0))/2.0+(qp5*qrp4*sin(q5*2.0))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(1,17) = qrpp1*sin(q2);
    Yr(1,18) = -qrpp1*sin(q2);
    Yr(1,21) = qrpp2+qrpp3+qrpp4-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(1,22) = qrpp2*sin(q4)*-2.0-qrpp3*sin(q4)*2.0-qrpp4*sin(q4)-qp2*qrp4*cos(q4)-qp4*qrp2*cos(q4)-qp3*qrp4*cos(q4)-qp4*qrp3*cos(q4)-qp4*qrp4*cos(q4)+qp1*qrp1*cos(q2*2.0+q3*2.0+q4);
    Yr(1,23) = cos(q3)*(qrpp2*2.0+qrpp3)-qp2*qrp3*sin(q3)-qp3*qrp2*sin(q3)-qp3*qrp3*sin(q3)+qp1*qrp1*sin(q2*2.0+q3);
    Yr(1,24) = qrpp2*cos(q3)*2.0+qrpp3*cos(q3)-qp2*qrp3*sin(q3)-qp3*qrp2*sin(q3)-qp3*qrp3*sin(q3)+qp1*qrp1*sin(q2*2.0+q3);
    Yr(1,25) = (qrpp1*cos(q2+q3+q4+q5))/2.0-qrpp5*cos(q5)+(qrpp1*cos(q2+q3+q4-q5))/2.0+qp5*qrp5*sin(q5)+(qp1*qrp5*sin(q2+q3+q4-q5))/2.0+(qp5*qrp1*sin(q2+q3+q4-q5))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0-(qp1*qrp5*sin(q2+q3+q4+q5))/2.0-(qp5*qrp1*sin(q2+q3+q4+q5))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0;
    Yr(1,26) = qrpp1*(cos(q2+q3+q4+q5)-cos(q2+q3+q4-q5))*(-1.0/2.0);
    Yr(1,27) = qrpp1*sin(q2+q3);
    Yr(1,28) = qrpp1*cos(q5)*sin(q2)-qp1*qrp5*sin(q2)*sin(q5)-qp5*qrp1*sin(q2)*sin(q5)+qrpp2*cos(q3)*cos(q4)*sin(q5)*2.0+qrpp3*cos(q3)*cos(q4)*sin(q5)+qrpp4*cos(q3)*cos(q4)*sin(q5)+qrpp5*cos(q3)*cos(q5)*sin(q4)+qrpp5*cos(q4)*cos(q5)*sin(q3)-qrpp2*sin(q3)*sin(q4)*sin(q5)*2.0-qrpp3*sin(q3)*sin(q4)*sin(q5)-qrpp4*sin(q3)*sin(q4)*sin(q5)+qp2*qrp5*cos(q3)*cos(q4)*cos(q5)+qp5*qrp2*cos(q3)*cos(q4)*cos(q5)+qp3*qrp5*cos(q3)*cos(q4)*cos(q5)+qp5*qrp3*cos(q3)*cos(q4)*cos(q5)+qp4*qrp5*cos(q3)*cos(q4)*cos(q5)+qp5*qrp4*cos(q3)*cos(q4)*cos(q5)-qp2*qrp3*cos(q3)*sin(q4)*sin(q5)-qp2*qrp3*cos(q4)*sin(q3)*sin(q5)-qp3*qrp2*cos(q3)*sin(q4)*sin(q5)-qp3*qrp2*cos(q4)*sin(q3)*sin(q5)-qp2*qrp4*cos(q3)*sin(q4)*sin(q5)-qp2*qrp4*cos(q4)*sin(q3)*sin(q5)-qp3*qrp3*cos(q3)*sin(q4)*sin(q5)-qp3*qrp3*cos(q4)*sin(q3)*sin(q5)-qp4*qrp2*cos(q3)*sin(q4)*sin(q5)-qp4*qrp2*cos(q4)*sin(q3)*sin(q5)-qp2*qrp5*cos(q5)*sin(q3)*sin(q4)-qp3*qrp4*cos(q3)*sin(q4)*sin(q5)-qp3*qrp4*cos(q4)*sin(q3)*sin(q5)-qp4*qrp3*cos(q3)*sin(q4)*sin(q5)-qp4*qrp3*cos(q4)*sin(q3)*sin(q5)-qp5*qrp2*cos(q5)*sin(q3)*sin(q4)-qp3*qrp5*cos(q5)*sin(q3)*sin(q4)-qp4*qrp4*cos(q3)*sin(q4)*sin(q5)-qp4*qrp4*cos(q4)*sin(q3)*sin(q5)-qp5*qrp3*cos(q5)*sin(q3)*sin(q4)-qp4*qrp5*cos(q5)*sin(q3)*sin(q4)-qp5*qrp4*cos(q5)*sin(q3)*sin(q4)-qp5*qrp5*cos(q3)*sin(q4)*sin(q5)-qp5*qrp5*cos(q4)*sin(q3)*sin(q5)+qp1*qrp1*cos(q2*2.0)*cos(q3)*sin(q4)*sin(q5)+qp1*qrp1*cos(q2*2.0)*cos(q4)*sin(q3)*sin(q5)+qp1*qrp1*sin(q2*2.0)*cos(q3)*cos(q4)*sin(q5)-qp1*qrp1*sin(q2*2.0)*sin(q3)*sin(q4)*sin(q5);
    Yr(1,29) = sin(q2+q3+q4)*cos(q1)*sin(q5);
    Yr(1,30) = sin(q1)*sin(q2);
    Yr(1,31) = -sin(q1)*sin(q2);
    Yr(1,32) = cos(q1)*sin(q2);
    Yr(1,33) = -cos(q1)*sin(q2);
    Yr(1,34) = sin(q2+q3+q4)*sin(q1)*sin(q5);
    Yr(1,35) = qrpp2*sin(q3+q4)*-2.0-qrpp3*sin(q3+q4)-qrpp4*sin(q3+q4)+qp1*qrp1*cos(q2*2.0+q3+q4)-qp2*qrp3*cos(q3+q4)-qp3*qrp2*cos(q3+q4)-qp2*qrp4*cos(q3+q4)-qp3*qrp3*cos(q3+q4)-qp4*qrp2*cos(q3+q4)-qp3*qrp4*cos(q3+q4)-qp4*qrp3*cos(q3+q4)-qp4*qrp4*cos(q3+q4);
    Yr(1,36) = qrpp2*cos(q4)*sin(q5)*2.0+qrpp3*cos(q4)*sin(q5)*2.0+qrpp4*cos(q4)*sin(q5)+qrpp5*cos(q5)*sin(q4)+qp1*qrp1*sin(q4)*sin(q5)-qp2*qrp4*sin(q4)*sin(q5)-qp4*qrp2*sin(q4)*sin(q5)-qp3*qrp4*sin(q4)*sin(q5)-qp4*qrp3*sin(q4)*sin(q5)-qp4*qrp4*sin(q4)*sin(q5)-qp5*qrp5*sin(q4)*sin(q5)+qrpp1*cos(q2)*cos(q5)*sin(q3)+qrpp1*cos(q3)*cos(q5)*sin(q2)+qp2*qrp5*cos(q4)*cos(q5)+qp5*qrp2*cos(q4)*cos(q5)+qp3*qrp5*cos(q4)*cos(q5)+qp5*qrp3*cos(q4)*cos(q5)+qp4*qrp5*cos(q4)*cos(q5)+qp5*qrp4*cos(q4)*cos(q5)-qp1*qrp1*pow(cos(q2),2.0)*sin(q4)*sin(q5)*2.0-qp1*qrp1*pow(cos(q3),2.0)*sin(q4)*sin(q5)*2.0-qp1*qrp5*cos(q2)*sin(q3)*sin(q5)-qp1*qrp5*cos(q3)*sin(q2)*sin(q5)-qp5*qrp1*cos(q2)*sin(q3)*sin(q5)-qp5*qrp1*cos(q3)*sin(q2)*sin(q5)+qp1*qrp1*pow(cos(q2),2.0)*pow(cos(q3),2.0)*sin(q4)*sin(q5)*4.0-qp1*qrp1*cos(q2)*cos(q4)*sin(q2)*sin(q5)*2.0-qp1*qrp1*cos(q3)*cos(q4)*sin(q3)*sin(q5)*2.0+qp1*qrp1*cos(q2)*pow(cos(q3),2.0)*cos(q4)*sin(q2)*sin(q5)*4.0+qp1*qrp1*pow(cos(q2),2.0)*cos(q3)*cos(q4)*sin(q3)*sin(q5)*4.0-qp1*qrp1*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*4.0;
    Yr(1,37) = qrpp1*cos(q2+q3+q4);
    Yr(1,38) = sin(q2+q3)*sin(q1);
    Yr(1,39) = sin(q2+q3)*cos(q1);
    Yr(1,40) = sin(q1+q2+q3+q4)/2.0-sin(-q1+q2+q3+q4)/2.0;
    Yr(1,41) = cos(q1+q2+q3+q4)/2.0+cos(-q1+q2+q3+q4)/2.0;
    Yr(1,42) = qrpp2;
    Yr(1,43) = qrpp2+qrpp3;
    Yr(1,44) = qrpp2+qrpp3+qrpp4;
    Yr(1,45) = qrp2;
    Yr(1,46) = cos(q2);
    Yr(1,47) = -cos(q2);
    Yr(1,48) = cos(q2+q3);
    Yr(1,49) = -sin(q2+q3+q4);
    Yr(1,50) = sin(q2+q3+q4+q5)/2.0-sin(q2+q3+q4-q5)/2.0;
    Yr(2,2) = (qp1*qrp1*sin(q2*2.0+q3*2.0))/2.0;
    Yr(2,3) = qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0)*(-1.0/2.0);
    Yr(2,6) = qp1*qrp1*sin(q2*2.0+q3*2.0)*(-1.0/2.0);
    Yr(2,7) = (qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(2,8) = qrpp2+qrpp3+(qp1*qrp1*sin(q2*2.0+q3*2.0))/2.0;
    Yr(2,10) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0-(qrpp2*cos(q5*2.0))/2.0-(qrpp3*cos(q5*2.0))/2.0-(qrpp4*cos(q5*2.0))/2.0+(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp2*qrp5*sin(q5*2.0))/2.0+(qp5*qrp2*sin(q5*2.0))/2.0+(qp3*qrp5*sin(q5*2.0))/2.0+(qp5*qrp3*sin(q5*2.0))/2.0+(qp4*qrp5*sin(q5*2.0))/2.0+(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(2,11) = qp1*qrp5*sin(q2+q3+q4)*(-1.0/2.0)-(qp5*qrp1*sin(q2+q3+q4))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(2,12) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0+(qrpp2*cos(q5*2.0))/2.0+(qrpp3*cos(q5*2.0))/2.0+(qrpp4*cos(q5*2.0))/2.0-(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp5*sin(q5*2.0))/2.0-(qp5*qrp2*sin(q5*2.0))/2.0-(qp3*qrp5*sin(q5*2.0))/2.0-(qp5*qrp3*sin(q5*2.0))/2.0-(qp4*qrp5*sin(q5*2.0))/2.0-(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(2,13) = qrpp2/4.0+qrpp3/4.0+qrpp4/4.0-(qrpp2*cos(q5*2.0))/4.0+(qrpp2*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0))/4.0+(qrpp3*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0))/4.0+(qrpp4*cos(q6*2.0))/4.0-(qrpp2*cos(q5*2.0)*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0)*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0)*cos(q6*2.0))/4.0+(qp2*qrp5*sin(q5*2.0))/4.0+(qp5*qrp2*sin(q5*2.0))/4.0+(qp3*qrp5*sin(q5*2.0))/4.0+(qp5*qrp3*sin(q5*2.0))/4.0-(qp2*qrp6*sin(q6*2.0))/4.0+(qp4*qrp5*sin(q5*2.0))/4.0+(qp5*qrp4*sin(q5*2.0))/4.0-(qp6*qrp2*sin(q6*2.0))/4.0-(qp3*qrp6*sin(q6*2.0))/4.0-(qp6*qrp3*sin(q6*2.0))/4.0-(qp4*qrp6*sin(q6*2.0))/4.0-(qp6*qrp4*sin(q6*2.0))/4.0-(qrpp5*sin(q6*2.0)*sin(q5))/2.0-(qp5*qrp5*sin(q6*2.0)*cos(q5))/2.0-(qp5*qrp6*cos(q6*2.0)*sin(q5))/2.0-(qp6*qrp5*cos(q6*2.0)*sin(q5))/2.0+(qp2*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp2*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp2*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp3*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp6*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp4*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp6*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp6*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp1*qrp1*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q2*2.0))/8.0-qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q6*2.0)*sin(q4*2.0)*(3.0/8.0)-qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q3*2.0)*(3.0/8.0)-qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q2*2.0)*(3.0/8.0)-(qp1*qrp5*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q5*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+qp1*qrp1*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*(3.0/8.0)+(qrpp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q4*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0))/8.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0;
    Yr(2,14) = qrpp2/4.0+qrpp3/4.0+qrpp4/4.0-(qrpp2*cos(q5*2.0))/4.0-(qrpp2*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0))/4.0-(qrpp3*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0))/4.0-(qrpp4*cos(q6*2.0))/4.0+(qrpp2*cos(q5*2.0)*cos(q6*2.0))/4.0+(qrpp3*cos(q5*2.0)*cos(q6*2.0))/4.0+(qrpp4*cos(q5*2.0)*cos(q6*2.0))/4.0+(qp2*qrp5*sin(q5*2.0))/4.0+(qp5*qrp2*sin(q5*2.0))/4.0+(qp3*qrp5*sin(q5*2.0))/4.0+(qp5*qrp3*sin(q5*2.0))/4.0+(qp2*qrp6*sin(q6*2.0))/4.0+(qp4*qrp5*sin(q5*2.0))/4.0+(qp5*qrp4*sin(q5*2.0))/4.0+(qp6*qrp2*sin(q6*2.0))/4.0+(qp3*qrp6*sin(q6*2.0))/4.0+(qp6*qrp3*sin(q6*2.0))/4.0+(qp4*qrp6*sin(q6*2.0))/4.0+(qp6*qrp4*sin(q6*2.0))/4.0+(qrpp5*sin(q6*2.0)*sin(q5))/2.0+(qp5*qrp5*sin(q6*2.0)*cos(q5))/2.0+(qp5*qrp6*cos(q6*2.0)*sin(q5))/2.0+(qp6*qrp5*cos(q6*2.0)*sin(q5))/2.0-(qp2*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp2*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp2*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp3*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp6*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp4*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp6*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp6*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp1*qrp1*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q2*2.0))/8.0+qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q6*2.0)*sin(q4*2.0)*(3.0/8.0)+qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q3*2.0)*(3.0/8.0)+qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q2*2.0)*(3.0/8.0)-(qp1*qrp5*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q5*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0-qp1*qrp1*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*(3.0/8.0)-(qrpp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q4*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-qp6*qrp1*pow(cos(q1),2.0)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6)-qp6*qrp1*cos(q5)*cos(q6)*pow(sin(q1),2.0)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6);
    Yr(2,15) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0+(qrpp2*cos(q5*2.0))/2.0+(qrpp3*cos(q5*2.0))/2.0+(qrpp4*cos(q5*2.0))/2.0+qrpp6*cos(q5)-(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp6*sin(q5))/2.0-(qp6*qrp5*sin(q5))/2.0-(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp6*sin(q2+q3+q4-q5))/4.0-(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp6*qrp1*sin(q2+q3+q4-q5))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp5*sin(q5*2.0))/2.0-(qp5*qrp2*sin(q5*2.0))/2.0-(qp3*qrp5*sin(q5*2.0))/2.0-(qp5*qrp3*sin(q5*2.0))/2.0-(qp4*qrp5*sin(q5*2.0))/2.0-(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp6*sin(q2+q3+q4+q5))/4.0+(qp6*qrp1*sin(q2+q3+q4+q5))/4.0;
    Yr(2,16) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0-(qrpp2*cos(q5*2.0))/2.0-(qrpp3*cos(q5*2.0))/2.0-(qrpp4*cos(q5*2.0))/2.0+(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp5*sin(q2+q3+q4))/2.0-(qp5*qrp1*sin(q2+q3+q4))/2.0+(qp2*qrp5*sin(q5*2.0))/2.0+(qp5*qrp2*sin(q5*2.0))/2.0+(qp3*qrp5*sin(q5*2.0))/2.0+(qp5*qrp3*sin(q5*2.0))/2.0+(qp4*qrp5*sin(q5*2.0))/2.0+(qp5*qrp4*sin(q5*2.0))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(2,21) = qrpp2+qrpp3+qrpp4-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(2,22) = qrpp2*sin(q4)*-2.0-qrpp3*sin(q4)*2.0-qrpp4*sin(q4)-qp2*qrp4*cos(q4)-qp4*qrp2*cos(q4)-qp3*qrp4*cos(q4)-qp4*qrp3*cos(q4)-qp4*qrp4*cos(q4)+qp1*qrp1*cos(q2*2.0+q3*2.0+q4);
    Yr(2,23) = qrpp2*cos(q3)+(qp1*qrp1*sin(q3))/2.0+qp2*qrp2*sin(q3)+(qp1*qrp1*sin(q2*2.0+q3))/2.0;
    Yr(2,24) = qrpp2*cos(q3)+(qp1*qrp1*sin(q3))/2.0+qp2*qrp2*sin(q3)+(qp1*qrp1*sin(q2*2.0+q3))/2.0;
    Yr(2,25) = (qrpp1*cos(q2+q3+q4+q5))/2.0-qrpp5*cos(q5)+(qrpp1*cos(q2+q3+q4-q5))/2.0+qp5*qrp5*sin(q5)+(qp1*qrp5*sin(q2+q3+q4-q5))/2.0+(qp5*qrp1*sin(q2+q3+q4-q5))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0-(qp1*qrp5*sin(q2+q3+q4+q5))/2.0-(qp5*qrp1*sin(q2+q3+q4+q5))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0;
    Yr(2,26) = qrpp1*(cos(q2+q3+q4+q5)-cos(q2+q3+q4-q5))*(-1.0/2.0);
    Yr(2,27) = qrpp1*sin(q2+q3);
    Yr(2,28) = sin(q5)*(qrpp2*cos(q3+q4)+(qp1*qrp1*sin(q2*2.0+q3+q4))/2.0+(qp1*qrp1*sin(q3+q4))/2.0+qp2*qrp2*sin(q3+q4));
    Yr(2,29) = sin(q2+q3+q4)*cos(q1)*sin(q5);
    Yr(2,34) = sin(q2+q3+q4)*sin(q1)*sin(q5);
    Yr(2,35) = -qrpp2*sin(q3+q4)+(qp1*qrp1*cos(q2*2.0+q3+q4))/2.0+(qp1*qrp1*cos(q3+q4))/2.0+qp2*qrp2*cos(q3+q4);
    Yr(2,36) = qrpp2*cos(q4)*sin(q5)*2.0+qrpp3*cos(q4)*sin(q5)*2.0+qrpp4*cos(q4)*sin(q5)+qrpp5*cos(q5)*sin(q4)+qp1*qrp1*sin(q4)*sin(q5)-qp2*qrp4*sin(q4)*sin(q5)-qp4*qrp2*sin(q4)*sin(q5)-qp3*qrp4*sin(q4)*sin(q5)-qp4*qrp3*sin(q4)*sin(q5)-qp4*qrp4*sin(q4)*sin(q5)-qp5*qrp5*sin(q4)*sin(q5)+qrpp1*cos(q2)*cos(q5)*sin(q3)+qrpp1*cos(q3)*cos(q5)*sin(q2)+qp2*qrp5*cos(q4)*cos(q5)+qp5*qrp2*cos(q4)*cos(q5)+qp3*qrp5*cos(q4)*cos(q5)+qp5*qrp3*cos(q4)*cos(q5)+qp4*qrp5*cos(q4)*cos(q5)+qp5*qrp4*cos(q4)*cos(q5)-qp1*qrp1*pow(cos(q2),2.0)*sin(q4)*sin(q5)*2.0-qp1*qrp1*pow(cos(q3),2.0)*sin(q4)*sin(q5)*2.0-qp1*qrp5*cos(q2)*sin(q3)*sin(q5)-qp1*qrp5*cos(q3)*sin(q2)*sin(q5)-qp5*qrp1*cos(q2)*sin(q3)*sin(q5)-qp5*qrp1*cos(q3)*sin(q2)*sin(q5)+qp1*qrp1*pow(cos(q2),2.0)*pow(cos(q3),2.0)*sin(q4)*sin(q5)*4.0-qp1*qrp1*cos(q2)*cos(q4)*sin(q2)*sin(q5)*2.0-qp1*qrp1*cos(q3)*cos(q4)*sin(q3)*sin(q5)*2.0+qp1*qrp1*cos(q2)*pow(cos(q3),2.0)*cos(q4)*sin(q2)*sin(q5)*4.0+qp1*qrp1*pow(cos(q2),2.0)*cos(q3)*cos(q4)*sin(q3)*sin(q5)*4.0-qp1*qrp1*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*4.0;
    Yr(2,37) = qrpp1*cos(q2+q3+q4);
    Yr(2,38) = sin(q2+q3)*sin(q1);
    Yr(2,39) = sin(q2+q3)*cos(q1);
    Yr(2,40) = sin(q1+q2+q3+q4)/2.0-sin(-q1+q2+q3+q4)/2.0;
    Yr(2,41) = cos(q1+q2+q3+q4)/2.0+cos(-q1+q2+q3+q4)/2.0;
    Yr(2,43) = qrpp2+qrpp3;
    Yr(2,44) = qrpp2+qrpp3+qrpp4;
    Yr(2,48) = cos(q2+q3);
    Yr(2,49) = -sin(q2+q3+q4);
    Yr(2,50) = sin(q2+q3+q4+q5)/2.0-sin(q2+q3+q4-q5)/2.0;
    Yr(2,51) = qrp3;
    Yr(3,3) = qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0)*(-1.0/2.0);
    Yr(3,7) = (qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(3,10) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0-(qrpp2*cos(q5*2.0))/2.0-(qrpp3*cos(q5*2.0))/2.0-(qrpp4*cos(q5*2.0))/2.0+(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp2*qrp5*sin(q5*2.0))/2.0+(qp5*qrp2*sin(q5*2.0))/2.0+(qp3*qrp5*sin(q5*2.0))/2.0+(qp5*qrp3*sin(q5*2.0))/2.0+(qp4*qrp5*sin(q5*2.0))/2.0+(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(3,11) = qp1*qrp5*sin(q2+q3+q4)*(-1.0/2.0)-(qp5*qrp1*sin(q2+q3+q4))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(3,12) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0+(qrpp2*cos(q5*2.0))/2.0+(qrpp3*cos(q5*2.0))/2.0+(qrpp4*cos(q5*2.0))/2.0-(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp5*sin(q5*2.0))/2.0-(qp5*qrp2*sin(q5*2.0))/2.0-(qp3*qrp5*sin(q5*2.0))/2.0-(qp5*qrp3*sin(q5*2.0))/2.0-(qp4*qrp5*sin(q5*2.0))/2.0-(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(3,13) = qrpp2/4.0+qrpp3/4.0+qrpp4/4.0-(qrpp2*cos(q5*2.0))/4.0+(qrpp2*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0))/4.0+(qrpp3*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0))/4.0+(qrpp4*cos(q6*2.0))/4.0-(qrpp2*cos(q5*2.0)*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0)*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0)*cos(q6*2.0))/4.0+(qp2*qrp5*sin(q5*2.0))/4.0+(qp5*qrp2*sin(q5*2.0))/4.0+(qp3*qrp5*sin(q5*2.0))/4.0+(qp5*qrp3*sin(q5*2.0))/4.0-(qp2*qrp6*sin(q6*2.0))/4.0+(qp4*qrp5*sin(q5*2.0))/4.0+(qp5*qrp4*sin(q5*2.0))/4.0-(qp6*qrp2*sin(q6*2.0))/4.0-(qp3*qrp6*sin(q6*2.0))/4.0-(qp6*qrp3*sin(q6*2.0))/4.0-(qp4*qrp6*sin(q6*2.0))/4.0-(qp6*qrp4*sin(q6*2.0))/4.0-(qrpp5*sin(q6*2.0)*sin(q5))/2.0-(qp5*qrp5*sin(q6*2.0)*cos(q5))/2.0-(qp5*qrp6*cos(q6*2.0)*sin(q5))/2.0-(qp6*qrp5*cos(q6*2.0)*sin(q5))/2.0+(qp2*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp2*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp2*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp3*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp6*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp5*qrp4*cos(q6*2.0)*sin(q5*2.0))/4.0+(qp6*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp6*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp1*qrp1*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q2*2.0))/8.0-qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q6*2.0)*sin(q4*2.0)*(3.0/8.0)-qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q3*2.0)*(3.0/8.0)-qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q2*2.0)*(3.0/8.0)-(qp1*qrp5*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q5*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+qp1*qrp1*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*(3.0/8.0)+(qrpp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qrpp1*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q4*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0))/8.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0;
    Yr(3,14) = qrpp2/4.0+qrpp3/4.0+qrpp4/4.0-(qrpp2*cos(q5*2.0))/4.0-(qrpp2*cos(q6*2.0))/4.0-(qrpp3*cos(q5*2.0))/4.0-(qrpp3*cos(q6*2.0))/4.0-(qrpp4*cos(q5*2.0))/4.0-(qrpp4*cos(q6*2.0))/4.0+(qrpp2*cos(q5*2.0)*cos(q6*2.0))/4.0+(qrpp3*cos(q5*2.0)*cos(q6*2.0))/4.0+(qrpp4*cos(q5*2.0)*cos(q6*2.0))/4.0+(qp2*qrp5*sin(q5*2.0))/4.0+(qp5*qrp2*sin(q5*2.0))/4.0+(qp3*qrp5*sin(q5*2.0))/4.0+(qp5*qrp3*sin(q5*2.0))/4.0+(qp2*qrp6*sin(q6*2.0))/4.0+(qp4*qrp5*sin(q5*2.0))/4.0+(qp5*qrp4*sin(q5*2.0))/4.0+(qp6*qrp2*sin(q6*2.0))/4.0+(qp3*qrp6*sin(q6*2.0))/4.0+(qp6*qrp3*sin(q6*2.0))/4.0+(qp4*qrp6*sin(q6*2.0))/4.0+(qp6*qrp4*sin(q6*2.0))/4.0+(qrpp5*sin(q6*2.0)*sin(q5))/2.0+(qp5*qrp5*sin(q6*2.0)*cos(q5))/2.0+(qp5*qrp6*cos(q6*2.0)*sin(q5))/2.0+(qp6*qrp5*cos(q6*2.0)*sin(q5))/2.0-(qp2*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp2*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp2*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp3*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp6*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp5*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp5*qrp4*cos(q6*2.0)*sin(q5*2.0))/4.0-(qp6*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp6*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp6*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp1*qrp1*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qrpp1*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qrpp1*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qrpp1*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp5*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*sin(q4*2.0))/8.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q3*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q2*2.0))/8.0+qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q6*2.0)*sin(q4*2.0)*(3.0/8.0)+qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q3*2.0)*(3.0/8.0)+qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q2*2.0)*(3.0/8.0)-(qp1*qrp5*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp5*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp1*cos(q5*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0-qp1*qrp1*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*(3.0/8.0)-(qrpp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qrpp1*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qrpp1*cos(q6*2.0)*sin(q5*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qrpp1*cos(q6*2.0)*sin(q5*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q4*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0)*cos(q5))/2.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q4*2.0))/8.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q3*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0))/8.0+(qp1*qrp5*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp5*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp6*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q5*2.0)*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0))/8.0-(qp1*qrp6*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*cos(q5))/2.0-(qp1*qrp6*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp6*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*sin(q6*2.0)*cos(q5))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q5)*sin(q3)*sin(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q3)*cos(q5)*sin(q2)*sin(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q3))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp1*qrp6*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp6*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-qp6*qrp1*pow(cos(q1),2.0)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6)-qp6*qrp1*cos(q5)*cos(q6)*pow(sin(q1),2.0)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6);
    Yr(3,15) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0+(qrpp2*cos(q5*2.0))/2.0+(qrpp3*cos(q5*2.0))/2.0+(qrpp4*cos(q5*2.0))/2.0+qrpp6*cos(q5)-(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0+(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0-(qp5*qrp6*sin(q5))/2.0-(qp6*qrp5*sin(q5))/2.0-(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp6*sin(q2+q3+q4-q5))/4.0-(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp6*qrp1*sin(q2+q3+q4-q5))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp2*qrp5*sin(q5*2.0))/2.0-(qp5*qrp2*sin(q5*2.0))/2.0-(qp3*qrp5*sin(q5*2.0))/2.0-(qp5*qrp3*sin(q5*2.0))/2.0-(qp4*qrp5*sin(q5*2.0))/2.0-(qp5*qrp4*sin(q5*2.0))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0+(qp1*qrp6*sin(q2+q3+q4+q5))/4.0+(qp6*qrp1*sin(q2+q3+q4+q5))/4.0;
    Yr(3,16) = qrpp2/2.0+qrpp3/2.0+qrpp4/2.0-(qrpp2*cos(q5*2.0))/2.0-(qrpp3*cos(q5*2.0))/2.0-(qrpp4*cos(q5*2.0))/2.0+(qrpp1*cos(q2+q3+q4-q5*2.0))/4.0-(qrpp1*cos(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp5*sin(q2+q3+q4+q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp5*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp5*sin(q2+q3+q4))/2.0-(qp5*qrp1*sin(q2+q3+q4))/2.0+(qp2*qrp5*sin(q5*2.0))/2.0+(qp5*qrp2*sin(q5*2.0))/2.0+(qp3*qrp5*sin(q5*2.0))/2.0+(qp5*qrp3*sin(q5*2.0))/2.0+(qp4*qrp5*sin(q5*2.0))/2.0+(qp5*qrp4*sin(q5*2.0))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/4.0;
    Yr(3,21) = qrpp2+qrpp3+qrpp4-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0))/2.0;
    Yr(3,22) = -qrpp2*sin(q4)-qrpp3*sin(q4)+(qp1*qrp1*cos(q4))/2.0+qp2*qrp2*cos(q4)+qp2*qrp3*cos(q4)+qp3*qrp2*cos(q4)+qp3*qrp3*cos(q4)+(qp1*qrp1*cos(q2*2.0+q3*2.0+q4))/2.0;
    Yr(3,25) = (qrpp1*cos(q2+q3+q4+q5))/2.0-qrpp5*cos(q5)+(qrpp1*cos(q2+q3+q4-q5))/2.0+qp5*qrp5*sin(q5)+(qp1*qrp5*sin(q2+q3+q4-q5))/2.0+(qp5*qrp1*sin(q2+q3+q4-q5))/2.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5))/2.0-(qp1*qrp5*sin(q2+q3+q4+q5))/2.0-(qp5*qrp1*sin(q2+q3+q4+q5))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5))/2.0;
    Yr(3,26) = qrpp1*(cos(q2+q3+q4+q5)-cos(q2+q3+q4-q5))*(-1.0/2.0);
    Yr(3,28) = sin(q5)*(qrpp2*cos(q3+q4)+(qp1*qrp1*sin(q2*2.0+q3+q4))/2.0+(qp1*qrp1*sin(q3+q4))/2.0+qp2*qrp2*sin(q3+q4));
    Yr(3,29) = sin(q2+q3+q4)*cos(q1)*sin(q5);
    Yr(3,34) = sin(q2+q3+q4)*sin(q1)*sin(q5);
    Yr(3,35) = -qrpp2*sin(q3+q4)+(qp1*qrp1*cos(q2*2.0+q3+q4))/2.0+(qp1*qrp1*cos(q3+q4))/2.0+qp2*qrp2*cos(q3+q4);
    Yr(3,36) = (sin(q5)*(qrpp2*cos(q4)*2.0+qrpp3*cos(q4)*2.0+qp1*qrp1*sin(q4)+qp2*qrp2*sin(q4)*2.0+qp2*qrp3*sin(q4)*2.0+qp3*qrp2*sin(q4)*2.0+qp3*qrp3*sin(q4)*2.0+qp1*qrp1*sin(q4)*(pow(cos(q2),2.0)*2.0-1.0)*(pow(cos(q3),2.0)*2.0-1.0)+qp1*qrp1*cos(q2)*cos(q4)*sin(q2)*(pow(cos(q3),2.0)*2.0-1.0)*2.0+qp1*qrp1*cos(q3)*cos(q4)*sin(q3)*(pow(cos(q2),2.0)*2.0-1.0)*2.0))/2.0-qp1*qrp1*pow(cos(q1),2.0)*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0-qp1*qrp1*cos(q2)*cos(q3)*pow(sin(q1),2.0)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0;
    Yr(3,37) = qrpp1*cos(q2+q3+q4);
    Yr(3,40) = sin(q1+q2+q3+q4)/2.0-sin(-q1+q2+q3+q4)/2.0;
    Yr(3,41) = cos(q1+q2+q3+q4)/2.0+cos(-q1+q2+q3+q4)/2.0;
    Yr(3,44) = qrpp2+qrpp3+qrpp4;
    Yr(3,49) = -sin(q2+q3+q4);
    Yr(3,50) = sin(q2+q3+q4+q5)/2.0-sin(q2+q3+q4-q5)/2.0;
    Yr(3,52) = qrp4;
    Yr(4,10) = qp1*qrp2*sin(q2+q3+q4-q5*2.0)*(-1.0/4.0)-(qp1*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp2*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp1*sin(q5*2.0))/4.0-(qp2*qrp2*sin(q5*2.0))/2.0-(qp2*qrp3*sin(q5*2.0))/2.0-(qp3*qrp2*sin(q5*2.0))/2.0-(qp2*qrp4*sin(q5*2.0))/2.0-(qp3*qrp3*sin(q5*2.0))/2.0-(qp4*qrp2*sin(q5*2.0))/2.0-(qp3*qrp4*sin(q5*2.0))/2.0-(qp4*qrp3*sin(q5*2.0))/2.0-(qp4*qrp4*sin(q5*2.0))/2.0;
    Yr(4,11) = qrpp5-qrpp1*cos(q2+q3+q4)+(qp1*qrp2*sin(q2+q3+q4))/2.0+(qp2*qrp1*sin(q2+q3+q4))/2.0+(qp1*qrp3*sin(q2+q3+q4))/2.0+(qp3*qrp1*sin(q2+q3+q4))/2.0+(qp1*qrp4*sin(q2+q3+q4))/2.0+(qp4*qrp1*sin(q2+q3+q4))/2.0;
    Yr(4,12) = (qp1*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp1*sin(q5*2.0))/4.0+(qp2*qrp2*sin(q5*2.0))/2.0+(qp2*qrp3*sin(q5*2.0))/2.0+(qp3*qrp2*sin(q5*2.0))/2.0+(qp2*qrp4*sin(q5*2.0))/2.0+(qp3*qrp3*sin(q5*2.0))/2.0+(qp4*qrp2*sin(q5*2.0))/2.0+(qp3*qrp4*sin(q5*2.0))/2.0+(qp4*qrp3*sin(q5*2.0))/2.0+(qp4*qrp4*sin(q5*2.0))/2.0;
    Yr(4,13) = qrpp5-qrpp5*pow(cos(q6),2.0)+(qp2*qrp6*sin(q5))/2.0+(qp6*qrp2*sin(q5))/2.0+(qp3*qrp6*sin(q5))/2.0+(qp6*qrp3*sin(q5))/2.0+(qp4*qrp6*sin(q5))/2.0+(qp6*qrp4*sin(q5))/2.0+(qp5*qrp6*sin(q6*2.0))/2.0+(qp6*qrp5*sin(q6*2.0))/2.0-qrpp1*cos(q2)*cos(q3)*cos(q4)-qp2*qrp6*pow(cos(q6),2.0)*sin(q5)-qp6*qrp2*pow(cos(q6),2.0)*sin(q5)-qp3*qrp6*pow(cos(q6),2.0)*sin(q5)-qp6*qrp3*pow(cos(q6),2.0)*sin(q5)-qp4*qrp6*pow(cos(q6),2.0)*sin(q5)-qp6*qrp4*pow(cos(q6),2.0)*sin(q5)+qrpp1*cos(q2)*sin(q3)*sin(q4)+qrpp1*cos(q3)*sin(q2)*sin(q4)+qrpp1*cos(q4)*sin(q2)*sin(q3)-qrpp2*cos(q6)*sin(q5)*sin(q6)-qrpp3*cos(q6)*sin(q5)*sin(q6)-qrpp4*cos(q6)*sin(q5)*sin(q6)+qp1*qrp1*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp2*qrp2*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp2*qrp3*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp3*qrp2*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp2*qrp4*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp3*qrp3*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp4*qrp2*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp3*qrp4*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp4*qrp3*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp4*qrp4*cos(q5)*pow(cos(q6),2.0)*sin(q5)+qrpp1*cos(q2)*cos(q3)*cos(q4)*pow(cos(q6),2.0)-qrpp1*cos(q2)*pow(cos(q6),2.0)*sin(q3)*sin(q4)-qrpp1*cos(q3)*pow(cos(q6),2.0)*sin(q2)*sin(q4)-qrpp1*cos(q4)*pow(cos(q6),2.0)*sin(q2)*sin(q3)+(qp1*qrp2*cos(q2)*cos(q3)*sin(q4))/2.0+(qp1*qrp2*cos(q2)*cos(q4)*sin(q3))/2.0+(qp1*qrp2*cos(q3)*cos(q4)*sin(q2))/2.0+(qp2*qrp1*cos(q2)*cos(q3)*sin(q4))/2.0+(qp2*qrp1*cos(q2)*cos(q4)*sin(q3))/2.0+(qp2*qrp1*cos(q3)*cos(q4)*sin(q2))/2.0+(qp1*qrp3*cos(q2)*cos(q3)*sin(q4))/2.0+(qp1*qrp3*cos(q2)*cos(q4)*sin(q3))/2.0+(qp1*qrp3*cos(q3)*cos(q4)*sin(q2))/2.0+(qp3*qrp1*cos(q2)*cos(q3)*sin(q4))/2.0+(qp3*qrp1*cos(q2)*cos(q4)*sin(q3))/2.0+(qp3*qrp1*cos(q3)*cos(q4)*sin(q2))/2.0+(qp1*qrp4*cos(q2)*cos(q3)*sin(q4))/2.0+(qp1*qrp4*cos(q2)*cos(q4)*sin(q3))/2.0+(qp1*qrp4*cos(q3)*cos(q4)*sin(q2))/2.0+(qp4*qrp1*cos(q2)*cos(q3)*sin(q4))/2.0+(qp4*qrp1*cos(q2)*cos(q4)*sin(q3))/2.0+(qp4*qrp1*cos(q3)*cos(q4)*sin(q2))/2.0-(qp1*qrp2*sin(q2)*sin(q3)*sin(q4))/2.0-(qp2*qrp1*sin(q2)*sin(q3)*sin(q4))/2.0-(qp1*qrp3*sin(q2)*sin(q3)*sin(q4))/2.0-(qp3*qrp1*sin(q2)*sin(q3)*sin(q4))/2.0-(qp1*qrp4*sin(q2)*sin(q3)*sin(q4))/2.0-(qp4*qrp1*sin(q2)*sin(q3)*sin(q4))/2.0-qp1*qrp1*pow(cos(q2),2.0)*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp1*qrp1*pow(cos(q3),2.0)*cos(q5)*pow(cos(q6),2.0)*sin(q5)-qp1*qrp1*pow(cos(q4),2.0)*cos(q5)*pow(cos(q6),2.0)*sin(q5)+(qp1*qrp6*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2.0+(qp1*qrp6*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2.0+(qp1*qrp6*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2.0+(qp6*qrp1*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2.0+(qp6*qrp1*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2.0+(qp6*qrp1*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2.0-(qp1*qrp6*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2.0-(qp6*qrp1*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2.0+qrpp1*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q6)-qp1*qrp2*cos(q2)*cos(q3)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q4)-qp1*qrp2*cos(q2)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q3)-qp1*qrp2*cos(q3)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)-qp2*qrp1*cos(q2)*cos(q3)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q4)-qp2*qrp1*cos(q2)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q3)-qp2*qrp1*cos(q3)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)-qp1*qrp3*cos(q2)*cos(q3)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q4)-qp1*qrp3*cos(q2)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q3)-qp1*qrp3*cos(q3)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)-qp3*qrp1*cos(q2)*cos(q3)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q4)-qp3*qrp1*cos(q2)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q3)-qp3*qrp1*cos(q3)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)-qp1*qrp4*cos(q2)*cos(q3)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q4)-qp1*qrp4*cos(q2)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q3)-qp1*qrp4*cos(q3)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)-qp4*qrp1*cos(q2)*cos(q3)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q4)-qp4*qrp1*cos(q2)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q3)-qp4*qrp1*cos(q3)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)+qp1*qrp2*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)+qp2*qrp1*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)+qp1*qrp3*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)+qp3*qrp1*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)+qp1*qrp4*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)+qp4*qrp1*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)-qp1*qrp6*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q6)-qp6*qrp1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q6)+qp1*qrp1*pow(cos(q2),2.0)*pow(cos(q3),2.0)*cos(q5)*pow(cos(q6),2.0)*sin(q5)*2.0+qp1*qrp1*pow(cos(q2),2.0)*pow(cos(q4),2.0)*cos(q5)*pow(cos(q6),2.0)*sin(q5)*2.0+qp1*qrp1*pow(cos(q3),2.0)*pow(cos(q4),2.0)*cos(q5)*pow(cos(q6),2.0)*sin(q5)*2.0+qp1*qrp1*cos(q2)*cos(q6)*sin(q2)*sin(q5)*sin(q6)+qp1*qrp1*cos(q3)*cos(q6)*sin(q3)*sin(q5)*sin(q6)+qp1*qrp1*cos(q4)*cos(q6)*sin(q4)*sin(q5)*sin(q6)+qp1*qrp6*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q6)+qp1*qrp6*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q6)+qp1*qrp6*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q6)+qp6*qrp1*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q6)+qp6*qrp1*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q6)+qp6*qrp1*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q6)-qp1*qrp6*cos(q2)*cos(q3)*cos(q5)*pow(cos(q6),2.0)*sin(q4)-qp1*qrp6*cos(q2)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q3)-qp1*qrp6*cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q2)-qp6*qrp1*cos(q2)*cos(q3)*cos(q5)*pow(cos(q6),2.0)*sin(q4)-qp6*qrp1*cos(q2)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q3)-qp6*qrp1*cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q2)-qrpp1*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4)*sin(q6)-qrpp1*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*sin(q6)-qrpp1*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q6)+qp1*qrp6*cos(q5)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)+qp6*qrp1*cos(q5)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)-qp1*qrp2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6)-qp2*qrp1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6)-qp1*qrp3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6)-qp3*qrp1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6)-qp1*qrp4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6)-qp4*qrp1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q6)+qp1*qrp2*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6)+qp1*qrp2*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6)+qp1*qrp2*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6)+qp2*qrp1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6)+qp2*qrp1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6)+qp2*qrp1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6)+qp1*qrp3*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6)+qp1*qrp3*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6)+qp1*qrp3*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6)+qp3*qrp1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6)+qp3*qrp1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6)+qp3*qrp1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6)+qp1*qrp4*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6)+qp1*qrp4*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6)+qp1*qrp4*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6)+qp4*qrp1*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4)*sin(q6)+qp4*qrp1*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*sin(q6)+qp4*qrp1*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q6)-qp1*qrp1*pow(cos(q2),2.0)*pow(cos(q3),2.0)*pow(cos(q4),2.0)*cos(q5)*pow(cos(q6),2.0)*sin(q5)*4.0-qp1*qrp1*cos(q2)*pow(cos(q3),2.0)*cos(q6)*sin(q2)*sin(q5)*sin(q6)*2.0-qp1*qrp1*cos(q2)*pow(cos(q4),2.0)*cos(q6)*sin(q2)*sin(q5)*sin(q6)*2.0-qp1*qrp1*pow(cos(q2),2.0)*cos(q3)*cos(q6)*sin(q3)*sin(q5)*sin(q6)*2.0-qp1*qrp1*cos(q3)*pow(cos(q4),2.0)*cos(q6)*sin(q3)*sin(q5)*sin(q6)*2.0-qp1*qrp1*pow(cos(q2),2.0)*cos(q4)*cos(q6)*sin(q4)*sin(q5)*sin(q6)*2.0-qp1*qrp1*pow(cos(q3),2.0)*cos(q4)*cos(q6)*sin(q4)*sin(q5)*sin(q6)*2.0+qp1*qrp1*cos(q2)*pow(cos(q3),2.0)*pow(cos(q4),2.0)*cos(q6)*sin(q2)*sin(q5)*sin(q6)*4.0+qp1*qrp1*pow(cos(q2),2.0)*cos(q3)*pow(cos(q4),2.0)*cos(q6)*sin(q3)*sin(q5)*sin(q6)*4.0+qp1*qrp1*pow(cos(q2),2.0)*pow(cos(q3),2.0)*cos(q4)*cos(q6)*sin(q4)*sin(q5)*sin(q6)*4.0-qp1*qrp1*cos(q2)*cos(q3)*cos(q5)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q5)*2.0-qp1*qrp1*cos(q2)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q2)*sin(q4)*sin(q5)*2.0-qp1*qrp1*cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q3)*sin(q4)*sin(q5)*2.0+qp1*qrp1*cos(q2)*cos(q3)*pow(cos(q4),2.0)*cos(q5)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q5)*4.0+qp1*qrp1*cos(q2)*pow(cos(q3),2.0)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q2)*sin(q4)*sin(q5)*4.0+qp1*qrp1*pow(cos(q2),2.0)*cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q3)*sin(q4)*sin(q5)*4.0-qp1*qrp1*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q6)*4.0;
    Yr(4,14) = qrpp5/2.0+(qrpp5*cos(q6*2.0))/2.0+(qrpp1*cos(q2+q3+q4+q5-q6*2.0))/8.0-(qrpp1*cos(q2+q3+q4+q5+q6*2.0))/8.0+(qrpp1*cos(q2+q3+q4-q5-q6*2.0))/8.0-(qrpp1*cos(q2+q3+q4-q5+q6*2.0))/8.0-(qrpp1*cos(q2+q3+q4-q6*2.0))/4.0-(qrpp1*cos(q2+q3+q4+q6*2.0))/4.0+(qrpp2*cos(q5-q6*2.0))/4.0-(qrpp2*cos(q5+q6*2.0))/4.0+(qrpp3*cos(q5-q6*2.0))/4.0-(qrpp3*cos(q5+q6*2.0))/4.0+(qrpp4*cos(q5-q6*2.0))/4.0-(qrpp4*cos(q5+q6*2.0))/4.0-(qrpp1*cos(q2+q3+q4))/2.0-(qp1*qrp2*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp1*qrp2*sin(q2+q3+q4-q5+q6*2.0))/8.0-(qp2*qrp1*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp2*qrp1*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp1*qrp2*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp1*qrp2*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp1*qrp2*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp1*qrp2*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp1*qrp3*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp1*qrp3*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp2*qrp1*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp1*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp1*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp2*qrp1*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp3*qrp1*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp3*qrp1*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp1*qrp3*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp1*qrp3*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp1*qrp3*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp1*qrp3*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp1*qrp4*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp1*qrp4*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp3*qrp1*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp1*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp3*qrp1*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp3*qrp1*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1-(qp4*qrp1*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp4*qrp1*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp1*qrp4*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp1*qrp4*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp1*qrp4*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp1*qrp4*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp1*sin(q2+q3+q4-q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp1*sin(q2+q3+q4-q5*2.0+q6*2.0))/1.6E+1+(qp4*qrp1*sin(q2+q3+q4+q5*2.0-q6*2.0))/1.6E+1+(qp4*qrp1*sin(q2+q3+q4+q5*2.0+q6*2.0))/1.6E+1+(qp1*qrp6*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp1*qrp6*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp6*qrp1*sin(q2+q3+q4-q5-q6*2.0))/8.0+(qp6*qrp1*sin(q2+q3+q4-q5+q6*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5-q6*2.0))/1.6E+1-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5+q6*2.0))/1.6E+1-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0-q6*2.0))/3.2E+1-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0+q6*2.0))/3.2E+1+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0-q6*2.0))/3.2E+1+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0+q6*2.0))/3.2E+1-(qp1*qrp2*sin(q2+q3+q4-q5*2.0))/8.0-(qp1*qrp2*sin(q2+q3+q4+q5*2.0))/8.0-(qp2*qrp1*sin(q2+q3+q4-q5*2.0))/8.0-(qp2*qrp1*sin(q2+q3+q4+q5*2.0))/8.0+(qp1*qrp2*sin(q2+q3+q4-q6*2.0))/8.0+(qp1*qrp2*sin(q2+q3+q4+q6*2.0))/8.0-(qp1*qrp3*sin(q2+q3+q4-q5*2.0))/8.0-(qp1*qrp3*sin(q2+q3+q4+q5*2.0))/8.0+(qp2*qrp1*sin(q2+q3+q4-q6*2.0))/8.0+(qp2*qrp1*sin(q2+q3+q4+q6*2.0))/8.0-(qp3*qrp1*sin(q2+q3+q4-q5*2.0))/8.0-(qp3*qrp1*sin(q2+q3+q4+q5*2.0))/8.0+(qp1*qrp3*sin(q2+q3+q4-q6*2.0))/8.0+(qp1*qrp3*sin(q2+q3+q4+q6*2.0))/8.0-(qp1*qrp4*sin(q2+q3+q4-q5*2.0))/8.0-(qp1*qrp4*sin(q2+q3+q4+q5*2.0))/8.0+(qp3*qrp1*sin(q2+q3+q4-q6*2.0))/8.0+(qp3*qrp1*sin(q2+q3+q4+q6*2.0))/8.0-(qp4*qrp1*sin(q2+q3+q4-q5*2.0))/8.0-(qp4*qrp1*sin(q2+q3+q4+q5*2.0))/8.0+(qp1*qrp4*sin(q2+q3+q4-q6*2.0))/8.0+(qp1*qrp4*sin(q2+q3+q4+q6*2.0))/8.0+(qp4*qrp1*sin(q2+q3+q4-q6*2.0))/8.0+(qp4*qrp1*sin(q2+q3+q4+q6*2.0))/8.0-(qp1*qrp6*sin(q2+q3+q4-q6*2.0))/4.0+(qp1*qrp6*sin(q2+q3+q4+q6*2.0))/4.0-(qp6*qrp1*sin(q2+q3+q4-q6*2.0))/4.0+(qp6*qrp1*sin(q2+q3+q4+q6*2.0))/4.0+(qp2*qrp6*sin(q5-q6*2.0))/4.0+(qp2*qrp6*sin(q5+q6*2.0))/4.0+(qp6*qrp2*sin(q5-q6*2.0))/4.0+(qp6*qrp2*sin(q5+q6*2.0))/4.0+(qp3*qrp6*sin(q5-q6*2.0))/4.0+(qp3*qrp6*sin(q5+q6*2.0))/4.0+(qp6*qrp3*sin(q5-q6*2.0))/4.0+(qp6*qrp3*sin(q5+q6*2.0))/4.0+(qp4*qrp6*sin(q5-q6*2.0))/4.0+(qp4*qrp6*sin(q5+q6*2.0))/4.0+(qp6*qrp4*sin(q5-q6*2.0))/4.0+(qp6*qrp4*sin(q5+q6*2.0))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/1.6E+1-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/1.6E+1+(qp1*qrp2*sin(q2+q3+q4))/4.0+(qp2*qrp1*sin(q2+q3+q4))/4.0+(qp1*qrp3*sin(q2+q3+q4))/4.0+(qp3*qrp1*sin(q2+q3+q4))/4.0+(qp1*qrp4*sin(q2+q3+q4))/4.0+(qp4*qrp1*sin(q2+q3+q4))/4.0+(qp1*qrp1*sin(q5*2.0))/8.0-(qp2*qrp2*sin(q5*2.0))/4.0-(qp2*qrp3*sin(q5*2.0))/4.0-(qp3*qrp2*sin(q5*2.0))/4.0-(qp2*qrp4*sin(q5*2.0))/4.0-(qp3*qrp3*sin(q5*2.0))/4.0-(qp4*qrp2*sin(q5*2.0))/4.0-(qp3*qrp4*sin(q5*2.0))/4.0-(qp4*qrp3*sin(q5*2.0))/4.0-(qp4*qrp4*sin(q5*2.0))/4.0-(qp5*qrp6*sin(q6*2.0))/2.0-(qp6*qrp5*sin(q6*2.0))/2.0-(qp1*qrp2*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp1*qrp2*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp2*qrp1*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp2*qrp1*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp1*qrp3*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp1*qrp3*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp3*qrp1*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp3*qrp1*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp1*qrp4*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp1*qrp4*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp4*qrp1*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp4*qrp1*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp1*qrp6*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp1*qrp6*sin(q2+q3+q4+q5+q6*2.0))/8.0+(qp6*qrp1*sin(q2+q3+q4+q5-q6*2.0))/8.0+(qp6*qrp1*sin(q2+q3+q4+q5+q6*2.0))/8.0-(qp1*qrp1*sin(q5*2.0-q6*2.0))/1.6E+1-(qp1*qrp1*sin(q5*2.0+q6*2.0))/1.6E+1+(qp2*qrp2*sin(q5*2.0-q6*2.0))/8.0+(qp2*qrp2*sin(q5*2.0+q6*2.0))/8.0+(qp2*qrp3*sin(q5*2.0-q6*2.0))/8.0+(qp2*qrp3*sin(q5*2.0+q6*2.0))/8.0+(qp3*qrp2*sin(q5*2.0-q6*2.0))/8.0+(qp3*qrp2*sin(q5*2.0+q6*2.0))/8.0+(qp2*qrp4*sin(q5*2.0-q6*2.0))/8.0+(qp2*qrp4*sin(q5*2.0+q6*2.0))/8.0+(qp3*qrp3*sin(q5*2.0-q6*2.0))/8.0+(qp3*qrp3*sin(q5*2.0+q6*2.0))/8.0+(qp4*qrp2*sin(q5*2.0-q6*2.0))/8.0+(qp4*qrp2*sin(q5*2.0+q6*2.0))/8.0+(qp3*qrp4*sin(q5*2.0-q6*2.0))/8.0+(qp3*qrp4*sin(q5*2.0+q6*2.0))/8.0+(qp4*qrp3*sin(q5*2.0-q6*2.0))/8.0+(qp4*qrp3*sin(q5*2.0+q6*2.0))/8.0+(qp4*qrp4*sin(q5*2.0-q6*2.0))/8.0+(qp4*qrp4*sin(q5*2.0+q6*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5-q6*2.0))/1.6E+1+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5+q6*2.0))/1.6E+1;
    Yr(4,15) = (qp2*qrp6*sin(q5))/2.0+(qp6*qrp2*sin(q5))/2.0+(qp3*qrp6*sin(q5))/2.0+(qp6*qrp3*sin(q5))/2.0+(qp4*qrp6*sin(q5))/2.0+(qp6*qrp4*sin(q5))/2.0+(qp1*qrp2*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp2*sin(q2+q3+q4+q5*2.0))/4.0+(qp2*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp2*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp3*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp3*sin(q2+q3+q4+q5*2.0))/4.0+(qp3*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp3*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp4*sin(q2+q3+q4-q5*2.0))/4.0+(qp1*qrp4*sin(q2+q3+q4+q5*2.0))/4.0+(qp4*qrp1*sin(q2+q3+q4-q5*2.0))/4.0+(qp4*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp6*sin(q2+q3+q4-q5))/4.0+(qp6*qrp1*sin(q2+q3+q4-q5))/4.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0-(qp1*qrp1*sin(q5*2.0))/4.0+(qp2*qrp2*sin(q5*2.0))/2.0+(qp2*qrp3*sin(q5*2.0))/2.0+(qp3*qrp2*sin(q5*2.0))/2.0+(qp2*qrp4*sin(q5*2.0))/2.0+(qp3*qrp3*sin(q5*2.0))/2.0+(qp4*qrp2*sin(q5*2.0))/2.0+(qp3*qrp4*sin(q5*2.0))/2.0+(qp4*qrp3*sin(q5*2.0))/2.0+(qp4*qrp4*sin(q5*2.0))/2.0+(qp1*qrp6*sin(q2+q3+q4+q5))/4.0+(qp6*qrp1*sin(q2+q3+q4+q5))/4.0;
    Yr(4,16) = qrpp5-qrpp1*cos(q2+q3+q4)-(qp1*qrp2*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp2*sin(q2+q3+q4+q5*2.0))/4.0-(qp2*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp2*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp3*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp3*sin(q2+q3+q4+q5*2.0))/4.0-(qp3*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp3*qrp1*sin(q2+q3+q4+q5*2.0))/4.0-(qp1*qrp4*sin(q2+q3+q4-q5*2.0))/4.0-(qp1*qrp4*sin(q2+q3+q4+q5*2.0))/4.0-(qp4*qrp1*sin(q2+q3+q4-q5*2.0))/4.0-(qp4*qrp1*sin(q2+q3+q4+q5*2.0))/4.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5*2.0))/8.0-(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5*2.0))/8.0+(qp1*qrp2*sin(q2+q3+q4))/2.0+(qp2*qrp1*sin(q2+q3+q4))/2.0+(qp1*qrp3*sin(q2+q3+q4))/2.0+(qp3*qrp1*sin(q2+q3+q4))/2.0+(qp1*qrp4*sin(q2+q3+q4))/2.0+(qp4*qrp1*sin(q2+q3+q4))/2.0+(qp1*qrp1*sin(q5*2.0))/4.0-(qp2*qrp2*sin(q5*2.0))/2.0-(qp2*qrp3*sin(q5*2.0))/2.0-(qp3*qrp2*sin(q5*2.0))/2.0-(qp2*qrp4*sin(q5*2.0))/2.0-(qp3*qrp3*sin(q5*2.0))/2.0-(qp4*qrp2*sin(q5*2.0))/2.0-(qp3*qrp4*sin(q5*2.0))/2.0-(qp4*qrp3*sin(q5*2.0))/2.0-(qp4*qrp4*sin(q5*2.0))/2.0;
    Yr(4,25) = qrpp1*cos(q2+q3+q4+q5)*(-1.0/2.0)-qrpp2*cos(q5)-qrpp3*cos(q5)-qrpp4*cos(q5)+(qrpp1*cos(q2+q3+q4-q5))/2.0-(qp1*qrp2*sin(q2+q3+q4-q5))/2.0-(qp2*qrp1*sin(q2+q3+q4-q5))/2.0-(qp1*qrp3*sin(q2+q3+q4-q5))/2.0-(qp3*qrp1*sin(q2+q3+q4-q5))/2.0-(qp1*qrp4*sin(q2+q3+q4-q5))/2.0-(qp4*qrp1*sin(q2+q3+q4-q5))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0-q5))/4.0+(qp1*qrp2*sin(q2+q3+q4+q5))/2.0+(qp2*qrp1*sin(q2+q3+q4+q5))/2.0+(qp1*qrp3*sin(q2+q3+q4+q5))/2.0+(qp3*qrp1*sin(q2+q3+q4+q5))/2.0+(qp1*qrp4*sin(q2+q3+q4+q5))/2.0+(qp4*qrp1*sin(q2+q3+q4+q5))/2.0+(qp1*qrp1*sin(q2*2.0+q3*2.0+q4*2.0+q5))/4.0;
    Yr(4,26) = qrpp1*cos(q2+q3+q4+q5)*(-1.0/2.0)-(qrpp1*cos(q2+q3+q4-q5))/2.0+qp1*qrp1*sin(q5);
    Yr(4,28) = -sin(q5)*(qrpp1*cos(q2)-qp1*qrp2*sin(q2))+qp2*qrp1*sin(q2)*sin(q5)+qrpp2*cos(q3)*cos(q5)*sin(q4)+qrpp2*cos(q4)*cos(q5)*sin(q3)-qp2*qrp2*cos(q3)*cos(q4)*cos(q5)+qp2*qrp2*cos(q5)*sin(q3)*sin(q4)+qp1*qrp1*pow(cos(q2),2.0)*cos(q5)*sin(q3)*sin(q4)-qp1*qrp1*pow(cos(q2),2.0)*cos(q3)*cos(q4)*cos(q5)+qp1*qrp1*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q4)+qp1*qrp1*cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q3);
    Yr(4,29) = -sin(q1)*sin(q5)-cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)+cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)+cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4)+cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3);
    Yr(4,34) = cos(q1)*sin(q5)-cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)+cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)+cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4)+cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3);
    Yr(4,36) = qrpp2*cos(q5)*sin(q4)+qrpp3*cos(q5)*sin(q4)-qrpp1*cos(q2)*cos(q3)*sin(q5)+qrpp1*sin(q2)*sin(q3)*sin(q5)-(qp1*qrp1*cos(q4)*cos(q5))/2.0-qp2*qrp2*cos(q4)*cos(q5)-qp2*qrp3*cos(q4)*cos(q5)-qp3*qrp2*cos(q4)*cos(q5)-qp3*qrp3*cos(q4)*cos(q5)+qp1*qrp2*cos(q2)*sin(q3)*sin(q5)+qp1*qrp2*cos(q3)*sin(q2)*sin(q5)+qp2*qrp1*cos(q2)*sin(q3)*sin(q5)+qp2*qrp1*cos(q3)*sin(q2)*sin(q5)+qp1*qrp3*cos(q2)*sin(q3)*sin(q5)+qp1*qrp3*cos(q3)*sin(q2)*sin(q5)+qp3*qrp1*cos(q2)*sin(q3)*sin(q5)+qp3*qrp1*cos(q3)*sin(q2)*sin(q5)-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4)*cos(q5))/2.0+(qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*cos(q5)*sin(q4))/2.0+(qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*cos(q5)*sin(q4))/2.0+(qp1*qrp1*sin(q2*2.0)*sin(q3*2.0)*cos(q4)*cos(q5))/2.0;
    Yr(4,50) = sin(q2+q3+q4+q5)/2.0+sin(q2+q3+q4-q5)/2.0;
    Yr(4,53) = qrp5;
    Yr(5,13) = qp1*qrp1*sin(q6*2.0)*(-1.0/8.0)+(qp2*qrp2*sin(q6*2.0))/4.0+(qp2*qrp3*sin(q6*2.0))/4.0+(qp3*qrp2*sin(q6*2.0))/4.0+(qp2*qrp4*sin(q6*2.0))/4.0+(qp3*qrp3*sin(q6*2.0))/4.0+(qp4*qrp2*sin(q6*2.0))/4.0+(qp3*qrp4*sin(q6*2.0))/4.0+(qp4*qrp3*sin(q6*2.0))/4.0+(qp4*qrp4*sin(q6*2.0))/4.0-(qp5*qrp5*sin(q6*2.0))/2.0+(qp2*qrp5*cos(q6*2.0)*sin(q5))/2.0+(qp5*qrp2*cos(q6*2.0)*sin(q5))/2.0+(qp3*qrp5*cos(q6*2.0)*sin(q5))/2.0+(qp5*qrp3*cos(q6*2.0)*sin(q5))/2.0+(qp4*qrp5*cos(q6*2.0)*sin(q5))/2.0+(qp5*qrp4*cos(q6*2.0)*sin(q5))/2.0+(qp1*qrp1*cos(q5*2.0)*sin(q6*2.0))/8.0-(qp2*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp2*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp2*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp3*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp4*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4))/2.0-qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*sin(q6*2.0)*(3.0/8.0)-(qp1*qrp5*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4))/2.0-(qp1*qrp5*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3))/2.0+qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0)*(3.0/8.0)+qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0)*(3.0/8.0)+qp1*qrp1*cos(q4*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0)*(3.0/8.0)+(qp1*qrp1*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q6*2.0))/8.0+(qp1*qrp2*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp2*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp2*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp2*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp2*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp2*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp3*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp3*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp3*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp3*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp3*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp3*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp4*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp1*qrp4*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp1*qrp4*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp4*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0+(qp4*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0+(qp4*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0+(qp1*qrp1*cos(q2*2.0)*cos(q5*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0))/8.0+(qp1*qrp1*cos(q3*2.0)*cos(q5*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0))/8.0+(qp1*qrp1*cos(q4*2.0)*cos(q5*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0))/8.0-(qp1*qrp2*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp2*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp3*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp3*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp4*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp4*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0-(qp1*qrp2*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp2*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp1*qrp3*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp3*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp1*qrp4*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp4*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2.0+(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2.0+(qp1*qrp5*cos(q6*2.0)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2.0+(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2.0+(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2.0+(qp5*qrp1*cos(q6*2.0)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2.0-(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q6*2.0)*sin(q4*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q3*2.0)*cos(q5))/2.0-(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q2*2.0)*cos(q5))/2.0+(qp1*qrp2*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp1*qrp2*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp1*qrp2*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp2*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp2*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp2*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp1*qrp3*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp1*qrp3*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp1*qrp3*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp3*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp3*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp3*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp1*qrp4*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp1*qrp4*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp1*qrp4*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp4*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0+(qp4*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0+(qp4*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp1*qrp5*cos(q6*2.0)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2.0-(qp5*qrp1*cos(q6*2.0)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2.0;
    Yr(5,14) = (qp1*qrp1*sin(q6*2.0))/8.0-(qp2*qrp2*sin(q6*2.0))/4.0-(qp2*qrp3*sin(q6*2.0))/4.0-(qp3*qrp2*sin(q6*2.0))/4.0-(qp2*qrp4*sin(q6*2.0))/4.0-(qp3*qrp3*sin(q6*2.0))/4.0-(qp4*qrp2*sin(q6*2.0))/4.0-(qp3*qrp4*sin(q6*2.0))/4.0-(qp4*qrp3*sin(q6*2.0))/4.0-(qp4*qrp4*sin(q6*2.0))/4.0+(qp5*qrp5*sin(q6*2.0))/2.0-(qp2*qrp5*cos(q6*2.0)*sin(q5))/2.0-(qp5*qrp2*cos(q6*2.0)*sin(q5))/2.0-(qp3*qrp5*cos(q6*2.0)*sin(q5))/2.0-(qp5*qrp3*cos(q6*2.0)*sin(q5))/2.0-(qp4*qrp5*cos(q6*2.0)*sin(q5))/2.0-(qp5*qrp4*cos(q6*2.0)*sin(q5))/2.0-(qp1*qrp1*cos(q5*2.0)*sin(q6*2.0))/8.0+(qp2*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp2*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp2*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp2*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp3*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp3*cos(q5*2.0)*sin(q6*2.0))/4.0+(qp4*qrp4*cos(q5*2.0)*sin(q6*2.0))/4.0-(qp1*qrp5*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4))/2.0-(qp5*qrp1*sin(q6*2.0)*cos(q2)*cos(q3)*cos(q4))/2.0+qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*sin(q6*2.0)*(3.0/8.0)+(qp1*qrp5*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4))/2.0+(qp1*qrp5*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q2)*sin(q3)*sin(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q3)*sin(q2)*sin(q4))/2.0+(qp5*qrp1*sin(q6*2.0)*cos(q4)*sin(q2)*sin(q3))/2.0-qp1*qrp1*cos(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0)*(3.0/8.0)-qp1*qrp1*cos(q3*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0)*(3.0/8.0)-qp1*qrp1*cos(q4*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0)*(3.0/8.0)-(qp1*qrp1*cos(q6*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q4*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q4*2.0)*cos(q5*2.0)*sin(q6*2.0))/8.0-(qp1*qrp2*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp2*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp2*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp2*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp2*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp2*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp3*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp3*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp3*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp3*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp3*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp3*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp4*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp1*qrp4*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp1*qrp4*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp4*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q3)*sin(q4))/4.0-(qp4*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q2)*cos(q4)*sin(q3))/4.0-(qp4*qrp1*sin(q5*2.0)*sin(q6*2.0)*cos(q3)*cos(q4)*sin(q2))/4.0-(qp1*qrp1*cos(q2*2.0)*cos(q5*2.0)*sin(q3*2.0)*sin(q4*2.0)*sin(q6*2.0))/8.0-(qp1*qrp1*cos(q3*2.0)*cos(q5*2.0)*sin(q2*2.0)*sin(q4*2.0)*sin(q6*2.0))/8.0-(qp1*qrp1*cos(q4*2.0)*cos(q5*2.0)*sin(q2*2.0)*sin(q3*2.0)*sin(q6*2.0))/8.0+(qp1*qrp2*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp2*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp3*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp3*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp4*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp4*qrp1*sin(q5*2.0)*sin(q6*2.0)*sin(q2)*sin(q3)*sin(q4))/4.0+(qp1*qrp2*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp2*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp1*qrp3*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp3*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp1*qrp4*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0+(qp4*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q4)*sin(q5))/2.0-(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2.0-(qp1*qrp5*cos(q6*2.0)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2.0-(qp1*qrp5*cos(q6*2.0)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2.0-(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q3)*cos(q5)*sin(q4))/2.0-(qp5*qrp1*cos(q6*2.0)*cos(q2)*cos(q4)*cos(q5)*sin(q3))/2.0-(qp5*qrp1*cos(q6*2.0)*cos(q3)*cos(q4)*cos(q5)*sin(q2))/2.0+(qp1*qrp1*cos(q2*2.0)*cos(q3*2.0)*cos(q6*2.0)*sin(q4*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q2*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q3*2.0)*cos(q5))/2.0+(qp1*qrp1*cos(q3*2.0)*cos(q4*2.0)*cos(q6*2.0)*sin(q2*2.0)*cos(q5))/2.0-(qp1*qrp2*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp1*qrp2*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp1*qrp2*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp2*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp2*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp2*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp1*qrp3*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp1*qrp3*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp1*qrp3*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp3*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp3*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp3*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp1*qrp4*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp1*qrp4*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp1*qrp4*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0-(qp4*qrp1*cos(q6*2.0)*cos(q2)*sin(q3)*sin(q4)*sin(q5))/2.0-(qp4*qrp1*cos(q6*2.0)*cos(q3)*sin(q2)*sin(q4)*sin(q5))/2.0-(qp4*qrp1*cos(q6*2.0)*cos(q4)*sin(q2)*sin(q3)*sin(q5))/2.0+(qp1*qrp5*cos(q6*2.0)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2.0+(qp5*qrp1*cos(q6*2.0)*cos(q5)*sin(q2)*sin(q3)*sin(q4))/2.0;
    Yr(5,15) = qrpp6+(qrpp1*cos(q2+q3+q4+q5))/2.0+qrpp2*cos(q5)+qrpp3*cos(q5)+qrpp4*cos(q5)-(qrpp1*cos(q2+q3+q4-q5))/2.0-(qp2*qrp5*sin(q5))/2.0-(qp5*qrp2*sin(q5))/2.0-(qp3*qrp5*sin(q5))/2.0-(qp5*qrp3*sin(q5))/2.0-(qp4*qrp5*sin(q5))/2.0-(qp5*qrp4*sin(q5))/2.0+(qp1*qrp2*sin(q2+q3+q4-q5))/4.0+(qp2*qrp1*sin(q2+q3+q4-q5))/4.0+(qp1*qrp3*sin(q2+q3+q4-q5))/4.0+(qp3*qrp1*sin(q2+q3+q4-q5))/4.0+(qp1*qrp4*sin(q2+q3+q4-q5))/4.0+(qp4*qrp1*sin(q2+q3+q4-q5))/4.0-(qp1*qrp5*sin(q2+q3+q4-q5))/4.0-(qp5*qrp1*sin(q2+q3+q4-q5))/4.0-(qp1*qrp2*sin(q2+q3+q4+q5))/4.0-(qp2*qrp1*sin(q2+q3+q4+q5))/4.0-(qp1*qrp3*sin(q2+q3+q4+q5))/4.0-(qp3*qrp1*sin(q2+q3+q4+q5))/4.0-(qp1*qrp4*sin(q2+q3+q4+q5))/4.0-(qp4*qrp1*sin(q2+q3+q4+q5))/4.0-(qp1*qrp5*sin(q2+q3+q4+q5))/4.0-(qp5*qrp1*sin(q2+q3+q4+q5))/4.0;
    Yr(5,54) = qrp6;

    return Yr;
}

bool SimpleEffortControl::init()
{
    std::string ns="~simple_effort_ctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"SimpleEffortControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }


    VDouble p;

    /////D GAINS
    s<<ns<<"/gains_d";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of d_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kd(i,i)=p[i];
    }

    ROS_WARN_STREAM("Kd: \n"<<m_Kd);

    s.str("");
    s<<ns<<"/gains_d_cart";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF+1)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of d_gains_cart --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF+1;i++)
    {
        m_Kd_cart(i,i)=p[i];
    }

    ROS_WARN_STREAM("Kd_cart: \n"<<m_Kd_cart);

    /////P GAINS
    s.str("");
    s<<ns<<"/gains_p";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of p_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kp(i,i)=p[i]/m_Kd(i,i);
    }

    ROS_WARN_STREAM("Kp: \n"<<m_Kp);

    s.str("");
    s<<ns<<"/gains_p_cart";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF+1)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of p_gains_cart --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF+1;i++)
    {
        m_Kp_cart(i,i)=p[i]/m_Kd_cart(i,i);
    }

    ROS_WARN_STREAM("Kp_cart: \n"<<m_Kp_cart);

    /////I GAINS
    s.str("");
    s<<ns<<"/gains_i";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of p_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Ki(i,i)=p[i];
    }

    ROS_WARN_STREAM("Ki: \n"<<m_Ki);

    s.str("");
    s<<ns<<"/gains_i_cart";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF+1)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of p_gains_cart --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF+1;i++)
    {
        m_Ki_cart(i,i)=p[i];
    }

    ROS_WARN_STREAM("Ki_cart: \n"<<m_Ki_cart);

    /////GOAL
    s.str("");
    s<<ns<<"/goal";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of joint goals --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_goal(i)=p[i];
    }

    ROS_WARN_STREAM("Goal [DEG]: \n"<<m_goal.transpose());

    m_goal=DEG2RAD(m_goal);

    ROS_WARN_STREAM("Goal [RAD]: \n"<<m_goal.transpose());

    /////Time
    s.str("");
    s<<ns<<"/execution_time";
    ros::param::get(s.str(),p);

    m_Tinit1 = p[0];
    m_Tinit2 = p[1];
    ROS_WARN_STREAM("Execution Time: \n"<<m_Tinit1<<", "<<m_Tinit2);

    /////Gamma
    s.str("");
    s<<ns<<"/gamma";
    ros::param::get(s.str(),p);

    m_invGamma=1/p[0]*Eigen::Matrix<double,55,55>::Identity();

    ROS_WARN_STREAM("inversed Gamma: \n"<<1/p[0]);
    
    s.str("");
    s<<ns<<"/THR_DeltaQ";
    ros::param::get(s.str(),p);
    m_THR_DeltaQ = p[0];
    ROS_WARN_STREAM("THR_DeltaQ: \n"<<m_THR_DeltaQ);
    
    s.str("");
    s<<ns<<"/THR_DeltaX";
    ros::param::get(s.str(),p);
    m_THR_DeltaX = p[0];
    ROS_WARN_STREAM("THR_DeltaX: \n"<<m_THR_DeltaX);
    
    s.str("");
    s<<ns<<"/THR_SING";
    ros::param::get(s.str(),p);
    m_THR_SING = p[0];
    ROS_WARN_STREAM("THR_SING: \n"<<m_THR_SING);
    
    s.str("");
    s<<ns<<"/THR_MANI";
    ros::param::get(s.str(),p);
    m_THR_MANI = p[0];
    ROS_WARN_STREAM("THR_MANI: \n"<<m_THR_MANI);
    
    s.str("");
    s<<ns<<"/start_pose";
    ros::param::get(s.str(),p);
    for(int i=0;i<STD_DOF+1;i++)
    {
        m_GoalX(i)=p[i];
    }
    ROS_WARN_STREAM("start_pose: \n"<<m_GoalX);
    
    m_GoalLED = false;

    m_curr_stat = INIT_JOINT;
    m_next_stat = INIT_JOINT;

    tum_ics_skin_msgs::setSkinCellLedColor msgShutDown;
    tum_ics_skin_msgs::SkinCellLedColor color;
    color.r = 0;
    color.g = 0;
    color.b = 0;
    for(int i=1;i<55;i++){
        color.cellId = i;
        msgShutDown.request.color.push_back(color);
    }
    srvModLED.call(msgShutDown);

}
bool SimpleEffortControl::start()
{
    
}
Vector6d SimpleEffortControl::update(const RobotTime &time, const JointState &current)
{    
    dt = time.tD()-ti;
    //ROS_INFO_STREAM("dt: "<<dt);
    tau.setZero();    
    JointState js_r;
    Vector7d Xrp,Xrpp;

    VVector6d vQd;
    VVector7d vXd;
    VVector6d vXd_tmp; //xyzRxRyRz
    Vector3d vXd_w; //Rw
    Vector7d curr_cartVel, curr_cartPos;
    Matrix3d Jv_tmp;
    Eigen::Matrix<double,3,6> Jv;
    
    
    geometry_msgs::PoseStamped curr_pose;

    /*
    Vector6d deltaQ;
    deltaQ  << dt, dt, dt, dt, dt, dt;
    curr_cartVel = (FK(current.q+deltaQ)-FK(current.q))/dt;
    
    m_J = Jacobian(current.q);
    Vector6d curr_cartVel_jacob = m_J*deltaQ/dt;
    ROS_INFO_STREAM("vel = "<<curr_cartVel);
    ROS_INFO_STREAM("jacobi vel = "<<curr_cartVel_jacob);
    */
    
    // ROS_INFO_STREAM("current state: "<<m_curr_stat);
    switch(m_curr_stat){
        case CALIBRATION:
        {
            if(!m_startFlag){
                m_startFlag=true;
                m_startTime = time.tD();
                m_iDeltaQ.setZero();
            }
            vQd.push_back(Vector6d());
            vQd.push_back(Vector6d());
            vQd.push_back(Vector6d()); 
            (vQd[0])[0] = std::sin(0.35*(time.tD()-m_startTime));
            (vQd[0])[1] = std::sin(0.35*(time.tD()-m_startTime))-3.14159265358/2;
            (vQd[0])[2] = std::sin(0.35*(time.tD()-m_startTime));
            
            (vQd[1])[0] = 0.35*std::cos(0.35*(time.tD()-m_startTime));
            (vQd[1])[1] = 0.35*std::cos(0.35*(time.tD()-m_startTime));
            (vQd[1])[2] = 0.35*std::cos(0.35*(time.tD()-m_startTime));
            
            (vQd[2])[0] = -0.35*0.35*std::sin(0.35*(time.tD()-m_startTime));
            (vQd[2])[1] = -0.35*0.35*std::sin(0.35*(time.tD()-m_startTime));
            (vQd[2])[2] = -0.35*0.35*std::sin(0.35*(time.tD()-m_startTime));
            
            m_DeltaQ=current.q-vQd[0];
            m_DeltaQp=current.qp-vQd[1];
            m_iDeltaQ = m_iDeltaQ+m_DeltaQ*dt;
            
            // ROS_INFO_STREAM("joint error = "<<m_DeltaQ);
            // ROS_INFO_STREAM("||joint error|| = "<<m_DeltaQ.norm());

            js_r=current;
            js_r.qp=vQd[1]-m_Kp*m_DeltaQ-m_Ki*m_iDeltaQ;
            js_r.qpp=vQd[2]-m_Kp*m_DeltaQp-m_Ki*m_DeltaQ;

            Sq = current.qp-js_r.qp;

            break;
        }
        case INIT_JOINT:
        {
            if(!m_startFlag){
                m_qStart=current.q;
                m_startFlag=true;
                m_startTime = time.tD();
                m_endTime = m_startTime+m_Tinit1;
                m_iDeltaQ.setZero();
            }
            
            //ROS_INFO_STREAM("qStart = "<<m_qStart);
            //ROS_INFO_STREAM("qGoal = "<<m_goal);
	        //ROS_INFO_STREAM("rel time = "<<time.tD()-m_startTime);
	        //ROS_INFO_STREAM("total time = "<<m_Tinit1);
            
            vQd = getJointPVT5(m_qStart,m_goal,time.tD()-m_startTime,m_Tinit1);
            
            //vQd[0] = m_goal;
            //vQd[1].setZero();
            //vQd[2].setZero();
            
            //ROS_INFO_STREAM("curr joint pos = "<<current.q);
            //ROS_INFO_STREAM("curr joint goal = "<<vQd[0]);

            m_DeltaQ=current.q-vQd[0];
            m_DeltaQp=current.qp-vQd[1];
            m_iDeltaQ = m_iDeltaQ+m_DeltaQ*dt;
            
            //ROS_INFO_STREAM("joint error = "<<current.q - vQd[0]);
            //ROS_INFO_STREAM("||joint error|| = "<<current.q - vQd[0]);

            js_r=current;
            js_r.qp=vQd[1]-m_Kp*m_DeltaQ-m_Ki*m_iDeltaQ;
            js_r.qpp=vQd[2]-m_Kp*m_DeltaQp-m_Ki*m_DeltaQ;

            Sq = current.qp-js_r.qp;
            // ROS_INFO_STREAM("Sq = \n"<<Sq);

            break;
        }
        case INIT_CARTE:
        {
            
        	curr_cartPos = FK(current.q);
            //ROS_INFO_STREAM("before jacobian");
        	m_J = Jacobian(current.q);
            //ROS_INFO_STREAM("after jacobian");
            Jv = m_J.block(0,0,3,m_J.cols());
            
        	JacobiSVD<MatrixXd> svd(m_J, ComputeFullU | ComputeFullV);

        	VectorXd singular_v;
			singular_v = svd.singularValues();
			VectorXd singular_v_inv = singular_v;
			//ROS_INFO_STREAM("Sigma: \n"<<singular_v);
 
			for(int i=0;i<singular_v.size();i++){
			    //ROS_INFO_STREAM("singular_v"<<i<<" = "<<singular_v(i));
			    //ROS_INFO_STREAM("m_THR_SING = "<<m_THR_SING);
			    singular_v_inv(i)=singular_v(i)< m_THR_SING ? 1/m_THR_SING:1/singular_v(i);
			    //if(singular_v(i)< m_THR_SING) ROS_INFO_STREAM("sing protection!!!!");
			    //singular_v_inv(i) = singular_v(i)/(singular_v(i)*singular_v(i)+0.04);
			    
			}
			
            J_.block(0,0,singular_v.size(),singular_v.size()) = singular_v_inv.asDiagonal();
            m_invJ_ = m_invJ;
            m_invJ = svd.matrixV() * J_ * svd.matrixU().transpose();
            
            if(!m_startFlag){
                m_invJ_d.setZero();
            }else{
                m_invJ_d = (m_invJ - m_invJ_)/dt;
            }
        	

            Jv_tmp = Jv*Jv.transpose();
            m_sing = std::sqrt(Jv_tmp.determinant());
            //ROS_INFO_STREAM("w = "<<m_sing);

        	//curr_cartVel = m_J*current.qp;
            // ROS_INFO_STREAM("curr_cartVel = \n"<<curr_cartVel);

        	if(!m_startFlag){
	            m_xStart = curr_cartPos;
	            //m_GoalX = curr_cartPos;
	            m_startFlag = true;
	            m_startTime = time.tD();
	            m_endTime = m_startTime+m_Tinit2;
	            m_iDeltaX.setZero();
	        }
            
            // ROS_INFO_STREAM("xStart = "<<m_xStart);
            // ROS_INFO_STREAM("xGoal = \n"<<m_GoalX);
	        //ROS_INFO_STREAM("rel time = "<<time.tD()-m_startTime);
	        //ROS_INFO_STREAM("total time = "<<m_Tinit2);
	        
	        vXd_tmp = getJointPVT5(m_xStart.head(6),m_GoalX.head(6),time.tD()-m_startTime,m_Tinit2);
            vXd_w = getJointPVT5(m_xStart[6],m_GoalX[6],time.tD()-m_startTime,m_Tinit2);
            
            vXd.push_back(Vector7d()); vXd.push_back(Vector7d()); vXd.push_back(Vector7d());
            vXd[0].head(6) = vXd_tmp[0]; (vXd[0])(6) = vXd_w[0];
            vXd[1].head(6) = vXd_tmp[1]; (vXd[1])(6) = vXd_w[1];
            vXd[2].head(6) = vXd_tmp[2]; (vXd[2])(6) = vXd_w[2];
	        
	        //vXd[0] = m_GoalX;
	        //vXd[1].setZero();
	        //vXd[2].setZero();
	        
            //ROS_INFO_STREAM("curr cartesian error = "<<curr_cartPos-m_GoalX);
            //ROS_INFO_STREAM("Planned goal = \n"<<vXd[0]);
            //ROS_INFO_STREAM("curr_cartPos = "<<curr_cartPos);
            //ROS_INFO_STREAM("curr cartesian goal vel = "<<vXd[1]);
	        m_DeltaX  = curr_cartPos-vXd[0];
            m_DeltaXp = curr_cartVel-vXd[1];
            //m_DeltaX  = curr_cartPos-m_GoalX;

            //ROS_INFO_STREAM("DeltaX = "<<m_DeltaX);
            //ROS_INFO_STREAM("DeltaXp = "<<m_DeltaXp);

            m_iDeltaX = m_iDeltaX+m_DeltaX*dt;

            Xrp  = vXd[1]-m_Kp_cart*m_DeltaX -m_Ki_cart*m_iDeltaX;
            Xrpp = vXd[2]-m_Kp_cart*m_DeltaXp-m_Ki_cart*m_DeltaX;
            
            //ROS_INFO_STREAM("vXd[0] = "<<vXd[0]);
            //ROS_INFO_STREAM("vXd[1] = "<<vXd[1]);
            //ROS_INFO_STREAM("vXd[2] = "<<vXd[2]);
            //xs_r.qp.setZero();
            //xs_r.qpp.setZero();

            js_r = current;
            js_r.qp = m_invJ*Xrp;
            js_r.qpp = m_invJ_d*Xrp + m_invJ*Xrpp;
            //js_r.qpp = m_invJ*xs_r.qpp;
            
            //js_r.qp.setZero();
            //js_r.qpp.setZero();

            Sq = current.qp-js_r.qp;
            // ROS_INFO_STREAM("Sq = \n"<<Sq);
            //Sq = m_invJ * (curr_cartVel - xs_r.qp);
            //ROS_INFO_STREAM("current.qp = "<<current.qp);
            //while(ros::ok()){}
            break;
        }
        case TRACKING:
        {
        	curr_cartPos = FK(current.q);

            curr_pose.header.stamp = ros::Time::now();
            curr_pose.header.frame_id = "world";

            curr_pose.pose.position.x = curr_cartPos[0];
            curr_pose.pose.position.y = curr_cartPos[1];
            curr_pose.pose.position.z = curr_cartPos[2];
            curr_pose.pose.orientation.x = curr_cartPos[3];
            curr_pose.pose.orientation.y = curr_cartPos[4];
            curr_pose.pose.orientation.z = curr_cartPos[5];
            curr_pose.pose.orientation.w = curr_cartPos[6];

            if(m_GoalLED){
                path.poses.push_back(curr_pose);
            }
            pubPath.publish(path);

            m_J = Jacobian(current.q);
            Jv = m_J.block(0,0,3,m_J.cols());
            
            JacobiSVD<MatrixXd> svd(m_J, ComputeFullU | ComputeFullV);

            VectorXd singular_v;
            singular_v = svd.singularValues();
            VectorXd singular_v_inv = singular_v;
            // ROS_INFO_STREAM("Sigma: \n"<<singular_v);
 
            for(int i=0;i<singular_v.size();i++){
                //ROS_INFO_STREAM("singular_v"<<i<<" = "<<singular_v(i));
                //ROS_INFO_STREAM("m_THR_SING = "<<m_THR_SING);
                singular_v_inv(i)=singular_v(i)< m_THR_SING ? 1/m_THR_SING:1/singular_v(i);
                //if(singular_v(i)< m_THR_SING) ROS_INFO_STREAM("sing protection!!!!");
                //singular_v_inv(i) = singular_v(i)/(singular_v(i)*singular_v(i)+0.04);
                
            }
            
            J_.block(0,0,singular_v.size(),singular_v.size()) = singular_v_inv.asDiagonal();
            m_invJ_ = m_invJ;
            m_invJ = svd.matrixV() * J_ * svd.matrixU().transpose();
            
            if(!m_startFlag){
                m_invJ_d.setZero();
            }else{
                m_invJ_d = (m_invJ - m_invJ_)/dt;
            }
        	

            Jv_tmp = Jv*Jv.transpose();
            m_sing = std::sqrt(Jv_tmp.determinant());
            //ROS_INFO_STREAM("w = "<<m_sing);
            
            curr_cartVel = m_J*current.qp;

            m_DeltaX  = curr_cartPos-m_GoalX;
            m_DeltaXp = curr_cartVel-m_GoalXp;
            m_iDeltaX = m_iDeltaX+m_DeltaX*dt;
            
            //ROS_INFO_STREAM("m_GoalX = "<<m_GoalX);
            //ROS_INFO_STREAM("curr_cartPos = "<<curr_cartPos);
            //ROS_INFO_STREAM("DeltaX = "<<m_DeltaX);
            //ROS_INFO_STREAM("DeltaXp = "<<m_DeltaXp);
            
            Xrp  = m_GoalXp-m_Kp_cart*m_DeltaX -m_Ki_cart*m_iDeltaX;
            Xrpp = m_GoalXpp-m_Kp_cart*m_DeltaXp-m_Ki_cart*m_DeltaX;

            js_r = current;
            js_r.qp = m_invJ*Xrp;
            js_r.qpp = m_invJ_d*Xrp + m_invJ*Xrpp;
            js_r.qpp.setZero();
            
            Sq = current.qp-js_r.qp;

            //ROS_INFO_STREAM("Sq = \n"<<Sq);
            //while(ros::ok()){}
            break;
        }
        case PROTECTION:
        {
            if(!m_startFlag){
                m_qStart=current.q;
                m_startFlag=true;
                m_startTime = time.tD();
                m_endTime = time.tD()+m_Tinit1;
                m_iDeltaQ.setZero();
            }

            vQd = getJointPVT5(m_qStart,m_goal,time.tD()-m_startTime,m_Tinit1);

            m_DeltaQ=current.q-vQd[0];
            m_DeltaQp=current.qp-vQd[1];
            m_iDeltaQ = m_iDeltaQ+m_DeltaQ*dt;

            js_r=current;
            js_r.qp=vQd[1]-m_Kp*m_DeltaQ-m_Ki*m_iDeltaQ;
            js_r.qpp=vQd[2]-m_Kp*m_DeltaQp-m_Ki*m_DeltaQ;

            Sq = current.qp-js_r.qp;
            break;
        }
    }

    switch(m_curr_stat){
        case CALIBRATION:
    	{
    		if(m_DeltaQ.norm()<0.1){
    			m_next_stat = INIT_JOINT;
    			m_startFlag = false;
    			ROS_INFO_STREAM("next state: INIT_JOINT");
    		}else{
    			m_next_stat = CALIBRATION;
    		}
    		break;
        }
    	case INIT_JOINT:
    	{
    		if(time.tD()>m_endTime && m_DeltaQ.sum()<m_THR_DeltaQ){
    			m_next_stat = INIT_CARTE;
    			m_startFlag = false;
    			ROS_INFO_STREAM("next state: INIT_CARTE");
    		}else{
    			m_next_stat = INIT_JOINT;
    		}
    		break;
        }
    	case INIT_CARTE:
    	{
    		if(m_sing <= m_THR_MANI){
                m_next_stat = PROTECTION;
                m_startFlag = false;
                ROS_INFO_STREAM("next state: Protection");
            }else{
                if(time.tD()>m_endTime && m_DeltaX.sum()<m_THR_DeltaX){
                    m_next_stat = TRACKING;
                    m_startFlag = false;
                    m_iDeltaX.setZero();
                    std_srvs::Empty tmp;
                    srvStart.call(tmp);
                    ROS_INFO_STREAM("next state: TRACKING");
                }else{
					m_next_stat = INIT_CARTE;
				}
    		}
    		break;
        }
    	case TRACKING:
    	{
            if(m_sing <= m_THR_MANI){
                m_next_stat = PROTECTION;
                m_startFlag = false;
                ROS_INFO_STREAM("next state: PROTECTION");
            }else{
                m_next_stat = TRACKING;
            }
            break;
        }
        case PROTECTION:
            m_next_stat = PROTECTION;

        default:
            m_next_stat = PROTECTION;
    }

    Yr = evalYr(current.q,current.qp,js_r.qp,js_r.qpp);
    
    tau = -m_Kd*Sq + Yr*Theta;
    //ROS_INFO_STREAM("Yr*Theta = "<<deltaTau);
    //tau = -m_Kd*Sq;
    //ROS_INFO_STREAM("tau = "<<tau);
    //ROS_INFO_STREAM("Sq = "<<Sq);
    //ROS_INFO_STREAM("Kd = "<<m_Kd);
    //ROS_INFO_STREAM("wtf = "<<((Yr.transpose())*Sq).norm());
    /*
    if(((Yr.transpose())*Sq).norm()>0.03){
        Theta = Theta - m_invGamma*(Yr.transpose())*Sq*dt;
    }
    */
    Theta = Theta - m_invGamma*(Yr.transpose())*Sq*dt;
    m_curr_stat = m_next_stat;
/*

    VVector6d vQd;
    vQd=getJointPVT5(m_qStart,m_goal,time.tD(),m_totalTime);



    m_DeltaQ=current.q-vQd[0];
    m_DeltaQp=current.qp-vQd[1];



    JointState js_r;

    js_r=current;
    js_r.qp=vQd[1]-m_Kp*m_DeltaQ;
    js_r.qpp=vQd[2]-m_Kp*m_DeltaQp;

    Vector6d Sq=current.qp-js_r.qp;

    Eigen::Matrix<double,35,1> Theta;
    Eigen::Matrix<double,6,35> Yr;

    `
    //def Yr and Theta
    

    Theta.setZero();
    Yr.setZero();

    tau=-m_Kd*Sq+Yr*Theta;
*/
    tum_ics_ur_robot_msgs::ControlData msg;

    msg.header.stamp=ros::Time::now();

    msg.time=time.tD();

    for(int i=0;i<STD_DOF;i++)
    {
        msg.q[i]=current.q(i);
        msg.qp[i]=current.qp(i);
        msg.qpp[i]=current.qpp(i);

        msg.qd[i]=vQd[0](i);
        msg.qpd[i]=vQd[1](i);

        msg.Dq[i]=m_DeltaQ(i);
        msg.Dqp[i]=m_DeltaQp(i);

        msg.torques[i]=current.tau(i);
    }

    pubCtrlData.publish(msg);



    //    tau.setZero();

    ti = time.tD();
    //ROS_INFO_STREAM("ti: "<<ti);
    //ROS_INFO_STREAM("tf: "<<tf);
    //tau.setZero();
    // ROS_INFO_STREAM("tau: "<<tau);
    return tau;

}

bool SimpleEffortControl::stop()
{

}




}
}
