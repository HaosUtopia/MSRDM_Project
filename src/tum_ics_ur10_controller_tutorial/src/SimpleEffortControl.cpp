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
    m_totalTime(100.0),
    m_DeltaQ(Vector6d::Zero()),
    m_DeltaQ_1(Vector6d::Zero()),
    m_DeltaQp(Vector6d::Zero()),
    m_DeltaQp_1(Vector6d::Zero()),
    m_iDeltaQ(Vector6d::Zero()),
    m_iDeltaQ_1(Vector6d::Zero())
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("SimpleEffortCtrlData",100);

    m_controlPeriod=0.002; //set the control period to the standard 2 ms

    m_controlPeriod_2=m_controlPeriod/2.0;

    ROS_INFO_STREAM("SimpleEffortCtrl Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

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
    m_totalTime=p[STD_DOF];

    if(!(m_totalTime>0))
    {
        m_totalTime=100.0;
    }

    ROS_WARN_STREAM("Goal [DEG]: \n"<<m_goal.transpose());
    ROS_WARN_STREAM("Total Time [s]: "<<m_totalTime);


    m_goal=DEG2RAD(m_goal);

    ROS_WARN_STREAM("Goal [RAD]: \n"<<m_goal.transpose());


}
bool SimpleEffortControl::start()
{

}
Vector6d SimpleEffortControl::update(const RobotTime &time, const JointState &current)
{
    if(!m_startFlag)
    {
        m_qStart=current.q;
        m_startFlag=true;
    }


    Vector6d tau;

    tau.setZero();

    VVector6d vQd;


    vQd=getJointPVT5(m_qStart,m_goal,time.tD(),m_totalTime);



    m_DeltaQ=current.q-vQd[0];
    m_DeltaQp=current.qp-vQd[1];



    JointState js_r;

    js_r=current;
    js_r.qp=vQd[1]-m_Kp*m_DeltaQ;
    js_r.qpp=vQd[2]-m_Kp*m_DeltaQp;

    Vector6d Sq=current.qp-js_r.qp;


    tau=-m_Kd*Sq;

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

    return tau;

}
bool SimpleEffortControl::stop()
{

}




}
}
