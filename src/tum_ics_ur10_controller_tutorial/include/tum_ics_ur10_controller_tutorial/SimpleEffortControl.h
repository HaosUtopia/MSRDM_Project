#ifndef UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
#define UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <trajectory_generator/TrajectoryPosition.h>
#include <tum_ics_skin_msgs/setSkinCellLedColor.h>
#include <std_srvs/Empty.h>

namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

class SimpleEffortControl: public ControlEffort
{
    // Member variables
private:

    typedef QVector<Vector7d> VVector7d;

    bool m_startFlag;
    //bool m_isCart; //0 --> Joint Space; 1--> Cartesian Space
    enum m_STATES {CALIBRATION, INIT_JOINT,INIT_CARTE,TRACKING,PROTECTION} m_curr_stat,m_next_stat;

    double L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,L12,L13,L14;
    double m1, m2, m3, m4, m5, m6;
    double b1, b2, b3, b4, b5, b6;
    double g, gx, gy, gz;
    double I111,I122,I133,I211,I222,I233,I311,I322,I333,I411,I422,I433,I511,I522,I533,I611,I622,I633;

    double ti,dt;
    
    Eigen::Matrix<double,55,1> Theta;
    Eigen::Matrix<double,6,55> Yr;

    Vector6d m_qStart;
    Vector7d m_xStart;
    JointState m_qInit;
    JointState m_qHome;
    JointState m_qPark;

    ros::NodeHandle n;
    ros::Publisher pubCtrlData;
    ros::Subscriber subGoal;
    ros::ServiceClient srvStart;
    ros::ServiceClient srvModLED;
    

    Matrix6d m_Kp;
    Matrix6d m_Kd;
    Matrix6d m_Ki;
    Matrix7d m_Kp_cart;
    Matrix7d m_Kd_cart;
    Matrix7d m_Ki_cart;
    Vector6d m_goal;
    Eigen::Matrix<double,55,55> m_invGamma;

    Vector6d m_DeltaQ;
    Vector6d m_DeltaQ_1;

    Vector6d m_DeltaQp;
    Vector6d m_DeltaQp_1;

    Vector6d m_iDeltaQ;
    Vector6d m_iDeltaQ_1;

    Vector7d m_DeltaX;
    Vector7d m_DeltaXp;
    Vector7d m_iDeltaX;

    double m_THR_DeltaQ;
    double m_THR_DeltaX;
    double m_THR_SING;
    double m_THR_MANI;

    Eigen::Matrix<double,7,6> m_J;
    Eigen::Matrix<double,6,7> m_invJ;
    Eigen::Matrix<double,6,7> J_;
    Eigen::Matrix<double,6,7> m_invJ_;
    Eigen::Matrix<double,6,7> m_invJ_d;

    //double m_controlPeriod;     //[s]
    //double m_controlPeriod_2;   //[s]

    Vector7d m_GoalX;
    Vector7d m_GoalXp;
    Vector7d m_GoalXpp;
    //double m_GoalY;
    //const double m_GoalZ;
    //const double m_GoalR;
    //const double m_GoalP;
    //const double m_GoalY;
    bool m_GoalLED;

    double m_Tinit1;
    double m_Tinit2;

    double m_endTime;
    double m_startTime;
    double m_sing;
    
    Vector6d Sq;
    
    Vector6d tau;

    // Member Methods
    void setGoal(const trajectory_generator::TrajectoryPosition::ConstPtr& msg);
public:
    SimpleEffortControl(double weight=1.0,
                        const QString& name="SimpleEffortCtrl");
    ~SimpleEffortControl();

    void setQInit(const JointState& qinit);
    void setQHome(const JointState& qhome);
    void setQPark(const JointState& qpark);

    Vector7d FK(Vector6d Q);
    Eigen::Matrix<double,7,6> Jacobian(Vector6d Q);
    Eigen::Matrix<double,6,55> evalYr(Vector6d Q, Vector6d Qp, Vector6d Qrp, Vector6d Qrpp);

    // Memeber Methods
private:
    bool init();
    bool start();
    Vector6d update(const RobotTime& time, const JointState &current);
    bool stop();


};

}
}



#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
