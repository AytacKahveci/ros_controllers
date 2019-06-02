#ifndef DYNAMIXELHARDWAREINTERFACECURRENT_H
#define DYNAMIXELHARDWAREINTERFACECURRENT_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <vector>
#include <string>

namespace dynamixel_current_controller
{
    class DynamixelCurrentController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:

        DynamixelCurrentController();
        ~DynamixelCurrentController();

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

        void printDebug();

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

        std::vector<std::string> joint_names_;
        int n_joints_;
        std::vector<hardware_interface::JointHandle> joints_;
        realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
        std::vector<double> commands_;

        double theta1, theta2, x3;
        double error;
        double commanded_effort;
        double current_;
        //1/149.795386991
        const double torque_to_current_value_ratio   = 0.642857143;
    private:
        int loop_count_;
        control_toolbox::Pid pid_controller_;

        ros::Subscriber sub_command_;

        void setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
    };
}
#endif
