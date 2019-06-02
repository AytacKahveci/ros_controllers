/* @author Aytac Kahveci */
#ifndef COIL_PID_CONTROLLER_H
#define COIL_PID_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/current_command_interface.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <vector>
#include <string>

namespace coil_pid_controller
{
    class CoilPidController: public controller_interface::Controller<hardware_interface::CurrentCommandInterface>
    {
    public:
        CoilPidController();
        ~CoilPidController();

        bool init(hardware_interface::CurrentCommandInterface *robot, ros::NodeHandle& n);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

        void printDebug();

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

        std::vector<std::string> coil_names_;
        int n_coils_;
        std::vector<hardware_interface::CurrentCommandHandle> coils_;
        realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
        std::vector<double> commands_;

        double last_pwm[2];

        double current_val_;
        double error_[2];
        double error_prev_[2];
        double d_error_[2];
        double u_[2];
        double commanded_effort_[2];
        double delta_u_[2];
        double direction_[2];


    private:
        int loop_count_;
        typedef boost::shared_ptr<control_toolbox::Pid> pidPtr;
        std::vector<pidPtr> pid_controller_;
        //std::vector<control_toolbox::Pid*> pid_controller_;
        ros::Subscriber sub_command_;
        ros::NodeHandle pid_nh_;
        ros::NodeHandle pid_nh2_;
        typedef boost::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controllerStatePtr;
        std::vector<controllerStatePtr> controller_state_publisher_;

        void setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg);

        double p_gain[2];
        double i_gain[2];
        double d_gain;
        double i_max;
        double i_min;
        bool antiwindup;
    };
}

#endif //End of namespace
