/* @author Aytac Kahveci */

#include <coil_pid_controller/coil_pid_controller.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>
namespace coil_pid_controller
{
    CoilPidController::CoilPidController() : loop_count_(0)
    {}

    CoilPidController::~CoilPidController()
    {
        sub_command_.shutdown();
    }

    bool CoilPidController::init(hardware_interface::CurrentCommandInterface *robot, ros::NodeHandle &n)
    {
        if(!n.getParam("coils", coil_names_))
        {
            ROS_ERROR("Error in coils parameter in your config.yaml file (namespace %s)", n.getNamespace().c_str());
            return false;
        }
        n_coils_ = coil_names_.size();

        if(n_coils_ == 0)
        {
            ROS_ERROR_STREAM("List of coil names is empty");
            return false;
        }

        sub_command_ = n.subscribe("command",1,&CoilPidController::setCommandCB, this);

        for(size_t i=0; i<n_coils_; i++)
        {
            coils_.push_back(robot->getHandle(coil_names_[i]));
        }

        pid_controller_.resize(n_coils_);
        controller_state_publisher_.resize(n_coils_);
        last_pwm[0] = 0.0;
        last_pwm[1] = 0.0;
        error_prev_[0] = 0.0;
        error_prev_[1] = 0.0;
        d_error_[0]= 0.0;
        d_error_[1]= 0.0;

        u_[0] = 0.0;
        u_[1] = 0.0;

        for(int i=0; i<n_coils_; i++)
        {
            ros::NodeHandle coils_nh(n, std::string("gains/") + coils_[i].getName());
            pid_controller_[i].reset(new control_toolbox::Pid());

            if(!pid_controller_[i]->init(coils_nh))
            {
                ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
                return false;
            }

            controller_state_publisher_[i].reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, coils_[i].getName() + std::string("/state"), 1));

            pid_controller_[i]->getGains(p_gain[i], i_gain[i], d_gain, i_max, i_min, antiwindup);
            ROS_INFO("p_gain[%d]: %f",i,p_gain[i]);
            ROS_INFO("i_gain[%d]: %f",i,i_gain[i]);
        }

        return true;
    }

    void CoilPidController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        for(size_t j=0; j<n_coils_; j++)
        {
            pid_controller_[j]->setGains(p, i, d, i_max, i_min, antiwindup);
        }
    }

    void CoilPidController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        for(size_t j=0; j<n_coils_; j++)
        {
            pid_controller_[j]->getGains(p,i,d,i_max,i_min,antiwindup);
        }
    }

    void CoilPidController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        for(size_t j=0; j<n_coils_; j++)
        {
            pid_controller_[j]->getGains(p,i,d,i_max,i_min);
        }
    }

    void CoilPidController::printDebug()
    {
        for(size_t j=0; j<n_coils_; j++)
        {
            pid_controller_[j]->printValues();
        }
    }

    void CoilPidController::starting(const ros::Time& time)
    {
        for(size_t i=0; i<n_coils_; i++)
        {
            commands_.push_back(0);
        }

        commands_buffer_.initRT(commands_);
        for(size_t j=0; j<=n_coils_; j++)
        {
            ROS_INFO("Inside the starting function");
            //pid_controller_[j]->reset();
            ROS_INFO("At the end of the starting function");
        }
    }

    void CoilPidController::update(const ros::Time& time, const ros::Duration& period)
    {
        commands_ = *(commands_buffer_.readFromRT());
        for(int i=0; i<n_coils_; i++)
        {
            commands_[i] = commands_[i] / 1000;
            if(abs(commands_[i]) < 0.88)
            {
                u_[i] = (617*pow(commands_[i],4) - 1308*pow(commands_[i],3) + 1696*pow(commands_[i],2) + 354.8*commands_[i] + 0.002664)/(commands_[i]+0.01811);
            }
            else
            {
                u_[i] = 21.17*pow(commands_[i],3) - 187.1*pow(commands_[i],2) + 966.3*commands_[i] + 506.2;
            }

            commands_[i] = commands_[i] * 1000;
            ROS_INFO("commands_[%d] is %f", i, commands_[i]);
            current_val_ = coils_[i].getCurrent();

            error_[i] = commands_[i] - current_val_;

            if(std::abs(error_[i]) > 20 && std::abs(error_[i]) < 100)
            {
                //pid_controller_[i]->getGains(p_gain, i_gain, d_gain, i_max, i_min, antiwindup);
                p_gain[i] = 0.15;
                ROS_INFO("P_GAIN Increasing: %f", p_gain[i]);
                pid_controller_[i]->setGains(p_gain[i], i_gain[i], d_gain, i_max, i_min, antiwindup);
            }
            else if(std::abs(error_[i]) < 20)
            {

            }
            else
            {
                //pid_controller_[i]->getGains(p_gain, i_gain, d_gain, i_max, i_min, antiwindup);
                p_gain[i] = 0.1;
                //ROS_INFO("P_GAIN Decreasing: %f", p_gain);
                pid_controller_[i]->setGains(p_gain[i], i_gain[i], d_gain, i_max, i_min, antiwindup);
            }
            /*else if(std::abs(error_[i]) > 20 && std::abs(error_[i]) <= 60)
            {
                pid_controller_[i]->getGains(p_gain, i_gain, d_gain, i_max, i_min, antiwindup);
                p_gain -= 0.01;
                ROS_INFO("P_GAIN Decreasing: %f", p_gain);
                pid_controller_[i]->setGains(p_gain, i_gain, d_gain, i_max, i_min, antiwindup);
            }*/

            d_error_[i] =  error_[i] - error_prev_[i];

            error_prev_[i] = error_[i];

            delta_u_[i] = pid_controller_[i]->computeCommand(d_error_[i], error_[i], 0.0, period);

            commanded_effort_[i] = delta_u_[i] + u_[i];
            u_[i] = commanded_effort_[i];

            double x = commanded_effort_[i]/1000;

            if(commanded_effort_[i] < 0.0)
            {
                commanded_effort_[i] = 0.0;
            }
            else if(commanded_effort_[i] > 3500.0)
            {
                commanded_effort_[i] = 3400.0;
            }


            ROS_INFO("error is %f commanded_effort %d is %f and u_[i] is %f d_error_[i] %f", error_[i], i, commanded_effort_[i],  u_[i], d_error_[i]);
            coils_[i].setCommand(commanded_effort_[i], direction_[i]);

        }

        // publish state
        if (loop_count_ % 10 == 0)
         {
           if(controller_state_publisher_[0] && controller_state_publisher_[0]->trylock())
           {
             controller_state_publisher_[0]->msg_.header.stamp = time;
             controller_state_publisher_[0]->msg_.set_point = commands_[0];
             controller_state_publisher_[0]->msg_.process_value = coils_[0].getCurrent();
             controller_state_publisher_[0]->msg_.process_value_dot = 0;
             controller_state_publisher_[0]->msg_.error = error_[0];
             controller_state_publisher_[0]->msg_.time_step = period.toSec();
             controller_state_publisher_[0]->msg_.command = commanded_effort_[0];

             double dummy;
             bool antiwindup;
             getGains(controller_state_publisher_[0]->msg_.p,
               controller_state_publisher_[0]->msg_.i,
               controller_state_publisher_[0]->msg_.d,
               controller_state_publisher_[0]->msg_.i_clamp,
               dummy,
               antiwindup);
             controller_state_publisher_[0]->msg_.antiwindup = static_cast<char>(antiwindup);
             controller_state_publisher_[0]->unlockAndPublish();
           }
           ////////////////////////
           if(controller_state_publisher_[1] && controller_state_publisher_[1]->trylock())
           {
             controller_state_publisher_[1]->msg_.header.stamp = time;
             controller_state_publisher_[1]->msg_.set_point = commands_[1];
             controller_state_publisher_[1]->msg_.process_value = coils_[1].getCurrent();
             controller_state_publisher_[1]->msg_.process_value_dot = 0;
             controller_state_publisher_[1]->msg_.error = error_[1];
             controller_state_publisher_[1]->msg_.time_step = period.toSec();
             controller_state_publisher_[1]->msg_.command = commanded_effort_[1];

             double dummy;
             bool antiwindup;
             getGains(controller_state_publisher_[1]->msg_.p,
               controller_state_publisher_[1]->msg_.i,
               controller_state_publisher_[1]->msg_.d,
               controller_state_publisher_[1]->msg_.i_clamp,
               dummy,
               antiwindup);
             controller_state_publisher_[1]->msg_.antiwindup = static_cast<char>(antiwindup);
             controller_state_publisher_[1]->unlockAndPublish();
           }
         }
         loop_count_++;
    }

    void CoilPidController::setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
        if(msg->data.size() != n_coils_)
        {
            ROS_ERROR_STREAM("Dimensions of command (" << msg->data.size() << ") does not match number of coils (" << n_coils_ << ")! Not executing!");
            return;
        }

        commands_buffer_.writeFromNonRT(msg->data);
    }
}

PLUGINLIB_EXPORT_CLASS(coil_pid_controller::CoilPidController, controller_interface::ControllerBase)
