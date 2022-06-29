#include <ekf_wrapper/ekf_wrapper.h>
#include <signal.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig){
    g_request_shutdown = 1;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ekf_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);

    EKF_NODE ekf_filter;
    ros::WallRate loop_rate(100);
    
    while(!g_request_shutdown)
    {
        ekf_filter.execute();
        ekf_filter.report();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ekf_filter.onFinish();
}
