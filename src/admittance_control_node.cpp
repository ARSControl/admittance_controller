#include "admittance_control/admittance_control.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "admittance_control");

	AdmittanceControl ce;
	ros::Rate r(500);

	while (ros::ok())
	{
    	ros::Time start = ros::Time::now();
		ce.spinner();
    	ROS_INFO("Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());
		r.sleep();
	}

	return 0;
}
