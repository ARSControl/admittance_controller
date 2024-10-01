#include "admittance_control/admittance_control.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "admittance_control");

	AdmittanceControl ce;
	ros::Rate r(500);

	while (ros::ok())
	{
		ce.spinner();
		r.sleep();
	}

	return 0;
}
