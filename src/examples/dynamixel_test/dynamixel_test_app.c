
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <uORB/uORB.h>
#include <uORB/topics/dynamixel_controls.h>

__EXPORT int dynamixel_test_app_main(int argc, char *argv[]);

static orb_advert_t dynamixel_command_pub;
bool advertised(void) { return dynamixel_command_pub != NULL; }

int dynamixel_test_app_main(int argc, char *argv[])
{
	/* advertise attitude topic */
	struct dynamixel_controls_s dynamixel_command;
	//memset(&dynamixel_command, 0, sizeof(dynamixel_command));
	if (!advertised()){
		dynamixel_command_pub = orb_advertise(ORB_ID(dynamixel_controls), &dynamixel_command);
	}

	if (argc ==3  && atoi(argv[1]) <= 8 && atoi(argv[1]) > 0 && 0 <= atoi(argv[2]) && 360 >= atoi(argv[2])){
		// first arguent, servo num, 2nd argument angle
		dynamixel_command.dynamixel_controls[atoi(argv[1])-1] = (uint16_t)atoi(argv[2])*(4095/360);
		dynamixel_command.timestamp = time(NULL);
		PX4_INFO("servo_num: %i          angle: %i",atoi(argv[1]), atoi(argv[2]));
		orb_publish(ORB_ID(dynamixel_controls), dynamixel_command_pub, &dynamixel_command);
	}
	else{
		PX4_ERR("input error");
	}

	PX4_INFO("exiting");

	return 0;
}
