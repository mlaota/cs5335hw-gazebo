
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

/*
To view the camera image in time, you could press CTRL-T in Gazebo
, choosing the Topic-"~/tankbot0/tankbot/camera_sensor/link/camera/image", 
then a Image Window will pop up, in order to view the Image in time.
*/

void
callback(Robot* robot)
{
    if (robot->range < 1.0) {
        robot->set_vel(-5.0, 5.0);
        return;
    }

    if (robot->range < 1.7) {
        robot->set_vel(5.0, 5.0);
        return;
    }

    robot->set_vel(5.0, -5.0);
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
