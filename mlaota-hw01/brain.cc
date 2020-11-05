
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

namespace Utils {
    
// Determines whether the given distance is acceptable to stop with.
bool reached_goal(double dx, double dy) {
    static const double ALLOWANCE = 0.75;
    return (abs(dx) < ALLOWANCE) && (abs(dy) < ALLOWANCE);
}

double degrees_to_rads(double degrees) {
    return degrees * M_PI / 180.0;
}

} // namespace Utils

enum class Directions {
    LEFT,
    RIGHT
};

namespace MotionMacros {

// Tells the robot to stop searching because we "won".
void stop_searching(Robot* robot, double dx, double dy) {
    cout << "we win!" << endl;
    robot->set_vel(0.0);
    robot->set_turn(0.0);
    robot->done();
}


void sharp_turn(Robot* robot, Directions dir) {
    auto radians = Utils::degrees_to_rads(30);
    robot->set_turn((dir == Directions::RIGHT) ? radians : (-1.0 * radians));
    robot->set_vel(5.0);
}

void soft_turn(Robot* robot, Directions dir) {
    auto radians = Utils::degrees_to_rads(15);
    robot->set_turn((dir == Directions::RIGHT) ? radians : (-1.0 * radians));
}

} // namespace MotionMacros

void
callback(Robot* robot)
{
    cout << endl;
    cout << "robot x =" << robot->pos_x << endl;
    cout << "robot y =" << robot->pos_y << endl;
    cout << "robot t =" << robot->pos_t << endl;

    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;
    
    // Stop searching if we reached the goal.
    if (Utils::reached_goal(dx, dy)) {
        MotionMacros::stop_searching(robot, dx, dy);
        return;
    }

    bool turn = false;

    // Detect obstacles from LIDAR.
    for (LaserHit hit : robot->hits) {
        if (hit.range < 1.5) {
            if (hit.angle < 0.5 || hit.angle > (6.2 - 0.5)) {
                turn = true;
            }
        }
    }

    // If an obstacle is detected, reset the turn count.
    static auto turn_count = 0;
    if (turn) {
        turn_count = 20;
    }

    if (turn_count > 0) {
        // Keep turning if not enough time has passed since the last obstacle detection.
        MotionMacros::sharp_turn(robot, Directions::RIGHT);
        --turn_count;
    } else { 
        // Reorient the robot to drive towards the direction of the goal.
        static const auto ALLOWANCE_THETA = 0.1; 
        auto dtheta = std::atan2(dy, dx);
        if (abs(dtheta - robot->pos_t) < ALLOWANCE_THETA) {
            robot->set_turn(0);
        } else {
            MotionMacros::soft_turn(robot, (dtheta > 0) ? Directions::LEFT : Directions::RIGHT);
        }
        robot->set_vel(10);
    }
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
