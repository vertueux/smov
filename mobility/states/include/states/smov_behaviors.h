#pragma once

#include <ctime>

#include <states/smov_states.h>

namespace smov {

enum ServoOrder {
  BODY_BICEPS_LEGS = 0,
  BODY_LEGS_BICEPS = 1,
  BICEPS_LEGS_BODY = 2,
  BICEPS_BODY_LEGS = 3,
  LEGS_BODY_BICEPS = 4,
  LEGS_BICEPS_BODY = 5,
};

struct ServoGroupValues : std::array<double, 3> {};

class RobotBehaviors {
 public:

  static void procedural_front_group_servo_to(ServoGroupValues values, FrontServoArray group, ServoOrder sequence, time_t timeout);
  static void procedural_back_group_servo_to(ServoGroupValues values, BackServoArray group, ServoOrder sequence, time_t timeout);
  static void procedural_group_servo_to(ServoGroupValues values, FrontServoArray front_group, BackServoArray back_group, 
    ServoOrder sequence, time_t timeout);

  // Used for proportional servos.
  static void set_servos_to_center(FrontServoArray f_servos, BackServoArray b_servos); 

  // Both are used for absolute servos.
  static void set_servos_to_min(FrontServoArray f_servos, BackServoArray b_servos); 
  static void set_servos_to_max(FrontServoArray f_servos, BackServoArray b_servos); 
};

} // namespace smov
