#pragma once

#include <states/smov_states.h>

namespace smov {

class Behaviors {
 public:
  static void set_servos_to_center(FrontServoArray f_servos, BackServoArray b_servos); 
  static void set_servos_to_min(FrontServoArray f_servos, BackServoArray b_servos); 
  static void set_servos_to_max(FrontServoArray f_servos, BackServoArray b_servos); 
  static void set_fs_ang_from_center(int servo, int angle);
  static void set_bs_ang_from_center(int servo, int angle);
};

} // namespace smov
