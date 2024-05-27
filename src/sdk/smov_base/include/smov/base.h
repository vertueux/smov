#ifndef BASE_H_
#define BASE_H_

namespace smov {

enum FrontServos {
  FRONT_BODY_LEFT = 0,
  FRONT_BODY_RIGHT = 1,
  FRONT_UPPER_LEG_LEFT = 2,
  FRONT_UPPER_LEG_RIGHT = 3,
  FRONT_LOWER_LEG_LEFT = 4,
  FRONT_LOWER_LEG_RIGHT = 5
};

enum BackServos {
  BACK_BODY_LEFT = 0,
  BACK_BODY_RIGHT = 1,
  BACK_UPPER_LEG_LEFT = 2,
  BACK_UPPER_LEG_RIGHT = 3,
  BACK_LOWER_LEG_LEFT = 4,
  BACK_LOWER_LEG_RIGHT = 5
};

extern void delay(int time);

}

#endif // BASE_H_
