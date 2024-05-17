#include <chrono>
#include <thread>
#include <unistd.h>

#include <smov/mathematics.h>

namespace smov {

Vector3::Vector3() {
  x = 0;
  y = 0;
  z = 0;
}

Vector3::Vector3(float _x, float _y, float _z) {
  x = _x;
  y = _y;
  z = _z;
}

Vector3::Vector3(float xyz) {
  x = xyz;
  y = xyz;
  z = xyz;
}

}
