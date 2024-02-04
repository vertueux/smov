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

Vector3 Vector3::zero() {
  return {0, 0, 0};
}

Vector3 Vector3::one() {
  return {1, 1, 1};
}

Vector3 Vector3::back() {
  return {0, 0, -1};
}

Vector3 Vector3::down() {
  return {0, -1, 0};
}

Vector3 Vector3::forward() {
  return {0, 0, 1};
}
Vector3 Vector3::left() {
  return {-1, 0, 0};
}

Vector3 Vector3::right() {
  return {1, 0, 0};
}
Vector3 Vector3::up() {
  return {0, 1, 0};
}

}
