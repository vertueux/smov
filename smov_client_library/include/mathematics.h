#pragma once

#include <vector>
#include <array>

namespace smov {

// This is the representation of 3D vectors and points. 
// A Vector3 are essentially directions with a length., consisting of 3 coordinates, x, y and z. 
// 
// Represents a vector with three single-precision floating-point values. 
// The Vector3 structure provides support for hardware acceleration. 
struct Vector3 {
 public:
  Vector3();
  Vector3(float _x, float _y, float _z);
  explicit Vector3(float xyz);

  float x, y, z;
};

} // namespace smov
