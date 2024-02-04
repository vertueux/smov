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

  // Make the Vector3 Values to Zero.
  static Vector3 zero();

  // Make the Vector3 Values to One.
  static Vector3 one();

  // Shorthand for writing Vector3(0, 0, -1).    
  static Vector3 back();

  // Shorthand for writing Vector3(0, -1, 0).    
  static Vector3 down();

  // Shorthand for writing Vector3(0, 0, 1).    
  static Vector3 forward();

  // Shorthand for writing Vector3(-1, 0, 0).
  static Vector3 left();

  // Shorthand for writing Vector3(1, 0, 0).  
  static Vector3 right();

  // Shorthand for writing Vector3(0, 0, 0).    
  static Vector3 up();
};

} // namespace smov
