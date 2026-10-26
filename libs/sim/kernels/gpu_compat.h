// Vector types and math shared by the Metal kernels and their C++ build (the emulated GPU device and tests).
// Kernel code uses only what this header defines, so it compiles unchanged as MSL and as C++20.
#pragma once

#if defined(__METAL_VERSION__)
#include <metal_stdlib>
using namespace metal;
#define DRAPE_DEVICE device
#define DRAPE_THREAD thread
#define constant_or_static constant
#else
#include <cmath>
#include <cstdint>
#define DRAPE_DEVICE
#define DRAPE_THREAD
#define constant_or_static inline constexpr
#endif

namespace dgpu {

#if !defined(__METAL_VERSION__)
using uint = std::uint32_t;

struct float2 {
  float x = 0, y = 0;
};

struct float3 {
  float x = 0, y = 0, z = 0;
};
inline float3 operator+(float3 a, float3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline float3 operator-(float3 a, float3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline float3 operator-(float3 a) { return {-a.x, -a.y, -a.z}; }
inline float3 operator*(float3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float3 operator*(float s, float3 a) { return {s * a.x, s * a.y, s * a.z}; }
inline float3 operator*(float3 a, float3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline float3 operator/(float3 a, float s) { return {a.x / s, a.y / s, a.z / s}; }
inline float3& operator+=(float3& a, float3 b) { return a = a + b; }
inline float3& operator-=(float3& a, float3 b) { return a = a - b; }
inline float3& operator*=(float3& a, float s) { return a = a * s; }

struct alignas(16) float4 {
  float x = 0, y = 0, z = 0, w = 0;
};

inline float dot(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float3 cross(float3 a, float3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length_squared(float3 a) { return dot(a, a); }
inline float sqrt(float x) { return std::sqrt(x); }
inline float length(float3 a) { return std::sqrt(dot(a, a)); }
inline float3 normalize(float3 a) { return a / length(a); }
inline float fabs(float x) { return std::fabs(x); }
inline float fmax(float a, float b) { return std::fmax(a, b); }
inline float fmin(float a, float b) { return std::fmin(a, b); }
inline float floor(float x) { return std::floor(x); }
inline float atan2(float y, float x) { return std::atan2(y, x); }
inline float clamp(float x, float lo, float hi) { return std::fmin(std::fmax(x, lo), hi); }
inline bool isfinite(float x) { return std::isfinite(x); }
#endif

inline float3 xyz(float4 v) { return float3{v.x, v.y, v.z}; }
inline float4 make4(float3 v, float w) { return float4{v.x, v.y, v.z, w}; }
inline float3 zero3() { return float3{0.0f, 0.0f, 0.0f}; }

// 3x3 matrix as three columns (not MSL's float3x3, so both compilers see the same type).
struct Mat3 {
  float3 c0, c1, c2;
};
inline Mat3 zeroMat3() { return Mat3{zero3(), zero3(), zero3()}; }
inline Mat3 identity3() {
  return Mat3{float3{1.0f, 0.0f, 0.0f}, float3{0.0f, 1.0f, 0.0f}, float3{0.0f, 0.0f, 1.0f}};
}
inline float3 mul(Mat3 m, float3 v) { return m.c0 * v.x + m.c1 * v.y + m.c2 * v.z; }
inline Mat3 add(Mat3 a, Mat3 b) { return Mat3{a.c0 + b.c0, a.c1 + b.c1, a.c2 + b.c2}; }
inline Mat3 sub(Mat3 a, Mat3 b) { return Mat3{a.c0 - b.c0, a.c1 - b.c1, a.c2 - b.c2}; }
inline Mat3 scale(Mat3 a, float s) { return Mat3{a.c0 * s, a.c1 * s, a.c2 * s}; }
inline Mat3 outer(float3 a, float3 b) { return Mat3{a * b.x, a * b.y, a * b.z}; }  // a b^T
inline Mat3 negate(Mat3 a) { return Mat3{-a.c0, -a.c1, -a.c2}; }

}  // namespace dgpu
