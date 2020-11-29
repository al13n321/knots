#pragma once
#include "vec.h"

template<typename T>
struct tmat3 {
  T m[3 * 3];

  tmat3() = default;
  tmat3(T m0, T m1, T m2,
        T m3, T m4, T m5,
        T m6, T m7, T m8)
    : m{m0,m1,m2,m3,m4,m5,m6,m7,m8} {}

  template<typename T2>
  inline tmat3(const tmat3<T2> &v) {
    for (int i = 0; i < 9; ++i)
      m[i] = (T)v.m[i];
  }

  T* operator[](size_t i) {
    return m + i*3;
  }
  const T* operator[](size_t i) const {
    return m + i*3;
  }
  
  tmat3 operator*(const tmat3& b) const {
    return tmat3(m[0]*b.m[0] + m[1]*b.m[3] + m[2]*b.m[6],
                 m[0]*b.m[1] + m[1]*b.m[4] + m[2]*b.m[7],
                 m[0]*b.m[2] + m[1]*b.m[5] + m[2]*b.m[8],
                 m[3]*b.m[0] + m[4]*b.m[3] + m[5]*b.m[6],
                 m[3]*b.m[1] + m[4]*b.m[4] + m[5]*b.m[7],
                 m[3]*b.m[2] + m[4]*b.m[5] + m[5]*b.m[8],
                 m[6]*b.m[0] + m[7]*b.m[3] + m[8]*b.m[6],
                 m[6]*b.m[1] + m[7]*b.m[4] + m[8]*b.m[7],
                 m[6]*b.m[2] + m[7]*b.m[5] + m[8]*b.m[8]);
  }
  tmat3 operator*(T b) const {
    tmat3 r;
    for (int i = 0; i < 9; ++i)
      r.m[i] = m[i] * b;
    return r;
  }
  tmat3& operator*=(T b) {
    for (int i = 0; i < 9; ++i)
      m[i] *= b;
    return *this;
  }
  tmat3 operator/(T b) const {
    tmat3 r;
    for (int i = 0; i < 9; ++i)
      r.m[i] = m[i] / b;
    return r;
  }
  tmat3& operator/=(T b) {
    for (int i = 0; i < 9; ++i)
      m[i] /= b;
    return *this;
  }  
  tmat3& operator+=(const tmat3& b) {
    for (int i = 0; i < 9; ++i)
      m[i] += b.m[i];
    return *this;
  }
  tmat3& operator-=(const tmat3& b) {
    for (int i = 0; i < 9; ++i)
      m[i] -= b.m[i];
    return *this;
  }
  tmat3 operator+(const tmat3& b) const {
    tmat3 r = *this;
    r += b;
    return r;
  }
  tmat3 operator-(const tmat3& b) const {
    tmat3 r = *this;
    r -= b;
    return r;
  }
  tmat3 operator-() const {
    tmat3 r;
    for (int i = 0; i < 9; ++i)
      r.m[i] = -m[i];
    return r;
  }
  tmat3 Inverse() const {
    T d = m[0]*m[4]*m[8]-m[0]*m[5]*m[7]-m[1]*m[3]*m[8]+m[1]*m[5]*m[6]+m[2]*m[3]*m[7]-m[2]*m[4]*m[6];
    return tmat3((m[4]*m[8]-m[5]*m[7])/d,
                 (m[2]*m[7]-m[1]*m[8])/d,
                 (m[1]*m[5]-m[2]*m[4])/d,
                 (m[5]*m[6]-m[3]*m[8])/d,
                 (m[0]*m[8]-m[2]*m[6])/d,
                 (m[2]*m[3]-m[0]*m[5])/d,
                 (m[3]*m[7]-m[4]*m[6])/d,
                 (m[1]*m[6]-m[0]*m[7])/d,
                 (m[0]*m[4]-m[1]*m[3])/d);
  }
  tmat3 Transposed() const {
    return tmat3(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]);
  }
  
  tvec3<T> Column(size_t j) const {
    return tvec3<T>(m[j], m[3+j], m[6+j]);
  }
  tvec3<T> Row(size_t i) const {
    return tvec3<T>(m[i*3], m[i*3+1], m[i*3+2]);
  }

  static tmat3 Zero() {
    tmat3 res;
    for (int i = 0; i < 9; ++i)
      res.m[i] = 0;
    return res;
  }
  static tmat3 Identity() {
    tmat3 res = Zero();
    res.m[0] = res.m[4] = res.m[8] = 1;
    return res;
  }
  static tmat3 Diag(T m0, T m4, T m8) {
    return tmat3(m0, 0, 0, 0, m4, 0, 0, 0, m8);
  }
};

template<typename T>
tmat3<T> operator*(T b, const tmat3<T>& m) {
  return m*b;
}
template<typename T>
tvec3<T> operator*(const tmat3<T>& m, const tvec3<T>& v) {
  return tvec3<T>(v.x*m.m[0] + v.y*m.m[1] + v.z*m.m[2],
                  v.x*m.m[3] + v.y*m.m[4] + v.z*m.m[5],
                  v.x*m.m[6] + v.y*m.m[7] + v.z*m.m[8]);
}

template<typename T>
std::ostream& operator<<(std::ostream& o, const tmat3<T>& m) {
  o << '(';
  for (int i = 0; i < 9; ++i) {
    if (i) {
      if (i%3) o << ',';
      else o << ';';
    }
    o << m.m[i];
  }
  return o << ')';
}

struct fmat4 {
  float m[4 * 4];

  fmat4() = default;
  fmat4(float m0 , float m1 , float m2 , float m3,
        float m4 , float m5 , float m6 , float m7,
        float m8 , float m9 , float m10, float m11,
        float m12, float m13, float m14, float m15) {
    m[0] = m0;
    m[1] = m1;
    m[2] = m2;
    m[3] = m3;
    m[4] = m4;
    m[5] = m5;
    m[6] = m6;
    m[7] = m7;
    m[8] = m8;
    m[9] = m9;
    m[10] = m10;
    m[11] = m11;
    m[12] = m12;
    m[13] = m13;
    m[14] = m14;
    m[15] = m15;
  }
	
  fmat4 operator*(const fmat4 &b) const;

  inline fvec3 Transform (const fvec3 &v) const {
    float d = v.x*m[12] + v.y*m[13] + v.z*m[14] + m[15];
    return fvec3((v.x*m[0] + v.y*m[1] + v.z*m[2]  + m[3] ) / d,
                 (v.x*m[4] + v.y*m[5] + v.z*m[6]  + m[7] ) / d,
                 (v.x*m[8] + v.y*m[9] + v.z*m[10] + m[11]) / d);
  }

  fmat4 Inverse();
  fmat4 Transposed();

  static fmat4 Zero();
  static fmat4 Identity();
  static fmat4 Translation(fvec3 delta);
  static fmat4 RotationX(float yaw);
  static fmat4 RotationY(float pitch);
  static fmat4 RotationZ(float roll);
  static fmat4 Rotation(float yaw, float pitch, float roll);
  static fmat4 Scale(float x, float y, float z);
  // fov - horizontal field of view in degrees
  // near_plane must be positive
  // far_plane must be either zero or strictly greater than near_plane; zero means infinite projection matrix (no far clip plane)
  static fmat4 PerspectiveProjection(float fov, float aspect_ratio, float near_plane, float far_plane);
};

using fmat3 = tmat3<float>;
using dmat3 = tmat3<double>;
