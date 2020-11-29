#pragma once

#include "vec.h"
#include "mat.h"
#include <ostream>

// Quaternion.
template<typename T>
struct tquat {
  T a{1}, b{0}, c{0}, d{0};

  tquat() {}
  tquat(T a, T b, T c, T d): a(a), b(b), c(c), d(d) {}
  template<typename T2>
  tquat(const tquat<T2>& q): a((T)q.a), b((T)q.b), c((T)q.c), d((T)q.d) {}

  // Angle in radians. Angle is measured clockwise
  // when viewed with view direction equal to axis.
  tquat(T angle, tvec3<T> axis) {
    T sn = std::sin(angle * .5f);
    T cs = std::cos(angle * .5f);
    a = cs;
    axis.NormalizeMe();
    axis *= sn;
    b = axis.x;
    c = axis.y;
    d = axis.z;
  }

  tquat operator+(const tquat &q) const {
    return tquat(a+q.a, b+q.b, c+q.c, d+q.d);
  }
  tquat& operator+=(const tquat &q) {
    a += q.a; b += q.b; c += q.c; d += q.d;
    return *this;
  }
  tquat operator*(const tquat &q) const {
    return tquat(
      a*q.a - b*q.b - c*q.c - d*q.d,
      a*q.b + b*q.a + c*q.d - d*q.c,
      a*q.c + c*q.a + d*q.b - b*q.d,
      a*q.d + d*q.a + b*q.c - c*q.b);
  }
  tquat& operator*=(const tquat &q) {
    return *this = *this * q;
  }
  tquat operator*(T s) const {
    return tquat(a*s, b*s, c*s, d*s);
  }
  tquat& operator*=(T s) {
    a*=s; b*=s; c*=s; d*=s;
    return *this;
  }
  tquat operator/(T s) const {
    return tquat(a/s, b/s, c/s, d/s);
  }

  T LengthSquare() const {
    return a*a + b*b + c*c + d*d;
  }
  T Length() const {
    return sqrt(LengthSquare());
  }
  void NormalizeMe() {
    T z = Length();
    a /= z;
    b /= z;
    c /= z;
    d /= z;
  }
  tquat Normalized() const {
    tquat res = *this;
    res.NormalizeMe();
    return res;
  }

  fmat4 ToMatrix4() const {
    // Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
    // (a bit modified)
    return fmat4(
       a, -d,  c, -b,
       d,  a, -b, -c,
      -c,  b,  a, -d,
       b,  c,  d,  a
    ) * fmat4(
       a, -d,  c,  b,
       d,  a, -b,  c,
      -c,  b,  a,  d,
      -b, -c, -d,  a
    );
  }

  tmat3<T> ToMatrix() const {
    // It's silly to have different quaternion->matrix conversion for 3x3 and 4x4 matrices,
    // but this one is templated and the other one looks neat.
    return
      tmat3<T>::Identity() +
      2.*tmat3<T>(-c*c-d*d, b*c, b*d,
                  b*c, -b*b-d*d, c*d,
                  b*d, c*d, -b*b-c*c) +
      2.*a*tmat3<T>(0, -d, c,
                    d, 0, -b,
                    -c, b, 0);
  }

  tquat Conjugate() const {
    return tquat(a, -b, -c, -d);
  }

  tquat Inverse() const {
    return Conjugate() / LengthSquare();
  }

  tvec3<T> Transform(tvec3<T> v) const {
    tquat q = *this * tquat(0, v.x, v.y, v.z) * this->Inverse();
    return tvec3<T>(q.b, q.c, q.d);
  }

  tvec3<T> Untransform(tvec3<T> v) const {
    tquat q = this->Inverse() * tquat(0, v.x, v.y, v.z) * *this;
    return tvec3<T>(q.b, q.c, q.d);
  }

  tmat3<T> Transform(tmat3<T> m) const {
    for (int i = 0; i < 3; ++i) {
      tvec3<T> c = Transform(tvec3<T>(m.m[i], m.m[i+3], m.m[i+6]));
      m.m[i] = c.x; m.m[i+3] = c.y; m.m[i+6] = c.z;
    }
    return m;
  }

  // Returns q such that q.Transform(a) points in the same direction as b.
  // a and b don't have to be normalized.
  static tquat RotationTo(tvec3<T> a, tvec3<T> b) {
    tvec3<T> c = a.Cross(b);
    tquat q;
    q.a = a.Dot(b) + sqrt(a.LengthSquare() * b.LengthSquare());
    q.b = c.x;
    q.c = c.y;
    q.d = c.z;
    q.NormalizeMe();
    return q;
  }
};

template<typename T>
inline std::ostream& operator<<(std::ostream& o, const tquat<T>& q){
  return o<<'('<<q.a<<','<<q.b<<','<<q.c<<','<<q.d<<')';
}

using fquat = tquat<float>;
using dquat = tquat<double>;
