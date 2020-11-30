#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <ostream>

template<typename T>
struct tmat3;

template<typename T>
struct tvec2 {
  // Closest floating-point type.
  typedef typename
    std::conditional<std::is_floating_point<T>::value, T, double>::type F;

  T x = 0, y = 0;

  tvec2() {}
  tvec2(T x, T y): x(x), y(y) {}

  template<typename X>
  inline tvec2<T>(const tvec2<X> &v) : x(v.x), y(v.y) {}

  tvec2<T> operator*(const T v) const {
    return tvec2<T>(x * v, y * v);
  }

  T LengthSquare() const { return x*x + y*y; }

  F Length() const {
    return sqrt((F)LengthSquare());
  }

  bool operator==(const tvec2<T> &rhs) const {
    return x == rhs.x && y == rhs.y;
  }
  tvec2<T> operator-() const {
    return tvec2<T>(-x, -y);
  }
};

template<typename ftype>
struct tvec3 {
  ftype x = 0, y = 0, z = 0;

  tvec3() {}
  tvec3(ftype x, ftype y, ftype z) : x(x), y(y), z(z) {}

  template<typename T>
  tvec3<ftype>(const tvec3<T> &v) : x((ftype)v.x), y((ftype)v.y), z((ftype)v.z) {}

  tvec3<ftype> operator - () const { return tvec3<ftype>(-x, -y, -z); }

  tvec3<ftype> operator + (const tvec3<ftype> &v) const { return tvec3<ftype>(x + v.x, y + v.y, z + v.z); }
  tvec3<ftype> operator - (const tvec3<ftype> &v) const { return tvec3<ftype>(x - v.x, y - v.y, z - v.z); }
  tvec3<ftype> operator * (ftype d) const { return tvec3(x * d, y * d, z * d); }
  tvec3<ftype> operator * (const tvec3<ftype> &v) const { return tvec3<ftype>(x * v.x, y * v.y, z * v.z); }
  tvec3<ftype> operator / (ftype d) const { d = 1.0f / d; return tvec3<ftype>(x * d, y * d, z * d); }

  tvec3<ftype>& operator += (const tvec3<ftype> &v) { x += v.x; y += v.y; z += v.z; return *this; }
  tvec3<ftype>& operator -= (const tvec3<ftype> &v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
  tvec3<ftype>& operator *= (ftype d) { x *= d; y *= d; z *= d; return *this; }
  tvec3<ftype>& operator *= (const tvec3<ftype> &v) { x *= v.x; y *= v.y; z *= v.z; return *this; }
  tvec3<ftype>& operator /= (ftype d) { d = 1.0f / d; x *= d; y *= d; z *= d; return *this; }

  // be careful with precision; obviously
  bool IsZero() const { return x == 0 && y == 0 && z == 0; }

  ftype Dot(const tvec3<ftype> &v) const { return x*v.x + y*v.y + z*v.z; }
  tvec3<ftype> Cross(const tvec3<ftype> &v) const { return tvec3<ftype>(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }

  // Cross product matrix: a.Cross(b) = a.Skew() * b = -b.Skew() * a.
  tmat3<ftype> Skew() const {
    return tmat3<ftype>(0, -z, y,
                        z, 0, -x,
                        -y, x, 0);
  }

  // Returns some vector (of arbitrary length) perpendicular to this.
  tvec3<ftype> ArbitraryPerpendicular() const {
    return Cross(abs(x) > abs(y) + abs(z) ? tvec3<ftype>(0, 1, 1) : tvec3(1, 0, 0));
  }
  
  ftype LengthSquare() const { return Dot(*this); }
  ftype Length() const { return sqrt(LengthSquare()); }

  ftype DistanceSquare(const tvec3<ftype> &b) { return (b - *this).LengthSquare(); }
  ftype Distance(const tvec3<ftype> &b) { return (b - *this).Length(); }

  void NormalizeMe() { *this /= Length(); }
  tvec3<ftype> Normalized() const { return *this / Length(); }

  bool AllGreaterThan(const tvec3<ftype> &v) { return x > v.x && y > v.y && z > v.z; }
  bool AllLessThan(const tvec3<ftype> &v) { return x < v.x && y < v.y && z < v.z; }
  ftype MinComponent() const { return x < y ? x < z ? x : z : y < z ? y : z; }
  ftype MaxComponent() const { return x > y ? x > z ? x : z : y > z ? y : z; }
  tvec3<ftype> Min(const tvec3 &v) const { return tvec3(std::min(x, v.x), std::min(y, v.y), std::min(z, v.z)); }
  tvec3<ftype> Max(const tvec3 &v) const { return tvec3(std::max(x, v.x), std::max(y, v.y), std::max(z, v.z)); }
  tvec3<ftype> Abs() const { return tvec3(std::abs(x), std::abs(y), std::abs(z)); }
  tvec3<ftype> Clamp(ftype a, ftype b) {
    return tvec3<ftype>(
      x < a ? a : x > b ? b : x,
      y < a ? a : y > b ? b : y,
      z < a ? a : z > b ? b : z);
  }

  bool AllSameSign(const tvec3<ftype> &v) { return x * v.x > 0 && y * v.y > 0 && z * v.z > 0; }

  // gets barycentric coordinates of this projected on triangle's plane;
  // if clamp, all out values are forced to range [0, 1] (while keeping their sum equal to 1)
  void ToBarycentric(const tvec3<ftype> *triangle, ftype *out, bool clamp = false) {
    tvec3<ftype> n = (triangle[1] - triangle[0]).Cross(triangle[2] - triangle[0]);

    ftype sum = 0;

    for (int i = 0; i < 3; ++i) {
      out[i] = (triangle[(i + 2) % 3] - triangle[(i + 1) % 3]).Cross(*this - triangle[(i + 1) % 3]).Dot(n);
      if (clamp && out[i] < 0)
        out[i] = 0;
      sum += out[i];
    }

    if (std::abs(sum) >= 1e-5) {
      for(int i = 0; i < 3; ++i)
        out[i] /= sum;
    } else {
      // this may be reachable only due to precision issues
      out[0] = out[1] = out[2] = static_cast<ftype>(1./3);
    }
  }

  void ToArray(ftype* p, size_t stride=1) const {
    p[0] = x;
    p[stride] = y;
    p[stride + stride] = z;
  }
  void FromArray(const ftype* p, size_t stride=1) {
    x = p[0];
    y = p[stride];
    z = p[stride + stride];
  }
  void AddToArrayMasked(ftype* p, uint8_t msk, size_t stride=1) {
    if (msk & 1) { *p += x; p += stride; }
    if (msk & 2) { *p += y; p += stride; }
    if (msk & 4) *p += z;
  }
};

template<typename ftype>
tvec3<ftype> operator*(ftype c, const tvec3<ftype>& v) {
  return v*c;
}

template<typename ftype>
struct tvec4 {
  ftype x, y, z, w;

  tvec4() {}
  tvec4(ftype x, ftype y, ftype z, ftype w): x(x), y(y), z(z), w(w) {}
};

template<typename T>
std::ostream& operator<<(std::ostream& o, const tvec2<T>& v) {
  return o << '(' << v.x << ',' << v.y << ')';
}
template<typename T>
std::ostream& operator<<(std::ostream& o, const tvec3<T>& v) {
  return o << '(' << v.x << ',' << v.y << ',' << v.z << ')';
}
template<typename T>
std::ostream& operator<<(std::ostream& o, const tvec4<T>& v) {
  return o << '(' << v.x << ',' << v.y << ',' << v.z << ',' << v.w << ')';
}

typedef tvec2<float> fvec2;
typedef tvec2<double> dvec2;
typedef tvec2<int> ivec2;
typedef tvec3<float> fvec3;
typedef tvec3<double> dvec3;
typedef tvec4<float> fvec4;
typedef tvec4<double> dvec4;
