#pragma once
#include <cassert>
#include <valarray>

template<typename T>
struct TMatrix {
  size_t n = 0, m = 0;
  std::valarray<T> a;

  TMatrix() {}
  TMatrix(size_t n, size_t m): n(n), m(m), a(n*m) {}

  T* operator[](size_t i) {
    return &a[i*m];
  }
  const T* operator[](size_t i) const {
    return &a[i*m];
  }

  // If new n*m is same as current, "reshapes" the matrix keeping contents.
  // Otherwise destroys contents.
  void Resize(size_t _n, size_t _m) {
    if (_n * _m != n * m)
      a.resize(_n * _m);
    n = _n;
    m = _m;
  }

  void Fill(T v) {
    a = v;
  }

  size_t Stride() const {
    return m;
  }

  void FillIdentity() {
    Fill(0);
    for (size_t i = 0; i < n && i < m; ++i)
      a[i*m + i] = 1;
  }

  // Solves system of linear equations. Let w=*this, and m = n+1. The equations are:
  //  w[  0][0] * x[0] + w[  0][1] * x[1] + ... + w[  0][n-1] * x[n-1] = w[  0][n],
  //  w[  1][0] * x[0] + w[  1][1] * x[1] + ... + w[  1][n-1] * x[n-1] = w[  1][n],
  //  ...
  //  w[n-1][0] * x[0] + w[n-1][1] * x[1] + ... + w[n-1][n-1] * x[n-1] = w[n-1][n].
  // After the call to LinSolve(), the solution x[0..n-1] will be in w[0..n-1][n],
  // and w[0..n-1][0..n-1] will be an identity matrix.
  // If solution is not unique, will find one of them and return false.
  // If there are no solutions, will find something that may or may not resemble a solution,
  // and return false.
  // Uses straightforward Gaussian elimination, runs in O(n^3).
  bool SolveLinearSystem(T epsilon = std::numeric_limits<T>::epsilon() * 100);
};

template<typename T>
std::ostream& operator<<(std::ostream& o, const TMatrix<T>& m) {
  o << "[";
  for (size_t i = 0; i < m.n; ++i) {
    o << '\n';
    for (size_t j = 0; j < m.m; ++j) {
      if (j)
        o << ", ";
      o << m[i][j];
    }
  }
  return o << "]";
}

template<typename T>
bool TMatrix<T>::SolveLinearSystem(T epsilon) {
  assert(m == n+1);
  bool ok = true;
  for (size_t i = 0; i < n; ++i) {
    T mx = std::abs(a[i*m + i]);
    size_t k = i;
    for (size_t j = i + 1; j < n; ++j) {
      T t = std::abs(a[j*m + i]);
      if (t > mx) {
        mx = t;
        k = j;
      }
    }
    if (mx < epsilon) {
      ok = false;
      for (size_t r = 0; r < n; ++r)
        a[r*m + i] = 0;
      a[i*m + i] = 1;
      a[i*m + n] = 0;
      continue;
    }
    if (k != i) {
      for (size_t j = i; j < m; ++j)
        std::swap(a[i*m + j], a[k*m + j]);
    }
    T c = 1/a[i*m + i];
    for (size_t j = i; j < m; ++j)
      a[i*m + j] *= c;
    for (size_t r = 0; r < n; ++r) {
      if (r == i)
        continue;
      T c = a[r*m + i];
      for (size_t j = i; j < m; ++j)
        a[r*m + j] -= a[i*m + j] * c;
    }
  }
  return ok;
}

using DMatrix = TMatrix<double>;
