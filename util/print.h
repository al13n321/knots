#pragma once
#include <ostream>
#include <vector>
#include <utility>

// Usage:
// cout << print(vector<int>({1,2,3}));

template<typename T>
struct Printable {
  const T& x;

  explicit Printable(const T& x): x(x) {}
};

template<typename T>
Printable<T> print(const T& x) {
  return Printable<T>(x);
}

// Some SFINAE to add operator<< for Printable<T> whenever there's an operator<< for T.
template<typename T, typename U = decltype(std::declval<std::ostream>() << std::declval<T>())>
std::ostream& operator<<(std::ostream& o, Printable<T> p) {
  return o << p.x;
}

template<typename T>
std::ostream& operator<<(std::ostream& o, Printable<std::vector<T>> v) {
  o << '[';
  for (size_t i = 0; i < v.x.size(); ++i) {
    if (i)
      o << ',';
    o << print(v.x[i]);
  }
  return o << ']';
}
