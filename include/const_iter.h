#ifndef CONST_ITER_UTILS_H_
#define CONST_ITER_UTILS_H_

#include <iostream>
#include <iterator>
#include <vector>

namespace std {

template <typename C>
constexpr auto cbegin(const C &container) -> decltype(std::begin(container));

template <typename C>
constexpr auto cend(const C &container) -> decltype(std::end(container));

#include "const_iter.hpp"

}  // namespace std

#endif  // CONST_ITER_UTILS_H_
