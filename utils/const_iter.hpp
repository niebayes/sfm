// !warn There is no need to include libs in this file.
// ! Because this file can be seen as remaining part of the
// ! corresponding "actual" header file.

/** @brief This func return non-const version std::begin() in favor of
 * that it works even for containers that offer a begin() member func
 * but fail to offer a cbegin() member.
 * For standard container types, the param container will be
 * a reference to const, which is in agree with our intent.
 */
template <typename C>
constexpr auto cbegin(const C &container) -> decltype(std::begin(container)) {
  return std::begin(container);
}

/** @brief This func return non-const version std::end() in favor of
 * that it works even for containers that offer a end() member func
 * but fail to offer a cend() member.
 * For standard container types, the param container will be
 * a reference to const, which is in agree with our intent.
 */
template <typename C>
constexpr auto cend(const C &container) -> decltype(std::end(container)) {
  return std::end(container);
}
