/*
template <typename T...>
void smoothing(T... params){

}*/

/**
 * Checks whether a given parameter pack can fit into the same type
 * Gives compile error if cannot
 * To be used in template metaprogramming in variadic templates
 * Example usage: isAssignableOrSameAs<T>(args...);
 * @tparam specifiedT specified type to check
 * @tparam argT type of first argument in parameter pack
 * @tparam Args type of parameter pack
 * @param param1 first argument in parameter pack
 * @param args parameter pack
 */
template<typename specifiedT, typename argT, typename ...Args>
void isAssignableOrSameAs(argT param1, Args... args);

/**
 * Filter function that takes variable amount of parameters
 * Example usage 1: double result = filter<double>(3.1, 3.2, 3.3);
 * Example usage 2: int result = filter<int>(1, 2, 3, 4, 5, 6, 7, 8, 9);
 * Note: Make sure operator+ and operator/ works properly for the specified type in angled brackets
 * @tparam T specified type to take and process
 * @tparam Args deduced type of variable arguments
 * @param args variable arguments to be passed into the filter
 * @return filtered result
 */
template<typename T, typename ...Args>
constexpr T filter(Args... args);

#include "Functions.tpp"