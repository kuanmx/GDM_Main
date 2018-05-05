#include <array>
#include <type_traits>

template<typename T>
void isAssignableOrSameAs(){};

template<typename specifiedT, typename argT, typename ...Args>
void isAssignableOrSameAs(argT param1, Args... args){
    static_assert(std::is_assignable<argT, specifiedT>::value || std::is_same<argT, specifiedT>::value,
                  "\n--------\nParameter cannot be assigned to specified type\n----------\n");
    //Checks if all arguments is assignable to or is same as the specified type
    isAssignableOrSameAs<specifiedT>(args...);
};

template<typename T, typename ...Args>
constexpr T filter(Args... args){
    isAssignableOrSameAs<T>(args...);
    std::array<T, sizeof...(Args)> valArray = {args...};
    const auto size = sizeof...(Args);
    static_assert(size!=0, "Must have input arguments");
    if(size<=2){
        return valArray[size-1];
    }
    else {
        T total = 0;
        for(auto &val : valArray) {
            total+=val;
        }
        return total/size;
    }
};