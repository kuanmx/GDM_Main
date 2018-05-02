//
// Created by poh14 on 4/15/2018.
//

#ifndef MBEDCLI2_EVENTVARIABLE_H
#define MBEDCLI2_EVENTVARIABLE_H
#include <mbed.h>
template<typename T>
class EventVariable {
protected:
    using statefulFuncPtr = void (*)(T&);
    using voidFuncPtr = void (*)();
public:
    explicit EventVariable(T& initialValue, voidFuncPtr funcPtr = nullptr);
    explicit EventVariable(T&& initialValue, voidFuncPtr funcPtr = nullptr);
    explicit EventVariable(voidFuncPtr funcPtr);
    explicit EventVariable(T& initialValue, statefulFuncPtr funcPtr = nullptr);
    explicit EventVariable(T&& initialValue, statefulFuncPtr funcPtr = nullptr);
    explicit EventVariable(statefulFuncPtr funcPtr);
    EventVariable() = default;

    EventVariable<T>& operator=(const T& rhs);
    EventVariable<T>& operator=(const T&& rhs);
    bool operator!();

    T value;
    void attachCallback(voidFuncPtr funcPtr);
    void attachCallback(statefulFuncPtr funcPtr);
    void detachCallback();
protected:
    voidFuncPtr _callback = nullptr;
    statefulFuncPtr _statefulCallback = nullptr;
};

//Constructors
template<typename T>
EventVariable<T>::EventVariable(T& initialValue, voidFuncPtr funcPtr)
        : value(initialValue), _callback(funcPtr)
{
}
template<typename T>
EventVariable<T>::EventVariable(T&& initialValue, voidFuncPtr funcPtr)
        : value(initialValue), _callback(funcPtr)
{
}
template<typename T>
EventVariable<T>::EventVariable(voidFuncPtr funcPtr)
        : _callback(funcPtr)
{
}

template<typename T>
EventVariable<T>::EventVariable(T& initialValue, EventVariable::statefulFuncPtr funcPtr)
        : value(initialValue), _statefulCallback(funcPtr)
{
}
template<typename T>
EventVariable<T>::EventVariable(T&& initialValue, EventVariable::statefulFuncPtr funcPtr)
        : value(initialValue), _statefulCallback(funcPtr)
{
}
template<typename T>
EventVariable<T>::EventVariable(EventVariable::statefulFuncPtr funcPtr)
        : _statefulCallback(funcPtr)
{
}

//Assignment operators
template<typename T>
EventVariable<T>& EventVariable<T>::operator=(const T& rhs)
{
    value = rhs;
    if (_callback!=nullptr) {
        _callback();
    }
    if (_statefulCallback!=nullptr) {
        _statefulCallback(value);
    }
    return *this;
}

template<typename T>
EventVariable<T>& EventVariable<T>::operator=(const T&& rhs)
{
    value = rhs;
    if (_callback!=nullptr) {
        _callback();
    }
    if (_statefulCallback!=nullptr) {
        _statefulCallback(value);
    }
    return *this;
}
template<typename T>
void EventVariable<T>::attachCallback(voidFuncPtr funcPtr)
{
    _callback = funcPtr;
}
template<typename T>
void EventVariable<T>::attachCallback(EventVariable::statefulFuncPtr funcPtr)
{
    _statefulCallback = funcPtr;
}
template<typename T>
void EventVariable<T>::detachCallback()
{
    _callback = nullptr;
    _statefulCallback = nullptr;
}
template<>
bool EventVariable<bool>::operator!()
{
    return !value;
}

#endif //MBEDCLI2_EVENTVARIABLE_H
