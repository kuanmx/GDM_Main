//
// Created by poh14 on 5/7/2018.
//

#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H
#include <deque>

/**
 * Moving average class
 * Example declaration: MovingAverage<float, 7> average1(3.14);
 * @tparam T data storage type
 * @tparam N number of elements to average, cannot be odd or less than 1
 */
template<typename T, int N>
class MovingAverage {
public:
    MovingAverage(){
        static_assert(N>0, "Number of elements has to be greater than 0");
        static_assert(N%2==1, "Number of elements has to be odd (for stability)");
    }
    explicit MovingAverage(T initialData);
    void AddData(T data);
    const std::deque<T>& getDataList() const
    {
        return _dataList;
    }
    T getValue() const
    {
        return _value;
    }
private:
    std::deque<T> _dataList;
    T _value;
    const double N_recipro = 1.0/N;
};


template<typename T, int N>
MovingAverage<T, N>::MovingAverage(T initialData) :  _value(initialData)
{
    MovingAverage();
    _dataList.push_back(initialData);
}

template<typename T, int N>
void MovingAverage<T, N>::AddData(T data)
{
    T sum = 0;
    if(_dataList.size()<N){
        _dataList.push_back(data);
        for(auto &val: _dataList){
            sum += val;
        }
        _value = sum/_dataList.size();
    }
    else{
        _dataList.pop_front();
        _dataList.push_back(data);
        for(auto &val: _dataList){
            sum += val;
        }
        _value = sum*N_recipro;
    }
}

#endif //MOVINGAVERAGE_H
