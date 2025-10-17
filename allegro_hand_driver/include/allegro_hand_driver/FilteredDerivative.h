/// Basic implementation of https://fr.mathworks.com/help/sps/ref/filteredderivativediscreteorcontinuous.html

# pragma once
#include <cassert>


template<typename TimeType, typename ValueType>
class FilteredDerivative {
  public:
    FilteredDerivative() = default;
    virtual ~FilteredDerivative() = default;

    // Default time constant is zero. Adding new point will raise exception. User needs to set it before.
    void set_time_constant(const TimeType constant);
    void new_point(const TimeType timestamp, const ValueType v);

    ValueType get_value_raw() const {return _value_raw; }
    ValueType get_value_filtered() const {return _value_filtered; }
    ValueType get_derivative_raw() const {return _derivative_raw; }
    ValueType get_derivative_filtered() const {return _derivative_filtered; }

  private:
    bool _init                     = false;
    TimeType  _time_constant       = 0;
    TimeType  _timestamp           = 0;
    ValueType _internal_state      = 0;
    ValueType _value_raw           = 0;
    ValueType _value_filtered      = 0;
    ValueType _derivative_raw      = 0;
    ValueType _derivative_filtered = 0;
};

template<typename TimeType, typename ValueType>
void FilteredDerivative<TimeType, ValueType>::new_point(const TimeType timestamp, const ValueType v)
{
    if(!_init) {
        _timestamp = timestamp;
        _internal_state = v;

        _value_raw = v;
        _value_filtered = v;

        _derivative_raw = 0;
        _derivative_filtered = 0;

        _init = true;
        return;
    }

    const TimeType __sample_time = timestamp - _timestamp;
    const ValueType __alpha = (ValueType) __sample_time / _time_constant;

    // Prevent divinding by zero;
    assert(_time_constant);
    assert(__sample_time);

    // Update internal state
    _internal_state = (1. - __alpha) * _internal_state + __alpha * v;

    // Update user values
    _derivative_filtered = (v - _internal_state) / _time_constant;
    _derivative_raw = (v - _value_raw) / __sample_time;
    _value_filtered = _internal_state;
    _value_raw = v;
    _timestamp = timestamp;
}

template<typename TimeType, typename ValueType>
void FilteredDerivative<TimeType, ValueType>::set_time_constant(const TimeType constant)
{
    _time_constant = constant;
};