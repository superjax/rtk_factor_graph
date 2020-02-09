#pragma once

#include <type_traits>

namespace mc {

// This is a simple wrapper for a reference to a type.  It is intended to flag an argument of a
// function as an output.  It acts a lot like a pointer to the object
template <typename T>
class Out
{
    static_assert(!std::is_const<T>::value, "Cannot make an Out variable out of constant");

 public:
    explicit Out(T& val) : value_(val) {}

    T& operator*() { return value_; }
    T* operator->() { return &value_; }

 private:
    T& value_;
};

}  // namespace mc
