#pragma once

#include <cstdint>
#include <cstdlib>
#include <functional>

namespace mc {
namespace utils {

class SerialListener
{
 public:
    virtual void readCb(const uint8_t* buf, const size_t size) = 0;
};

class SerialInterface
{
 public:
    using SerialCb = std::function<void(const uint8_t* buf, const size_t size)>;

    virtual void write(const uint8_t* buf, const size_t size) = 0;
    void addListener(SerialListener* l) { listeners_.push_back(l); }
    void addCallback(SerialCb cb) { cb_ = cb; }

    std::vector<SerialListener*> listeners_;
    SerialCb cb_;
};

}  // namespace utils
}  // namespace mc
