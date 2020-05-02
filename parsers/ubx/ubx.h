/* Copyright (c) 2019 James Jackson, Matt Rydalch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <iostream>
#include <set>
#include <vector>

#include "async_comm/comm.h"
#include "parsers/ubx/ubx_defs.h"

namespace mc {
namespace parsers {
namespace ublox {

class UbxListener
{
 public:
    void subscribe(const uint8_t cls, const uint8_t id)
    {
        uint16_t key = cls << 8 | id;
        subscriptions_.insert(key);
    }
    bool subscribed(const uint8_t cls, const uint8_t id) const
    {
        uint16_t key = cls << 8 | id;
        return subscriptions_.find(key) != subscriptions_.end();
    }

    virtual void gotUbx(const uint8_t cls, const uint8_t id, const UbxMessage &) = 0;

 private:
    std::set<uint16_t> subscriptions_;
};

class Ubx : public async_comm::CommListener
{
    static constexpr int TIMEOUT_MS = 1000;

 public:
    Ubx(async_comm::Comm &ser);

    void configure(uint8_t version,
                   uint8_t layer,
                   uint64_t cfgData,
                   uint32_t cfgDataKey,
                   uint8_t size);
    void getConfiguration(uint8_t version, uint8_t layer, uint32_t cfgDataKey);
    void setDynamicMode();
    void enableMessage(uint8_t msg_cls, uint8_t msg_id, uint8_t rate);
    void registerListener(UbxListener *listener);

    void setNavRate(uint8_t period_ms);
    void configRover();
    void configBase();
    void configBaseStationary();
    void configBaseMobile();
    void pollValue();
    void restart();
    void disableNmea();
    void startSurveyIn();
    bool getVersion();

    void receive_callback(const uint8_t *buf, size_t size) override;

    bool parsingMessage();
    size_t numMessagesReceived();

    bool sendMessage(uint8_t msg_class, uint8_t msg_id, UbxMessage &message, uint16_t len);

    int majorVersion() const { return major_version_; }
    int minorVersion() const { return minor_version_; }
    const char *moduleVame() const { return module_name_; }

 protected:
    bool waitForResponse();

    // low-level parsing functions
    void versionCb();
    bool decodeMessage();
    void calculateChecksum(const uint8_t msg_cls,
                           const uint8_t msg_id,
                           const uint16_t len,
                           const UbxMessage &payload,
                           uint8_t &ck_a,
                           uint8_t &ck_b) const;
    void extractVersionString(const char *str);
    void extractModuleName(const char *str);

    // Parsing State Working Memory
    uint8_t prev_byte_ = 0;
    uint16_t buffer_head_ = 0;
    bool start_message_ = false;
    bool end_message_ = false;
    bool got_ack_ = false;
    bool got_ver_ = false;
    bool got_nack_ = false;
    parse_state_t parse_state_ = parse_state_t::START;
    uint8_t message_class_ = 0;
    uint8_t message_type_ = 0;
    uint16_t length_ = 0;
    uint8_t ck_a_ = 0;
    uint8_t ck_b_ = 0;
    uint32_t num_errors_ = 0;
    uint32_t num_messages_received_ = 0;
    // local storage
    volatile bool new_data_;

    int major_version_;
    int minor_version_;
    char module_name_[10];

    // Serial Port
    void write(const uint8_t byte);
    void write(const uint8_t *byte, const size_t size);
    async_comm::Comm &ser_;
    std::vector<UbxListener *> listeners_;

    // Main buffers for communication
    UbxMessage out_message_;
    UbxMessage in_message_;
};

}  // namespace ublox
}  // namespace parsers
}  // namespace mc
