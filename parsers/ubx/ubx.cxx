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

#include "parsers/ubx/ubx.h"

#include <stdio.h>

#include <cassert>
#include <chrono>
#include <cstring>
#include <sstream>

#include "common/print.h"

namespace mc {
namespace parsers {
namespace ublox {

Ubx::Ubx(async_comm::Comm &ser) : ser_(ser)
{
    ser.register_listener(*this);
}

bool Ubx::parsingMessage()
{
    return (start_message_ == true && end_message_ == false);
}

size_t Ubx::numMessagesReceived()
{
    return num_messages_received_;
}

bool Ubx::getVersion()
{
    using namespace std::chrono;

    got_ver_ = false;
    sendMessage(CLASS_MON, MON_VER, out_message_, 0);

    auto start = high_resolution_clock::now();
    int dt_ms = 0;
    while (!got_ver_ && dt_ms < TIMEOUT_MS)
    {
        dt_ms = duration_cast<milliseconds>(high_resolution_clock::now() - start).count();
    }

    if (got_ver_)
    {
        return true;
    }
    else
    {
        error("Did not get requested Ubx version");
        return false;
    }
}

bool Ubx::waitForResponse()
{
    using namespace std::chrono;
    got_ack_ = got_nack_ = false;
    auto start = high_resolution_clock::now();
    int dt_ms = 0;
    while (!got_ack_ && !got_nack_ && dt_ms < TIMEOUT_MS)
    {
        dt_ms = duration_cast<milliseconds>(high_resolution_clock::now() - start).count();
    }
    if (got_ack_)
    {
        return true;
    }
    else if (got_nack_ || dt_ms >= TIMEOUT_MS)
    {
        error("No Requested Ubx Response");
        return false;
    }
    return false;
}

bool Ubx::sendMessage(uint8_t msg_class, uint8_t msg_id, UbxMessage &message, uint16_t len)
{
    // First, calculate the checksum
    uint8_t ck_a, ck_b;
    calculateChecksum(msg_class, msg_id, len, message, ck_a, ck_b);

    // Send message
    write(START_BYTE_1);
    write(START_BYTE_2);
    write(msg_class);
    write(msg_id);
    write(len & 0xFF);
    write((len >> 8) & 0xFF);
    write(message.buffer, len);
    write(ck_a);
    write(ck_b);
    return true;
}

void Ubx::startSurveyIn()
{
    out_message_.CFG_TMODE3.flags = CFG_TMODE3_t::SURVEY_IN;
    out_message_.CFG_TMODE3.svin_min_dur = 60;   // Wait at least 1 minute
    out_message_.CFG_TMODE3.svin_acc_limit = 3;  // At least within 3 centimeters
    sendMessage(CLASS_CFG, CFG_TMODE3, out_message_, sizeof(CFG_TMODE3_t));
}

void Ubx::disableNmea()
{
    if (major_version_ <= 23)
    {
        using CF = CFG_PRT_t;
        memset(&out_message_, 0, sizeof(CF));
        out_message_.CFG_PRT.port_id = CF::PORT_USB | CF::PORT_UART1;
        out_message_.CFG_PRT.baudrate = 921600;
        out_message_.CFG_PRT.out_proto_mask = CF::OUT_UBX | CF::OUT_RTCM3;
        out_message_.CFG_PRT.in_proto_mask = CF::OUT_UBX | CF::OUT_RTCM3;
        out_message_.CFG_PRT.flags = CF::CHARLEN_8BIT | CF::PARITY_NONE | CF::STOP_BITS_1;
        sendMessage(CLASS_CFG, CFG_PRT, out_message_, sizeof(CF));
    }
    else
    {
        using CV = CFG_VALSET_t;
        configure(CV::VERSION_0, CV::RAM, 0, CV::USB_INPROT_NMEA, 1);
        configure(CV::VERSION_0, CV::RAM, 0, CV::USB_OUTPROT_NMEA, 1);
    }
}

void Ubx::setDynamicMode()
{
    if (major_version_ <= 23)
    {
        memset(&out_message_, 0, sizeof(CFG_NAV5_t));
        out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
        out_message_.CFG_NAV5.dyn_model = CFG_NAV5_t::DYNMODE_AIRBORNE_4G;
        sendMessage(CLASS_CFG, CFG_NAV5, out_message_, sizeof(CFG_NAV5_t));
    }
    else
    {
        using CV = CFG_VALSET_t;
        configure(CV::VERSION_0, CV::RAM, CV::DYNMODE_AIRBORNE_1G, CV::DYNMODEL, 1);
    }
}

void Ubx::setNavRate(uint8_t period_ms)
{
    info("Setting nav rate to {}", fmt(period_ms));
    if (major_version_ <= 23)
    {
        memset(&out_message_, 0, sizeof(CFG_RATE_t));
        out_message_.CFG_RATE.measRate = period_ms;
        out_message_.CFG_RATE.navRate = 1;
        out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_UTC;
        sendMessage(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
    }
    else
    {
        using CV = CFG_VALSET_t;
        configure(CV::VERSION_0, CV::RAM, period_ms, CV::RATE_MEAS, 1);
        configure(CV::VERSION_0, CV::RAM, 1, CV::RATE_NAV, 1);
        configure(CV::VERSION_0, CV::RAM, CV::TIME_REF_UTC, CV::RATE_TIMEREF, 1);
    }
}

void Ubx::enableMessage(uint8_t msg_cls, uint8_t msg_id, uint8_t rate)
{
    // if (major_version_ <= 23)
    memset(&out_message_, 0, sizeof(CFG_MSG_t));
    out_message_.CFG_MSG.class_id = msg_cls;
    out_message_.CFG_MSG.msg_id = msg_id;
    out_message_.CFG_MSG.rate = rate;
    sendMessage(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
}

void Ubx::receive_callback(const uint8_t *buf, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        const uint8_t byte = buf[i];

        switch (parse_state_)
        {
        case START:
            if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1)
            {
                buffer_head_ = 0;
                parse_state_ = GOT_START_FRAME;
                message_class_ = 0;
                message_type_ = 0;
                length_ = 0;
                ck_a_ = 0;
                ck_b_ = 0;
                start_message_ = true;
                end_message_ = false;
            }
            break;
        case GOT_START_FRAME:
            message_class_ = byte;
            parse_state_ = GOT_CLASS;
            break;
        case GOT_CLASS:
            message_type_ = byte;
            parse_state_ = GOT_MSG_ID;
            break;
        case GOT_MSG_ID:
            length_ = byte;
            parse_state_ = GOT_LENGTH1;
            break;
        case GOT_LENGTH1:
            length_ |= (uint16_t)byte << 8;
            parse_state_ = GOT_LENGTH2;
            if (length_ > BUFFER_SIZE)
            {
                warn("UBX message 0x{x}-0x{x} is too long ({} > {})",
                     fmt(message_class_, message_type_, length_, BUFFER_SIZE));
                num_errors_++;
                prev_byte_ = byte;
                restart();
            }
            break;
        case GOT_LENGTH2:
            if (buffer_head_ < length_)
            {
                // push the byte onto the data buffer
                in_message_.buffer[buffer_head_] = byte;
                if (buffer_head_ == length_ - 1)
                {
                    parse_state_ = GOT_PAYLOAD;
                }
                buffer_head_++;
            }
            break;
        case GOT_PAYLOAD:
            ck_a_ = byte;
            parse_state_ = GOT_CK_A;
            break;
        case GOT_CK_A:
            ck_b_ = byte;
            parse_state_ = GOT_CK_B;
            break;
        default:
            num_errors_++;
            parse_state_ = START;
            end_message_ = false;
            start_message_ = false;
            break;
        }

        // If we have a complete packet, then try to parse it
        if (parse_state_ == GOT_CK_B)
        {
            if (decodeMessage())
            {
                parse_state_ = START;
                end_message_ = true;
                start_message_ = false;
                prev_byte_ = byte;
            }
            else
            {
                // indicate error if it didn't work
                warn("Ubx failed to parse message");
                num_errors_++;
                parse_state_ = START;
                start_message_ = false;
                end_message_ = false;
            }
        }
        prev_byte_ = byte;
    }
}

void Ubx::restart()
{
    parse_state_ = START;
    end_message_ = false;
    start_message_ = false;
}

bool Ubx::decodeMessage()
{
    // First, check the checksum
    uint8_t ck_a, ck_b;
    calculateChecksum(message_class_, message_type_, length_, in_message_, ck_a, ck_b);
    if (ck_a != ck_a_ || ck_b != ck_b_)
        return false;

    num_messages_received_++;

    // Parse the payload
    switch (message_class_)
    {
    case CLASS_ACK:
        switch (message_type_)
        {
        case ACK_ACK:
            got_ack_ = true;
            break;
        case ACK_NACK:
            got_nack_ = true;
            break;
        }
        break;
    case CLASS_MON:
        switch (message_type_)
        {
        case MON_VER:
            versionCb();
            break;
        }
        break;
    default:
        break;
    }

    // call callbacks
    for (auto &l : listeners_)
    {
        if (l->subscribed(message_class_, message_type_))
            l->gotUbx(message_class_, message_type_, in_message_);
    }

    return true;
}

void Ubx::registerListener(UbxListener *l)
{
    listeners_.push_back(l);
}

void Ubx::calculateChecksum(const uint8_t msg_cls,
                            const uint8_t msg_id,
                            const uint16_t len,
                            const UbxMessage &payload,
                            uint8_t &ck_a,
                            uint8_t &ck_b) const
{
    ck_a = ck_b = 0;

    // Add in class
    ck_a += msg_cls;
    ck_b += ck_a;

    // Id
    ck_a += msg_id;
    ck_b += ck_a;

    // Length
    ck_a += len & 0xFF;
    ck_b += ck_a;
    ck_a += (len >> 8) & 0xFF;
    ck_b += ck_a;

    // Payload
    for (int i = 0; i < len; i++)
    {
        ck_a += payload.buffer[i];
        ck_b += ck_a;
    }
}

void Ubx::configure(uint8_t version,
                    uint8_t layer,
                    uint64_t cfgData,
                    uint32_t cfgDataKey,
                    uint8_t size)
{
    memset(&out_message_, 0, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.version = version;
    out_message_.CFG_VALSET.layer = layer;
    if (size == 1)
    {
        out_message_.CFG_VALSET.cfg_data.bytes[0] = cfgData;
    }
    if (size == 2)
    {
        out_message_.CFG_VALSET.cfg_data.word = cfgData;
    }
    out_message_.CFG_VALSET.cfg_data_key = cfgDataKey;
    sendMessage(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
}

void Ubx::getConfiguration(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
{
    memset(&out_message_, 0, sizeof(CFG_VALGET_t));
    out_message_.CFG_VALGET.version = version;
    out_message_.CFG_VALGET.layer = layer;
    out_message_.CFG_VALGET.cfg_data_key = cfgDataKey;
    sendMessage(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t));
}

void Ubx::write(const uint8_t byte)
{
    ser_.send_byte(byte);
}
void Ubx::write(const uint8_t *byte, const size_t size)
{
    ser_.send_bytes(byte, size);
}

void Ubx::versionCb()
{
    for (int i = 0; i < 20; ++i)
    {
        if (strncmp(in_message_.MON_VER.extension[i], "PROTVER=", 8) == 0)
        {
            extractVersionString(in_message_.MON_VER.extension[i]);
        }
        if (strncmp(in_message_.MON_VER.extension[i], "MOD=", 4) == 0)
        {
            extractModuleName(in_message_.MON_VER.extension[i]);
        }
    }

    fmt::print("Connected. ");
    fmt::print("Module: {} ", module_name_);
    fmt::print("Protocol Version: {}.{}\n", major_version_, minor_version_);
    got_ver_ = true;
}

void Ubx::extractVersionString(const char *str)
{
    // Get major version
    char tmp[5];
    const char *s = str + 8;
    for (int j = 0; j < 5; j++)
    {
        if (*s == '.')
        {
            tmp[j] = 0;
            ++s;
            break;
        }
        tmp[j] = *s;
        ++s;
    }
    major_version_ = atoi(tmp);

    // Get minor version
    memset(tmp, 0, sizeof(tmp));
    for (int j = 0; j < 5; j++)
    {
        if (*s == 0)
        {
            tmp[j] = 0;
            break;
        }
        tmp[j] = *s;
        ++s;
    }
    minor_version_ = atoi(tmp);
}

void Ubx::extractModuleName(const char *str)
{
    const char *s = str + 4;
    for (size_t i = 0; i < sizeof(module_name_); i++)
    {
        if (*s == 0)
        {
            module_name_[i] = 0;
            break;
        }

        module_name_[i] = *s;
        ++s;
    }
}

}  // namespace ublox
}  // namespace parsers
}  // namespace mc
