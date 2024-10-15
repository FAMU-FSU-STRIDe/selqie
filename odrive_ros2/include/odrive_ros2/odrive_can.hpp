#pragma once

#include <string>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <unistd.h>
#include <cassert>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#define MAX_CANID 63

typedef uint8_t cmdid_t;
typedef uint32_t arbid_t;

enum ODriveCanID : cmdid_t
{
    HEARTBEAT = 0x001,
    GET_ENCODER_ESTIMATES = 0x009,
    GET_IQ = 0x014,
    GET_TEMPERATURE = 0x015,
    GET_BUS_VOLTAGE_CURRENT = 0x017,
    GET_TORQUES = 0x01C,
    SET_AXIS_STATE = 0x007,
    SET_CONTROL_MODE = 0x00b,
    SET_LIMITS = 0x00f,
    SET_POS_GAIN = 0x01a,
    SET_VEL_GAINS = 0x01b,
    SET_POSITION = 0x00c,
    SET_VELOCITY = 0x00d,
    SET_TORQUE = 0x00e,
    CLEAR_ERRORS = 0x018
};

class ODriveSocket
{
private:
    int _socket;

    struct
    {
        uint32_t axis_error;
        uint8_t axis_state;
        float pos_estimate, vel_estimate, torq_estimate;
        float iq_setpoint, iq_measured;
        float fet_temperature, motor_temperature;
        float bus_voltage, bus_current;
    } _infos[MAX_CANID + 1];

    bool _close_poll = false;
    std::thread _poll_thread;
    std::mutex _infos_mutex;

    void _open(const std::string &if_name)
    {
        _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (_socket < 0)
        {
            throw std::runtime_error("Failed to create CAN socket on interface " + if_name);
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, if_name.c_str());
        ioctl(_socket, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            throw std::runtime_error("Failed to bind socket on interface " + if_name);
        }
    }

    ssize_t _send(const canid_t id, const uint8_t *data, const uint8_t size) const
    {
        assert(size > CAN_MAX_DLC);
        struct can_frame frame;
        frame.can_id = id;
        frame.can_dlc = size;
        std::memcpy(frame.data, data, size);
        return write(_socket, &frame, sizeof(frame));
    }

    inline arbid_t _getArbID(const canid_t can_id, const cmdid_t cmd_id) const
    {
        return static_cast<arbid_t>(can_id << 5 | cmd_id);
    }

    inline canid_t _getCanID(const arbid_t arb_id) const
    {
        return static_cast<canid_t>(arb_id >> 5 & 0b111111);
    }

    inline cmdid_t _getCmdID(const arbid_t arb_id) const
    {
        return static_cast<cmdid_t>(arb_id & 0b11111);
    }

    void _poll()
    {
        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(_infos_mutex);
                if (_close_poll)
                {
                    break;
                }
            }

            struct can_frame frame;
            if (read(_socket, &frame, sizeof(frame)) <= 0)
            {
                continue;
            }

            const canid_t odrv_id = _getCanID(frame.can_id);
            const cmdid_t cmd_id = _getCmdID(frame.can_id);
            assert(odrv_id <= MAX_CANID);

            switch (cmd_id)
            {
            case ODriveCanID::HEARTBEAT:
                // Heartbeat
                uint32_t axis_error;
                uint8_t axis_state;
                std::memcpy(&axis_error, frame.data, sizeof(axis_error));
                std::memcpy(&axis_state, frame.data + 4, sizeof(axis_state));
                {
                    std::lock_guard<std::mutex> lock(_infos_mutex);
                    _infos[odrv_id].axis_error = axis_error;
                    _infos[odrv_id].axis_state = axis_state;
                }
                break;
            case ODriveCanID::GET_IQ:
                // Get_Iq
                float iq_setpoint, iq_measured;
                std::memcpy(&iq_setpoint, frame.data, sizeof(iq_setpoint));
                std::memcpy(&iq_measured, frame.data + 4, sizeof(iq_measured));
                {
                    std::lock_guard<std::mutex> lock(_infos_mutex);
                    _infos[odrv_id].iq_setpoint = iq_setpoint;
                    _infos[odrv_id].iq_measured = iq_measured;
                }
                break;
            case ODriveCanID::GET_TEMPERATURE:
                // Get_Temperature
                float fet_temperature, motor_temperature;
                std::memcpy(&fet_temperature, frame.data, sizeof(fet_temperature));
                std::memcpy(&motor_temperature, frame.data + 4, sizeof(motor_temperature));
                {
                    std::lock_guard<std::mutex> lock(_infos_mutex);
                    _infos[odrv_id].fet_temperature = fet_temperature;
                    _infos[odrv_id].motor_temperature = motor_temperature;
                }
                break;
            case ODriveCanID::GET_BUS_VOLTAGE_CURRENT:
                // Get_Bus_Voltage_Current
                float bus_voltage, bus_current;
                std::memcpy(&bus_voltage, frame.data, sizeof(bus_voltage));
                std::memcpy(&bus_current, frame.data + 4, sizeof(bus_current));
                {
                    std::lock_guard<std::mutex> lock(_infos_mutex);
                    _infos[odrv_id].bus_voltage = bus_voltage;
                    _infos[odrv_id].bus_current = bus_current;
                }
                break;
            case ODriveCanID::GET_ENCODER_ESTIMATES:
                // Get_Encoder_Estimates
                float pos_estimate, vel_estimate;
                std::memcpy(&pos_estimate, frame.data, sizeof(pos_estimate));
                std::memcpy(&vel_estimate, frame.data + 4, sizeof(vel_estimate));
                {
                    std::lock_guard<std::mutex> lock(_infos_mutex);
                    _infos[odrv_id].pos_estimate = pos_estimate;
                    _infos[odrv_id].vel_estimate = vel_estimate;
                }
                break;
            case ODriveCanID::GET_TORQUES:
                // Get_Torques
                float torq_estimate;
                std::memcpy(&torq_estimate, frame.data, sizeof(torq_estimate));
                {
                    std::lock_guard<std::mutex> lock(_infos_mutex);
                    _infos[odrv_id].torq_estimate = torq_estimate;
                }
                break;
            }
        }
    }

public:
    ODriveSocket(const std::string &if_name)
    {
        _open(if_name);
        _poll_thread = std::thread(&ODriveSocket::_poll, this);
    }

    ~ODriveSocket()
    {
        _close_poll = true;
        _poll_thread.join();
        close(_socket);
    }

    uint32_t getAxisError(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].axis_error;
    }

    uint8_t getAxisState(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].axis_state;
    }

    float getPositionEstimate(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].pos_estimate;
    }

    float getVelocityEstimate(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].vel_estimate;
    }

    float getTorqueEstimate(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].torq_estimate;
    }

    float getIqSetpoint(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].iq_setpoint;
    }

    float getIqMeasured(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].iq_measured;
    }

    float getFETTemperature(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].fet_temperature;
    }

    float getMotorTemperature(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].motor_temperature;
    }

    float getBusVoltage(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].bus_voltage;
    }

    float getBusCurrent(const canid_t id)
    {
        assert(id <= MAX_CANID);
        std::lock_guard<std::mutex> lock(_infos_mutex);
        return _infos[id].bus_current;
    }

    void setAxisState(const canid_t id, const uint8_t axis_state)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::SET_AXIS_STATE);
        uint8_t data[4];
        std::memcpy(data, &axis_state, sizeof(axis_state));
        _send(arb_id, data, sizeof(data));
    }

    void setControlMode(const canid_t id, const uint32_t control_mode, const uint32_t input_mode = 0x1)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::SET_CONTROL_MODE);
        uint8_t data[8];
        std::memcpy(data, &control_mode, sizeof(control_mode));
        std::memcpy(data + 4, &input_mode, sizeof(input_mode));
        _send(arb_id, data, sizeof(data));
    }

    void setPosition(const canid_t id, const float pos_setpoint, const float vel_feedforward = 0.F, const float torq_feedforward = 0.F)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::SET_POSITION);
        const int16_t vel_ff_int = (int16_t)(vel_feedforward * 1E3F);
        const int16_t torq_ff_int = (int16_t)(torq_feedforward * 1E3F);
        uint8_t data[8];
        std::memcpy(data, &pos_setpoint, sizeof(pos_setpoint));
        std::memcpy(data + 4, &vel_ff_int, sizeof(vel_ff_int));
        std::memcpy(data + 6, &torq_ff_int, sizeof(torq_ff_int));
        _send(arb_id, data, sizeof(data));
    }

    void setVelocity(const canid_t id, const float vel_setpoint, const float torq_feedforward = 0.F)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::SET_VELOCITY);
        uint8_t data[8];
        std::memcpy(data, &vel_setpoint, sizeof(vel_setpoint));
        std::memcpy(data + 4, &torq_feedforward, sizeof(torq_feedforward));
        _send(arb_id, data, sizeof(data));
    }

    void setTorque(const canid_t id, const float torq_setpoint)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::SET_TORQUE);
        uint8_t data[4];
        std::memcpy(data, &torq_setpoint, sizeof(torq_setpoint));
        _send(arb_id, data, sizeof(data));
    }

    void setLimits(const canid_t id, const float vel_limit, const float curr_limit)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::SET_LIMITS);
        uint8_t data[8];
        std::memcpy(data, &vel_limit, sizeof(vel_limit));
        std::memcpy(data + 4, &curr_limit, sizeof(curr_limit));
        _send(arb_id, data, sizeof(data));
    }

    void setPosGain(const canid_t id, const float pos_gain)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::SET_POS_GAIN);
        uint8_t data[4];
        std::memcpy(data, &pos_gain, sizeof(pos_gain));
        _send(arb_id, data, sizeof(data));
    }

    void setVelGains(const canid_t id, const float vel_gain, const float vel_int_gain)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::SET_VEL_GAINS);
        uint8_t data[8];
        std::memcpy(data, &vel_gain, sizeof(vel_gain));
        std::memcpy(data + 4, &vel_int_gain, sizeof(vel_int_gain));
        _send(arb_id, data, sizeof(data));
    }

    void clearErrors(const canid_t id)
    {
        const uint32_t arb_id = _getArbID(id, ODriveCanID::CLEAR_ERRORS);
        uint8_t data[4];
        std::memset(data, 0, 4);
        _send(arb_id, data, sizeof(data));
    }
};