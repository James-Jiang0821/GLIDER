#include "protocols/can_messages.hpp"

namespace can
{

/*
------------------------------------------------------------
STATUS_FAULT
------------------------------------------------------------
*/
void packStatusFault(uint8_t *buf, const StatusFault &m)
{
    buf[0] = m.hard_fault_a;
    buf[1] = m.hard_fault_b;
    buf[2] = m.soft_fault;
    buf[3] = 0;

    buf[4] = m.first_hard_fault;
    buf[5] = m.state;

    buf[6] = m.sequence;
    buf[7] = 0;
}


/*
------------------------------------------------------------
CMD_SETPOINT
------------------------------------------------------------
*/
void packCmdSetpoint(uint8_t *buf, const CmdSetpoint &m)
{
    buf[0] = m.stroke_percent;

    buf[1] = 0;
    buf[2] = 0;

    buf[3] = m.command_mode;

    buf[4] = 0;
    buf[5] = 0;

    buf[6] = m.sequence;

    buf[7] = 0;
}

void unpackCmdSetpoint(const uint8_t *buf, CmdSetpoint &m)
{
    m.stroke_percent = buf[0];

    m.command_mode = buf[3];

    m.sequence = buf[6];
}


/*
------------------------------------------------------------
STATUS_CONTROL
------------------------------------------------------------
*/
void packStatusControl(uint8_t *buf, const StatusControl &m)
{
    buf[0] = m.est_position_01mm & 0xFF;
    buf[1] = m.est_position_01mm >> 8;

    buf[2] = m.tof_position_mm & 0xFF;
    buf[3] = m.tof_position_mm >> 8;

    buf[4] = m.flags;
    buf[5] = m.pwm;

    buf[6] = m.sequence;

    buf[7] = static_cast<uint8_t>(m.motor_current);
}


/*
------------------------------------------------------------
STATUS_BMS
------------------------------------------------------------
*/
void packStatusBMS(uint8_t *buf, const StatusBMS &m)
{
    buf[0] = m.pack_mV & 0xFF;
    buf[1] = m.pack_mV >> 8;

    buf[2] = m.pack_temp & 0xFF;
    buf[3] = m.pack_temp >> 8;

    buf[4] = m.bms_fault_id;   // raw TinyBMS fault code

    buf[5] = 0;

    buf[6] = m.sequence;

    buf[7] = 0;
}

}