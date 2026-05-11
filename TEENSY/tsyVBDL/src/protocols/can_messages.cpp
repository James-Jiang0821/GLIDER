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
Encoding:
  pack_V:       uint8 ×0.1 V     → decode: value / 10.0f
  min/max_cell: uint8 (mV−2500)/10 → decode: value * 10 + 2500
  pack_current: int8  ×0.5 A     → decode: value * 0.5f  (+ = discharge)
  temp_*:       uint8 °C         → 0xFF means invalid / not connected
------------------------------------------------------------
*/
void packStatusBMS(uint8_t *buf, const StatusBMS &m)
{
    buf[0] = m.pack_V;
    buf[1] = m.min_cell_V;
    buf[2] = m.max_cell_V;
    buf[3] = static_cast<uint8_t>(m.pack_current);
    buf[4] = m.temp_internal;
    buf[5] = m.temp_ext1;
    buf[6] = m.temp_ext2;
    buf[7] = m.sequence;
}


/*
------------------------------------------------------------
STATUS_BMS_MORE
------------------------------------------------------------
cell_V[0..5]:  uint8 (mV−2500)/10, same encoding as min/max_cell_V
online_status: 0=Unknown 1=Charging 2=FullyCharged
               3=Discharging 4=Idle 5=Fault
------------------------------------------------------------
*/
void packStatusBmsMore(uint8_t *buf, const StatusBmsMore &m)
{
    for (int i = 0; i < 6; i++) buf[i] = m.cell_V[i];
    buf[6] = m.online_status;
    buf[7] = m.sequence;
}

}