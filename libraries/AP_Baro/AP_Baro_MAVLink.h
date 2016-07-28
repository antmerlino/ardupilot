/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  dummy backend for MAVLink. This doesn't actually need to do
  any work, as handle_msg() is in the frontend
 */

#ifndef __AP_BARO_MAVLink_H__
#define __AP_BARO_MAVLink_H__

#include "AP_Baro.h"

class AP_Baro_MAVLink : public AP_Baro_Backend
{
public:
    AP_Baro_MAVLink(AP_Baro &baro);
    void update(void);
    // Get update from mavlink
    void handle_msg(mavlink_message_t *msg);

private:
    uint8_t _instance;
    float _pressure;                 // pressure in Pascal
    float _temperature;              // temperature in degrees C
    bool  _updated;
};

#endif //  __AP_BARO_MAVLink_H__
