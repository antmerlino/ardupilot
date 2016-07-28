/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_Baro.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_Baro_MAVLink::AP_Baro_MAVLink(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
    _updated = false;
}

void AP_Baro_MAVLink::update(void){
    if(_updated){
        _updated = false;
        _copy_to_frontend(_instance, _pressure, _temperature);
    }
}

// ==========================================================================
// based on tables.cpp from http://www.pdas.com/atmosdownload.html

/* 
   Compute the temperature, density, and pressure in the standard atmosphere
   Correct to 20 km.  Only approximate thereafter.
*/
void SimpleAtmosphere(
	const float alt,                           // geometric altitude, km.
	float& sigma,                   // density/sea-level standard density
	float& delta,                 // pressure/sea-level standard pressure
	float& theta)           // temperature/sea-level standard temperature
{
    const float REARTH = 6369.0f;        // radius of the Earth (km)
    const float GMR    = 34.163195f;     // gas constant
    float h=alt*REARTH/(alt+REARTH);     // geometric to geopotential altitude

    if (h < 11.0f) {
        // Troposphere
        theta=(288.15f-6.5f*h)/288.15f;
        delta=powf(theta, GMR/6.5f);
    } else {
        // Stratosphere
        theta=216.65f/288.15f;
        delta=0.2233611f*expf(-GMR*(h-11.0f)/216.65f);
    }

    sigma = delta/theta;
}

/*
   Set the altitude/pressure/temperature based on a MAVLINK message
*/
void AP_Baro_MAVLink::handle_msg(mavlink_message_t *msg)
{
    mavlink_altitude_sensor_t packet;
    mavlink_msg_altitude_sensor_decode(msg, &packet);

    if(packet.fromAltitude){
        float sigma, delta, theta;
        const float p0 = 101325;

        SimpleAtmosphere(packet.altitude*0.001f, sigma, delta, theta);
        float p = p0 * delta;
        float T = 303.16f * theta - 273.16f; // Assume 30 degrees at sea level - converted to degrees Kelvin

        _pressure = p;
        _temperature = T;
    } else {
        _pressure = packet.pressure;
        _temperature = packet.temperature;
    }

    _updated = true;
}
