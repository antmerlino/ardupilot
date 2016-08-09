// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS_Control.h
/// @brief	GCS_Control manager, with EEPROM-backed storage of constants.

#ifndef __GCS_CONTROL_H__
#define __GCS_CONTROL_H__

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#define GCS_ATTITUDE_THRESHOLD 1000
#define GCS_YAWRATE_THRESHOLD 10000
#define GCS_THROTTLE_TRIM_THRESHOLD 150

#define GCS_THROTTLE_MODE_TRIM 0
#define GCS_THROTTLE_MODE_ABSOLUTE 1


/// @class	GCS_Control
/// @brief	Object managing one RC channel
class GCS_Control {
    public:
        float get_pitch(); // centi-degree
        float get_roll(); // centi-degree
        float get_yaw_rate(); // centi-degree

        float get_x_rate(void);
        float get_y_rate(void);
        float get_z_rate(void);

        int16_t get_throttle();
        int16_t get_throttle_trim();

        int8_t get_throttle_mode();

        void init_gcs_control(void);

        void set_rates(float x_rate_cms, float y_rate_cms, float z_rate_cms);

        void set_attitude(float p, float r, float yawRate);  // centi-degree

        void set_throttle(int16_t thr);
        void set_throttle_trim(int16_t trim);
        void set_throttle_mode(int8_t mode); // 0: Trimmed Throttle 1: Absolute Throttle

    private:
        float _x_rate;
        float _y_rate;
        float _z_rate;

        float _pitch;
        float _roll;
        float _yaw_rate;
        int16_t _throttle;
        int16_t _throttle_trim;

        int8_t _throttle_mode = GCS_THROTTLE_MODE_TRIM;

    protected:

};

#endif
