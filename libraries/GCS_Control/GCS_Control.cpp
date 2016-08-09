// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       GCS_Control.cpp - GCS Control Library
 *       Code by Anthony Merlino, Tony Samaritano. DPI
 *
 */

#include <stdlib.h>
#include <math.h>

#include "GCS_Control.h"

void GCS_Control::init_gcs_control()
{
    _x_rate = 0;
    _y_rate = 0;
    _z_rate = 0;

    _pitch = 0;
    _roll = 0;
    _yaw_rate = 0;
    _throttle = 0;

    _throttle_trim = 0.0f;
    _throttle_mode = GCS_THROTTLE_MODE_TRIM;
}

void GCS_Control::set_rates(float x_rate_cms, float y_rate_cms, float z_rate_cms){
    _x_rate = x_rate_cms;
    _y_rate = y_rate_cms;
    _z_rate = z_rate_cms;
}


float GCS_Control::get_x_rate(void){
    return _x_rate;
}

float GCS_Control::get_y_rate(void){
    return _y_rate;
}

float GCS_Control::get_z_rate(void){
    return _z_rate;
}

void GCS_Control::set_attitude(float p, float r, float yawRate){
    if(p < GCS_ATTITUDE_THRESHOLD && p > -1*GCS_ATTITUDE_THRESHOLD ){
        _pitch = p;
    } else {
        _pitch = 0;
    }

    if(r < GCS_ATTITUDE_THRESHOLD && r > -1*GCS_ATTITUDE_THRESHOLD ){
        _roll = r;
    } else {
        _roll = 0;
    }

    if(yawRate < GCS_YAWRATE_THRESHOLD && yawRate > -1*GCS_YAWRATE_THRESHOLD ){
        _yaw_rate = yawRate;
    } else {
        _yaw_rate = 0;
    }
}

float GCS_Control::get_pitch(){
    return _pitch;
}

float GCS_Control::get_roll(){
    return _roll;
}

float GCS_Control::get_yaw_rate(){
    return _yaw_rate;
}

void GCS_Control::set_throttle(float thr){
    if(thr >= 0.0f && thr <= 1.0f){
        _throttle = thr;
    }
}

void GCS_Control::set_throttle_trim(float trim){
    if(trim > -1*GCS_THROTTLE_TRIM_THRESHOLD && trim < GCS_THROTTLE_TRIM_THRESHOLD){
        _throttle_trim = trim;
    }
}

void GCS_Control::set_throttle_mode(int8_t mode){
    if(mode == GCS_THROTTLE_MODE_TRIM || mode == GCS_THROTTLE_MODE_ABSOLUTE){
        _throttle_mode = mode;
    } else {
        _throttle_mode = GCS_THROTTLE_MODE_TRIM;
        _throttle_trim = 0.0f;
    }
}

float GCS_Control::get_throttle(){
    if(_throttle_mode == GCS_THROTTLE_MODE_TRIM){
        return _throttle + _throttle_trim;
    } else {
        return _throttle;
    }
}

float GCS_Control::get_throttle_trim(){
    return _throttle_trim;
}

int8_t GCS_Control::get_throttle_mode(){
    return _throttle_mode;
}
