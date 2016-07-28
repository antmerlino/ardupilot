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
 * AP_RangeFinder_offboard.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: verge
 */

#include "RangeFinder.h"
#include "AP_RangeFinder_offboard.h"

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_offboard::AP_RangeFinder_offboard(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    set_status(RangeFinder::RangeFinder_NoData);
}

/*
   detect if an offboard rangefinder is connected. For now
   we just assume that it is connected
*/
bool AP_RangeFinder_offboard::detect(RangeFinder &_ranger, uint8_t instance)
{
    return true;
}

/*
  Do nothing, everything is done when MAVLink receives the offboard
  range finder message
 */
void AP_RangeFinder_offboard::update(void)
{
    // update range_valid state based on distance measured
    update_status();
}


