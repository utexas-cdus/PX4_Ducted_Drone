/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/log.h>

#include "ActuatorEffectivenessControlSurfacesDucted.hpp"

using namespace matrix;

ActuatorEffectivenessControlSurfacesDucted::ActuatorEffectivenessControlSurfacesDucted(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < MAX_COUNT; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", i);
		_param_handles[i].type = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRQ_R", i);
		_param_handles[i].torque[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRQ_P", i);
		_param_handles[i].torque[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRQ_Y", i);
		_param_handles[i].torque[2] = param_find(buffer);
		// snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRIM", i);
		// _param_handles[i].trim = param_find(buffer);

        snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TX", i);
        _param_handles[i].thrust[0] = param_find(buffer);
        snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TY", i);
        _param_handles[i].thrust[1] = param_find(buffer);
        snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TZ", i);
        _param_handles[i].thrust[2] = param_find(buffer);
	}

	_count_handle = param_find("CA_SV_CS_COUNT");
	updateParams();
}

void ActuatorEffectivenessControlSurfacesDucted::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_count = count;

	for (int i = 0; i < _count; i++) {
		param_get(_param_handles[i].type, (int32_t *)&_params[i].type);

		Vector3f &torque = _params[i].torque;
        Vector3f &thrust = _params[i].thrust; 

		for (int n = 0; n < 3; ++n) {
			param_get(_param_handles[i].torque[n], &torque(n));
            param_get(_param_handles[i].thrust[n], &thrust(n));            
		}

		// param_get(_param_handles[i].trim, &_params[i].trim);

		// TODO: enforce limits (note that tailsitter uses different limits)?
		switch (_params[i].type) {

		case Type::LeftAileron:
			break;

		case Type::RightAileron:
			break;

		case Type::Elevator:
			break;

		case Type::Rudder:
			break;

		case Type::LeftElevon:
			break;

		case Type::RightElevon:
			break;

		case Type::LeftVTail:
			break;

		case Type::RightVTail:
			break;

		case Type::LeftFlaps:
		case Type::RightFlaps:
			torque.setZero();
			break;

		case Type::Airbrakes:
			torque.setZero();
			break;

		case Type::Custom:
			break;
		}
	}
}

bool ActuatorEffectivenessControlSurfacesDucted::addActuators(Configuration &configuration)
{
	for (int i = 0; i < _count; i++) {
		(void)configuration.addActuator(ActuatorType::SERVOS, _params[i].torque, _params[i].thrust);
		// No Trim Condition defined yet 
		// if (actuator_idx >= 0) {
		// 	configuration.trim[configuration.selected_matrix](actuator_idx) = _params[i].trim;
		// }
	}

	return true;
}

// Commenting out below because Ducted Drone does not have Flaps or Air Brakes 
// void ActuatorEffectivenessControlSurfacesDucted::applyFlapsAndAirbrakes(float flaps_control, float airbrakes_control,
// 		int first_actuator_idx,
// 		ActuatorVector &actuator_sp) const
// {
// 	for (int i = 0; i < _count; ++i) {
// 		switch (_params[i].type) {
// 		// TODO: check sign
// 		case ActuatorEffectivenessControlSurfacesDucted::Type::LeftFlaps:
// 			actuator_sp(i + first_actuator_idx) += flaps_control;
// 			break;

// 		case ActuatorEffectivenessControlSurfacesDucted::Type::RightFlaps:
// 			actuator_sp(i + first_actuator_idx) -= flaps_control;
// 			break;

// 		case ActuatorEffectivenessControlSurfacesDucted::Type::Airbrakes:
// 			actuator_sp(i + first_actuator_idx) += airbrakes_control;
// 			break;

// 		default:
// 			break;
// 		}
// 	}

// }
