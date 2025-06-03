/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
 * @file ActuatorEffectivenessTiltingMultirotor.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Jeffrey Young <jeffrey.a.young34@gmail.com>
 */

#include "ActuatorEffectivenessDuctedDroneVTOL.hpp"

using namespace matrix;

ActuatorEffectivenessDuctedDroneVTOL::ActuatorEffectivenessDuctedDroneVTOL(ModuleParams *parent)
	: ModuleParams(parent)
{
	_servo_count_handle = param_find("CA_SV_TL_COUNT");

	updateParams();
}

void ActuatorEffectivenessDuctedDroneVTOL::updateParams()
{
	ModuleParams::updateParams();

	// if (param_get(_tilting_type_handle, &_tilting_type) != 0) {
	// 	PX4_ERR("param_get failed");
	// 	return;
	// }

	if (param_get(_servo_count_handle, &_servo_count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	// Instatiate duct_rotors of class ActuatorEffectivenessRotors and duct_servos of class ActuatorEffectivenessServos
	duct_rotors = new ActuatorEffectivenessRotorsDucted(this);
	duct_servos = new ActuatorEffectivenessControlSurfacesDucted(this);
}

bool
ActuatorEffectivenessDuctedDroneVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	// Update Effectiveness Matrix with Cyclic Pitch based on the Servo Input Configuration Parameter 
	// Note: configuration updates are only possible when disarmed.

	bool rotors_added_successfully = false;
	bool servos_added_successfully = false; 

	//Add rotors & Servos 
	configuration.selected_matrix = 0;
	rotors_added_successfully = duct_rotors->addActuators(configuration);
	servos_added_successfully = duct_servos->addActuators(configuration);
	return (rotors_added_successfully && servos_added_successfully);
}

