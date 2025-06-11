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
	: ModuleParams(parent), _rotors(this), _control_surfaces(this)
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

	// Adding in unused code for StdVTOL Config
		if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	configuration.selected_matrix = 0;
	_rotors.enablePropellerTorqueNonUpwards(false);
	const bool mc_rotors_added_successfully = _rotors.addActuators(configuration);
	_upwards_motors_mask = _rotors.getUpwardsMotors();
	_forwards_motors_mask = _rotors.getForwardsMotors();

	// Control Surfaces
	configuration.selected_matrix = 1;
	_first_control_surface_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);
}

// Adding in Extra Functions not needed for Ducted Drone Vehicle Configuration but might effect SWIL Capabilites with jMAVSim 

void ActuatorEffectivenessDuctedDroneVTOL::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	if (matrix_index == 1) {
		// apply flaps
		normalized_unsigned_setpoint_s flaps_setpoint;

		if (_flaps_setpoint_sub.copy(&flaps_setpoint)) {
			_control_surfaces.applyFlaps(flaps_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}

		// apply spoilers
		normalized_unsigned_setpoint_s spoilers_setpoint;

		if (_spoilers_setpoint_sub.copy(&spoilers_setpoint)) {
			_control_surfaces.applySpoilers(spoilers_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}
	}
}

void ActuatorEffectivenessDuctedDroneVTOL::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	if (matrix_index == 0) {
		stopMaskedMotorsWithZeroThrust(_forwards_motors_mask, actuator_sp);
	}
}

void ActuatorEffectivenessDuctedDroneVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// update stopped motors
	switch (flight_phase) {
	case FlightPhase::FORWARD_FLIGHT:
		_stopped_motors_mask |= _upwards_motors_mask;
		break;

	case FlightPhase::HOVER_FLIGHT:
	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF:
		_stopped_motors_mask &= ~_upwards_motors_mask;
		break;
	}
}