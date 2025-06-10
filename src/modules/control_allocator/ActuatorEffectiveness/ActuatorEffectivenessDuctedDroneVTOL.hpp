/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfacesDucted.hpp"
#include "ActuatorEffectivenessRotorsDucted.hpp"

// #include "ActuatorEffectivenessTilts.hpp" // Commenting out 
#include <px4_platform_common/module_params.h>

#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/Subscription.hpp>

class ActuatorEffectivenessDuctedDroneVTOL : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessDuctedDroneVTOL(ModuleParams *parent);
	virtual ~ActuatorEffectivenessDuctedDroneVTOL() = default;

	/**
     * Populate the effectiveness matrix for cyclic pitch servos
     */
	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

    int numMatrices() const override { return 1; }

	/** The only valid Control Allocation method is Pseudo Inverse */
	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{	
		allocation_method_out[0] = AllocationMethod::PSEUDO_INVERSE;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;

	}

	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;
	
	void setFlightPhase(const FlightPhase &flight_phase) override;

	const char *name() const override { return "Ducted Drone VTOL "; }

protected:
	void updateParams() override;
	ActuatorEffectivenessRotorsDucted *duct_rotors{nullptr};
	ActuatorEffectivenessControlSurfacesDucted *duct_servos{nullptr};

	static constexpr int NUM_SERVOS_MAX = 2;

	struct ServoParamHandles{
		param_t torque_x;
    	param_t torque_y;
    	param_t torque_z;

    	param_t thrust_x;
    	param_t thrust_y;
    	param_t thrust_z;
	};
	param_t _servo_count_handle;
	int32_t _servo_count{0};

	struct ServoParam{
		matrix::Vector3f servo_torque_gain;
		matrix::Vector3f servo_thrust_gain;
	};
	
	// ServoParamHandles _servo_param_handles[NUM_SERVOS_MAX];
	// ServoParam _servo_param[NUM_SERVOS_MAX];

	uORB::Subscription _torque_sp_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _thrust_sp_sub{ORB_ID(vehicle_thrust_setpoint)};
};
