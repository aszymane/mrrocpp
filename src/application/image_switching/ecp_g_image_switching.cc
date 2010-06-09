/*
 * ecp_g_image_switching.cc
 *
 *  Created on: May 12, 2010
 *      Author: aszymane
 */

#include "ecp_g_image_switching.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 30

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace generator {

const char ecp_g_image_switching::configSectionName[] = { "[image_switching]" };

ecp_g_image_switching::ecp_g_image_switching(mrrocpp::ecp::common::task::task & _ecp_task)
//: mrrocpp::ecp::common::generator::generator(_ecp_task)
:
	generator(_ecp_task), logEnabled(true)
{
	licznik=0;
	promien=0.05;


//	char kp_name[] = { "kp" };
//	char maxt_name[] = { "maxt" };
//
//	if (_ecp_task.config.exists(std::string(kp_name), std::string(configSectionName))) {
//		Kp = _ecp_task.config.value <double> (kp_name, configSectionName);
//	} else {
//		Kp = 0.0001;
//		log("Parameter %s->%s not found. Using default value: %g\n", configSectionName, kp_name);
//	}
//
//	if (_ecp_task.config.exists(std::string(maxt_name), std::string(configSectionName))) {
//		maxT = _ecp_task.config.value <double> (maxt_name, configSectionName);
//	} else {
//		maxT = 0.01;
//		log("Parameter %s->%s not found. Using default value: %g\n", configSectionName, maxt_name, maxT);
//	}
//
//	log("\nKp: %g; maxT: %g\n", Kp, maxT);
}

ecp_g_image_switching::~ecp_g_image_switching()
{
}

bool ecp_g_image_switching::first_step()
{
	log("ecp_g_image_switching::first_step()\n");

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 3;

//	for (int i = 0; i < 6; i++) {
//		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
//	}

//	the_robot->ecp_command.command = lib::NEXT_POSE;
//	the_robot->ecp_command.instruction.instruction_type = lib::GET;
//	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
//	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
//	the_robot->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
//	the_robot->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
//	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
//	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
//	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
//	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
//
//	for (int i=0; i<3; i++) {
//		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
//		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
//		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i+3] = 0;
//		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i+3] = 0;
//		the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
//		the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
//		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
//		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i+3] = lib::UNGUARDED_MOTION;
//		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = FORCE_INERTIA;
//		the_robot->ecp_command.instruction.arm.pf_def.inertia[i+3] = TORQUE_INERTIA;
//	}
//	the_robot->ecp_command.instruction.arm.pf_def.behaviour[2] = lib::CONTACT;
//	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[2] = 12.5;
//
//	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
//	
	return true;
}

bool ecp_g_image_switching::next_step()
{
	log("ecp_g_image_switching::next_step()\n");

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;//polozenie w xyz w macierzy 3na4
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;

	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;//polozenie od srodka postumenta

	vsp_fradia->get_reading();
	tracking=vsp_fradia->received_object.tracking;
	readings=vsp_fradia->received_object.error;

	if(licznik==0)
	{
		currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		currentFrame.get_translation_vector(firstTransVector);//srodek okregu
		//std::cout << currentFrame << std::endl;
		licznik++;
	}

	log("ecp_g_image_switching::next_step() %d\n",index);
	lib::Homog_matrix nextFrame;
	nextFrame = currentFrame;

	if(tracking)
	{
		printf("tracking\n");
		double trans_vect [3];

		/*modyfikuj nextFrame*/
		nextFrame.get_translation_vector(trans_vect);
		if(readings.z>2)
			readings.z=2;
		if(readings.y>2)
			readings.y=2;
		if(readings.x>2)
			readings.x=2;

		trans_vect[0]=trans_vect[0]+readings.x/100;
		trans_vect[1]=trans_vect[1]+readings.y/100;
		trans_vect[2]=trans_vect[2]+readings.z/100;

//		trans_vect[1]= firstTransVector[1] + 0.5*sin(0.1*licznik);
//		trans_vect[2]= firstTransVector[2] + 0.5*cos(0.1*licznik) - 0.5;// -r : aby zniwelowac podskok ze srodka okregu na okrag
//		licznik= licznik+1;

		nextFrame.set_translation_vector(trans_vect);
		/*koniec modyfikacji*/
	}

	nextFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	currentFrame = nextFrame;

	return true;
}

void ecp_g_image_switching::log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	if (!logEnabled) {
		va_end(ap);
		return;
	}

	vfprintf(stdout, fmt, ap);
	fflush(stdout);
	va_end(ap);
}

}
}
}
}
