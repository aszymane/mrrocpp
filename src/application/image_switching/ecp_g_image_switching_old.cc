/*
 * ecp_g_image_switching.cc
 *
 *  Created on: May 12, 2010
 *      Author: aszymane
 */

#include "ecp_g_image_switching.h"
#include "base/ecp/ecp_robot.h"

#include <iostream>

using namespace std;

#define MOTION_STEPS 30

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace generator {

const char ecp_g_image_switching::configSectionName[] = { "[image_switching]" };

ecp_g_image_switching::ecp_g_image_switching(mrrocpp::ecp::common::task::task & _ecp_task, ecp_mp::sensor::fradia_sensor <image_switching_types::fradia_configuration, image_switching_types::position_based_reading, image_switching_types::position_based_searching> *vspfradia)
//: mrrocpp::ecp::common::generator::generator(_ecp_task)
:
	generator(_ecp_task)//, logEnabled(true)
{

	logger::log("\nkonstruktor");
	licznik=0;
	promien=0.05;
	search=true;
	vsp_fradia=vspfradia;
	logger::log("\nvsp_fradia: %d\n",vsp_fradia);
	sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;//lib::SENSOR_CVFRADIA


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
	logger::log("ecp_g_image_switching::first_step()\n");

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 3;

	return true;
}

bool ecp_g_image_switching::next_step()
{
	logger::log("ecp_g_image_switching::next_step() poczatek\n");

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;//polozenie w xyz w macierzy 3na4
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;

	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 3;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;//polozenie od srodka postumenta

	if(licznik==0)
	{
		currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
		currentFrame.get_translation_vector(firstTransVector);//srodek okregu
		//std::cout << currentFrame << std::endl;
		licznik++;
	}

	logger::log("ecp_g_image_switching::next_step() %d\n",index);
	lib::Homog_matrix nextFrame;
	nextFrame = currentFrame;

	// pobranie danych z fradii
	vsp_fradia->get_reading();
	tracking=vsp_fradia->get_reading_message().tracking;
	found=vsp_fradia->get_reading_message().found;
	readings=vsp_fradia->get_reading_message().error;

	if(tracking)
	{
		printf("tracking\n");
		double trans_vect [3];

		/*modyfikuj nextFrame*/
		nextFrame.get_translation_vector(trans_vect);
		if(readings.z>0.2)
			readings.z=0.2;
		if(readings.y>0.2)
			readings.y=0.2;
		if(readings.x>0.2)
			readings.x=0.2;

		trans_vect[0]=trans_vect[0]-readings.x/100;
		trans_vect[1]=trans_vect[1]-readings.y/100;
		trans_vect[2]=trans_vect[2]-readings.z/100;

//		trans_vect[1]= firstTransVector[1] + 0.5*sin(0.1*licznik);
//		trans_vect[2]= firstTransVector[2] + 0.5*cos(0.1*licznik) - 0.5;// -r : aby zniwelowac podskok ze srodka okregu na okrag
//		licznik= licznik+1;

		nextFrame.set_translation_vector(trans_vect);
		/*koniec modyfikacji*/
	}
	if(found)
	{
		search=false;
		printf("Znaleziono\n");
		return false;
	}

	searching.search = search;
	vsp_fradia->set_initiate_message(searching);
	nextFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	currentFrame = nextFrame;

	return true;
}

/*void ecp_g_image_switching::log(const char *fmt, ...)
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
*/

}
}
}
}
