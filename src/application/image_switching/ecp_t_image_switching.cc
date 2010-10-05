/*
 * ecp_t_image_switching.cc
 *
 *  Created on: May 14, 2010
 *      Author: aszymane
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>


//#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp_t_image_switching.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

const double
		ecp_t_image_switching::initialPositionJoints[MAX_SERVOS_NR] = { 0, -0.010, -1.693, -0.075, 0.011, 4.680, -1.577, 0.090 };

//Constructors
ecp_t_image_switching::ecp_t_image_switching(lib::configurator &_config): task(_config)
{
	logger::log_enabled=true;
	logger::log("konstruktor taska\n");
	ecp_m_robot = new ecp::irp6ot_m::robot(*this);

	//delay(20000);
//smoothgen2 = new common::generator::smooth(*this, true);
	vsp_fradia = new ecp_mp::sensor::fradia_sensor<image_switching_types::fradia_configuration, image_switching_types::position_based_reading,image_switching_types::position_based_searching> (_config, "[vsp_fradia_sensor_servovision]");
	vsp_fradia->configure_sensor();

	smooth_gen = new mrrocpp::ecp::common::generator::smooth(*this, true);
	image_switching_generator = new generator::ecp_g_image_switching(*this, vsp_fradia);
	sr_ecp_msg->message("ECP loaded ecp_g_image_switching");
};

void ecp_t_image_switching::main_task_algorithm(void ) {
	//ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	//smoothgen2 = new ecp_smooth_generator(*this, true);
	//sr_ecp_msg->message("ECP loaded smooth_test");

	sr_ecp_msg->message("ecp_g_image_switching ready");

	smooth_gen->reset();
	smooth_gen->set_absolute();
	smooth_gen->load_coordinates(lib::ECP_JOINT, (double *) initialPositionJoints, true);
	smooth_gen->Move();


	sr_ecp_msg->message("ecp_g_image_switching in initialPosition");
	//smoothgen2->set_relative();

//smoothgen2->load_file_with_path("/net/koleszko/home/mm/workspace/mrrocpp/src/application/mm_test/mm_test.trj");
		/*char size[10];
		double size2 = smoothgen2->pose_list_length();
		sprintf(size,"%f",size2);
		sr_ecp_msg->message(size);*/

//smoothgen2->Move();
	image_switching_generator->Move();
	sr_ecp_msg->message("moved");
	//printf("wielkosc listy: %d\n", smoothgen2->pose_list_length());
	//fflush();

	  //smoothgen2->Move();
	  //sr_ecp_msg->message("jest git");
//smoothgen2->reset();
//	my_generator->reset();

	ecp_termination_notice();
	sr_ecp_msg->message("noticed");
};

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::ecp_t_image_switching(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


