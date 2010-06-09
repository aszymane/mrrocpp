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
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_t_image_switching.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Constructors
ecp_g_image_switching::ecp_g_image_switching(lib::configurator &_config): task(_config)
{
	ecp_m_robot = new ecp::irp6ot_m::robot(*this);

	//delay(20000);
//smoothgen2 = new common::generator::smooth(*this, true);
	my_generator = new generator::ecp_g_image_switching(*this);
	sr_ecp_msg->message("ECP loaded ecp_g_image_switching");
};

void ecp_g_image_switching::main_task_algorithm(void ) {
	//ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	//smoothgen2 = new ecp_smooth_generator(*this, true);
	//sr_ecp_msg->message("ECP loaded smooth_test");

	sr_ecp_msg->message("ecp_g_image_switching ready");

	//smoothgen2->set_relative();

//smoothgen2->load_file_with_path("/net/koleszko/home/mm/workspace/mrrocpp/src/application/mm_test/mm_test.trj");
		/*char size[10];
		double size2 = smoothgen2->pose_list_length();
		sprintf(size,"%f",size2);
		sr_ecp_msg->message(size);*/

//smoothgen2->Move();
	my_generator->Move();
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
	return new irp6ot::task::ecp_g_image_switching(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


