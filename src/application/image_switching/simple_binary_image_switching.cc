/*
 * simple_binary_image_switching.cc
 *
 *  Created on: Oct 13, 2010
 *      Author: aszymane
 */

#include "simple_binary_image_switching.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

simple_binary_image_switching::simple_binary_image_switching(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name1, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> eih, const char * section_name2, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> sac) :
	visual_servo_manager(ecp_task, section_name1)
{
	servos.push_back(eih);
	servos.push_back(sac);
	state=0;
	printf("aaa\n");
}

simple_binary_image_switching::~simple_binary_image_switching()
{
	printf("ddd\n");
}

lib::Homog_matrix simple_binary_image_switching::get_aggregated_position_change()
{
	printf("bbb\n"); fflush(stdout);
	// sprawdzic warunek zmiany stanu
	if(servos[0]->is_object_visible())
		state=0;
	else
		state=1;
	printf("bbb2\n"); fflush(stdout);

	printf("czy widzi 0: %d\n", (int)servos[0]->is_object_visible());
	printf("czy widzi 1: %d\n", (int)servos[1]->is_object_visible());
	fflush(stdout);

	if(state == 0){
		printf("Z 0\n"); fflush(stdout);
		return servos[0]->get_position_change(get_current_position(), dt);
	} else { // state == 1
		printf("Z 1\n"); fflush(stdout);
		get_current_position();
		printf("Z 12\n"); fflush(stdout);
		const Eigen::Matrix <double, 6, 1> err = servos[1]->get_error();
//		printf("e: %d, %d, %d\n",err(1,1),err(2,1), err(3,1));
		printf("Z 13\n"); fflush(stdout);
//		lib::Homog_matrix servoss=;

		return servos[1]->get_position_change(get_current_position(), dt);
	}
	return lib::Homog_matrix();
}

void simple_binary_image_switching::configure_all_servos()
{
	printf("ccc\n");
	//logger::logDbg("simple_binary_image_switching::configure_all_servos()\n");
}



}//namespace generator

}//namespace common

}//namespace ecp

}//namespace mrrocpp
