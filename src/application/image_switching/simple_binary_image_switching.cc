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
		mrrocpp::ecp::servovision::visual_servo> vs1, const char * section_name2, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> vs2) :
	visual_servo_manager(ecp_task, section_name1)
{
	servos.push_back(vs1);
	servos.push_back(vs2);
	state=0;
}

simple_binary_image_switching::~simple_binary_image_switching()
{
}

lib::Homog_matrix simple_binary_image_switching::get_aggregated_position_change()
{
	// sprawdzic warunek zmiany stanu
	if(servos[1]->is_object_visible())
		state=1;
	else
		state=0;

	if(state == 0){
		return servos[0]->get_position_change(get_current_position(), dt);
	} else {
		return servos[1]->get_position_change(get_current_position(), dt);
	}
}

void simple_binary_image_switching::configure_all_servos()
{
	//logger::logDbg("simple_binary_image_switching::configure_all_servos()\n");
}

}//namespace generator

}//namespace common

}//namespace ecp

}//namespace mrrocpp
