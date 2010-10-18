/*
 * ecp_g_image_switching_new.cc
 *
 *  Created on: Oct 13, 2010
 *      Author: aszymane
 */

#include "ecp_g_image_switching_test.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

ecp_g_image_switching_test::ecp_g_image_switching_test(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> vs1, boost::shared_ptr <mrrocpp::ecp::servovision::visual_servo> vs2) :
	visual_servo_manager(ecp_task, section_name)
{
	servos.push_back(vs1);
	servos.push_back(vs2);
	state=0;
}

ecp_g_image_switching_test::~ecp_g_image_switching_test()
{
}

lib::Homog_matrix ecp_g_image_switching_test::get_aggregated_position_change()
{
	// sprawdzic warunek zmiany stanu

	if(state == 0){
		return servos[0]->get_position_change(get_current_position(), dt);
	} else {
		return servos[1]->get_position_change(get_current_position(), dt);
	}
}

void ecp_g_image_switching_test::configure_all_servos()
{
	//logger::logDbg("single_visual_servo_manager::configure_all_servos()\n");
}

}//namespace generator

}//namespace common

}//namespace ecp

}//namespace mrrocpp
