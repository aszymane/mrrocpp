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

#include "ecp_t_image_switching.h"
#include "../../visual_servoing_demo/ecp_mp_g_visual_servo_tester.h"
//#include "../../visual_servoing_demo/defines.h"
//#include "../visual_servoing/position_constraint.h"

#include "sensor/discode/discode_sensor.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"

using namespace mrrocpp::ecp::common::generator;
using namespace logger;
using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

//Constructors
ecp_t_image_switching::ecp_t_image_switching(
		mrrocpp::lib::configurator& configurator) :
	common::task::task(configurator) {
	try {
		logger::log_enabled = true;
		logger::log_dbg_enabled = true;
		ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(
				*this);

		sr_ecp_msg->message("ECP loaded ecp_g_image_switching");

		//	char config_section_name1[] = { "[object_follower_sac_1]" };
		//	char config_section_name2[] = { "[object_follower_pb]" };
		char config_section_name1[] = { "[object_follower_pb]" };
		char config_section_name2[] = { "[object_follower_sac_1]" };

		boost::shared_ptr<position_constraint> cube(
				new cubic_constraint(config, config_section_name1));

		reg = boost::shared_ptr<visual_servo_regulator>(
				new regulator_p(config, config_section_name1));

		boost::shared_ptr<discode_sensor> ds1 = boost::shared_ptr<
				discode_sensor>(
				new discode_sensor(config, config_section_name1));
		eih
				= boost::shared_ptr<visual_servo>(
						new pb_eih_visual_servo(reg, ds1, config_section_name1,
								config));

		boost::shared_ptr<discode_sensor> ds2 = boost::shared_ptr<
				discode_sensor>(
				new discode_sensor(config, config_section_name2));
		sac
				= boost::shared_ptr<visual_servo>(
						new pb_sac_visual_servo(reg, ds2, config_section_name2,
								config));

		sm = boost::shared_ptr<visual_servo_manager>(new aggregated_image_switching(*this, config_section_name1, eih, config_section_name2, sac));
		sm->add_position_constraint(cube);
		sm->configure();
	} catch (std::exception& ex) {
		sr_ecp_msg->message(lib::FATAL_ERROR,
				std::string("ERROR in ecp_t_image_switching: ") + ex.what());
		throw ex;
	}
	log_dbg("ecp_t_image_switching: initialization completed.\n");
}
;

void ecp_t_image_switching::main_task_algorithm(void) {

	sr_ecp_msg->message("ecp_g_image_switching ready");
	std::vector<double> coordinates1(6);

	sr_ecp_msg->message("ecp_g_image_switching in initialPosition");

	sm->Move();
	sr_ecp_msg->message("moved");

	//	ecp_termination_notice();
	sr_ecp_msg->message("noticed");
}
;

}
} // namespace irp6p

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &config) {
	return new irp6p::task::ecp_t_image_switching(config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


