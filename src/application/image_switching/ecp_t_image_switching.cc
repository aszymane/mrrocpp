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
#include "../servovision/ecp_mp_g_visual_servo_tester.h"
#include "../servovision/defines.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"


using namespace mrrocpp::ecp::common::generator;
using namespace logger;

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

//const double
//		ecp_t_image_switching::initialPositionJoints[MAX_SERVOS_NR] = { 0, -0.010, -1.693, -0.075, 0.011, 4.680, -1.577, 0.090 };

//Constructors
ecp_t_image_switching::ecp_t_image_switching(mrrocpp::lib::configurator& configurator) :
	common::task::task(configurator) 
{
	logger::log_enabled = true;
	logger::log_dbg_enabled = true;
//	ecp_m_robot = new mrrocpp::ecp::irp6p_m::robot(*this);
	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);	


//	smooth_gen = new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, 6);
	sr_ecp_msg->message("ECP loaded ecp_g_image_switching");

//	char config_section_name1[] = { "[object_follower_sac_1]" };
//	char config_section_name2[] = { "[object_follower_pb]" };
	char config_section_name1[] = { "[object_follower_pb]" };
	char config_section_name2[] = { "[object_follower_sac_1]" };

	boost::shared_ptr<position_constraint> cube(new cubic_constraint(config, config_section_name1));

	//		log_dbg("ecp_t_objectfollower_ib::ecp_t_objectfollower_ib(): 1\n");
	reg = boost::shared_ptr<visual_servo_regulator> (new regulator_p(config, config_section_name1));
	//		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 2\n");
//	sac = boost::shared_ptr<visual_servo> (new pb_eih_visual_servo(reg, config_section_name1, config));
//	eih = boost::shared_ptr<visual_servo> (new pb_sac_visual_servo(reg, config_section_name2, config));
	eih = boost::shared_ptr<visual_servo> (new pb_eih_visual_servo(reg, config_section_name1, config));
	sac = boost::shared_ptr<visual_servo> (new pb_sac_visual_servo(reg, config_section_name2, config));

//	term_cond = boost::shared_ptr<termination_condition> (
//			new object_reached_termination_condition(config, config_section_name1));
	//		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 3\n");
//	sm = boost::shared_ptr<visual_servo_manager> (new single_visual_servo_manager( *this, config_section_name2, sac));
	sm = boost::shared_ptr<visual_servo_manager> (new simple_binary_image_switching( *this,
			config_section_name1, eih, config_section_name2, sac));
//	sm = boost::shared_ptr<visual_servo_manager> (new aggregated_image_switching( *this, config_section_name1, eih, config_section_name2, sac));
	//		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 4\n");
//	sm->add_position_constraint(cube);
//	sm->add_termination_condition(term_cond);
	//		log_dbg("ecp_t_objectfollower_pb::ecp_t_objectfollower_pb(): 5\n");
//	sm->configure();
	sm->configure();

}
;

void ecp_t_image_switching::main_task_algorithm(void) {

	sr_ecp_msg->message("ecp_g_image_switching ready");
	std::vector<double> coordinates1(6);
//
//	smooth_gen->reset();
//	smooth_gen->set_absolute();
//
//	coordinates1[0] = 0.922;
//	coordinates1[1] = -0.000;
//	coordinates1[2] = 0.268;
//	coordinates1[3] = -2.209;
//	coordinates1[4] = -0.000;
//	coordinates1[5] = -2.233;
//	smooth_gen->load_absolute_angle_axis_trajectory_pose(coordinates1);
//
//	smooth_gen->set_debug(false);
//	if (smooth_gen->calculate_interpolate())
//		smooth_gen->Move();

	sr_ecp_msg->message("ecp_g_image_switching in initialPosition");

	sm->Move();
	sr_ecp_msg->message("moved");

	ecp_termination_notice();
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


