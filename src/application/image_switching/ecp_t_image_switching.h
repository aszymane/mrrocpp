/*
 * ecp_t_image_switching.h
 *
 *  Created on: May 14, 2010
 *      Author: aszymane
 */

#ifndef ECP_T_IMAGE_SWITCHING_H_
#define ECP_T_IMAGE_SWITCHING_H_

#include "base/ecp/ecp_task.h"
#include <boost/shared_ptr.hpp>
#include "generator/ecp/ecp_g_newsmooth.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
//#include "base/ecp/ecp_g_smooth.h"
//#include "src/application/servovision/visual_servo_manager.h"
//#include "../servovision/visual_servo_manager.h"
//#include "ecp_g_image_switching_old.h"
//#include "ecp_g_image_switching.h"
#include "simple_binary_image_switching.h"


#include "../servovision/single_visual_servo_manager.h"
#include "../servovision/pb_eih_visual_servo.h"
#include "../servovision/pb_sac_visual_servo.h"
#include "../servovision/cubic_constraint.h"
#include "../servovision/object_reached_termination_condition.h"
#include "../servovision/visual_servo_regulator_p.h"


using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using namespace mrrocpp::ecp::servovision;
using boost::shared_ptr;

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class ecp_t_image_switching: public common::task::task {

  protected:
	 // common::generator::smooth* smoothgen2;
//	  generator::ecp_g_image_switching* image_switching_generator;
//	  generator::visual_servo_manager* visual_servo_gen;
//	  mrrocpp::ecp::common::generator::smooth* smooth_gen;
	  common::generator::newsmooth* smooth_gen;

//	  ecp_mp::sensor::fradia_sensor<image_switching_types::fradia_configuration, image_switching_types::position_based_reading, image_switching_types::position_based_searching> *vsp_fradia;

//	  static const double initialPositionJoints[MAX_SERVOS_NR];

		shared_ptr<visual_servo_regulator> reg;
		shared_ptr<visual_servo_manager> sm;
		shared_ptr<visual_servo> vs1;
		shared_ptr<visual_servo> vs2;
		shared_ptr<termination_condition> term_cond;

	public:
	  ecp_t_image_switching(lib::configurator &_config);

		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif /* ECP_T_IMAGE_SWITCHING_H_ */
