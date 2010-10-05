/*
 * ecp_t_image_switching.h
 *
 *  Created on: May 14, 2010
 *      Author: aszymane
 */

#ifndef ECP_T_IMAGE_SWITCHING_H_
#define ECP_T_IMAGE_SWITCHING_H_

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_smooth.h"
#include "ecp_g_image_switching.h"
#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class ecp_t_image_switching: public common::task::task {

  protected:
	 // common::generator::smooth* smoothgen2;
	  generator::ecp_g_image_switching* image_switching_generator;
	  mrrocpp::ecp::common::generator::smooth* smooth_gen;

	  ecp_mp::sensor::fradia_sensor<image_switching_types::fradia_configuration, image_switching_types::position_based_reading, image_switching_types::position_based_searching> *vsp_fradia;

	  static const double initialPositionJoints[MAX_SERVOS_NR];

	public:
	  ecp_t_image_switching(lib::configurator &_config);

		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif /* ECP_T_IMAGE_SWITCHING_H_ */
