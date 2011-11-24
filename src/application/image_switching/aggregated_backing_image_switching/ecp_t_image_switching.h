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
#include "aggregated_backing_image_switching.h"

#include "application/visual_servoing/visual_servoing.h"



using mrrocpp::ecp::common::generator::single_visual_servo_manager;
using mrrocpp::ecp::common::generator::visual_servo_manager;
using namespace mrrocpp::ecp::servovision;

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class ecp_t_image_switching: public common::task::task
{
protected:
	boost::shared_ptr<visual_servo_regulator> reg;
	boost::shared_ptr<visual_servo_manager> sm;
	boost::shared_ptr<visual_servo_manager> sm2;
	boost::shared_ptr<visual_servo> eih;
	boost::shared_ptr<visual_servo> sac;
	boost::shared_ptr<termination_condition> term_cond;

public:
	ecp_t_image_switching(lib::configurator &config);

	void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp


#endif /* ECP_T_IMAGE_SWITCHING_H_ */
