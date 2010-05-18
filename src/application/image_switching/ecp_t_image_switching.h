/*
 * ecp_t_image_switching.h
 *
 *  Created on: May 14, 2010
 *      Author: aszymane
 */

#ifndef ECP_T_IMAGE_SWITCHING_H_
#define ECP_T_IMAGE_SWITCHING_H_

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp_g_image_switching.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class ecp_g_image_switching: public common::task::task {

  protected:
	 // common::generator::smooth* smoothgen2;
	  generator::ecp_g_image_switching* my_generator;

	public:
	  ecp_g_image_switching(lib::configurator &_config);

		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif /* ECP_T_IMAGE_SWITCHING_H_ */
