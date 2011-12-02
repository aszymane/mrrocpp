/*
 * ternary_changing_image_switching.h
 *
 *  Created on: Oct 13, 2010
 *      Author: aszymane
 */

#ifndef TERNARY_CHANGING_IMAGE_SWITCHING_H_
#define TERNARY_CHANGING_IMAGE_SWITCHING_H_

#include "../../visual_servoing/visual_servo_manager.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class ternary_changing_image_switching : public visual_servo_manager
{
public:
	ternary_changing_image_switching(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name1, boost::shared_ptr <
					mrrocpp::ecp::servovision::visual_servo> vs1, const char * section_name2, boost::shared_ptr <
					mrrocpp::ecp::servovision::visual_servo> vs2);
	virtual ~ternary_changing_image_switching();
protected:
	lib::Homog_matrix get_aggregated_position_change();
	void configure_all_servos();
	int state;
	FILE *feih, *fsac, *fboth;
};

/** @} */

}//namespace generator

}

}

}

#endif /* TERNARY_CHANGING_IMAGE_SWITCHING_H_ */
