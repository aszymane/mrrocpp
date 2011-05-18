/*!
 * @file
 * @brief File contains ecp robot class definition for Sarkofag
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup sarkofag
 */

#include "robot/sarkofag/ecp_r_sarkofag.h"

namespace mrrocpp {
namespace ecp {
namespace sarkofag {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp::common::robot::ecp_robot(lib::sarkofag::ROBOT_NAME, lib::sarkofag::NUM_OF_SERVOS, _config, _sr_ecp)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task_base& _ecp_object) :
	ecp::common::robot::ecp_robot(lib::sarkofag::ROBOT_NAME, lib::sarkofag::NUM_OF_SERVOS, _ecp_object)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota sarkofag.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::sarkofag::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);

}

} // namespace sarkofag
} // namespace ecp
} // namespace mrrocpp


