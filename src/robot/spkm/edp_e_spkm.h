/*!
 * @file edp_e_spkm.h
 * @brief File containing the declaration of edp::spkm::effector class.
 *
 * @author Tomasz Winiarski
 * @date 2009
 *
 */

#ifndef __EDP_E_SPKM_H
#define __EDP_E_SPKM_H

#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include "base/edp/edp_e_manip.h"
#include "const_spkm.h"
#include "dp_spkm.h"

#include "robot/canopen/gateway.h"
#include "robot/maxon/epos.h"
#include "robot/maxon/ipm_executor.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

/*!
 * @brief class of EDP SwarmItFix parallel kinematic manipulator
 *
 * It is the base of the head mounted on the mobile base.
 */
class effector : public common::manip_effector
{
private:
	//! Access to the CAN gateway unit
	boost::shared_ptr <canopen::gateway> gateway;

	//! PKM axes.
	boost::shared_ptr <maxon::epos> axisA, axisB, axisC, axis1, axis2, axis3;

	//! Names of PKM axes.
	boost::array <std::string, mrrocpp::lib::spkm::NUM_OF_SERVOS> axesNames;

	//! Axes container.
	boost::array <maxon::epos *, mrrocpp::lib::spkm::NUM_OF_SERVOS> axes;

	//! Default axis velocity [rpm]
	static const uint32_t Vdefault[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Default axis acceleration [rpm/s]
	static const uint32_t Adefault[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Default axis deceleration [rpm/s]
	static const uint32_t Ddefault[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Maximal motor velocity [rpm].
	static const uint32_t MotorVmax[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	//! Max motor acceleration [rpm/s].
	static const uint32_t MotorAmax[mrrocpp::lib::spkm::NUM_OF_SERVOS];

	/*!
	 * \brief "Desired" joint values that were required by previously received SET command.
	 *
	 *	It is threated as current position of joints - which can be retrieved from the hardware only by the GET command.
	 *
	 */
	lib::JointArray desired_joints_old;

	//! Variable denoting whether current end-effector pose in the cartesian space is known.
	bool is_current_cartesian_pose_known;

	/*!
	 * \brief Tool transformation (SHEAD).
	 * \author tkornuta
	 */
	lib::Homog_matrix shead_frame;

	/*!
	 * \brief Desired tool frame (pose of the SHEAD tip in the PKM base reference frame).
	 * \author tkornuta
	 */
	lib::Homog_matrix desired_shead_frame;

	/*!
	 * \brief Current tool frame (pose of the SHEAD tip in the PKM base reference frame).
	 * \author tkornuta
	 */
	lib::Homog_matrix current_shead_frame;

	//! Handler for the asynchronous execution of the interpolated profile motion
	maxon::ipm_executor <lib::spkm::NUM_OF_MOTION_SEGMENTS, lib::spkm::NUM_OF_SERVOS> ipm_handler;

	/*!
	 * \brief Method initializes all SPKM variables (including motors, joints and frames), depending on working mode (robot_test_mode) and robot state.
	 * Called only once after process creation.
	 */
	void check_controller_state();

protected:
	lib::spkm::cbuffer ecp_edp_cbuffer;
	lib::spkm::rbuffer edp_ecp_rbuffer;

	/*!
	 * @brief method,  creates a list of available kinematic models for spkm effector.
	 *
	 * Here it is parallel manipulator direct and inverse kinematic transform
	 * and motor to joint transform
	 */
	virtual void create_kinematic_models_for_given_robot(void);

public:

	/*!
	 * @brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	effector(common::shell &_shell, lib::robot_name_t l_robot_name);

	/*!
	 * @brief motors synchronization
	 *
	 * This method synchronizes motors of the robots.
	 */
	void synchronise();

	/*!
	 * @brief method to create threads other then EDP master thread.
	 *
	 * Here there is only one extra thread - reader_thread.
	 */
	void create_threads();

	/*!
	 * @brief method to move robot arm
	 *
	 * it chooses the single thread variant from the manip_effector
	 */
	void move_arm(const lib::c_buffer &instruction);

	/*!
	 * \brief Method responsible for parsing of the command for motors controlling the legs and SPKM rotation.
	 * \author tkornuta
	 */
	void parse_motor_command();

	/*!
	 * \brief Method responsible for motion of motors controlling the legs and SPKM rotation.
	 * \author tkornuta
	 */
	void execute_motor_motion();

	/*!
	 * \brief Method responsible for interpolated motion in the operational space.
	 * \author tkornuta
	 */
	void interpolated_motion_in_operational_space();


	void get_controller_state(lib::c_buffer &instruction);

	/*!
	 * @brief method to get position of the arm
	 *
	 * Here it calls common::manip_effector::get_arm_position_get_arm_type_switch
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction);

	/*!
	 * @brief method to choose master_order variant
	 *
	 * IHere the single thread variant is chosen
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * @brief method to deserialize part of the reply
	 *
	 * Currently simple memcpy implementation
	 */
	void instruction_deserialization();

	/*!
	 * @brief method to serialize part of the reply
	 *
	 * Currently simple memcpy implementation
	 */
	void reply_serialization();
};

} // namespace spkm
} // namespace edp
} // namespace mrrocpp


#endif
