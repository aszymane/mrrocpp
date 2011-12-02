#if !defined(_ECP_R_SPKM_H)
#define _ECP_R_SPKM_H

/*!
 * @file
 * @brief File contains ecp robot class declaration for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include <boost/shared_ptr.hpp>

#include "base/ecp/ecp_robot.h"
#include "base/ecp/ecp_task.h"

#include "base/lib/agent/InputBuffer.h"
#include "base/lib/agent/RemoteAgent.h"

#include "base/kinematics/kinematics_manager.h"

#include "base/lib/swarmtypes.h"
#include "dp_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine gripper ecp robot class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup spkm
 */
class robot : public common::robot::ecp_robot, public kinematics::common::kinematics_manager
{
private:
	void create_kinematic_models_for_given_robot(void);

	/**
	 * @brief EDP command buffer
	 */
	lib::spkm::cbuffer ecp_edp_cbuffer;

	/**
	 * @brief EDP reply buffer
	 */
	lib::spkm::rbuffer edp_ecp_rbuffer;

public:
	/**
	 * @brief epos motor motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_simple_command> epos_motor_command_data_port;

	/**
	 * @brief epos joint motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_simple_command> epos_joint_command_data_port;

	/**
	 * @brief epos external motion command data port
	 */
	lib::single_thread_port <lib::epos::epos_simple_command> epos_external_command_data_port;

	/**
	 * @brief epos brake command data port
	 */
	lib::single_thread_port <bool> epos_brake_command_data_port;

	/**
	 * @brief epos brake command data port
	 */
	lib::single_thread_port <bool> epos_clear_fault_data_port;

	/**
	 * @brief epos motion status reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> epos_motor_reply_data_request_port;

	/**
	 * @brief epos motion status with joint reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> epos_joint_reply_data_request_port;

	/**
	 * @brief epos motion status with external reply data request port
	 */
	lib::single_thread_request_port <lib::epos::epos_reply> epos_external_reply_data_request_port;

	/**
	 * @brief constructor called from UI
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	robot(const lib::robot_name_t & _robot_name, lib::configurator &_config, lib::sr_ecp &_sr_ecp);

	/**
	 * @brief constructor called from ECP
	 * @param _ecp_object ecp tak object reference
	 */
	robot(const lib::robot_name_t & _robot_name, common::task::task_base& _ecp_object);

	/**
	 * @brief set the edp command buffer
	 * basing on data_ports
	 */
	void create_command();

	/**
	 * @brief set the data_request_ports
	 * basing on edp reply buffer
	 */
	void get_reply();

	/**
	 * Input buffer for MP commands
	 */
	boost::shared_ptr<InputBuffer<lib::spkm::next_state_t> > nextstateBuffer;

	/**
	 * Output buffer for MP notifications
	 */
	boost::shared_ptr<OutputBuffer<lib::notification_t> > notifyBuffer;
};
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

#endif
