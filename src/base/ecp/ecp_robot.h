#if !defined(_ECP_ROBOT_H)
#define _ECP_ROBOT_H

/*!
 * @file
 * @brief File contains ecp base robot declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <cerrno>

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp_mp/ecp_mp_robot.h"
#include "base/lib/single_thread_port.h"

#include "base/lib/messip/messip_dataport.h"

class ui_common_robot;

namespace mrrocpp {

namespace lib {
class sr_ecp;
struct c_buffer;
struct r_buffer;
class configurator;
}

namespace ecp {
namespace common {

namespace generator {
class transparent;
}

namespace task {
class task_base;
}

namespace robot {

/*!
 * @brief ECP robot error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ECP_error
{
public:

	/**
	 * @brief error class (type)
	 */
	const lib::error_class_t error_class;

	/**
	 * @brief error number
	 */
	const uint64_t error_no;

	/**
	 * @brief edp error structure
	 */
	lib::edp_error error;

	/**
	 * @brief constructor
	 * @param err_cl error class
	 * @param err_no error number
	 * @param err0 EDP error0 number
	 * @param err1 EDP error1 number
	 */
	ECP_error(lib::error_class_t err_cl, uint64_t err_no, uint64_t err0 = 0, uint64_t err1 = 0);
};

/*!
 * @brief ECP robot main error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ECP_main_error
{
public:

	/**
	 * @brief error class (type)
	 */
	const lib::error_class_t error_class;

	/**
	 * @brief error number
	 */
	const uint64_t error_no;

	/**
	 * @brief constructor
	 * @param err_cl error class
	 * @param err_no error number
	 */
	ECP_main_error(lib::error_class_t err_cl, uint64_t err_no);
};

/*!
 * @brief Base class of all ecp robots
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ecp_robot_base : public ecp_mp::robot
{
	// friend classes
	friend class ui_common_robot;

	/**
	 * @brief method to spawn and connect to EDP
	 *
	 * when called from Ui it first spawns then connects to EDP,\n
	 * when called from ECP it only connects to existing EDP
	 * @param config configurator of communcation channels, edp binary file name etc.
	 */
	void connect_to_edp(lib::configurator &config);

	/**
	 * @brief pid of EDP process
	 */
	pid_t EDP_MASTER_Pid; // Identyfikator procesu driver'a edp_m

	/**
	 * @brief  the EDP spawn and kill flag
	 *
	 * if the flag is set the EDP is spawned with robot object creation then killed with destruction\n
	 * it is set when UI calls robot constructor
	 */
	const bool is_created_by_ui;

public:
	/**
	 * @brief to exchange data with generators
	 *
	 * it is used in some robots
	 */
	lib::single_thread_port_manager port_manager;

	/**
	 * @brief the communication with EDP flag
	 *
	 * if the flag is set (default) the ECP communicates with EDP in Move method of generator\n
	 * Sometimes it is needed to disable communication e.g. when there is a need to communicate only With MP or VSP\n
	 * in the following iterations of Move
	 */
	bool communicate_with_edp;

	/**
	 * @brief set the command and get reply from EDP
	 *
	 * it communicates directly with EDP
	 */
	virtual void send() = 0;

	/**
	 * @brief Query EDP
	 *
	 * it first set the query flag in EDP command then calls send() method
	 */
	virtual void query() = 0;

	/**
	 * @brief reference to sr_ecp object for sending messages to UI_SR console
	 */
	lib::sr_ecp & sr_ecp_msg; // obiekt do komunikacji z SR

	/**
	 * @brief flag if the robot is synchronised or not
	 */
	bool synchronised; // Flaga synchronizacji robota (true - zsynchronizowany, false - nie)

	/**
	 * @brief nummber of servos (joints)
	 */
	const int number_of_servos;

	/**
	 * @brief the configuration file section name of associated EDP process
	 */
	std::string edp_section;

	/**
	 * @brief file descriptor of EDP communication channel
	 */
	lib::fd_client_t EDP_fd; // by Y&W

	/**
	 * @brief executed the communication sequence with EDP: set and query with error handling
	 *
	 * it can be reimplemented to maintain new error handling e.g.: in nose_run force generator
	 */
	virtual void execute_motion(void) = 0;

	/**
	 * @brief constructor called from UI
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	ecp_robot_base(const lib::robot_name_t & _robot_name, int _number_of_servos, lib::configurator &_config, lib::sr_ecp &_sr_ecp);

	/**
	 * @brief constructor called from ECP
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _ecp_object ecp tak object reference
	 */
	ecp_robot_base(const lib::robot_name_t & _robot_name, int _number_of_servos, common::task::task_base& _ecp_object);

	/**
	 * @brief checks the flag
	 * then sets the flag or throw exception. Called from create_command() method.
	 */
	void check_then_set_command_flag(bool& flag);

	/**
	 * @brief returns EDP_MASTER_Pid - EDP pid
	 */
	pid_t get_EDP_pid(void) const;

	/**
	 * @brief desctructor
	 *
	 * Closes communication channels and optionally kills EDP process
	 */
	virtual ~ecp_robot_base(void);

	/**
	 * @brief send the synchronise command to EDP
	 *
	 * Also waits for reply status of synchronization completion
	 */
	virtual void synchronise(void) = 0;

	/**
	 * @brief returns synchronised flag - synchronisation status
	 */
	bool is_synchronised(void) const;
};

template <typename ROBOT_COMMAND_T = lib::c_buffer, typename ROBOT_REPLY_T = lib::r_buffer>
class _ecp_robot : public ecp_robot_base
{
public:
	friend class ecp::common::generator::transparent;

private:
	/**
	 * @brief method to directly copy mp command to edp buffer
	 *
	 * used e.g. in transparent generator in strict coordination
	 * @param[in] mp_buffer buffer including mp command
	 */
	void copy_mp_to_edp_buffer(const ROBOT_COMMAND_T & mp_buffer)
	{
		ecp_command = mp_buffer;
	}

	/**
	 * @brief method to directly copy edp reply to mp reply
	 *
	 * used e.g. in transparent generator in strict coordination
	 * @param[out] mp_buffer buffer including mp reply
	 */
	void copy_edp_to_mp_buffer(ROBOT_REPLY_T & mp_buffer)
	{
		mp_buffer = reply_package;
	}

public:
	/**
	 * @brief constructor called from UI
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _config configuration object reference
	 * @param _sr_ecp sr_ecp communication object reference
	 */
	_ecp_robot(const lib::robot_name_t & _robot_name, int _number_of_servos, lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
			ecp_robot_base(_robot_name, _number_of_servos, _config, _sr_ecp)
	{
	}

	/**
	 * @brief constructor called from ECP
	 * @param _robot_name robot label
	 * @param _number_of_servos number of robot servos (joints)
	 * @param _edp_section associated EDP configuration file section
	 * @param _ecp_object ecp tak object reference
	 */
	_ecp_robot(const lib::robot_name_t & _robot_name, int _number_of_servos, common::task::task_base& _ecp_object) :
			ecp_robot_base(_robot_name, _number_of_servos, _ecp_object)
	{
	}

	/**
	 * @brief desctructor
	 *
	 * Calls base class destructor
	 */
	virtual ~_ecp_robot()
	{
	}

	/**
	 * @brief command to EDP
	 */
	ROBOT_COMMAND_T ecp_command;

	/**
	 * @brief Type of the command to be sent to a robot
	 */
	typedef ROBOT_COMMAND_T robot_command_t;

	/**
	 * @brief Reply from EDP
	 */
	ROBOT_REPLY_T reply_package;

	/**
	 * @brief Type of the reply from a robot
	 */
	typedef ROBOT_REPLY_T robot_reply_t;

	/**
	 * @brief set the EDP command buffer from data_port structures
	 *
	 * currently it is executed only in sporadically coordinated robots using data_ports
	 */
	virtual void create_command()
	{
	}

	/**
	 * @brief set the data_port structures from EDP reply buffer
	 *
	 * currently it is executed only in sporadically coordinated robots using data_ports
	 */
	virtual void get_reply()
	{
	}

	/**
	 * @brief executed the communication sequence with EDP: set and query with error handling
	 *
	 */
	void execute_motion(void)
	{
		send();

		if (reply_package.reply_type == lib::ERROR) {
			query();
			throw ECP_error(lib::NON_FATAL_ERROR, EDP_ERROR);
		}

		query();

		if (reply_package.reply_type == lib::ERROR) {

			throw ECP_error(lib::NON_FATAL_ERROR, EDP_ERROR);
		}
	}

	void query()
	{
		ecp_command.instruction_type = lib::QUERY;
		send(); // czyli wywolanie funkcji ecp_buffer::send, ktora jest powyzej :)
	}

	void synchronise(void)
	{
		// komunikacja wlasciwa
		ecp_command.instruction_type = lib::SYNCHRO;

		send(); // Wyslanie zlecenia synchronizacji
		query(); // Odebranie wyniku zlecenia

		synchronised = (reply_package.reply_type == lib::SYNCHRO_OK);
	}

	void send()
	{
		if (messip::port_send(EDP_fd, 0, 0, ecp_command, reply_package) == -1) {
			int e = errno; // kod bledu systemowego
			perror("ecp: Send to EDP_MASTER error");
			sr_ecp_msg.message(lib::SYSTEM_ERROR, e, "ecp: Send to EDP_MASTER error");
			throw ECP_error(lib::SYSTEM_ERROR, 0);
		}
	}

	/**
	 * @brief checks robot reply_package and detects edp_error
	 * @return edp_error occurred
	 */
	bool is_EDP_error() const
	{
		if (reply_package.error_no.error0 || reply_package.error_no.error1) {
			return true;
		} else {
			return false;
		}
	}
};

typedef _ecp_robot <> ecp_robot;

} // namespace robot
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_ROBOT_H */
