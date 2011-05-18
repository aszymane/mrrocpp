#if !defined(_SMB_CONST_H)
#define _SMB_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "dp_smb.h"

#include "base/lib/impconst.h"

namespace mrrocpp {
namespace lib {
namespace smb {

/*!
 * @brief SwarmItFix Mobile Base robot label
 * @ingroup smb
 */
const robot_name_t ROBOT_NAME = "smb";

/*!
 * @brief SwarmItFix Mobile Base EDP command buffer variant enum
 * @ingroup smb
 */
enum CBUFFER_VARIANT
{
	CBUFFER_EPOS_CUBIC_COMMAND, CBUFFER_EPOS_TRAPEZOIDAL_COMMAND, CBUFFER_PIN_INSERTION, CBUFFER_PIN_LOCKING
};

/*!
 * @brief SwarmItFix Mobile Base EDP command buffer
 * @ingroup smb
 */
struct cbuffer
{
	CBUFFER_VARIANT variant;
	union
	{
		epos::epos_cubic_command epos_cubic_command_structure;
		epos::epos_trapezoidal_command epos_trapezoidal_command_structure;
		epos::epos_operational_command epos_operational_command_structure;
		multi_pin_insertion_td multi_pin_insertion;
		multi_pin_locking_td multi_pin_locking;
	};
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Mobile Base EDP reply buffer
 * @ingroup smb
 */
struct rbuffer
{
	multi_leg_reply_td multi_leg_reply;
	epos::single_controller_epos_reply epos_controller[lib::smb::NUM_OF_SERVOS];
}__attribute__((__packed__));

} // namespace smb
} // namespace lib
} // namespace mrrocpp

#endif /* _SMB_CONST_H */
