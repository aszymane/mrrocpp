// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_postument
//
// Ostatnia modyfikacja: 23.02.2005
// zmiana: funkcja WAIT_FOR_STOP -> chyba dziala.
// autor: tkornuta
// -------------------------------------------------------------------------

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <unistd.h>
#include <fstream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"


#include "base/lib/messip/messip_dataport.h"


#include "base/lib/sr/sr_ecp.h"
#include "generator/ecp/ecp_g_delta.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

delta::delta(common::task::task& _ecp_task) :
	common::generator::generator(_ecp_task)
{
}


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
