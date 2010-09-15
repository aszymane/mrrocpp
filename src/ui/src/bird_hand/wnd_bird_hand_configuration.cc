/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"
#include "ui/src/bird_hand/ui_r_bird_hand.h"
#include "ui/src/bird_hand/wnd_bird_hand_configuration.h"
#include "robot/bird_hand/const_bird_hand.h"
#include "ui/src/ui_class.h"

/* Local headers */
#include "../ablibs.h"
#include "../abimport.h"
#include "../gcc_ntox86/proto.h"

namespace mrrocpp {
namespace uin {
namespace bird_hand {

//
//
// KLASA UiRobotBirdHand
//
//


WndConfiguration::WndConfiguration(common::Ui& _ui, UiRobot& _bird_hand) :
	ui(_ui), bird_hand(_bird_hand), is_open(false)
{

}

int WndConfiguration::get_configuration()
{

	return 1;
}

int WndConfiguration::set_configuration()
{

	return 1;
}

int WndConfiguration::copy_command()
{

	return 1;
}

}
} //namespace uin
} //namespace mrrocpp
