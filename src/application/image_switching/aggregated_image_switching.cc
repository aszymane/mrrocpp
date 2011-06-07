/*
 * aggregated_image_switching.cc
 *
 *  Created on: Oct 13, 2010
 *      Author: aszymane
 */

#include "aggregated_image_switching.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

aggregated_image_switching::aggregated_image_switching(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name1, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> eih, const char * section_name2, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> sac) :
	visual_servo_manager(ecp_task, section_name1)
{
	servos.push_back(eih);
	servos.push_back(sac);
	state=0;
	indeks=1.0;

	fboth=fopen("/home/aszymane/workspace/mrrocpp-git/build/bin/zzboth.txt","w");
	fclose(fboth);
}

aggregated_image_switching::~aggregated_image_switching()
{
}

lib::Homog_matrix aggregated_image_switching::get_aggregated_position_change()
{
	lib::Homog_matrix position_change_eih = servos[0]->get_position_change(get_current_position(), dt);
	lib::Homog_matrix position_change_sac = servos[1]->get_position_change(get_current_position(), dt);

	int visible_eih=servos[0]->is_object_visible();
	int visible_sac=servos[1]->is_object_visible();

	printf("czy widzi 0: %d\n", visible_eih);
	printf("czy widzi 1: %d\n", visible_sac); fflush(stdout);

	// sprawdzic warunek zmiany stanu
	if(visible_eih == 1)
	{
		state=0;
		indeks=indeks + 0.1;
		dzeta = 1.0/indeks;
	}
	else
	{
		state=1;
		dzeta = 1;
	}

	double tr_eih[3], tr_sac[3];
	position_change_eih.get_translation_vector(tr_eih);
	position_change_sac.get_translation_vector(tr_sac);

	double rms_eih = sqrt((tr_eih[0]*tr_eih[0]+tr_eih[1]*tr_eih[1]+tr_eih[2]*tr_eih[2])/3);
	double rms_sac = sqrt((tr_sac[0]*tr_sac[0]+tr_sac[1]*tr_sac[1]+tr_sac[2]*tr_sac[2])/3);
	printf("msr_eih=%g\nmsr_sac=%g\n",rms_eih,rms_sac); fflush(stdout);
	printf("indeks=%g\n",indeks); fflush(stdout);
	printf("dzeta=%g\n",dzeta); fflush(stdout);

	printf("%g\t%g\t%g\t%g\n",position_change_sac(0,0),position_change_sac(0,1),position_change_sac(0,2),position_change_sac(0,3));
	printf("%g\t%g\t%g\t%g\n",position_change_sac(1,0),position_change_sac(1,1),position_change_sac(1,2),position_change_sac(1,3));
	printf("%g\t%g\t%g\t%g\n",position_change_sac(2,0),position_change_sac(2,1),position_change_sac(2,2),position_change_sac(2,3));
	printf("%g\t%g\t%g\t%g\n",position_change_eih(0,0),position_change_eih(0,1),position_change_eih(0,2),position_change_eih(0,3));
	printf("%g\t%g\t%g\t%g\n",position_change_eih(1,0),position_change_eih(1,1),position_change_eih(1,2),position_change_eih(1,3));
	printf("%g\t%g\t%g\t%g\n",position_change_eih(2,0),position_change_eih(2,1),position_change_eih(2,2),position_change_eih(2,3));
//	printf("%g\t%g\t%g\t%g\n",position_change_sac(3,0),position_change_sac(3,1),position_change_sac(3,2),position_change_sac(3,3));

	lib::Homog_matrix position_change;
	if(visible_sac == 1)
	{
		position_change(0,0) = (1-dzeta)* position_change_eih(0,0) + dzeta*position_change_sac(0,0);
		position_change(0,1) = (1-dzeta)* position_change_eih(0,1) + dzeta*position_change_sac(0,1);
		position_change(0,2) = (1-dzeta)* position_change_eih(0,2) + dzeta*position_change_sac(0,2);
		position_change(0,3) = (1-dzeta)* position_change_eih(0,3) + dzeta*position_change_sac(0,3);
		position_change(1,0) = (1-dzeta)* position_change_eih(1,0) + dzeta*position_change_sac(1,0);
		position_change(1,1) = (1-dzeta)* position_change_eih(1,1) + dzeta*position_change_sac(1,1);
		position_change(1,2) = (1-dzeta)* position_change_eih(1,2) + dzeta*position_change_sac(1,2);
		position_change(1,3) = (1-dzeta)* position_change_eih(1,3) + dzeta*position_change_sac(1,3);
		position_change(2,0) = (1-dzeta)* position_change_eih(2,0) + dzeta*position_change_sac(2,0);
		position_change(2,1) = (1-dzeta)* position_change_eih(2,1) + dzeta*position_change_sac(2,1);
		position_change(2,2) = (1-dzeta)* position_change_eih(2,2) + dzeta*position_change_sac(2,2);
		position_change(2,3) = (1-dzeta)* position_change_eih(2,3) + dzeta*position_change_sac(2,3);
	}
	else
	{
		position_change(0,0) = position_change_eih(0,0);
		position_change(0,1) = position_change_eih(0,1);
		position_change(0,2) = position_change_eih(0,2);
		position_change(0,3) = position_change_eih(0,3);
		position_change(1,0) = position_change_eih(1,0);
		position_change(1,1) = position_change_eih(1,1);
		position_change(1,2) = position_change_eih(1,2);
		position_change(1,3) = position_change_eih(1,3);
		position_change(2,0) = position_change_eih(2,0);
		position_change(2,1) = position_change_eih(2,1);
		position_change(2,2) = position_change_eih(2,2);
		position_change(2,3) = position_change_eih(2,3);
	}
//	position_change(3,0) = (1-dzeta)* position_change_eih(3,0) + dzeta*position_change_sac(3,0);
//	position_change(3,1) = (1-dzeta)* position_change_eih(3,1) + dzeta*position_change_sac(3,1);
//	position_change(3,2) = (1-dzeta)* position_change_eih(3,2) + dzeta*position_change_sac(3,2);
//	position_change(3,3) = (1-dzeta)* position_change_eih(3,3) + dzeta*position_change_sac(3,3);

	fboth=fopen("/home/aszymane/workspace/mrrocpp-git/build/bin/zzboth.txt","a");
	fprintf(fboth,"%g\t%g\t%g\t%g\t%g\t%g\t%g\t%g\t%g\n",
			position_change_sac(0,3),position_change_sac(1,3),position_change_sac(2,3),
			position_change_eih(0,3),position_change_eih(1,3),position_change_eih(2,3),
			position_change(0,3),position_change(1,3),position_change(2,3));
	fflush(fboth);

	printf("%g\t%g\t%g\t%g\n",position_change(0,0),position_change(0,1),position_change(0,2),position_change(0,3));
	printf("%g\t%g\t%g\t%g\n",position_change(1,0),position_change(1,1),position_change(1,2),position_change(1,3));
	printf("%g\t%g\t%g\t%g\n",position_change(2,0),position_change(2,1),position_change(2,2),position_change(2,3));
//	printf("%g\t%g\t%g\t%g\n",position_change(3,0),position_change(3,1),position_change(3,2),position_change(3,3));
	fflush(stdout);

	fclose(fboth);
	return position_change;
	return lib::Homog_matrix();
}

void aggregated_image_switching::configure_all_servos()
{
	//logger::logDbg("simple_binary_image_switching::configure_all_servos()\n");
}



}//namespace generator

}//namespace common

}//namespace ecp

}//namespace mrrocpp
