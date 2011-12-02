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

	fboth=fopen("/home/aszymane/workspace/mrrocpp/build/bin/zzboth.txt","w");
	if(fboth!=NULL)
		fclose(fboth);
	else
		printf("aggregated_image_switching: Failed to open file\n");
}

aggregated_image_switching::~aggregated_image_switching()
{
}

lib::Homog_matrix aggregated_image_switching::get_aggregated_position_change()
{
	lib::Homog_matrix position_change_eih = servos[0]->get_position_change(get_current_position(), get_dt());
	lib::Homog_matrix position_change_sac = servos[1]->get_position_change(get_current_position(), get_dt());

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
		state = 1;
//		dzeta = 1;
	}

	Eigen::Matrix<double, 6, 1> error_eih;
	Eigen::Matrix<double, 6, 1> error_sac;
	Eigen::Matrix<double, 6, 1> agg_error;

	if(visible_eih == 1)
		error_eih = servos[0]->get_error();
	else
	{
		error_eih(0,0)=0;
		error_eih(1,0)=0;
		error_eih(2,0)=0;
	}
	if(visible_sac == 1)
		error_sac = servos[1]->get_error();
	else
	{
		error_sac(0,0)=0;
		error_sac(1,0)=0;
		error_sac(2,0)=0;
	}

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
		agg_error(0,0)=(1-dzeta)* error_eih(0,0) + dzeta*error_sac(0,0);
		agg_error(1,0)=(1-dzeta)* error_eih(1,0) + dzeta*error_sac(1,0);
		agg_error(2,0)=(1-dzeta)* error_eih(2,0) + dzeta*error_sac(2,0);
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
		agg_error(0,0)=error_eih(0,0);
		agg_error(1,0)=error_eih(1,0);
		agg_error(2,0)=error_eih(2,0);
	}

	fboth=fopen("/home/aszymane/workspace/mrrocpp/build/bin/zzboth.txt","a");

	if(fboth==NULL)
		printf("aggregated_image_switching: Failed to open file\n");
	else{
		fprintf(fboth,"%g\t%g\t%g\t%g\t%g\t%g\t%g\t%g\t%g\n",
				error_sac(0,0),error_sac(1,0),error_sac(2,0),
				error_eih(0,0),error_eih(1,0),error_eih(2,0),
				agg_error(0,0),agg_error(1,0),agg_error(2,0));
		fflush(fboth);

		printf("%g\t%g\t%g\t%g\n",position_change(0,0),position_change(0,1),position_change(0,2),position_change(0,3));
		printf("%g\t%g\t%g\t%g\n",position_change(1,0),position_change(1,1),position_change(1,2),position_change(1,3));
		printf("%g\t%g\t%g\t%g\n",position_change(2,0),position_change(2,1),position_change(2,2),position_change(2,3));
		fflush(stdout);

		fclose(fboth);
	}
	return position_change;
	return lib::Homog_matrix();
}

void aggregated_image_switching::configure_all_servos()
{
	//logger::logDbg("aggregated_image_switching::configure_all_servos()\n");
}



}//namespace generator

}//namespace common

}//namespace ecp

}//namespace mrrocpp
