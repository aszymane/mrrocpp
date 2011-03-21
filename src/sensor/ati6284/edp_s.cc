/*	! \file src/sensor/ati6284/edp_s.cc
 * \brief metody sterownika czujnika sily Gamma FT6284 firmy ATI
 * Ostatnia modyfikacja: marzec 2006
 * Autor: Krzysztof Dziubek	*/

#include <unistd.h>
#include <ctime>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/mis_fun.h"

#include "base/lib/sr/srlib.h"
#include "sensor/ati6284/edp_s.h"
#include "base/edp/edp_e_manip.h"

#include "base/lib/configurator.h"
#include "base/lib/mis_fun.h"
#include "base/lib/timer.h"
#include "tSTC.h"
#include "tESeries.h"
#include "osiBus.h"
#include "display.h"
#include "ftconfig.h"

#include "robot/hi_moxa/hi_moxa.h"

short int invalid_value;
Calibration *cal; //!< struct containing calibration information

namespace mrrocpp {
namespace edp {
namespace sensor {

#define START_TO_READ_TIME_INTERVAL 0.0007
#define INTERRUPT_INTERVAL 0.00007
#define START_TO_READ_FAILURE 0.002

static struct sigevent hi_event;
struct sigevent ati6284event;

iBus* bus;
tSTC *theSTC;
tESeries *board;
tAddressSpace Bar1;

// // // // // // // // // // // // // // /   obsluga prerwania // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////

const struct sigevent *szafa_handler(void *area, int szafa_id)
{
	return (&hi_event);
}

const struct sigevent *isr_handler(void *area, int id)
{
	short int boardInterrupt;

	//!< czy to karta pci wygenerowala przerwanie

	tAddressSpace Bar0;
	Bar0 = bus->createAddressSpace(kPCI_BAR0);
	boardInterrupt = (Bar0.read32(0x14)) & 0x80000000;
	bus->destroyAddressSpace(Bar0);

	//!< nie nasza karta
	if (boardInterrupt) {
		return (NULL);
	}

	//!< nasza karta wygenerowala przerwanie
	//!< czy to z kolejki?
	if (theSTC->AI_Status_1.readAI_FIFO_Empty_St()) {
		theSTC->Interrupt_A_Ack.writeAI_SC_TC_Interrupt_Ack(1);
		theSTC->Interrupt_A_Ack.writeAI_Error_Interrupt_Ack(1);
		return (NULL);
	}

	//!< przerwanie z kolejki
	theSTC->Interrupt_Control.writeRegister(0);
	theSTC->Interrupt_A_Ack.writeAI_SC_TC_Interrupt_Ack(1);
	theSTC->Interrupt_A_Ack.writeAI_Error_Interrupt_Ack(1);

	return (&ati6284event);
}

// // // // // // // // // // // // // // /   konfiguracja czujnika // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////

ATI6284_force::ATI6284_force(common::manip_effector &_master) :
	force(_master)
{
	sensor_frame = lib::Homog_matrix(-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0.09);
	force_sensor_name = edp::sensor::FORCE_SENSOR_ATI6284;

	//	sr_msg->message("ATI6284_force 1");
}

void ATI6284_force::connect_to_hardware(void)
{
	// unsigned  uCount;  //!< Count index
	// unsigned  uStatus; //!< Flag to indicate FIFO not empty

	Total_Number_of_Samples = 6;
	index = 1;
	/*
	 std::string calfilepath(master.config.return_mrrocpp_network_path());
	 calfilepath += "../src/sensor/ati6284/ft6284.cal";
	 */
	std::string calfilepath("../../src/sensor/ati6284/ft6284.cal");

	for (int i = 0; i < 5; i++)
		last_correct[i] = 0;
	overload = 0;
	show_no_result = 0;

	//!< create Calibration struct
	cal = createCalibration(calfilepath.c_str(), index);

	if (cal == NULL) {
		printf("\nSpecified calibration could not be loaded.\n");
	}

	if (ThreadCtl(_NTO_TCTL_IO, NULL) == -1) //!< dostep do sprzetu
		printf("Unable to connect to card\n");

	bus = acquireBoard(0x10932CA0); //!< funkcja uruchamiajca kart

	if (bus == NULL) {
		printf("Error accessing the PCI device.  Exiting.\n");
		exit(EXIT_FAILURE);
	}

	Bar1 = bus->createAddressSpace(kPCI_BAR1);
	board = new tESeries(Bar1);
	theSTC = new tSTC(Bar1);

	//!< Intitialise Mite Chip.
	InitMite();

	//!< Configure the board with the channel settings.
	Configure_Board();

	//!< Now Program the DAQ-STC

	//!< Configure the timebase options for DAQ-STC
	MSC_Clock_Configure();

	//!< Clear ADC FIFO
	Clear_FIFO();

	//!< Stop any activities in progress
	AI_Reset_All();

	//!< Set DAQ STC for E-series board
	AI_Board_Personalize();

	//!< Access the first value in the configuration FIFO
	AI_Initialize_Configuration_Memory_Output();

	//!< Setup for any external multiplexer
	AI_Board_Environmentalize();

	//!< Set triggering options
	AI_Trigger_Signals();

	//!< Select the number of scans
	Number_of_Scans();

	//!< Select the scan start event
	AI_Scan_Start();

	//!< Select the end of scan event
	AI_End_of_Scan();

	//!< Select the convert signal
	Convert_Signal();

	memset(&hi_event, 0, sizeof(hi_event));
	hi_event.sigev_notify = SIGEV_INTR;

	//Wacek: karta ISA zniknela!
	/*
	 irq_no = 0;//  edp::irp6p_m::IRQ_REAL; //!< Numer przerwania sprzetowego od karty ISA

	 if ((szafa_id = InterruptAttach(irq_no, szafa_handler, NULL, NULL, 0)) == -1) {
	 //!< Obsluga bledu
	 perror("Unable to attach szafa interrupt handler: ");
	 }
	 */

}

// // // // // // // // // // // // // // /   odlaczenie czujnika // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////

ATI6284_force::~ATI6284_force(void)
{
	if (!(force_sensor_test_mode)) {
		disconnect_from_hardware();
	}
}

void ATI6284_force::disconnect_from_hardware(void)
{
	delete theSTC;
	delete board;

	bus->destroyAddressSpace(Bar1);
#if INTERRUPT
	InterruptDetach (szafa_id);
#endif

	releaseBoard(bus);
}

// // // // // // // // // // // // // // /   inicjacja odczytu // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////
void ATI6284_force::configure_particular_sensor(void)
{

	//		sr_msg->message("configure_sensor 1");

	// double kartez_force[6];
	// short  measure_report;
	short int sensor_overload = 0;
	float force_torque[6];

	lib::timer local_timer;
	float sec;

	overload = 0;
	do {

#if	 WITHOUT_INTERRUPT
		do {
#endif
			//!< Clear ADC FIFO
			Clear_FIFO();

			//!< Enable interrupts
			AI_Interrupt_Enable();

			//!< Arm the analog input counters
			AI_Arming();

			//!< Add back _enable and _disable from comment
			//!< _disable;

			//!< Start the acquistion
			AI_Start_The_Acquisition();
			Samples_Acquired = 0;
			invalid_value = 0;

			local_timer.start();

			do {
				//!< odczekaj
				usleep(1000);
				local_timer.stop();
				local_timer.get_time(sec);
			} while (sec < START_TO_READ_TIME_INTERVAL);

			//			sr_msg->message("configure_sensor 21");
#if	 WITHOUT_INTERRUPT

			local_timer.start();
			do {
				//!< this is the ISR
				uStatus = theSTC->AI_Status_1.readRegister();
				if (!((uStatus & 0x1000) == 0x1000)) {
					//!< If the FIFO is not empty, call the ISR.
					Interrupt_Service_Routine();
				}
				local_timer.stop();
				local_timer.get_time(sec);
			} while ((Samples_Acquired < Total_Number_of_Samples) && (sec < START_TO_READ_FAILURE));

			if (sec >= START_TO_READ_FAILURE) {
				printf("aa :%f\n", sec);
				if (show_no_result == 0) {
					sr_msg->message("edp Sensor configure_sensor - brak wyniku");
					show_no_result = 1;
				}
			} else {
				if (show_no_result == 1) {
					sr_msg->message("edp Sensor configure_sensor - wynik otrzymany");
					show_no_result = 0;
				}
			}
		} while (sec >= START_TO_READ_FAILURE); //!< przeciwdzialanie zawieszeniu w petli podczas odczytu

#endif

#if	 INTERRUPT

		local_timer.start();
		do
		{
			//!< odczekaj
			local_timer.stop();
			local_timer.get_time(sec);
		}
		while(sec<INTERRUPT_INTERVAL);
#if DEBUG

		printf("Interrupt enabling!!!\n");
#endif

		theSTC->Interrupt_Control.setInterrupt_A_Output_Select(0);
		theSTC->Interrupt_Control.setInterrupt_A_Enable(1);
		theSTC->Interrupt_Control.flush();
#if DEBUG

		printf("Interrupt enabled!!!\n");
#endif

		InterruptWait (0, NULL);
		do
		{
			uValues[Samples_Acquired] = board->AIFifoData.readRegister();
			Samples_Acquired++;
		}
		while (Samples_Acquired<Total_Number_of_Samples);
#if DEBUG

		printf("After InterruptWait!!!\n");
#endif

		theSTC->Interrupt_Control.setInterrupt_A_Output_Select(4);
		theSTC->Interrupt_Control.setInterrupt_A_Enable(1);
		theSTC->Interrupt_Control.flush();

#endif

		Clear_FIFO();
		Samples_Acquired = 0;
		Input_to_Volts();

		for (int i = 0; i < 6; i++) {
			sBias[i] = 0;
		}
		ftconvert(sVolt, sBias, force_torque);

		//!< jesli pomiar byl poprawny
		if (invalid_value == 0) {
			sr_msg->message("edp Sensor configure_sensor - OK");
			sensor_overload = 0;
			overload = 0;
			is_sensor_configured = true;
		} else {
			if (overload == 0) {
				sr_msg->message("edp Sensor configure_sensor - OVERLOAD!!!");
			}
			sensor_overload = 1;
			overload = 1;
		}
	} while (sensor_overload == 1);

	for (int j = 0; j < 6; j++)
		sBias[j] = sVolt[j];

}

// // // // // // // // // // // // // // /   inicjalizacja zbierania danych z czujnika, wait_for_event // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////

void ATI6284_force::wait_for_particular_event()
{
	//	sr_msg->message("wait_for_event");
	if (!is_sensor_configured)
		throw lib::sensor::sensor_error(lib::FATAL_ERROR, SENSOR_NOT_CONFIGURED);

	//	sr_msg->message("wait_for_event ni test");

	lib::timer local_timer;
	float sec;

	//!< Clear ADC FIFO
	Clear_FIFO();

	//!< Enable interrupts
	AI_Interrupt_Enable();

	//!< Arm the analog input counters
	AI_Arming();

	//!< Add back _enable and _disable from comment
	//!< _disable;

	//!< Start the acquistion
	AI_Start_The_Acquisition();
	Samples_Acquired = 0;
	invalid_value = 0;

	local_timer.start();

	do {
		//!< odczekaj
		lib::timespec_increment_ns(&wake_time, COMMCYCLE_TIME_NS);

		int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake_time, NULL);
		if (err != 0) {
			fprintf(stderr, "clock_nanosleep(): %s\n", strerror(err));
		}

		local_timer.stop();
		local_timer.get_time(sec);
	} while (sec < START_TO_READ_TIME_INTERVAL);

	is_reading_ready = true;

}

// // // // // // // // // // // // // // /   odczyt z czujnika // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////
void ATI6284_force::get_particular_reading(void)
{

	short int no_result = 0; //brak wyniku
	static short int show = 0; //wyswietl
	float force_torque[6]; //wektor z siami i napiciami
	force_readring_status_t sensor_status = EDP_FORCE_SENSOR_READING_CORRECT;

	//!< jezeli chcemy jakikolwiek odczyt	-> is_reading_ready
	if (!is_reading_ready)
		throw lib::sensor::sensor_error(lib::FATAL_ERROR, READING_NOT_READY);

#if	 WITHOUT_INTERRUPT

	lib::timer local_timer;
	float sec;

	local_timer.start();

	do {
		//!< this is the ISR
		//!< Flag to indicate FIFO not empty
		uint16_t uStatus = theSTC->AI_Status_1.readRegister();
		if (!((uStatus & 0x1000) == 0x1000)) {
			//!< If the FIFO is not empty, call the ISR.
			Interrupt_Service_Routine();
		}

		local_timer.stop();
		local_timer.get_time(sec);
	} while ((Samples_Acquired < Total_Number_of_Samples) && (sec < START_TO_READ_FAILURE));
	if (sec >= START_TO_READ_FAILURE) {
		no_result = 1;
		invalid_value = 1;
	} else {
		no_result = 0;
		invalid_value = 0;
	}

#endif

#if	 INTERRUPT
	local_timer.start();
	do
	{
		//!< odczekaj
		local_timer.stop();
		local_timer.get_time(sec);
	}
	while(sec<INTERRUPT_INTERVAL);
#if DEBUG

	printf("Interrupt enabling!!!\n");
#endif

	theSTC->Interrupt_Control.setInterrupt_A_Output_Select(0);
	theSTC->Interrupt_Control.setInterrupt_A_Enable(1);
	theSTC->Interrupt_Control.flush();
#if DEBUG

	printf("Interrupt enabled!!!\n");
#endif

	InterruptWait (0, NULL);
	do
	{
		uValues[Samples_Acquired] = board->AIFifoData.readRegister();
		Samples_Acquired++;
	}
	while (Samples_Acquired<Total_Number_of_Samples);
#if DEBUG

	printf("After InterruptWait!!!\n");
#endif

	theSTC->Interrupt_Control.setInterrupt_A_Output_Select(4);
	theSTC->Interrupt_Control.setInterrupt_A_Enable(1);
	theSTC->Interrupt_Control.flush();

#endif

	if (no_result == 0) {
		Input_to_Volts();
		ftconvert(sVolt, sBias, force_torque);
	}
	if (invalid_value == 1) {
		if (no_result == 1) {
			if (show_no_result == 0) {
				sr_msg->message("edp Sensor initiate_reading - brak wyniku");
				show_no_result = 1;
				sensor_status = EDP_FORCE_SENSOR_READING_ERROR;
			}
		} else {
			if (overload == 0) {
				sr_msg->message("edp Sensor initiate_reading - OVERLOAD!!!");
				overload = 1;
				sensor_status = EDP_FORCE_SENSOR_OVERLOAD;
			}
		}
		for (int i = 0; i < 6; i++) {
			force_torque[i] = last_correct[i];
		}
	} else {
		if (show_no_result == 1) {
			sr_msg->message("edp Sensor initiate_reading - wynik otrzymany");
			show_no_result = 0;
		} else {
			if (overload == 1) {
				sr_msg->message("edp Sensor initiate_reading - OVERLOAD REMOVED");
				overload = 0;
			}
		}
		for (int i = 0; i < 6; i++) {
			last_correct[i] = force_torque[i];
		}
	}
	//!< ok
	from_vsp.vsp_report = lib::sensor::VSP_REPLY_OK;
	//!< tutaj: czujnik skalibrowany, odczyt dokonany, przepisanie wszystkich pol
	//!< przepisanie do bufora komunikacyjnego
#if WYNIKI_MRROCPP

	printf ("aaa: %f, %f, %f: \n", force_torque[0], force_torque[1], force_torque[2]);
#endif
	// // // // // // // // // // // // // // / PRZEPISANIE WYNIKU // // // // // // // // // // // // // // // // // // // // // // // //
	lib::Ft_vector kartez_force;
	if (gravity_transformation) {
		for (int i = 0; i < 6; i++) {
			ft_table[i] = force_torque[i];
		}

		/*		if (show==1000){
		 cerr << "Output\t";
		 for(int i=0;i<3;i++) {
		 output[i] *= 20;
		 cerr << ceil(output[i]) << "  ";
		 }
		 for(int i=3;i<6;i++) {
		 output[i] *= 333;
		 cerr << ceil(output[i]) << "  ";
		 }
		 cerr << endl;// << "Input\t";
		 for(int i=6;i<12;i++) cerr << ceil(output[i]) << "  ";
		 cerr << endl << "Gravity\t";
		 for(int i=12;i<18;i++) cerr << ceil(output[i]) << "  ";
		 cerr << endl << "Bias\t";
		 for(int i=18;i<24;i++) cerr << ceil(output[i]) << "  ";
		 cerr << endl << endl;
		 cerr << frame;
		 show=0;
		 }
		 */
	}

	show++;

}

// // // // // // // // // // // // // // /  inne potrzebne funkcje // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////

void ATI6284_force::Interrupt_Service_Routine(void)
{
	uValues[Samples_Acquired] = board->AIFifoData.readRegister();
	Samples_Acquired++;
	return;
}

void ATI6284_force::InitMite(void)
{
	tAddressSpace Bar0;
	u32 physicalBar1;

	//!< Skip MITE initialization for PCMCIA boards
	//!< (which do not have a MITE DMA controller)
	if (!bus->get(kIsPciPxiBus, 0))
		return;
	Bar0 = bus->createAddressSpace(kPCI_BAR0);

	//!< Get the physical address of the DAQ board
	physicalBar1 = bus->get(kBusAddressPhysical, kPCI_BAR1);

	//!< Tell the MITE to enable BAR1, where the rest of the board's registers are
	Bar0.write32(0xC0, (physicalBar1 & 0xffffff00L) | 0x80);

	bus->destroyAddressSpace(Bar0);
}

void ATI6284_force::Input_to_Volts(void)
{
	unsigned uCount;
	unsigned licznik = 0;
	float jednostka = 0.0003051;
	jednostka = jednostka;
	int suma[6] = { 0, 0, 0, 0, 0, 0 };
	float temp = 0; //!< do przesuniecia xzy xyz na xyz xyz

	for (uCount = 0; uCount < Total_Number_of_Samples; uCount++) {
		licznik = (uCount) % 6;
		suma[licznik] = suma[licznik] + uValues[uCount];
	}

	for (uCount = 0; uCount < 6; uCount++) {
		sVolt[uCount] = uValues[uCount] * jednostka;
	}

	//!< odwrcenie wartoci tak by byly w kolejnosci fx fy fz dx dy dz
	for (int i = 0; i < 3; i++)
		for (int j = 5; j > 2; j--) {
			temp = sVolt[j];
			sVolt[j] = sVolt[i];
			sVolt[i] = temp;
		}

	temp = sVolt[3];
	sVolt[3] = sVolt[5];
	sVolt[5] = temp;

	temp = sVolt[2];
	sVolt[2] = sVolt[0];
	sVolt[0] = temp;

#if BIAS

	for(uCount=0;uCount<6;uCount++)
	{
		cout<<sBias[uCount]/jednostka<<"\t";
	}
	printf("\n");
	for(uCount=0;uCount<6;uCount++)
	{
		cout<<sVolt[uCount]/jednostka<<"\t";
	}
	printf("\n\n");
#endif

	return;
}

// // // // // // // // DO KONFIGURACJI PLYTY// // // // // // // // // // // // // // // // // // // // // // // // // // // // /


//!< Call this function to configure board options.
void ATI6284_force::Configure_Board(void)
{
	//!< Clear configuration memory
	theSTC->Write_Strobe_0.writeRegister(0x0001);

	//!< Clear ADC FIFO
	theSTC->Write_Strobe_1.writeRegister(0x0001);

	//!< Writing to Config_Memory_High_Register for channel 5 settings
	board->ConfigFifoHigh.setChannel(5);
	board->ConfigFifoHigh.setBank(0);
	board->ConfigFifoHigh.setChannelType(board->ConfigFifoHigh.kChannelTypeDifferential);
	board->ConfigFifoHigh.flush();

	//!< Writing to Config_Memory_Low_Register for following channel 5 settings
	board->ConfigFifoLow.setLastChannel(0);
	board->ConfigFifoLow.setGeneralTrigger(0);
	board->ConfigFifoLow.setGain(board->ConfigFifoLow.kGain000_5);
	board->ConfigFifoLow.setPolarity(board->ConfigFifoLow.kPolarityBipolar);
	board->ConfigFifoLow.setDither(1);
	board->ConfigFifoLow.flush();

	//!< Writing to Config_Memory_High_Register for channel 4 settings
	board->ConfigFifoHigh.setChannel(4);
	board->ConfigFifoHigh.setBank(0);
	board->ConfigFifoHigh.setChannelType(board->ConfigFifoHigh.kChannelTypeDifferential);
	board->ConfigFifoHigh.flush();

	//!< Writing to Config_Memory_Low_Register for following channel 4 settings
	board->ConfigFifoLow.setLastChannel(0);
	board->ConfigFifoLow.setGeneralTrigger(0);
	board->ConfigFifoLow.setGain(board->ConfigFifoLow.kGain000_5);
	board->ConfigFifoLow.setPolarity(board->ConfigFifoLow.kPolarityBipolar);
	board->ConfigFifoLow.setDither(1);
	board->ConfigFifoLow.flush();

	//!< Writing to Config_Memory_High_Register for channel 3 settings
	board->ConfigFifoHigh.setChannel(3);
	board->ConfigFifoHigh.setBank(0);
	board->ConfigFifoHigh.setChannelType(board->ConfigFifoHigh.kChannelTypeDifferential);
	board->ConfigFifoHigh.flush();

	//!< Writing to Config_Memory_Low_Register for following channel 3 settings
	board->ConfigFifoLow.setLastChannel(0);
	board->ConfigFifoLow.setGeneralTrigger(0);
	board->ConfigFifoLow.setGain(board->ConfigFifoLow.kGain000_5);
	board->ConfigFifoLow.setPolarity(board->ConfigFifoLow.kPolarityBipolar);
	board->ConfigFifoLow.setDither(1);
	board->ConfigFifoLow.flush();

	//!< Writing to Config_Memory_High_Register for channel 2 settings
	board->ConfigFifoHigh.setChannel(2);
	board->ConfigFifoHigh.setBank(0);
	board->ConfigFifoHigh.setChannelType(board->ConfigFifoHigh.kChannelTypeDifferential);
	board->ConfigFifoHigh.flush();

	//!< Writing to Config_Memory_Low_Register for following channel 2 settings
	board->ConfigFifoLow.setLastChannel(0);
	board->ConfigFifoLow.setGeneralTrigger(0);
	board->ConfigFifoLow.setGain(board->ConfigFifoLow.kGain000_5);//!< zmienione z 001_0==1 na 000-5==0.5 wtedy zakres +-10V
	board->ConfigFifoLow.setPolarity(board->ConfigFifoLow.kPolarityBipolar);
	board->ConfigFifoLow.setDither(1);
	board->ConfigFifoLow.flush();

	//!< Writing to Config_Memory_High_Register for channel 1 settings
	board->ConfigFifoHigh.setChannel(1);
	board->ConfigFifoHigh.setBank(0);
	board->ConfigFifoHigh.setChannelType(board->ConfigFifoHigh.kChannelTypeDifferential);
	board->ConfigFifoHigh.flush();

	//!< Writing to Config_Memory_Low_Register for following channel 1 settings
	board->ConfigFifoLow.setLastChannel(0);
	board->ConfigFifoLow.setGeneralTrigger(0);
	board->ConfigFifoLow.setGain(board->ConfigFifoLow.kGain000_5);
	board->ConfigFifoLow.setPolarity(board->ConfigFifoLow.kPolarityBipolar);
	board->ConfigFifoLow.setDither(1);
	board->ConfigFifoLow.flush();

	//!< Writing to Config_Memory_High_Register for channel 0 settings
	board->ConfigFifoHigh.setChannel(0);
	board->ConfigFifoHigh.setBank(0);
	board->ConfigFifoHigh.setChannelType(board->ConfigFifoHigh.kChannelTypeDifferential);
	board->ConfigFifoHigh.flush();

	//!< Writing to Config_Memory_Low_Register for following channel 0 settings
	board->ConfigFifoLow.setLastChannel(1);//!< Channel 0 has last channel set to 1
	board->ConfigFifoLow.setGeneralTrigger(0);
	board->ConfigFifoLow.setGain(board->ConfigFifoLow.kGain000_5);
	board->ConfigFifoLow.setPolarity(board->ConfigFifoLow.kPolarityBipolar);
	board->ConfigFifoLow.setDither(1);
	board->ConfigFifoLow.flush();
	return;
}

//!< Call this function to configure the timebase options for DAQ-STC.
void ATI6284_force::MSC_Clock_Configure(void)
{
	//!< Select timebase for DAQ-STC
	theSTC->Clock_and_FOUT.setSlow_Internal_Timebase(1);
	theSTC->Clock_and_FOUT.setSlow_Internal_Time_Divide_By_2(1);
	theSTC->Clock_and_FOUT.setClock_To_Board(1);
	theSTC->Clock_and_FOUT.setClock_To_Board_Divide_By_2(1);
	theSTC->Clock_and_FOUT.flush();
	return;
}

//!< Call this function to clear the AI FIFO.
void ATI6284_force::Clear_FIFO(void)
{
	theSTC->Write_Strobe_1.writeRegister(0x0001);
	return;
}

//!< Call this function to stop any activities in progress.
void ATI6284_force::AI_Reset_All(void)
{
	theSTC->Joint_Reset.setAI_Reset(1);//!< Reset important registers
	theSTC->Joint_Reset.setAI_Configuration_Start(1);//!< Starting AI configuration
	theSTC->Joint_Reset.flush();

	//!< Acknowledges the following interrupt request if the interrupt enable is set
	theSTC->Interrupt_A_Ack.setAI_SC_TC_Error_Confirm(1);
	theSTC->Interrupt_A_Ack.setAI_SC_TC_Interrupt_Ack(1);
	theSTC->Interrupt_A_Ack.setAI_START1_Interrupt_Ack(1);
	theSTC->Interrupt_A_Ack.setAI_START2_Interrupt_Ack(1);
	theSTC->Interrupt_A_Ack.setAI_START_Interrupt_Ack(1);
	theSTC->Interrupt_A_Ack.setAI_STOP_Interrupt_Ack(1);
	theSTC->Interrupt_A_Ack.setAI_Error_Interrupt_Ack(1);
	theSTC->Interrupt_A_Ack.flush();

	//!< Enables Start or Stop Analog Input Operation
	theSTC->AI_Mode_1.setReserved_One(1);
	theSTC->AI_Mode_1.setAI_Start_Stop(1);
	theSTC->AI_Mode_1.flush();

	//!< Ending AI configuration
	theSTC->Joint_Reset.setAI_Configuration_Start(0);
	theSTC->Joint_Reset.setAI_Configuration_End(1);
	theSTC->Joint_Reset.flush();
	return;
}

//!< Call this function to setup the board.
void ATI6284_force::AI_Board_Personalize(void)
{
	theSTC->Joint_Reset.writeAI_Configuration_Start(1);

	theSTC->Clock_and_FOUT.setAI_Source_Divide_By_2(0);
	theSTC->Clock_and_FOUT.setAI_Output_Divide_By_2(1);
	theSTC->Clock_and_FOUT.flush();

	theSTC->AI_Personal.setAI_CONVERT_Pulse_Timebase(theSTC->AI_Personal.kAI_CONVERT_Pulse_TimebasePulse_Width);
	theSTC->AI_Personal.setAI_CONVERT_Pulse_Width(theSTC->AI_Personal.kAI_CONVERT_Pulse_WidthAbout_1_Clock_Period);
	theSTC->AI_Personal.setAI_FIFO_Flags_Polarity(theSTC->AI_Personal.kAI_FIFO_Flags_PolarityActive_Low);
	theSTC->AI_Personal.setAI_LOCALMUX_CLK_Pulse_Width(theSTC->AI_Personal.kAI_LOCALMUX_CLK_Pulse_WidthAbout_1_Clock_Period);
	theSTC->AI_Personal.setAI_AIFREQ_Polarity(theSTC->AI_Personal.kAI_AIFREQ_PolarityActive_High);
	theSTC->AI_Personal.setAI_SHIFTIN_Polarity(theSTC->AI_Personal.kAI_SHIFTIN_PolarityActive_Low);
	theSTC->AI_Personal.setAI_SHIFTIN_Pulse_Width(theSTC->AI_Personal.kAI_SHIFTIN_Pulse_WidthAbout_2_Clock_Periods);
	theSTC->AI_Personal.setAI_EOC_Polarity(theSTC->AI_Personal.kAI_EOC_PolarityRising_Edge);
	theSTC->AI_Personal.setAI_SOC_Polarity(theSTC->AI_Personal.kAI_SOC_PolarityFalling_Edge);
	theSTC->AI_Personal.setAI_Overrun_Mode(theSTC->AI_Personal.kAI_Overrun_ModeSOC_To_SHIFTIN_Trailing_Edge);
	theSTC->AI_Personal.flush();

	theSTC->AI_Output_Control.setAI_CONVERT_Output_Select(theSTC->AI_Output_Control.kAI_CONVERT_Output_SelectActive_Low);
	theSTC->AI_Output_Control.setAI_SC_TC_Output_Select(theSTC->AI_Output_Control.kAI_SC_TC_Output_SelectActive_High);
	theSTC->AI_Output_Control.setAI_SCAN_IN_PROG_Output_Select(theSTC->AI_Output_Control.kAI_SCAN_IN_PROG_Output_SelectActive_High);
	theSTC->AI_Output_Control.setAI_LOCALMUX_CLK_Output_Select(theSTC->AI_Output_Control.kAI_LOCALMUX_CLK_Output_SelectActive_Low);
	theSTC->AI_Output_Control.flush();

	theSTC->Joint_Reset.setAI_Configuration_Start(0);
	theSTC->Joint_Reset.setAI_Configuration_End(1);
	theSTC->Joint_Reset.flush();
	return;
}

//!< Call this function to access the first value in the configuration FIFO.
void ATI6284_force::AI_Initialize_Configuration_Memory_Output(void)
{
	theSTC->AI_Command_1.writeAI_CONVERT_Pulse(1);
	return;
}

//!< Call this function to setup for external multiplexers.
void ATI6284_force::AI_Board_Environmentalize(void)
{
	theSTC->Joint_Reset.writeAI_Configuration_Start(1);

	theSTC->AI_Mode_2.writeAI_External_MUX_Present(theSTC->AI_Mode_2.kAI_External_MUX_PresentEvery_Convert);

	theSTC->Joint_Reset.setAI_Configuration_Start(0);
	theSTC->Joint_Reset.setAI_Configuration_End(1);
	theSTC->Joint_Reset.flush();
	return;
}

//!< Call this function to enable or disable retriggering.
void ATI6284_force::AI_Trigger_Signals(void)
{
	theSTC->Joint_Reset.writeAI_Configuration_Start(1);

	//!< Controls the retriggerability of Counters
	theSTC->AI_Mode_1.writeAI_Trigger_Once(1);

	//!< Selects and configures the functanality of START1 trigger
	theSTC->AI_Trigger_Select.setAI_START1_Select(theSTC->AI_Trigger_Select.kAI_START1_SelectPulse);
	theSTC->AI_Trigger_Select.setAI_START1_Polarity(theSTC->AI_Trigger_Select.kAI_START1_PolarityActive_High_Or_Rising_Edge);
	theSTC->AI_Trigger_Select.setAI_START1_Edge(1);
	theSTC->AI_Trigger_Select.setAI_START1_Sync(1);
	theSTC->AI_Trigger_Select.flush();

	theSTC->Joint_Reset.setAI_Configuration_Start(0);
	theSTC->Joint_Reset.setAI_Configuration_End(1);
	theSTC->Joint_Reset.flush();
	return;
}

//!< Call this function to select the number of scans.
void ATI6284_force::Number_of_Scans(void)
{
	theSTC->Joint_Reset.writeAI_Configuration_Start(1);
	theSTC->AI_SC_Load_A.writeRegister(0x00000001);//!< Number of Scans(jeden skan to 6 sygnalow)  are 1

	theSTC->AI_Command_1.writeAI_SC_Load(1);

	theSTC->Joint_Reset.setAI_Configuration_Start(0);
	theSTC->Joint_Reset.setAI_Configuration_End(1);
	theSTC->Joint_Reset.flush();
	return;
}

//!< Call this function to select the scan start event.
void ATI6284_force::AI_Scan_Start(void)
{
	theSTC->Joint_Reset.writeAI_Configuration_Start(1);

	//!< Setting the bitfields corresponding to START triggers
	theSTC->AI_START_STOP_Select.setAI_START_Select(theSTC->AI_START_STOP_Select.kAI_START_SelectSI_TC);
	theSTC->AI_START_STOP_Select.setAI_START_Edge(1);
	theSTC->AI_START_STOP_Select.setAI_START_Sync(1);
	theSTC->AI_START_STOP_Select.setAI_START_Polarity(theSTC->AI_START_STOP_Select.kAI_START_PolarityActive_High_Or_Rising_Edge);
	theSTC->AI_START_STOP_Select.flush();

	theSTC->AI_SI_Load_A.writeRegister(0x00000001);

	theSTC->AI_Command_1.writeAI_SI_Load(1);

	theSTC->AI_SI_Load_A.writeRegister(0x000007CF);

	theSTC->Joint_Reset.setAI_Configuration_Start(0);
	theSTC->Joint_Reset.setAI_Configuration_End(1);
	theSTC->Joint_Reset.flush();
	return;
}

//!< Call this function to select the end of scan event.
void ATI6284_force::AI_End_of_Scan(void)
{
	theSTC->Joint_Reset.writeAI_Configuration_Start(1);

	//!< Setting the bitfields corresponding to STOP triggers
	theSTC->AI_START_STOP_Select.setAI_STOP_Select(theSTC->AI_START_STOP_Select.kAI_STOP_SelectIN);
	theSTC->AI_START_STOP_Select.setAI_STOP_Edge(0);
	theSTC->AI_START_STOP_Select.setAI_STOP_Polarity(theSTC->AI_START_STOP_Select.kAI_STOP_PolarityActive_High_Or_Rising_Edge);
	theSTC->AI_START_STOP_Select.setAI_STOP_Sync(1);
	theSTC->AI_START_STOP_Select.flush();

	theSTC->Joint_Reset.setAI_Configuration_Start(0);
	theSTC->Joint_Reset.setAI_Configuration_End(1);
	theSTC->Joint_Reset.flush();
	return;
}

//!< Call this function to select the convert signal for the Acquisition
void ATI6284_force::Convert_Signal(void)
{
	theSTC->Joint_Reset.writeAI_Configuration_Start(1);

	theSTC->AI_SI2_Load_A.writeRegister(0x03E7);

	theSTC->AI_SI2_Load_B.writeRegister(0x03E7);

	theSTC->AI_Mode_2.writeAI_SI2_Reload_Mode(theSTC->AI_Mode_2.kAI_SI2_Reload_ModeAlternate_First_Period_Every_STOP);

	theSTC->AI_Command_1.writeAI_SI2_Load(1);

	theSTC->AI_Mode_2.writeAI_SI2_Initial_Load_Source(theSTC->AI_Mode_2.kAI_SI2_Initial_Load_SourceLoad_B);

	theSTC->Joint_Reset.setAI_Configuration_Start(0);
	theSTC->Joint_Reset.setAI_Configuration_End(1);
	theSTC->Joint_Reset.flush();
	return;
}

//!< Call this function to enable interrupts for the Acquisition
void ATI6284_force::AI_Interrupt_Enable(void)
{
	theSTC->Interrupt_A_Enable.setRegister(0); //!< reset register
	theSTC->Interrupt_A_Enable.setAI_FIFO_Interrupt_Enable(1); //!< enable interrupts based on the AI FIFO
	theSTC->Interrupt_A_Enable.setAI_STOP_Interrupt_Enable(0);
	theSTC->Interrupt_A_Enable.setAI_START_Interrupt_Enable(0);
	theSTC->Interrupt_A_Enable.setAI_SC_TC_Interrupt_Enable(0);
	theSTC->Interrupt_A_Enable.setAI_START1_Interrupt_Enable(0);
	theSTC->Interrupt_A_Enable.setAI_START2_Interrupt_Enable(0);
	theSTC->Interrupt_A_Enable.setAI_Error_Interrupt_Enable(0);
	theSTC->Interrupt_A_Enable.flush();

	theSTC->AI_Mode_3.setRegister(0); //!< reset register
	theSTC->AI_Mode_3.setAI_FIFO_Mode(theSTC->AI_Mode_3.kAI_FIFO_ModeNot_Empty); //!< set the interrupt when the AI FIFO is not empty
	theSTC->AI_Mode_3.flush();

	theSTC->Interrupt_Control.setRegister(0); //!< reset register
	//!< IRQ signal is routed from DAQ STC to the MITE chip which sends the interrupt onto the PCI bus
	//!< The polarity must be Active Low for the signal from the STC to the MITE
	theSTC->Interrupt_Control.setInterrupt_Output_Polarity(theSTC->Interrupt_Control.kInterrupt_Output_PolarityActive_Low);
	//!< Only IRQ_Out 0 from the STC is connected to the MITE and will generate actual
	//!< interrupts. Change the following line to select IRQ_Out 0 to generate interrupts
	//!< on the PCI bus when you have an Interrupt Service Routine installed, with IRQ_Out 4
	//!< selected, no interrupts will be generated by the MITE.


#if	INTERRUPT

	theSTC->Interrupt_Control.setInterrupt_A_Output_Select(4);
#endif

#if WITHOUT_INTERRUPT

	theSTC->Interrupt_Control.setInterrupt_A_Output_Select(4);
#endif

	theSTC->Interrupt_Control.setInterrupt_A_Enable(1);
	theSTC->Interrupt_Control.flush();

	return;
}

//!< Call this function to arm the analog input counters.
void ATI6284_force::AI_Arming(void)
{
	theSTC->AI_Command_1.setAI_SC_Arm(1);
	theSTC->AI_Command_1.setAI_SI_Arm(1);
	theSTC->AI_Command_1.setAI_SI2_Arm(1);
	theSTC->AI_Command_1.setAI_DIV_Arm(1);
	theSTC->AI_Command_1.flush();
	return;
}

//!< Call this function to start the acquistion.
void ATI6284_force::AI_Start_The_Acquisition(void)
{
	theSTC->AI_Command_2.writeAI_START1_Pulse(1);
	theSTC->AI_Command_2.flush();
	return;
}

force* return_created_edp_force_sensor(common::manip_effector &_master)
{
	return new ATI6284_force(_master);
}//!< : return_created_sensor

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
