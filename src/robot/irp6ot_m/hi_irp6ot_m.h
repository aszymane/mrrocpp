// -------------------------------------------------------------------------
//                            hi_rydz.h
// Definicje struktur danych i metod dla interfejsu sprzetowego
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __HI_LOCAL_IRP6OT_M_H
#define __HI_LOCAL_IRP6OT_M_H

#include "robot/hi_rydz/hi_rydz.h"
#include "robot/hi_moxa/hi_moxa.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_m {

// Struktury danych wykorzystywane w hardware_interface
const int IRQ_REAL = 10; // Numer przerwania sprzetowego
const unsigned short int INT_FREC_DIVIDER = 4; // Dzielnik czestotliwosci przerwan

const long HI_RYDZ_INTR_TIMEOUT_HIGH = 10000000; // by Y - timeout przerwania z szafy badz zegara

const int FIRST_SERVO_PTR = 0xC0;
const int INTERRUPT_GENERATOR_SERVO_PTR = 0xC0;
const int IN_OUT_PACKET = 0xC8;

const int ISA_CARD_OFFSET = 0x20;
// w zaleznosci od ustawienia na karcie isa

const int AXIS_1_MAX_CURRENT = 0x2430;// ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
const int AXIS_2_MAX_CURRENT = 0x2430; // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
const int AXIS_3_MAX_CURRENT = 0x2430; // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
const int AXIS_4_MAX_CURRENT = 0x2430;// ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
const int AXIS_5_MAX_CURRENT = 0x2430;// ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
const int AXIS_6_MAX_CURRENT = 0x2430; // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
const int AXIS_7_MAX_CURRENT = 0x2410; // ustawienie pradu maksymalnego dla przedostatniej osi - obrot chwytaka
// 13,7 j na amper


// ------------------------------------------------------------------------
//                HARDWARE_INTERFACE class
// ------------------------------------------------------------------------

class hardware_interface : public hi_moxa::HI_moxa
{
public:
	hardware_interface(common::motor_driven_effector &_master); // Konstruktor

}; // koniec: class hardware_interface

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp

#endif // __HI_RYDZ_H
