/*
  Servo.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
  Copyright (c) 2009 Michael Margolis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
  A servo is activated by creating an instance of the Servo class passing
  the desired pin to the attach() method.
  The servos are pulsed in the background using the value most recently
  written using the write() method.

  Note that analogWrite of PWM on pins associated with the timer are
  disabled when the first servo is attached.
  Timers are seized as needed in groups of 12 servos - 24 servos use two
  timers, 48 servos will use four.
  The sequence used to sieze timers is defined in timers.h

  The methods are:

    Servo - Class for manipulating servo motors connected to Arduino pins.

    attach(pin )  - Attaches a servo motor to an i/o pin.
    attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
    default min is 544, max is 2400

    write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
    writeMicroseconds() - Sets the servo pulse width in microseconds
    read()      - Gets the last written servo pulse width as an angle between 0 and 180.
    readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
    attached()  - Returns true if there is a servo attached.
    detach()    - Stops an attached servos from pulsing its i/o pin.
 */

#ifndef Stepper_h
#define Stepper_h

#include <inttypes.h>

/*
 * Defines for 16 bit timers used with  Servo library
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the current board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available.
 */

// Architecture specific include
#include "StepperTimers.h"

#define STEPPER_VERSION           1     // software version of this library

#define MIN_PULSE_WIDTH       300     // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      30000000     // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH  300     // default pulse width when servo is attached
#define REFRESH_INTERVAL    20000     // minumim time to refresh servos in microseconds
#define DEFAULT_PERIOD      1000
#define STEPPERS_PER_TIMER       10      // the maximum number of servos controlled by one timer
#define MAX_STEPPERS      (_Nbr_16timers  * STEPPERS_PER_TIMER)

#define INVALID_STEPPER         255     // flag indicating an invalid servo index

#define DIV_FULL      0x00
#define DIV_1_2_STEP  0X01
#define DIV_1_4_STEP  0x02
#define DIV_1_8_STEP  0x03

typedef struct  {
  uint8_t nbr        :6 ;             // a pin number from 0 to 63
  uint8_t isActive   :1 ;             // true if this channel is enabled, pin not pulsed if false
  uint8_t etat       :1 ;
} StepperPin_t   ;

typedef struct {
  StepperPin_t Pin;
  volatile int32_t ticks;   // nombre de pas à faire
  volatile uint32_t period; //valeur de la fréquence
  volatile bool completed;
} stepper_t;

class Stepper
{
public:
  Stepper(void);
  uint8_t attach(uint16_t nb_step,uint8_t enaPin, uint8_t stepPin, uint8_t dirPin,uint8_t M0Pin, uint8_t M1Pin);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  void detach();
  void SetSpeed(uint16_t value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
  void Move(int32_t value);           // number of step to move
  void Move(void);                    // restart from last tick
  void Stop(void);                    // Stop immediately the OC IT
  void writeMicroseconds(int value); // Write pulse width in microseconds
  int readMicroseconds(void);     
  uint16_t read(void);
  bool attached(void);                   // return true if this servo is attached, otherwise false
  bool Completed(void);
  bool Ready(void);
  void SetDirPolarity(bool Avance);
  void Deactivate(void);
  uint8_t GetDivider(void);
private:
  
  uint32_t SetDiv(uint32_t speed);
  void Setmstep(byte ustep);
  bool Avance=true;
  bool Recule=false;

  uint8_t stepperIndex;               // index into the channel data for this servo
  stimer_t _timer;
  uint16_t nstep;
  uint8_t M0Pin;
  uint8_t M1Pin;
  uint8_t DirPin;
  uint8_t enaPin;
  uint8_t diviseur=DIV_FULL;     // store the driver divider value [M1M0]
  uint32_t speed=0;
  bool ready=false;
};

#endif
