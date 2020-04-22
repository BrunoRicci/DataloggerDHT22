#include <Arduino.h>

#ifndef _STATEMACHINE_HPP
#define _STATEMACHINE_HPP

#define INITCONF_FLAG_BYTE  _flags	  //Flags register.
#define	INITCONF_FLAG_BIT	  0b1				//bit 0 reserved to initial configuration routine flag.

//Maximum states: 255.

class StateMachine {
  public:
    uint8_t setState(uint8_t new_state);    //Sets new state and returns the previous one.    
    uint8_t getState(void);               //Returns actual state value.
    uint8_t setSubState(uint8_t new_substate);    //Sets new substate and returns the previous one.    
    uint8_t stateInit(void);              //Returns true in the first call, so an initialization routine for each new state can be run.
    StateMachine(uint8_t initial_state=0);        //Constructor.

  private:
    uint8_t _state = 0;      //actual state value.
    uint8_t _substate = 0;   //sub states for each state.
    uint8_t _flags = 0;      //Flags bits.
    uint8_t _gpf[4] = {0,0,0,0};     //4B for general purposes.
};


#endif