#include <Arduino.h>

#include <statemachine.h>


StateMachine::StateMachine(uint8_t initial_state){      //Constuctor
    _state = initial_state; 
    _substate = 0;
    _flags = 0;   
    _gpf[0] = 0;
    _gpf[1] = 0;
    _gpf[2] = 0;
    _gpf[3] = 0;
}

uint8_t StateMachine::stateInit(){
    if (((INITCONF_FLAG_BYTE) & INITCONF_FLAG_BIT) == 0)		//If flag is 0...
	{
		INITCONF_FLAG_BYTE|=INITCONF_FLAG_BIT;	//Set flag to 1.
		return 1;
	}
	else
		return 0;
}

uint8_t StateMachine::setState(uint8_t new_state){
    uint8_t laststate = _state;
    _state = new_state;
    return(laststate);
}

uint8_t StateMachine::setSubState(uint8_t new_substate){
    uint8_t lastsubstate = _substate;
    _substate = new_substate;
    return(lastsubstate);
}

uint8_t StateMachine::getState(){
    return(_state);
}
