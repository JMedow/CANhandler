/*
 CANhandler.cpp
 2023

 Author: Jeremy Medow (jeremy@tungstencustoms.com)
*/

#include "CANhandler.h"

Node::Node(){
}

// For a given prop, set it up as the right type, initialize all variables, and pass the CAN object for communication
void Node::initNode(uint8_t nodeID, CANtroller aCAN){
	_myID = nodeID;
	_myCAN = &aCAN;
  _status = NEVER_STATUS;		// No status ever sent
	_hasChanged = true;			// Have to handle it
}

// Is it me?
bool Node::isMe(uint8_t aID){
    return (aID == _myID);
}

// What was the last status sent
uint8_t Node::lastReg(uint8_t reg){
	return _regs[reg];
}

// Set register for ACK, update _errorStatus if no ACK
bool Node::setRegister(uint8_t theReg, uint8_t thePayload){
		bool acked = _myCAN->regWrite(_myID,theReg,thePayload,true);
		if(acked){
			if(_regs[theReg]!=theStatus)
				_hasChanged = true;
			_regs[theReg] = theStatus;
			_errorStatus = 0;
			return true;
		} else {
			_errorStatus++;
			return false;
		}
}

// Return whether there's been a change, clear the change flag either way
bool Node:registerChange(void){
	if(_hasChanged){
		_hasChanged = false;
		return true;
	} else
		return false;
}

uint8_t Node:updateReg(uint8_t reg){
	if((millis()-_lastCANtime)<ASK_TIME)
		return _regs[reg];
	else{
		_lastCANtime = millis();
		_errorStatus++;
		_myCAN->sendReadRequest(_myID,reg,false);
	}
}

void Node:updateAllReg(void){
	if((millis()-_lastCANtime)<ASK_TIME)
		return;
	else{
		_lastCANtime = millis();
		_errorStatus++;
		_myCAN->sendReadRequest(_myID,REG_MULT,true);
	}
}

///////////////////////

// Step through an array of props, find the one with matching id to the sender, and assign the message to the status correctly
void assignMessage(Node p[], uint8_t numNodes, CANmsg msg, uint8_t data[], uint8_t len){
	for(uint8_t i = 0; i < numNodes; i++){
		if(p[i].isMe(msg.sndID)){
			_lastCANtime = millis();			// Prop responding, so online, not in error
			_errorStatus = 0;
			switch (msg.ackRW){
				case W_ACK:
					myCan->sndACK(msg.reg,msg.payload);
				case W_NACK:
				case ACK:
					if(msg.reg == REG_MULT)
						_myCan->assignMult(msg.payload,data);
					else
						_myCan->setReg(msg.reg,msg.payload);
					break;
			}
			return;				// No need to step through the rest of the loop
		}
	}
}

/*
  END FILE
*/
