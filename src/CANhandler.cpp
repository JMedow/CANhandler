/*
 CANhandler.cpp
 2023

 Author: Jeremy Medow (jeremy@tungstencustoms.com)
*/

#include "CANhandler.h"

Node::Node(){
}

// For a given prop, set it up as the right type, initialize all variables, and pass the CAN object for communication
void Node::initNode(uint8_t nodeID, CANnode aCAN){
	_myID = nodeID;
	_myCAN = &aCAN;
	_errorStatus = 0;
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
			if(_regs[theReg]!=thePayload)
				_hasChanged = true;
			_regs[theReg] = thePayload;
			_errorStatus = 0;
			return true;
		} else {
			_errorStatus++;
			return false;
		}
}

// Return whether there's been a change, clear the change flag either way
bool Node::registerChange(void){
	if(_hasChanged){
		_hasChanged = false;
		return true;
	} else
		return false;
}

uint8_t Node::updateReg(uint8_t reg){
	if((millis()-_lastCANtime)<ASK_TIME)
		return _regs[reg];
	else{
		_lastCANtime = millis();
		_errorStatus++;
		_myCAN->sendReadRequest(_myID,reg,false);
	}
}

void Node::updateAllReg(void){
	if((millis()-_lastCANtime)<ASK_TIME)
		return;
	else{
		_lastCANtime = millis();
		_errorStatus++;
		_myCAN->sendReadRequest(_myID,REG_MULT,true);
	}
}

bool Node::assignMult(uint8_t regMask, uint8_t data[]){
    bool toRet = false;

    for(uint8_t reg = 0; reg < 7; reg++)
      if(bitRead(regMask,reg)){
        if(_regs[reg] != data[reg])     // Something will change
          toRet = true;
        _regs[reg] = data[reg];
      }

    return toRet;
}

bool Node::assignOne(uint8_t reg, uint8_t value){
	bool toRet = false;

	if(_regs[reg] != value)     // Something will change
		toRet = true;
	_regs[reg] = value;

	return toRet;
}

void Node::clearErrors(void){
	_lastCANtime = millis();
	_errorStatus = 0;
}

void Node::sndACK(uint8_t theReg, uint8_t thePayload){
	_myCAN->sndACK(theReg,thePayload);			// Send as the master
}

///////////////////////

// Step through an array of props, find the one with matching id to the sender, and assign the message to the status correctly
void assignMessage(Node p[], uint8_t numNodes, CANmsg msg, uint8_t data[], uint8_t len){
	for(uint8_t i = 0; i < numNodes; i++){
		if(p[i].isMe(msg.sndID)){

			p[i].clearErrors();
			switch (msg.ackRW){
				case W_ACK:
					p[i].sndACK(msg.reg,msg.payload);
				case W_NACK:
				case ACK:
					if(msg.reg == REG_MULT)
						p[i].assignMult(msg.payload,data);
					else
						p[i].assignOne(msg.reg,msg.payload);
					break;
			}
			return;				// No need to step through the rest of the loop
		}
	}
}



/*
  END FILE
*/
