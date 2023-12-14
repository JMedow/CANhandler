/*
  CANhandler.h
  2023

  Author: Jeremy Medow (jeremy@tungstencustoms.com)
*/
#ifndef CANhandler_h
#define CANhandler_h

#include <CANnode.h>

#define UPDATE_FREQUENCY 1000     // How often to go to CAN for updates
#define ASK_TIME 200
#define NONRESPONSE_THRESHOLD 5

class Node{
public:
    Node();
    void initNode(uint8_t nodeID, CANnode aCAN);
    // Set up the prop according to its propDef, and pass the master's CAN object for communication

    bool isMe(uint8_t aID);
    // Respond TRUE if aID is this prop's address

    uint8_t lastReg(uint8_t reg);
    // Returns the most recently fetched / aquired status info, but doesn't ask over CAN

    bool setRegister(uint8_t theReg, uint8_t thePayload);
    // Sets a register to the payload, returns true if ACK'd

    bool registerChange(void);
    // Returns whether there has been a change, clears the flag

    uint8_t updateReg(uint8_t reg);
    // Ask for the contents of a register

    void updateAllReg(void);
    // Ask for the contents of all registers

    bool assignMult(uint8_t regMask, uint8_t data[]);
    // Put the data into local storage

    bool assignOne(uint8_t reg, uint8_t value);
    // Put the data into local storage

    void clearErrors(void);
    // Update _lastCANtime, zero _errorStatus

    void sndACK(uint8_t theReg, uint8_t thePayload);
    // Handler acknowledge

private:
    CANnode* _myCAN;					      // CAN object of the MASTER for communication
    uint8_t _myID = 0xFF;			        // This prop's id
    uint8_t _regs[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};;		// This prop's current status (stored locally)
    unsigned long _lastCANtime = 0;		// Last time (millis()) that the prop communicated over CAN
    uint8_t _errorStatus = 0;			    // Number of non-responses in a row
    uint8_t _hasChanged = 0;			    // Whether a given register is updated
};

void assignMessage(Node p[], uint8_t size, CANmsg msg, uint8_t data[], uint8_t len);
// Take an incoming message, see which prop it was sent from, and assign the relevant information to that prop


#endif
/*
  END FILE
*/
