/*
*   DriverLora.cpp
*/

#include <DriverLora.h>

DriverLora::DriverLora()
    :
    _mode(RLModeInitialising),
    _thisAddress(RH_BROADCAST_ADDRESS),
    _txHeaderTo(RH_BROADCAST_ADDRESS),
    _txHeaderFrom(RH_BROADCAST_ADDRESS),
    _txHeaderId(0),
    _txHeaderFlags(0),
    _rxBad(0),
    _rxGood(0),
    _txGood(0),
    _cad_timeout(0)
{
}

bool DriverLora::init(){
    return true;
}

// Blocks until a valid message is received
void DriverLora::waitAvailable(uint16_t polldelay){
    while (!available()){
        YIELD;
        if (polldelay)
        delay(polldelay);
    }
}

// Blocks until a valid message is received or timeout expires
// Return true if there is a message available
// Works correctly even on millis() rollover
bool DriverLora::waitAvailableTimeout(uint16_t timeout, uint16_t polldelay){
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout){

        if (available())
            return true;
        YIELD;

        if (polldelay)
            delay(polldelay);
    }
    return false;
}

bool DriverLora::waitPacketSent(){
    while (_mode == RLModeTx)
	    YIELD; // Wait for any previous transmit to finish
    return true;
}

bool DriverLora::waitPacketSent(uint16_t timeout){
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout)
    {
        if (_mode != RLModeTx) // Any previous transmit finished?
           return true;
	    YIELD;
    }
    return false;
}

// Wait until no channel activity detected or timeout
bool DriverLora::waitCAD(){
    if (!_cad_timeout)
	return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
    unsigned long t = millis();
    while (isChannelActive())
    {
        if (millis() - t > _cad_timeout) 
            return false;

        #if (RL_PLATFORM == RH_PLATFORM_STM32) // stdlib on STMF103 gets confused if random is redefined
            delay(_random(1, 10) * 100);
        #else
            delay(random(1, 10) * 100); // Should these values be configurable? Macros?
        #endif
    }

    return true;
}

bool DriverLora::isChannelActive(){
    return false;
}

void DriverLora::setPromiscuous(bool promiscuous){
    _promiscuous = promiscuous;
}

void DriverLora::setThisAddress(uint8_t address){
    _thisAddress = address;
}

void DriverLora::setHeaderTo(uint8_t to){
    _txHeaderTo = to;
}

void DriverLora::setHeaderFrom(uint8_t from){
    _txHeaderFrom = from;
}

void DriverLora::setHeaderId(uint8_t id){
    _txHeaderId = id;
}

void DriverLora::setHeaderFlags(uint8_t set, uint8_t clear){
    _txHeaderFlags &= ~clear;
    _txHeaderFlags |= set;
}

uint8_t DriverLora::headerTo(){
    return _rxHeaderTo;
}

uint8_t DriverLora::headerFrom(){
    return _rxHeaderFrom;
}

uint8_t DriverLora::headerId(){
    return _rxHeaderId;
}

uint8_t DriverLora::headerFlags(){
    return _rxHeaderFlags;
}

int16_t DriverLora::lastRssi(){
    return _lastRssi;
}

DriverLora::RLMode DriverLora::mode(){
    return _mode;
}

void DriverLora::setMode(RLMode mode){
    _mode = mode;
}

bool DriverLora::sleep(){
    return false;
}

// Diagnostic help
void DriverLora::printBuffer(const char* prompt, const uint8_t* buf, uint8_t len){
    #ifdef RL_HAVE_SERIAL
        Serial.println(prompt);
        uint8_t i;
        for (i = 0; i < len; i++){
            if (i % 16 == 15){
                Serial.println(buf[i], HEX);
            } else {
                Serial.print(buf[i], HEX);
                Serial.print(' ');
            }
        }
        Serial.println("");
    #endif
}

uint16_t DriverLora::rxBad(){
    return _rxBad;
}

uint16_t DriverLora::rxGood(){
    return _rxGood;
}

uint16_t DriverLora::txGood(){
    return _txGood;
}

void DriverLora::setCADTimeout(unsigned long cad_timeout){
    _cad_timeout = cad_timeout;
}
