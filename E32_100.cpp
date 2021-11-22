/*
*   E32_100.cpp
*
*/

#include <RadioLora.h>
#ifdef RH_HAVE_SERIAL

#include <E32_100.h>

E32_100::E32_100(Stream *s, uint8_t m0_pin, uint8_t m1_pin, uint8_t aux_pin)
    : 
    _s(s),
    _m0_pin(m0_pin),
    _m1_pin(m1_pin),
    _aux_pin(aux_pin) 
{
    pinMode(_aux_pin, INPUT);
    //digitalWrite(_m0_pin, HIGH);
    //digitalWrite(_m1_pin, HIGH);
    pinMode(_m0_pin, OUTPUT);
    pinMode(_m1_pin, OUTPUT);
}

bool E32_100::init()
{
  // When a message is available, Aux will go low 5 msec before the first character is output
  // So if we ever wait more than this period of time after Aux low, can conclude there will be no data
  _s -> setTimeout(10);

  // Wait until the module is connected
  waitAuxHigh();

  if (!getVersion())
      return false;
  
  setMode(RHModeRx);
  clearRxBuf();

  if (!setDataRate(DataRate5kbps))
    return false;

  if (!setPower(Power21dBm))
    return false;

  //  if (!setBaudRate(BaudRate9600, Parity8N1))
  //  return false;

  if (!setFrequency(433))
    return false;
  
  return true;
}

void E32_100::waitAuxHigh() {
  while (digitalRead(_aux_pin) == LOW);
}

void E32_100::waitAuxLow() {
  while (digitalRead(_aux_pin) == HIGH);
}

void E32_100::setOperatingMode(OperatingMode mode) {
  waitAuxHigh();    // Esperamos que el modulo se desocupe antes de cambiar de modo
  switch (mode) {
    case ModeNormal:  // M0=0 and M1=0 modo normal
      digitalWrite(_m0_pin, LOW);
      digitalWrite(_m1_pin, LOW);
      break;
      
    case ModeWakeUp:  // M0=1 and M1=0 modo wake-up
      digitalWrite(_m0_pin, HIGH);
      digitalWrite(_m1_pin, LOW);
      break;
      
    case ModePowerSaving: // M0=0 and M1=1 modo power saving
      digitalWrite(_m0_pin, LOW);
      digitalWrite(_m1_pin, HIGH);
      break;
      
    case ModeSleep: // M0=1 and M1=1 modo sleep
      digitalWrite(_m0_pin, HIGH);
      digitalWrite(_m1_pin, HIGH);
      break;
  }
  // Según datasheet demora alrededor de 2ms 
  // en cambiar de un modo a otro (en pasar de Low a High el pin Aux)

  //delay(10); // es necesario el delay? estaba en la libreria RadioHead (no lo veo relevante)
  waitAuxHigh();
}


//////////////////////////////////////////////
// Funciones Básicas - Configuraciones y lectura de Parametros
bool E32_100::reset() {
  setOperatingMode(ModeSleep);
  uint8_t resetCommand[] = {E32_COMMAND_RESET, E32_COMMAND_RESET, E32_COMMAND_RESET};
  size_t result = _s -> write(resetCommand, sizeof(resetCommand));
  // Luego de mandar la intruccion no unico que debe hacerse es esperar el Aux a High
  setOperatingMode(ModeNormal);
  return (result == sizeof(resetCommand));
}

bool E32_100::readParameters(Parameters& params) {
  setOperatingMode(ModeSleep);
  uint8_t readParamsCommand[] = { E32_COMMAND_READ_PARAMS, E32_COMMAND_READ_PARAMS, E32_COMMAND_READ_PARAMS };
  _s -> write(readParamsCommand, sizeof(readParamsCommand));
  size_t result = _s -> readBytes((char*)&params, sizeof(params)); // default 1 sec timeout
  setOperatingMode(ModeNormal);
  return (result == sizeof(Parameters));
}

bool E32_100::writeParameters(Parameters& params, bool save) {
  setOperatingMode(ModeSleep);

  // Elegir entre C0 (guardado permanente) y C2
  params.head = save ? E32_COMMAND_WRITE_PARAMS_SAVE : E32_COMMAND_WRITE_PARAMS_NOSAVE;
  size_t result = _s -> write((uint8_t*)&params, sizeof(params));
  if (result != sizeof(params))
    return false;
  
  // Now we expect to get the same data back
  result = _s -> readBytes((char*)&params, sizeof(params));
  if (result != sizeof(params))
    return false;

  // Without a little delay here, writing params often fails
  // delay(20);
    
  setOperatingMode(ModeNormal);
  return result == sizeof(params);
}

bool E32_100::getVersion() {
  setOperatingMode(ModeSleep);

  uint8_t readVersionCommand[] = { E32_COMMAND_READ_VERSION, E32_COMMAND_READ_VERSION, E32_COMMAND_READ_VERSION };
  _s -> write(readVersionCommand, sizeof(readVersionCommand));
  uint8_t version[4];
  size_t result = _s -> readBytes((char *)version, sizeof(version)); // default 1 sec timeout

  setOperatingMode(ModeNormal);

  if (result == 4) {
    // Successful read => C3 32 XX YY si es un módulo E32 
    // (XX -> version number, YY -> module feature)
    if (version[0] != 0xc3 || version [1] != 0x32) 
      return false; // Not an E32

    return true;
  } else {
    return false; // Read failed: no module? Wrong baud?
  }
}

//////////////////////////////////////////////
// Configuración de Módulo 
bool E32_100::setAddress(uint8_t addh, uint8_t addl){
  Parameters params;
  if(!readParameters(params))
    return false;
  
  params.addh = addh;
  params.addl = addl;
  return writeParameters(params);
}

bool E32_100::setDataRate(DataRate rate) {
  Parameters params;
  if (!readParameters(params))
    return false;

  params.sped &= ~E32_PARAM_SPED_DATARATE_MASK;
  params.sped |= (rate & E32_PARAM_SPED_DATARATE_MASK);
  return writeParameters(params);
}

bool E32_100::setPower(PowerLevel level) {
  Parameters params;
  if (!readParameters(params))
    return false;

  params.option &= ~E32_PARAM_OPTION_POWER_MASK;
  params.option |= (level & E32_PARAM_OPTION_POWER_MASK);
  return writeParameters(params);
}

bool E32_100::setBaudRate(BaudRate rate, Parity parity) {
  Parameters params;
  if (!readParameters(params))
    return false;

  // Set BaudRate
  params.sped &= ~E32_PARAM_SPED_UART_BAUD_MASK;
  params.sped |= (rate & E32_PARAM_SPED_UART_BAUD_MASK);

  // Set the parity
  params.sped &= ~E32_PARAM_SPED_UART_MODE_MASK;
  params.sped |= (parity & E32_PARAM_SPED_UART_MODE_MASK);
  
  return writeParameters(params);
}

bool E32_100::setFrequency(uint16_t frequency) {    
  // funcion en duda... es necesario mandar el channel pero en hexadecimal
  if (frequency < 410 || frequency > 441)
    return false;
  
  Parameters params;
  if (!readParameters(params))
    return false;

  params.chan = frequency - 410;
  return writeParameters(params); 
}

bool E32_100::setTypeTransmission(Transmission transmissionType){
  Parameters params;
  if (!readParameters(params))
    return false;

  params.option &= ~E32_PARAM_OPTION_FIXED_MASK;
  params.option |= (transmissionType & E32_PARAM_OPTION_FIXED_MASK);
  return writeParameters(params);
}

bool E32_100::setFEC(FEC fec){
  Parameters params;
  if (!readParameters(params))
    return false;

  params.option &= ~E32_PARAM_OPTION_FEC_MASK;
  params.option |= (fec & E32_PARAM_OPTION_FEC_MASK);
  return writeParameters(params);
}

//////////////////////////////////////////////
// Recepción y envío de datos
uint8_t E32_100::maxMessageLength(){
  return E32_MAX_MESSAGE_LEN;
}

bool E32_100::waitPacketSent(){
  if(_mode == RLModeTx)
    waitAuxHigh();

  setMode(RLModeRx);
  return true;
}

bool E32_100::send(const uint8_t* data, uint8_t len){

  //La libreria RadioHead no permite mandar datos mayores a 53 bytes
  if (len > E32_MAX_MESSAGE_LEN)
    return false;

  waitPacketSent();

  // Set up the headers
  _buf[0] = len + E32_HEADER_LEN; // Number of octets in teh whole message
  _buf[1] = _txHeaderTo;
  _buf[2] = _txHeaderFrom;
  _buf[3] = _txHeaderId;
  _buf[4] = _txHeaderFlags;

  // REVISIT: do we really have to do this? perhaps just write it after writing the header?
  memcpy(_buf + E32_HEADER_LEN, data, len);
  
  _s -> write(_buf, len + E32_HEADER_LEN);
  setMode(RLModeTx);
  _txGood++;
  // Aux will return high when the TX buffer is empty
  
  return true;
}

bool E32_100::recv(uint8_t* buf, uint8_t* len){
  if (!available())
	  return false;
  
  if (buf && len){
    // Skip the 4 headers that are at the beginning of the rxBuf
    if (*len > _bufLen - RH_E32_HEADER_LEN)
        *len = _bufLen - RH_E32_HEADER_LEN;

    memcpy(buf, _buf + RH_E32_HEADER_LEN, *len);
  }
  clearRxBuf(); // This message accepted and cleared
  return true;
}


bool E32_100::available(){
  // Caution: long packets could be sent in several bursts
  if (!_rxBufValid){
    if (_mode == RLModeTx)
      return false;

    if (!_s -> available())
      return false;

    // Suck up all the characters we can
    uint8_t data;
    while (_s->readBytes((char *)&data, 1) == 1) {
      _buf[_bufLen++] = data;
    }
	  
	  if (_bufLen < E32_HEADER_LEN){
	    return false;
	  } else if (_bufLen < _buf[0]){
	    return false;
	  } else if (   _bufLen > _buf[0] || _bufLen > RH_E32_MAX_PAYLOAD_LEN){
	    clearRxBuf();
	    _rxBad++;
	    return false;
	  }

    // Else it a partial or complete message, test it
    //	printBuffer("read success", _buf, _bufLen);
    validateRxBuf(); 
  }
  
  return _rxBufValid;
}

// Check whether the latest received message is complete and uncorrupted
void E32_100::validateRxBuf(){
  if (_bufLen < RH_E32_HEADER_LEN)
	  return; // Too short to be a real message
  
  if (_bufLen != _buf[0])
    return; // Do we have all the message?
    
  // Extract the 4 headers
  _rxHeaderTo    = _buf[1];
  _rxHeaderFrom  = _buf[2];
  _rxHeaderId    = _buf[3];
  _rxHeaderFlags = _buf[4];
  
  if (_promiscuous || _rxHeaderTo == _thisAddress || _rxHeaderTo == RH_BROADCAST_ADDRESS){
	  _rxGood++;
	  _rxBufValid = true;
  }

}

void E32_100::clearRxBuf(){
  _rxBufValid = false;
  _bufLen = 0;
}


#endif