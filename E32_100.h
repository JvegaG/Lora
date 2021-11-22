/*
*   Personalizacion de Instrucciones para modulo E32-TTL-100
*   Todos los datos estan basados en el datasheet del modulo RF
*/

#ifndef E32_100_h
#define E32_100_h

#include <DriverLora.h>
#include <Stream.h>

// se pueden mandar 58 bytes por paquete (el modulo soporta hasta 512bytes)
#define E32_MAX_PAYLOAD_LEN 58

#define E32_HEADER_LEN 5 // 5 bytes son los utilizados para la configuracion del módulo

// Numero maximo de bytes que pueden ser enviados por el modulo por paquete
#define E32_MAX_MESSAGE_LEN (E32_MAX_PAYLOAD_LEN - E32_HEADER_LEN)

// Comandos para alterar comportamiento del módulo
#define E32_COMMAND_WRITE_PARAMS_SAVE         0xC0  // alterar parametros de trabajo (se guardan a pesar de estar apagado el módulo)
#define E32_COMMAND_READ_PARAMS               0xC1  // leer los parametros seteados en el módulo ( C1 C1 C1 )
#define E32_COMMAND_WRITE_PARAMS_NOSAVE       0xC2  // alterar parametros de trabajo (NO se guardan cuando se apaga el módulo)
#define E32_COMMAND_READ_VERSION              0xC3  // retorna la version del módulo ( C3 C3 C3 )
#define E32_COMMAND_RESET                     0xC4  // resetea el módulo ( C4 C4 C4 )

///////////////////////////////////////////////////
// Parametros de configuración

// Speed Parameter
// speed 7,6 bits: UART parity
#define E32_PARAM_SPED_UART_MODE_MASK         0xC0  // es igual al 8N1 según datasheet
#define E32_PARAM_SPED_UART_MODE_8N1          0x00
#define E32_PARAM_SPED_UART_MODE_8O1          0x40
#define E32_PARAM_SPED_UART_MODE_8E1          0x80

// speed 5,4,3 bits: UART baud rate
#define E32_PARAM_SPED_UART_BAUD_MASK         0x38
#define E32_PARAM_SPED_UART_BAUD_1200         0x00
#define E32_PARAM_SPED_UART_BAUD_2400         0x08
#define E32_PARAM_SPED_UART_BAUD_4800         0x10
#define E32_PARAM_SPED_UART_BAUD_9600         0x18
#define E32_PARAM_SPED_UART_BAUD_19200        0x20
#define E32_PARAM_SPED_UART_BAUD_38400        0x28
#define E32_PARAM_SPED_UART_BAUD_57600        0x30
#define E32_PARAM_SPED_UART_BAUD_115200       0x38

// speed 2,1,0 bits: Air date rate
#define E32_PARAM_SPED_DATARATE_MASK          0x07
#define E32_PARAM_SPED_DATARATE_0_3KBPS       0x00
#define E32_PARAM_SPED_DATARATE_1_2KBPS       0x01
#define E32_PARAM_SPED_DATARATE_2_4KBPS       0x02
#define E32_PARAM_SPED_DATARATE_4_8KBPS       0x03
#define E32_PARAM_SPED_DATARATE_9_6KBPS       0x04
#define E32_PARAM_SPED_DATARATE_19_2KBPS_1    0x05
#define E32_PARAM_SPED_DATARATE_19_2KBPS_2    0x06
#define E32_PARAM_SPED_DATARATE_19_2KBPS_3    0x07

// Channel Parameter
// Este parametro no se configura aqui, los bits de orden 7,6,5 no tienen valores relevantes
// por lo tanto los valores que valen son del 4-0 , van desde 0x00 al 0x1F (410MHz a 441MHz aprox.)
// el valor por default es 0x17 (433 MHz)
// formula: 410MHz + Channel*1MHz

// Option Parameter 7 bit
// Fixed transmission
#define E32_PARAM_OPTION_FIXED_MASK           0x80
#define E32_PARAM_OPTION_TRANSPARENT          0x00


// IO Drive Mode 6 bit
#define E32_PARAM_OPTION_IODRIVE_MASK         0x40  // TXD and AUX push-pull outputs, RXD pull-up inputs (default)
#define E32_PARAM_OPTION_IODRIVE_OFF          0x00  // TXD and AUX open-Collector outputs, RXD open-Collector inputs

// Wireless wake-up time 5,4,3 bits
#define E32_PARAM_OPTION_WAKEUP_TIME_250MS    0x00  // (default)
#define E32_PARAM_OPTION_WAKEUP_TIME_500MS    0x08
#define E32_PARAM_OPTION_WAKEUP_TIME_750MS    0x10
#define E32_PARAM_OPTION_WAKEUP_TIME_1000MS   0x18
#define E32_PARAM_OPTION_WAKEUP_TIME_1250MS   0x20
#define E32_PARAM_OPTION_WAKEUP_TIME_1500MS   0x28
#define E32_PARAM_OPTION_WAKEUP_TIME_1750MS   0x30
#define E32_PARAM_OPTION_WAKEUP_TIME_2000MS   0x38

// FEC switch 2 bit
#define E32_PARAM_OPTION_FEC_MASK             0x04
#define E32_PARAM_OPTION_FEC_OFF              0x00

// Tansmision pover 1,0 bit
#define E32_PARAM_OPTION_POWER_MASK           0x03
#define E32_PARAM_OPTION_POWER_20DBM          0x00
#define E32_PARAM_OPTION_POWER_17DBM          0x01
#define E32_PARAM_OPTION_POWER_14DBM          0x02
#define E32_PARAM_OPTION_POWER_10DBM          0x03

class E32_100 : public DriverLora {
    public:

        typedef enum
        {
            DataRate_0_3kbps  = E32_PARAM_SPED_DATARATE_0_3KBPS,
            DataRate_1_2kbps  = E32_PARAM_SPED_DATARATE_1_2KBPS,
            DataRate_2_4kbps  = E32_PARAM_SPED_DATARATE_2_4KBPS,
            DataRate_4_8kbps  = E32_PARAM_SPED_DATARATE_4_8KBPS,
            DataRate_9_6kbps = E32_PARAM_SPED_DATARATE_9_6KBPS,
            DataRate_19_2kbps_1 = E32_PARAM_SPED_DATARATE_19_2KBPS_1,
            DataRate_19_2kbps_2 = E32_PARAM_SPED_DATARATE_19_2KBPS_2,
            DataRate_19_2kbps_3 = E32_PARAM_SPED_DATARATE_19_2KBPS_3
        } DataRate;
        
        typedef enum
        {
            Power20dBm = E32_PARAM_OPTION_POWER_20DBM,
            Power17dBm = E32_PARAM_OPTION_POWER_17DBM,
            Power14dBm = E32_PARAM_OPTION_POWER_14DBM,
            Power10dBm = E32_PARAM_OPTION_POWER_10DBM,
        } PowerLevel;

        typedef enum
        {
            BaudRate1200   = E32_PARAM_SPED_UART_BAUD_1200,
            BaudRate2400   = E32_PARAM_SPED_UART_BAUD_2400,
            BaudRate4800   = E32_PARAM_SPED_UART_BAUD_4800,
            BaudRate9600   = E32_PARAM_SPED_UART_BAUD_9600,
            BaudRate19200  = E32_PARAM_SPED_UART_BAUD_19200,
            BaudRate38400  = E32_PARAM_SPED_UART_BAUD_38400,
            BaudRate57600  = E32_PARAM_SPED_UART_BAUD_57600,
            BaudRate115200 = E32_PARAM_SPED_UART_BAUD_115200,
        } BaudRate;

        typedef enum
        {
            Parity8N1 = E32_PARAM_SPED_UART_MODE_8N1,
            Parity8O1 = E32_PARAM_SPED_UART_MODE_8O1,
            Parity8E1 = E32_PARAM_SPED_UART_MODE_8E1,
        } Parity;

        typedef enum
        {
            transTransmission = E32_PARAM_OPTION_TRANSPARENT,
            fixedTransmission = E32_PARAM_OPTION_FIXED_MASK,
        } Transmission;

        typedef enum
        {
            fec_On = E32_PARAM_OPTION_FEC_MASK,
            fec_off = E32_PARAM_OPTION_FEC_OFF,
        } FEC;

        E32_100(Stream *s=&Serial, uint8_t m0_pin = 4, uint8_t m1_pin = 5, uint8_t aux_pin = 8);

        bool init();

        bool available();

        bool recv(uint8_t* buf, uint8_t* len);

        bool send(const uint8_t* data, uint8_t len);

        uint8_T maxMessageLength();

        bool waitPacketSent();

        bool setAddress(uint8_t addh, uint8_t addl);

        bool setDataRate(DataRate rate);

        bool setPower(PowerLevel level);

        bool setBaudRate(BaudRate rate = BaudRate9600, Parity Parity = Parity8N1);

        bool setFrequency(uint16_t frequency);

        bool setTypeTransmission(Transmission transmissionType);

        bool setFEC(FEC fec);

    protected:
        typedef enum
        {
            ModeNormal = 0,
            ModeWakeUp,
            ModePowerSaving,
            ModeSleep
        } OperatingMode;
        
        typedef struct
        {
            uint8_t head;
            uint8_t addh;
            uint8_t addl;
            uint8_t sped;
            uint8_t chan;
            uint8_t option;
        } Parameters;

        void setOperatingMode(OperatingMode mode);

        bool getVersion();

        void waitAuxHigh();

        void waitAuxLow();

        bool reset();

        bool readParameters(Parameters& params);

        bool writeParameters(Parameters& params, bool save = false);

        void validateRxBuf();

        void clearRxBuf();

    private:

        Stream* _s;

        uint8_t _m0_pin;

        uint8_t _m1_pin;

        uint8_t _aux_pin;

        uint8_t _bufLen;

        uint8_t _buf[E32_MAX_PAYLOAD_LEN];

        bool _rxBufValid;
};

#endif