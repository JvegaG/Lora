#ifndef DriverLora_h
#define DriverLora_h

#include <RadioLora.h>

// Defines bits of the FLAGS header reserved for use by the RadioHead library and 
// the flags available for use by applications
#define RL_FLAGS_RESERVED                 0xf0
#define RL_FLAGS_APPLICATION_SPECIFIC     0x0f
#define RL_FLAGS_NONE                     0

// Default timeout for waitCAD() in ms
#define RL_CAD_DEFAULT_TIMEOUT            10000


class DriverLora
{
    public:
        typedef enum
        {
            RLModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
            RLModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
            RLModeIdle,             ///< Transport is idle.
            RLModeTx,               ///< Transport is in the process of transmitting a message.
            RLModeRx,               ///< Transport is in the process of receiving a message.
            RLModeCad               ///< Transport is in the process of detecting channel activity (if supported)
        } RLMode;

        DriverLora();

        virtual ~DriverLora() {};
        
        virtual bool init();

        virtual bool available() = 0;

        virtual bool recv(uint8_t* buf, uint8_t* len) = 0;

        virtual bool send(const uint8_t* data, uint8_t len) = 0;

        virtual uint8_t maxMessageLength() = 0;

        virtual void waitAvailable(uint16_t polldelay = 0);

        virtual bool waitPacketSent();

        virtual bool waitPacketSent(uint16_t timeout);

        virtual bool waitAvailableTimeout(uint16_t timeout, uint16_t polldelay = 0);

        virtual bool waitCAD();

        void setCADTimeout(unsigned long cad_timeout);

        virtual bool isChannelActive();

        virtual void setThisAddress(uint8_t thisAddress);

        virtual void setHeaderTo(uint8_t to);

        virtual void setHeaderFrom(uint8_t from);

        virtual void setHeaderId(uint8_t id);

        virtual void setHeaderFlags(uint8_t set, uint8_t clear = RL_FLAGS_APPLICATION_SPECIFIC);

        virtual void setPromiscuous(bool promiscuous);

        virtual uint8_t headerTo();

        virtual uint8_t headerFrom();

        virtual uint8_t headerId();

        virtual uint8_t headerFlags();

        virtual int16_t lastRssi();

        virtual RLMode mode();

        virtual void setMode(RLMode mode);

        virtual bool sleep();

        static void printBuffer(const char* prompt, const uint8_t* buf, uint8_t len);

        virtual uint16_t rxBad();

        virtual uint16_t rxGood();

        virtual uint16_t txGood();

    protected:

        /// The current transport operating mode
        volatile RLMode _mode;

        /// This node id
        uint8_t _thisAddress;
        
        /// Whether the transport is in promiscuous mode
        bool _promiscuous;

        /// TO header in the last received mesasge
        volatile uint8_t _rxHeaderTo;

        /// FROM header in the last received mesasge
        volatile uint8_t _rxHeaderFrom;

        /// ID header in the last received mesasge
        volatile uint8_t _rxHeaderId;

        /// FLAGS header in the last received mesasge
        volatile uint8_t _rxHeaderFlags;

        /// TO header to send in all messages
        uint8_t _txHeaderTo;

        /// FROM header to send in all messages
        uint8_t _txHeaderFrom;

        /// ID header to send in all messages
        uint8_t _txHeaderId;

        /// FLAGS header to send in all messages
        uint8_t _txHeaderFlags;

        /// The value of the last received RSSI value, in some transport specific units
        volatile int16_t _lastRssi;

        /// Count of the number of bad messages (eg bad checksum etc) received
        volatile uint16_t _rxBad;

        /// Count of the number of successfully transmitted messaged
        volatile uint16_t _rxGood;

        /// Count of the number of bad messages (correct checksum etc) received
        volatile uint16_t _txGood;
        
        /// Channel activity detected
        volatile bool _cad;

        /// Channel activity timeout in ms
        unsigned int _cad_timeout;

    private:

};

#endif 