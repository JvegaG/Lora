/*
*   Personalización de Libreria RadioHead, Proyecto Lora
*   Autor: Jesus Vega
*   20/11/2021 2:40pm
*/

#ifndef RadioLora_h
#define RadioLora_h

// Nombres simbolicos de los tipo de plataforma soportados
#define RH_PLATFORM_ARDUINO 1
#define RH_PLATFORM_GENERIC_AVR8 2
#define RH_PLATFORM_STM32 3
#define RH_PLATFORM_ESP8266 4
#define RH_PLATFORM_ESP32 5
						   
////////////////////////////////////////////////////
// Seleccionar plataforma automaticamente
#ifndef RH_PLATFORM
  #if (defined(MPIDE) && MPIDE >= 150 && defined(ARDUINO))
    #define RH_PLATFORM RH_PLATFORM_CHIPKIT_CORE
  #elif defined(ARDUINO)
    #define RH_PLATFORM RH_PLATFORM_ARDUINO
  #elif defined(MCU_STM32F103RE)
    #define RH_PLATFORM RH_PLATFORM_STM32
  #elif defined(ESP8266)
    #define RH_PLATFORM RH_PLATFORM_ESP8266
  #elif defined(ESP32)
    #define RH_PLATFORM RH_PLATFORM_ESP32
  #else
    #error Platform not defined!
  #endif
#endif
						   
////////////////////////////////////////////////////
// Cabeceras de acuerdo a una plataforma especifica
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
  // Version de Arduino IDE, mayor a 1.00
  #if (ARDUINO >= 100)
    #include <Arduino.h>
  #else
    #include <wiring.h>
  #endif
  #include <SPI.h>
  #define RH_HAVE_HARDWARE_SPI
  #define RH_HAVE_SERIAL

#elif (RH_PLATFORM == RH_PLATFORM_ESP8266) // Micro ESP8266 en Arduino IDE
  #include <Arduino.h>
  #include <SPI.h>
  #define RH_HAVE_HARDWARE_SPI
  #define RH_HAVE_SERIAL
  #define RH_MISSING_SPIUSINGINTERRUPT

#elif (RH_PLATFORM == RH_PLATFORM_ESP32)   // Micro ESP32 en Aduino IDE
  #include <Arduino.h>
  #include <SPI.h>
  #define RH_HAVE_HARDWARE_SPI
  #define RH_HAVE_SERIAL
  #define RH_MISSING_SPIUSINGINTERRUPT
  // ESP32 has 2 user SPI buses: VSPI and HSPI. They are essentially identical, but use different pins.
  // The usual, default bus VSPI (available as SPI object in Arduino) uses pins:
  // SCLK:      18
  // MISO:      19
  // MOSI:      23
  // SS:	       5
  // The other HSPI bus uses pins
  // SCLK:      14
  // MISO:      12
  // MOSI:      12
  // SS:	       15
  // By default RadioHead uses VSPI, but you can make it use HSPI by defining this:
  //#define RH_ESP32_USE_HSPI

#elif (RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE)
  #include <WProgram.h>
  #include <string.h>
  #include <SPI.h>
  #define RH_HAVE_HARDWARE_SPI
  #define memcpy_P memcpy
  #define RH_HAVE_SERIAL

#elif (RH_PLATFORM == RH_PLATFORM_STM32)
  #include <STM32ArduinoCompat/wirish.h>	
  #include <stdint.h>
  #include <string.h>
  #include <STM32ArduinoCompat/HardwareSPI.h>
  #define RH_HAVE_HARDWARE_SPI
  // Definir que timer utilizar en Maple
  #define MAPLE_TIMER 1
  #define PROGMEM
  #define memcpy_P memcpy
  #define Serial SerialUSB
  #define RH_HAVE_SERIAL

#elif (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8) 
  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include <util/delay.h>
  #include <string.h>
  #include <stdbool.h>
  #define RH_HAVE_HARDWARE_SPI
  #include <SPI.h>

#else
  #error Platform Unkonwn!
#endif

////////////////////////////////////////////////////
// This is an attempt to make a portable atomic block
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
  #if defined(__arm__)
    #include <RHutil/atomic.h>
  #else
    #include <util/atomic.h>
  #endif
  
#elif (RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE)
  // UsingChipKIT Core on Arduino IDE
  #define ATOMIC_BLOCK_START unsigned int __status = disableInterrupts(); { 
  #define ATOMIC_BLOCK_END } restoreInterrupts(__status);

#elif (RH_PLATFORM == RH_PLATFORM_ESP8266)
  // See hardware/esp8266/2.0.0/cores/esp8266/Arduino.h
  #define ATOMIC_BLOCK_START { 
    uint32_t __savedPS = xt_rsil(15);
    #define ATOMIC_BLOCK_END xt_wsr_ps(__savedPS);
  }
#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
  // jPerotto see hardware/esp32/1.0.4/tools/sdk/include/esp32/xtensa/xruntime.h
  #define ATOMIC_BLOCK_START uint32_t volatile register ilevel = XTOS_DISABLE_ALL_INTERRUPTS;
  #define ATOMIC_BLOCK_END XTOS_RESTORE_INTLEVEL(ilevel);
#else 
  // TO BE DONE:
  #define ATOMIC_BLOCK_START
  #define ATOMIC_BLOCK_END
#endif

////////////////////////////////////////////////////
// Try to be compatible with systems that support yield() and multitasking
// instead of spin-loops
// Recent Arduino IDE
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO && ARDUINO >= 155) 
  #define YIELD yield();
#elif (RH_PLATFORM == RH_PLATFORM_ESP8266)
  // ESP8266 also has it
  #define YIELD yield();
#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
  // ESP32 also has it
  #define YIELD yield();
#else
  #define YIELD
#endif

////////////////////////////////////////////////////
// digitalPinToInterrupt is not available prior to Arduino 1.5.6 and 1.0.6
// See http://arduino.cc/en/Reference/attachInterrupt
#ifndef NOT_AN_INTERRUPT
  #define NOT_AN_INTERRUPT -1
#endif
#ifndef digitalPinToInterrupt
  #if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && !defined(__arm__)
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      // Arduino Mega, Mega ADK, Mega Pro
      // 2->0, 3->1, 21->2, 20->3, 19->4, 18->5
      #define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) >= 18 && (p) <= 21 ? 23 - (p) : NOT_AN_INTERRUPT)))

    #elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 
      // Arduino 1284 and 1284P - See Manicbug and Optiboot
      // 10->0, 11->1, 2->2
      #define digitalPinToInterrupt(p) ((p) == 10 ? 0 : ((p) == 11 ? 1 : ((p) == 2 ? 2 : NOT_AN_INTERRUPT)))

    #elif defined(__AVR_ATmega32U4__)
      // Leonardo, Yun, Micro, Pro Micro, Flora, Esplora
      // 3->0, 2->1, 0->2, 1->3, 7->4
      #define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))

    #else
      // All other arduino except Due:
      // Serial Arduino, Extreme, NG, BT, Uno, Diecimila, Duemilanove, Nano, Menta, Pro, Mini 04, Fio, LilyPad, Ethernet etc
      // 2->0, 3->1
      #define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))

    #endif
    
  #elif (RH_PLATFORM == RH_PLATFORM_UNO32) || (RH_PLATFORM == RH_PLATFORM_CHIPKIT_CORE)
    // Hmmm, this is correct for Uno32, but what about other boards on ChipKIT Core?
    #define digitalPinToInterrupt(p) ((p) == 38 ? 0 : ((p) == 2 ? 1 : ((p) == 7 ? 2 : ((p) == 8 ? 3 : ((p) == 735 ? 4 : NOT_AN_INTERRUPT)))))

  #elif (RH_PLATFORM == RH_PLATFORM_ESP32)
    #define digitalPinToInterrupt(p) (((p) < 40) ? (p) : -1)

  #elif (RH_PLATFORM == RH_PLATFORM_ESP8266)
    #define digitalPinToInterrupt(p) (((p) < EXTERNAL_NUM_INTERRUPTS)? (p) : NOT_AN_INTERRUPT)

  #else
    // Everything else (including Due and Teensy) interrupt number the same as the interrupt pin number
    #define digitalPinToInterrupt(p) (p)
  #endif
#endif

// On some platforms, attachInterrupt() takes a pin number, not an interrupt number
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO) && defined (__arm__) && (defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_SAM_DUE)) || defined(ARDUINO_ARCH_STM32L0) 
  #define RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
#endif

// Slave select pin, some platforms such as ATTiny do not define it.
// ESP32 pins_arduino.h uses static const uint8_t SS = <UINT>; instead
// of a #define to declare the SS constant.
#if (RH_PLATFORM != RH_PLATFORM_ESP32)
  #ifndef SS
    #define SS 10
  #endif
#endif

// Some platforms require special attributes for interrupt routines						   
#if (RH_PLATFORM == RH_PLATFORM_ESP8266)
  // interrupt handler and related code must be in RAM on ESP8266,
  // according to issue #46.
  #define RH_INTERRUPT_ATTR ICACHE_RAM_ATTR
						   
#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
  #define RH_INTERRUPT_ATTR IRAM_ATTR
#else
  #define RH_INTERRUPT_ATTR
#endif

// These defs cause trouble on some versions of Arduino
#undef abs
#undef round
#undef double

// Some platforms need a mutex for multihreaded case
#ifdef RH_USE_MUTEX
  #include <pthread.h>
  #define RH_DECLARE_MUTEX(X) pthread_mutex_t X;						   
  #define RH_MUTEX_INIT(X) pthread_mutex_init(&X, NULL)
  #define RH_MUTEX_LOCK(X) pthread_mutex_lock(&X)
  #define RH_MUTEX_UNLOCK(X) pthread_mutex_unlock(&X)						   
#else
  #define RH_DECLARE_MUTEX(X)
  #define RH_MUTEX_INIT(X)
  #define RH_MUTEX_LOCK(X)
  #define RH_MUTEX_UNLOCK(X)
#endif

// This is the address that indicates a broadcast
#define RH_BROADCAST_ADDRESS 0xff

// Specifies an invalid IO pin selection
#define RH_INVALID_PIN 0xff

// Uncomment this is to enable Encryption (see RHEncryptedDriver):
// But ensure you have installed the Crypto directory from arduinolibs first:
// http://rweather.github.io/arduinolibs/index.html
//#define RH_ENABLE_ENCRYPTION_MODULE

// Some platforms like RocketScream need this to see debug Serial output from within RH
// and if it goes to Serial, get a hang after a few minutes.
//#define Serial SerialUSB

#endif
