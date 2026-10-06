/*
  my_machine_map.h - Board map for BIGTREETECH Octopus Pro v1.0 (F446, 12 MHz)

  Based on btt_octopus_pro_map.h (v1.1). Only pins that differ for the
  v1.0 board / this machine's wiring were changed. Pin assignments were
  derived from the working Marlin configuration (BOARD_BTT_OCTOPUS_PRO_V1_0
  with modified motor slots).

  Part of grblHAL

  Copyright (c) 2024 Joe Corelli
  Copyright (c) 2024 Jon Escombe
  Copyright (c) 2025 Michael Griffin

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL.  If not, see <http://www.gnu.org/licenses/>.
*/

#if N_ABC_MOTORS > 8
#error "Octopus Pro board map is only configured for 8 motors max."
#endif

#if !((defined(STM32F446xx) || defined(STM32F429xx)) && HSE_VALUE == 12000000) && !(defined(DEBUG) && IS_NUCLEO_DEVKIT == 144)
#error "This board has a STM32F429 or STM32F446 processor with 12MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "BTT Octopus Pro v1.0 (Marlin-derived)"
#define BOARD_URL "https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-Pro"

#define SERIAL_PORT                 1       // GPIOA: TX = 9, RX = 10,  USART 1
#define SERIAL1_PORT                21      // GPIOD: TX = 5, RX = 6,   USART 2
#define SERIAL2_PORT                32      // GPIOD: TX = 8, RX = 9,   USART 3
#define I2C_PORT                    1       // GPIOB: SCL = 8, SDA = 9
#define SPI_PORT                    1       // GPIOA: SCK = 5, MISO = 6, MOSI = 7

//we have multiple SPI but only one can be used, for now
//#define SPI_PORT                  12       // GPIOB: SCK = 3, MISO = 4, MOSI = 5

//#define TRINAMIC_SOFT_SPI

// Motor slot reference (board label -> grblHAL axis, as wired on this machine):
// MOTOR0 -> X
// MOTOR6 -> Y
// MOTOR1 -> Z
// MOTOR7 -> M3 (Y2, enable Y_GANGED + Y_AUTO_SQUARE in my_machine.h)
// MOTOR3 -> M4 (spare)
// MOTOR4 -> M5 (spare)
// MOTOR5 -> M6 (spare)
// MOTOR2 -> M7 (spare, defective)

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOF
#define X_STEP_PIN                  13          // MOTOR0
#define Y_STEP_PORT                 GPIOE
#define Y_STEP_PIN                  2           // MOTOR6
#define Z_STEP_PORT                 GPIOG
#define Z_STEP_PIN                  0           // MOTOR1
#define STEP_OUTMODE                GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOF
#define X_DIRECTION_PIN             12
#define Y_DIRECTION_PORT            GPIOE
#define Y_DIRECTION_PIN             3
#define Z_DIRECTION_PORT            GPIOG
#define Z_DIRECTION_PIN             1
#define DIRECTION_OUTMODE           GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOF
#define X_ENABLE_PIN                14
#define Y_ENABLE_PORT               GPIOD
#define Y_ENABLE_PIN                4
#define Z_ENABLE_PORT               GPIOF
#define Z_ENABLE_PIN                15

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOG
#define X_LIMIT_PIN                 6           // X-STOP
#define Y_LIMIT_PORT                GPIOG
#define Y_LIMIT_PIN                 9           // Y-STOP
#define Z_LIMIT_PORT                GPIOG
#define Z_LIMIT_PIN                 10          // Z-STOP
#define LIMIT_INMODE                GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE                            // MOTOR7 = Y2
#define M3_STEP_PORT                GPIOE
#define M3_STEP_PIN                 6
#define M3_DIRECTION_PORT           GPIOA
#define M3_DIRECTION_PIN            14
#define M3_LIMIT_PORT               GPIOG
#define M3_LIMIT_PIN                13          // E1DET (Y2 endstop in Marlin)
#define M3_ENABLE_PORT              GPIOE
#define M3_ENABLE_PIN               0
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE                            // MOTOR3
#define M4_STEP_PORT                GPIOG
#define M4_STEP_PIN                 4
#define M4_DIRECTION_PORT           GPIOC
#define M4_DIRECTION_PIN            1
#define M4_LIMIT_PORT               GPIOG
#define M4_LIMIT_PIN                11          // Z2-STOP (shared with probe input below)
#define M4_ENABLE_PORT              GPIOA
#define M4_ENABLE_PIN               0           // v1.0: MOTOR3 enable on PA0
#endif

// Define ganged axis or C axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 2
#define M5_AVAILABLE                            // MOTOR4
#define M5_STEP_PORT                GPIOF
#define M5_STEP_PIN                 9
#define M5_DIRECTION_PORT           GPIOF
#define M5_DIRECTION_PIN            10
#define M5_LIMIT_PORT               GPIOG
#define M5_LIMIT_PIN                12          // E0DET
#define M5_ENABLE_PORT              GPIOG
#define M5_ENABLE_PIN               2
#endif

#if N_ABC_MOTORS > 3
#define M6_AVAILABLE                            // MOTOR5
#define M6_STEP_PORT                GPIOC
#define M6_STEP_PIN                 13
#define M6_DIRECTION_PORT           GPIOF
#define M6_DIRECTION_PIN            0
#define M6_LIMIT_PORT               GPIOG
#define M6_LIMIT_PIN                14          // E2DET
#define M6_ENABLE_PORT              GPIOF
#define M6_ENABLE_PIN               1
#endif

#if N_ABC_MOTORS > 4
#define M7_AVAILABLE                            // MOTOR2 (defective)
#define M7_STEP_PORT                GPIOF
#define M7_STEP_PIN                 11
#define M7_DIRECTION_PORT           GPIOG
#define M7_DIRECTION_PIN            3
#define M7_LIMIT_PORT               GPIOG
#define M7_LIMIT_PIN                15          // E3DET
#define M7_ENABLE_PORT              GPIOG
#define M7_ENABLE_PIN               5
#endif

#define AUXOUTPUT0_PORT             GPIOA       // Spindle PWM - FAN0 (T1CH1)
#define AUXOUTPUT0_PIN              8

#define AUXOUTPUT1_PORT             GPIOE       // - FAN1
#define AUXOUTPUT1_PIN              5

#define AUXOUTPUT2_PORT             GPIOD       // - FAN2
#define AUXOUTPUT2_PIN              12

#define AUXOUTPUT3_PORT             GPIOD       // - FAN3
#define AUXOUTPUT3_PIN              13

#define AUXOUTPUT4_PORT             GPIOA       // Spindle/laser enable - Bed-out (as in Marlin: HEATER_BED_PIN)
#define AUXOUTPUT4_PIN              1

#define AUXOUTPUT5_PORT             GPIOD       // Spindle direction - FAN5 (PD15 as in Marlin, PE15 is TFT DC on EXP1)
#define AUXOUTPUT5_PIN              15

#define AUXOUTPUT6_PORT             GPIOA       // Coolant flood - HE0 (v1.0: PA2)
#define AUXOUTPUT6_PIN              2

#define AUXOUTPUT7_PORT             GPIOA       // Coolant mist - HE1
#define AUXOUTPUT7_PIN              3

#define AUXOUTPUT8_PORT             GPIOB       // - HE2 (v1.0: PB10)
#define AUXOUTPUT8_PIN              10

#define AUXOUTPUT10_PORT            GPIOD       // - FAN4
#define AUXOUTPUT10_PIN             14

#if RGB_LED_ENABLE

#define LED_PWM_PORT                GPIOB_BASE  // - RGB (v1.0: PB0)
#define LED_PWM_PIN                 0

#define AUXOUTPUT9_PORT             GPIOB       // - HE3
#define AUXOUTPUT9_PIN              11

#else

#define AUXOUTPUT9_PORT             GPIOB       // - RGB (v1.0: PB0)
#define AUXOUTPUT9_PIN              0

#define AUXOUTPUT0_PWM_PORT         GPIOB       // - HE3 (T2CH4)
#define AUXOUTPUT0_PWM_PIN          11

#endif

// Define driver spindle pins.
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT         AUXOUTPUT4_PORT
#define SPINDLE_ENABLE_PIN          AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT            AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN             AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT      AUXOUTPUT5_PORT
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT5_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT          AUXOUTPUT6_PORT
#define COOLANT_FLOOD_PIN           AUXOUTPUT6_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT           AUXOUTPUT7_PORT
#define COOLANT_MIST_PIN            AUXOUTPUT7_PIN
#endif

#define AUXINPUT0_PORT              GPIOC       // Safety door - PWR-DET
#define AUXINPUT0_PIN               0
#define AUXINPUT1_PORT              GPIOG       // Probe - Z2-STOP (as in Marlin: Z_MIN_PROBE_PIN = Z2_DIAG_PIN)
#define AUXINPUT1_PIN               11
#define AUXINPUT2_PORT              GPIOB       // Z probe "right"
#define AUXINPUT2_PIN               7
#define AUXINPUT3_PORT              GPIOB       // Button on PCB
#define AUXINPUT3_PIN               2
#define AUXINPUT4_PORT              GPIOF       // Reset - TB
#define AUXINPUT4_PIN               3
#define AUXINPUT5_PORT              GPIOF       // Feed hold - T0
#define AUXINPUT5_PIN               4
//#define AUXINPUT6_PORT              GPIOF       // Cycle start - T1
//#define AUXINPUT6_PIN               5
#define AUXINPUT7_PORT              GPIOG       // Joystick enable (Stop7)
#define AUXINPUT7_PIN               15

#define AUXINPUT0_ANALOG_PORT       GPIOF       // TH1 = X stick
#define AUXINPUT0_ANALOG_PIN        5
#define AUXINPUT1_ANALOG_PORT       GPIOF       // TH2 = Y stick
#define AUXINPUT1_ANALOG_PIN        6
#define AUXINPUT2_ANALOG_PORT       GPIOF       // TH3 = Z stick
#define AUXINPUT2_ANALOG_PIN        7

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT                  AUXINPUT4_PORT
#define RESET_PIN                   AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT              AUXINPUT5_PORT
#define FEED_HOLD_PIN               AUXINPUT5_PIN
#endif
//#if CONTROL_ENABLE & CONTROL_CYCLE_START
//#define CYCLE_START_PORT            AUXINPUT6_PORT
//#define CYCLE_START_PIN             AUXINPUT6_PIN
//#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT            AUXINPUT0_PORT
#define SAFETY_DOOR_PIN             AUXINPUT0_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT                  AUXINPUT1_PORT
#define PROBE_PIN                   AUXINPUT1_PIN
#endif

#if SDCARD_ENABLE
#define SDCARD_SDIO                 1
// Card detect disabled: on the Octopus PC14 is high with a card inserted (see Marlin SD_DETECT_STATE HIGH),
// grblHAL expects low. With it enabled inserting a card unmounts it. The card is mounted on demand instead.
//#ifndef M6_LIMIT_PORT
//#define SD_DETECT_PORT              GPIOC
//#define SD_DETECT_PIN               14
//#endif
#endif

//Pins not used
// DC Probe pin? Connected to an EL357C on GPIOC5 conflicts with T1/cycle start but this may be preferred as it is optocoupled. Only one?
// PS-ON is not used due to conflict with limit switches. Can be used as an analog in.

#if TRINAMIC_UART_ENABLE //Not tested, use with care

#define MOTOR_UARTX_PORT            GPIOC
#define MOTOR_UARTX_PIN             4           // MOTOR0
#define MOTOR_UARTY_PORT            GPIOE
#define MOTOR_UARTY_PIN             1           // MOTOR6
#define MOTOR_UARTZ_PORT            GPIOD
#define MOTOR_UARTZ_PIN             11          // MOTOR1

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT           GPIOD
#define MOTOR_UARTM3_PIN            3           // MOTOR7
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT           GPIOC
#define MOTOR_UARTM4_PIN            7           // MOTOR3
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_UARTM5_PORT           GPIOF
#define MOTOR_UARTM5_PIN            2           // MOTOR4
#endif

#ifdef  M6_AVAILABLE
#define MOTOR_UARTM6_PORT           GPIOE
#define MOTOR_UARTM6_PIN            4           // MOTOR5
#endif

#ifdef  M7_AVAILABLE
#define MOTOR_UARTM7_PORT           GPIOC
#define MOTOR_UARTM7_PIN            6           // MOTOR2
#endif

#elif TRINAMIC_SPI_ENABLE

#ifdef TRINAMIC_SOFT_SPI // Software SPI implementation

#define TRINAMIC_MOSI_PORT          GPIOA
#define TRINAMIC_MOSI_PIN           7
#define TRINAMIC_SCK_PORT           GPIOA
#define TRINAMIC_SCK_PIN            5
#define TRINAMIC_MISO_PORT          GPIOA
#define TRINAMIC_MISO_PIN           6

#endif //TRINAMIC_SOFT_SPI

#define MOTOR_CSX_PORT              GPIOC
#define MOTOR_CSX_PIN               4
#define MOTOR_CSY_PORT              GPIOE
#define MOTOR_CSY_PIN               1
#define MOTOR_CSZ_PORT              GPIOD
#define MOTOR_CSZ_PIN               11

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOD
#define MOTOR_CSM3_PIN              3
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_CSM4_PORT             GPIOC
#define MOTOR_CSM4_PIN              7
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_CSM5_PORT             GPIOF
#define MOTOR_CSM5_PIN              2
#endif

#ifdef  M6_AVAILABLE
#define MOTOR_CSM6_PORT             GPIOE
#define MOTOR_CSM6_PIN              4
#endif

#ifdef  M7_AVAILABLE
#define MOTOR_CSM7_PORT             GPIOC
#define MOTOR_CSM7_PIN              6
#endif

#endif

#define CAN_PORT                    GPIOD
#define CAN_RX_PIN                  0
#define CAN_TX_PIN                  1

// EOF
