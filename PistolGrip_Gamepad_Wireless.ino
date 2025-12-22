/*
Pistol Grip Gamepad
By: Colby R.
Date: 10/03/2025
	V3: redesigned for ESP32-S3 modules
	V2: designed for TQi 2 channel contrller (the black 2.4GHz ones)
	V1: designed and working for my old traxxas controller (the grey ones)
*/

// #include <XInput.h>
// #include <Arduino.h>
#include <BleGamepad.h>
#include <Joystick_ESP32S2.h>

#define VERSION_MAJOR 3

// +--------------------------------------------------------------+
// |                       Connection Notes                       |
// +--------------------------------------------------------------+

// +==============================+
// |         TQi Pinouts          |
// +==============================+
/*
	### Wires ###
	Red    -  +3.3V    - Connected to PCB
	Green  -  Throttle - 
	Black  -  Ground   - Connected to PCB
	Blue   -  Ground   - Connected to PCB
	Orange -  +3.3V    - Connected to PCB
	White  -  Steering - 

	### PCB ###
    & Top &
	P1_1 - Ground     - GND
	P1_2 - Switch1    - SET
	P1_3 - Switch2    - MENU
	P1_4 - DS2_Right  - LED_RED   (53Ω ~60mA needed!) NOTE: SMD 83Ω added to PCB traces
	P1_5 - DS2_Center - LED_GREEN (53Ω ~60mA needed!) NOTE: SMD 83Ω added to PCB traces
	P1_6 - R2_Center  - STR_TRIM
	P1_7 - R1_Center  - MULTI_TRIM
	P1_8 - VDD        - 3.3V
    & Bottom &
*/

// +==============================+
// |     Arduino Connections      |
// +==============================+
/*
	Conn Color    Name        Arduino  PIN#
	00 - N/A    - Ground       - GND  - NA  - NA
	01 - N/A    - +5V          - VDD  - NA  - NA
	02 - Green  - Throttle     - A0   - 18  - REVOLT_STEER
	03 - White  - Steering     - A1   - 19  - 
	04 - N/A    - Throttle-Trm - A2   - 20  - REVOLT_FWDREV
	05 - N/A    - Steering-Trm - A3   - 21  -
	
	06 - Red    - Thumb_Btn   - D2   - 02  - REVOLT_ACCEPT_ITEM
	07 - N/A    - Set_Btn     - D3   - 03  - REVOLT_BACK_PAUSE
	08 - N/A    - Menu_Btn    - D4   - 04  - REVOLT_RESET_CAR
	09 - N/A    - LED_RED     - D5   - 05  - PWM_OUT
	10 - N/A    - LED_GREEN   - D6   - 06  - PWM_OUT
	09 - Yellow - Top_Btn     - D7   - 07  - REVOLT_FLIP_CAR
	10 - Purple - Middle_Btn  - D8   - 08  - REVOLT_HORN
	11 - Brown  - Bottom_Btn  - D9   - 09  - REVOLT_LOOK_BACK
*/

// +==============================+
// |       OLD Wire colors        |
// +==============================+
/*
	Conn Color    Name       Arduino  PIN#
	0 - Black  - Ground      - GND  - 
	1 - Brown  - +5V         - 3.3V - 
	2 - White  - Steer       - A0   - 17
	3 - Orange - Steer-Trm   - A1   - 16
	4 - Green  - Throttle    - A2   - 15
	5 - Blue   - Thottle-Trm - A3   - 14
	6 - Purple - Switch-NO   - D2   - 02
	7 - Yellow - Switch_NC   - D3   - 03
	8 - Red    - Pwr_Swtch   - D4   - 04
*/

// +==============================+
// |        XINPUT Buttons        |
// +==============================+

#define BUTTON_LOGO   0
#define BUTTON_A      1
#define BUTTON_B      2
#define BUTTON_X      3
#define BUTTON_Y      4
#define BUTTON_LB     5
#define BUTTON_RB     6
#define BUTTON_BACK   7
#define BUTTON_START  8
#define BUTTON_L3     9
#define BUTTON_R3     10
#define DPAD_UP       11
#define DPAD_DOWN     12
#define DPAD_LEFT     13
#define DPAD_RIGHT    14
#define TRIGGER_LEFT  15
#define TRIGGER_RIGHT 16
#define JOY_LEFT      17
#define JOY_RIGHT     18

// +--------------------------------------------------------------+
// |                     Compile Time Options                     |
// +--------------------------------------------------------------+
#define NEG_AXIS     false // set axis range -32767 to 32767 (Some non-Windows operating systems and web based gamepad testers don't like min axis set below 0, so 0 is set by default)
#define SIM_CONTROLS false // enable and map throttle/steering to "sim" controls

// BLE_Gamepad_Config
#define NUM_BUTTONS      8 // do be able to natrually map BUTTON_BACK in Re-Volt. we only have 6 actual
#define NUM_HAT_SWITCHES 0
#define ENABLE_X true
#define ENABLE_Y true
#define ENABLE_Z false
#define ENABLE_RX true
#define ENABLE_RY true
#define ENABLE_RZ false
#define ENABLE_SLIDER1 false
#define ENABLE_SLIDER2 false
#if SIM_CONTROLS
	#define ENABLE_RUDDER false
	#define ENABLE_THROTTLE true
	#define ENABLE_ACCELERATOR false
	#define ENABLE_BRAKE true
	#define ENABLE_STEERING true
#else
	#define ENABLE_RUDDER false
	#define ENABLE_THROTTLE false
	#define ENABLE_ACCELERATOR false
	#define ENABLE_BRAKE false
	#define ENABLE_STEERING false
#endif

// +--------------------------------------------------------------+
// |                       Mapping Defines                        |
// +--------------------------------------------------------------+

// +==============================+
// |   Re-Volt XINPUT MAapping    |
// +==============================+
#define REVOLT_HORN        BUTTON_LOGO   // N/A
#define REVOLT_ACCEPT_ITEM BUTTON_A      // N/A
#define REVOLT_FLIP_CAR    BUTTON_B      // N/A
#define REVOLT_RESET_CAR   BUTTON_X      // N/A
#define REVOLT_BACK_PAUSE  BUTTON_Y      // N/A
// #define REVOLT_NA0         BUTTON_LB     // N/A
// #define REVOLT_NA1         BUTTON_RB     // N/A
#define REVOLT_LOOK_BACK   BUTTON_BACK   // N/A
// #define REVOLT_NA2         BUTTON_START  // N/A
// #define REVOLT_NA3         BUTTON_L3     // N/A
// #define REVOLT_NA4         BUTTON_R3     // N/A
#define REVOLT_UP          DPAD_UP       // Menu Nav
#define REVOLT_DOWN        DPAD_DOWN     // Menu Nav
#define REVOLT_LEFT        DPAD_LEFT     // Menu Nav
#define REVOLT_RIGHT       DPAD_RIGHT    // Menu Nav
// #define REVOLT_NA5         TRIGGER_LEFT  // N/A
// #define REVOLT_NA6         TRIGGER_RIGHT // N/A
#define REVOLT_FWDREV      JOY_LEFT      // leftYAxis
#define REVOLT_STEER       JOY_LEFT      // leftXAxis
// #define REVOLT_NA7         JOY_RIGHT,

// +==============================+
// |            PINOUT            |
// +==============================+
#if 0 // ESP32-C3 prototype
	#define PIN_STR_TRM   (4)  // GPIO3  // ADC1_3
	#define PIN_THT_TRM   (5)  // GPIO2  // ADC1_4
	#define PIN_STR       (6)  // GPIO1  // ADC1_5
	#define PIN_THT       (7)  // GPIO0  // ADC1_6
	#define PIN_THMB_BTN  (4)  // GPIO4  // (2) // D2
	#define PIN_MENU_BTN  (5)  // GPIO5  // (3) // D3
	#define PIN_SET_BTN   (6)  // GPIO6  // (4) // D4
	#define PIN_LED_RED   (7)  // GPIO7  // (5) // D5 NOTE: PWM!
	#define PIN_LED_GREEN (10) // GPIO10 // (6) // D6 NOTE: PWM!
	#define PIN_TOP_BTN   (19) // GPIO19 // (7) // D7 NOTE: USB Uses this?
	#define PIN_MID_BTN   (20) // GPIO20 // (8) // D8
	#define PIN_BTM_BTN   (21) // GPIO21 // (9) // D9
#else // ESP32-S3
	#define PIN_STR_TRM   (4)  // GPIO4  // ADC1_3
	#define PIN_THT_TRM   (5)  // GPIO5  // ADC1_4
	#define PIN_STR       (6)  // GPIO6  // ADC1_5
	#define PIN_THT       (7)  // GPIO7  // ADC1_6
	#define PIN_THMB_BTN  (38) // GPI38  // (38)
	#define PIN_MENU_BTN  (2)  // GPIO2  // (0)
	#define PIN_SET_BTN   (1)  // GPIO1  // (1)
	#define PIN_LED_RED   (42) // GPIO42 // (42) NOTE: PWM!
	#define PIN_LED_GREEN (41) // GPIO41 // (41) NOTE: PWM!
	#define PIN_TOP_BTN   (16) // GPIO16 // (16)
	#define PIN_MID_BTN   (17) // GPIO17 // (17)
	#define PIN_BTM_BTN   (18) // GPIO18 // (18)
	#define PIN_VIBRATOR  (39) // GPIO39 // (39)
#endif

// +==============================+
// |        Button Mapping        |
// +==============================+
#define XIN_THMB_BTN  REVOLT_ACCEPT_ITEM
#define XIN_MENU_BTN  REVOLT_BACK_PAUSE
#define XIN_SET_BTN   REVOLT_FLIP_CAR
#define XIN_TOP_BTN   REVOLT_RESET_CAR
#define XIN_MID_BTN   REVOLT_HORN
#define XIN_BTM_BTN   REVOLT_LOOK_BACK

// +--------------------------------------------------------------+
// |                        Controller IDs                        |
// +--------------------------------------------------------------+
#define MANF_NAME "ReadySetProjects"
// #define PROD_NAME "RSP Controller"

// #define PROD_NAME "RSP Controller TEST-C3" 0x1234 // Test ESP32-C3
// #define PROD_NAME "RSP Controller A0148840" // 0x8840 First ESP32-S3 controller
// #define PROD_NAME "RSP Controller A0392285" // 0x2285 Has throttle adjuster thing
// #define PROD_NAME "RSP Controller A0381549" // 0x1549
// #define PROD_NAME "RSP Controller A0381459" // 0x1459
// #define PROD_NAME "RSP Controller A0359313" // 0x9313
// #define PROD_NAME "RSP Controller A0359295" // 0x9295
// #define PROD_NAME "RSP Controller A0306966" // 0x6966
// #define PROD_NAME "RSP Controller A0306712" // 0x6712
// #define PROD_NAME "RSP Controller A0148987" // 0x8987 Currently an Arduino controller
// #define PROD_NAME "RSP Controller A0148750" // 0x8750 // messed up POT
// #define PROD_NAME "RSP Controller A0329340" // 0x9340 // actual first S3 Controller
#define PROD_NAME "RSP Controller A0148860" // 0x8860 // First PCB S3 controller
// #define PROD_NAME "RSP Controller A0350786" 48840
// #define PROD_NAME "RSP Controller A0243249" 48840
// #define PROD_NAME "RSP Controller A0337517" 48840
// #define PROD_NAME "RSP Controller A0337117" 48840


#define VENDOR_ID 0xC01B // Colby!

// #define CONTROLLER_ID 0x1234 // Test ESP32-C3
// #define CONTROLLER_ID 0x8840 // A0148840 First ESP32-S3 controller
// #define CONTROLLER_ID 0x2285 // A0392285 Has throttle adjuster thing
// #define CONTROLLER_ID 0x1549 // A0381549
// #define CONTROLLER_ID 0x1459 // A0381459
// #define CONTROLLER_ID 0x9313 // A0359313
// #define CONTROLLER_ID 0x9295 // A0359295
// #define CONTROLLER_ID 0x6966 // A0306966
// #define CONTROLLER_ID 0x6712 // A0306712
// #define CONTROLLER_ID 0x8987 // A0148987 Currently an Arduino controller
// #define CONTROLLER_ID 0x8750 // A0148750 // Messed Up POT
// #define CONTROLLER_ID 0x9340 // A0329340 // actual first S3 Controller
#define CONTROLLER_ID 0x8860 // A0148860 // First PCA S3 Controller
// #define CONTROLLER_ID 0x0786
// #define CONTROLLER_ID 0x3249
// #define CONTROLLER_ID 0x7517
// #define CONTROLLER_ID 0x7117

// +--------------------------------------------------------------+
// |                          Constants                           |
// +--------------------------------------------------------------+
#define ADC_MAX      0xFFF // For ESP32-C3 12-Bit (4096 values, ie. 0-4095)
#define ADC_HALF     ((ADC_MAX+1)/2)
#define ADC_FILT_LEN 5 // Rolling average length
#define XINPUT_MAX 0xFFFF // 65536 i.e. 16 bit resolution
#define XINPUT_MIN -65536 // 16 bit resolution i.e. 0x10000
#define CONV_MULTI (XINPUT_MAX/(ADC_MAX+1)) // Conversion multiplier from ADC to XINPUT resolutions
#define STARTING_LIMITS 100 // every potentiometer is different so we'll ring the values in a bit to start

#define PWM_LED_MAX 255
#define PWM_LED_MIN 0 // LEDs don't turn on till this?
#define PWM_CONV_MULTI (PWM_LED_MAX - PWM_LED_MIN) // Conversion multiplier from ADC to XINPUT resolutions

#define VIB_DURATION 100 // ms

#define TRIM_PERCENT 0.30 //% amount of range trim can adjust center point
#define TRIM_MIN 10 // trim pots get crazy around the edges
#define TRIM_MAX (ADC_MAX - TRIM_MIN)

#if NEG_AXIS
	#define SIM_MIN     0x8000 // -32767
	#define AXIS_CENTER 0x0000      
	#define SIM_MAX     0x7FFF // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
	#define AXIS_MIN    0x8000 // -32767
	#define AXIS_CENTER 0x0000      
	#define AXIS_MAX    0x7FFF
#else
	#define SIM_MIN     0x0000
	#define SIM_CENTER  0x3FFF
	#define SIM_MAX     0x7FFF // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
	#define AXIS_MIN    0x0000
	#define AXIS_CENTER 0x3FFF
	#define AXIS_MAX    0x7FFF
#endif

// +--------------------------------------------------------------+
// |                           Globals                            |
// +--------------------------------------------------------------+

BleGamepad bleGamepad(PROD_NAME, MANF_NAME, 100);
Joystick_  usbGamepad(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
					  NUM_BUTTONS, NUM_HAT_SWITCHES,                      // Button Count, Hat Switch Count
					  ENABLE_X, ENABLE_Y, ENABLE_Z,                       // X and Y, but no Z Axis
					  ENABLE_RX, ENABLE_RY, ENABLE_RZ,                    // Rx and Ry, but no Rz Axis
					  ENABLE_RUDDER, ENABLE_THROTTLE,                     // No rudder or throttle
					  ENABLE_ACCELERATOR, ENABLE_BRAKE, ENABLE_STEERING); // No accelerator, brake, or steering
hw_timer_t *Timer0_Cfg = NULL;

int VibCountdown = 0;
bool VibActivated = false;

int ThrottleMin = STARTING_LIMITS;
int ThrottleMax = ADC_MAX - STARTING_LIMITS;
int SteeringMin = STARTING_LIMITS;
int SteeringMax = ADC_MAX - STARTING_LIMITS;
int ThrottleTrimMin = STARTING_LIMITS;
int ThrottleTrimMax = ADC_MAX - STARTING_LIMITS;
int SteeringTrimMin = STARTING_LIMITS;
int SteeringTrimMax = ADC_MAX - STARTING_LIMITS;

int  LastSteeringTrim = 0;
int  LastThrottleTrim = 0;
int  LastSteering     = 0;
int  LastThrottle     = 0;
bool LastThumbBtn     = false;
bool LastMenuBtn      = false;
bool LastSetBtn       = false;
bool LastTopBtn       = false;
bool LastMidBtn       = false;
bool LastBtmBtn       = false;

uint AdcBufferThrottle[ADC_FILT_LEN] = {0};
uint AdcBufferSteering[ADC_FILT_LEN] = {0};
uint AdcBufferSteeringTrim[ADC_FILT_LEN] = {0};
uint AdcBufferThrottleTrim[ADC_FILT_LEN] = {0};

// +--------------------------------------------------------------+
// |                         Timer 0 ISR                          |
// +--------------------------------------------------------------+
void IRAM_ATTR Timer0_ISR()
{
	if(VibCountdown > 0) { VibCountdown --; }
}

// +--------------------------------------------------------------+
// |                             Init                             |
// +--------------------------------------------------------------+
void setup()
{

	// +==============================+
	// |            Timers            |
	// +==============================+
	Timer0_Cfg = timerBegin(10000); // 10KHz
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
    timerAlarm(Timer0_Cfg, 10, true, 0); // fire ISR every 1ms (or every 10 timer ticks), repeat, unlimited times
    // timerStart(Timer0_Cfg);

	// +==============================+
	// |            XInput            |
	// +==============================+
	// // put your setup code here, to run once:
	// XInput.setAutoSend(false);
	// XInput.begin();


	// +==============================+
	// |            Micro             |
	// +==============================+
	pinMode(PIN_STR,       INPUT); // No Pullup for ADC?
	pinMode(PIN_STR_TRM,   INPUT); // No Pullup for ADC?
	pinMode(PIN_THT,       INPUT); // No Pullup for ADC?
	pinMode(PIN_THT_TRM,   INPUT); // No Pullup for ADC?
	pinMode(PIN_THMB_BTN,  INPUT_PULLUP);
	pinMode(PIN_MENU_BTN,  INPUT_PULLUP);
	pinMode(PIN_SET_BTN,   INPUT_PULLUP);
	pinMode(PIN_LED_RED,   OUTPUT);
	pinMode(PIN_LED_GREEN, OUTPUT);
	pinMode(PIN_TOP_BTN,   INPUT_PULLUP);
	pinMode(PIN_MID_BTN,   INPUT_PULLUP);
	pinMode(PIN_BTM_BTN,   INPUT_PULLUP);
	pinMode(PIN_VIBRATOR,  OUTPUT);

	digitalWrite(PIN_VIBRATOR, HIGH); // turn off vibrator

	// +==============================+
	// |            Serial            |
	// +==============================+
	// Serial.begin(115200);
	// Serial.println("Starting BLE work!");
	
	// +==============================+
	// |          Bluetooth           |
	// +==============================+
	BleGamepadConfiguration bleGamepadConfig;
	bleGamepadConfig.setAutoReport(false);
	bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD); // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
	bleGamepadConfig.setButtonCount(NUM_BUTTONS);
	bleGamepadConfig.setHatSwitchCount(NUM_HAT_SWITCHES);
    bleGamepadConfig.setWhichAxes(ENABLE_X, ENABLE_Y, ENABLE_Z, ENABLE_RX, ENABLE_RY, ENABLE_RZ, ENABLE_SLIDER1, ENABLE_SLIDER2);      // Can also be done per-axis individually. All are true by default
    bleGamepadConfig.setAxesMin(AXIS_MIN);
	bleGamepadConfig.setAxesMax(AXIS_MAX);

    #if SIM_CONTROLS
    bleGamepadConfig.setWhichSimulationControls(ENABLE_RUDDER, ENABLE_THROTTLE, ENABLE_ACCELERATOR, ENABLE_BRAKE, ENABLE_STEERING); // Can also be done per-control individually. All are false by default
    bleGamepadConfig.setSimulationMin(SIM_MIN);
    bleGamepadConfig.setSimulationMax(SIM_MAX);
    #endif

    // bleGamepadConfig.setHidReportId(0x5); // unneeded right?
	bleGamepadConfig.setVid(VENDOR_ID);
	bleGamepadConfig.setPid(CONTROLLER_ID);
	bleGamepad.begin(&bleGamepadConfig); // Changing bleGamepadConfig after the begin function has no effect, unless you call the begin function again

    #if SIM_CONTROLS
    // Set steering to center
    bleGamepad.setSteering(SIM_CENTER);
    bleGamepad.setBrake(SIM_MIN);
    bleGamepad.setAccelerator(SIM_MIN);
    #endif

	// +==============================+
	// |          usbGamepad          |
	// +==============================+
	USB.PID(CONTROLLER_ID);
	USB.VID(VENDOR_ID);
	USB.productName(PROD_NAME);
	USB.manufacturerName(MANF_NAME);
	USB.begin();

	usbGamepad.setXAxisRange(AXIS_MIN, AXIS_MAX);
	usbGamepad.setYAxisRange(AXIS_MIN, AXIS_MAX);
	usbGamepad.setRxAxisRange(AXIS_MIN, AXIS_MAX);
	usbGamepad.setRyAxisRange(AXIS_MIN, AXIS_MAX);
	usbGamepad.begin(false);

	// +==============================+
	// |         Init Values          |
	// +==============================+
	LastSteeringTrim = analogRead(PIN_THT_TRM);
	LastThrottleTrim = analogRead(PIN_STR_TRM);
	LastSteering     = analogRead(PIN_STR);
	LastThrottle     = analogRead(PIN_THT);
	LastThumbBtn     = !digitalRead(PIN_THMB_BTN);
	LastMenuBtn      = !digitalRead(PIN_MENU_BTN);
	LastSetBtn       = !digitalRead(PIN_SET_BTN);
	// LastTopBtn       = !digitalRead(PIN_TOP_BTN);
	LastMidBtn       = !digitalRead(PIN_MID_BTN);
	LastBtmBtn       = !digitalRead(PIN_BTM_BTN);
}

// Function to map a number from one range to another
int mapRange(int numIn, int minIn, int maxIn, int minOut, int maxOut)
{
    if (maxIn - minIn == 0) { return 0; } // Input range cannot be zero.
    return (int)((((float)(numIn - minIn) / (maxIn - minIn)) * (maxOut - minOut)) + minOut);
}

uint AverageAdc(int AdcIn, uint *buffer, int len)
{
	int bIndex = 0;
	uint32_t average = 0;

	for(bIndex = (len-1); bIndex > 0; bIndex--)
	{
		buffer[bIndex] = buffer[bIndex-1];
		average += buffer[bIndex];
	}
	buffer[0] = AdcIn;
	average += AdcIn;
	return (uint)(average/len);
}

void loop()
{
	int throttlePosition = AverageAdc(analogRead(PIN_THT), AdcBufferSteering, ADC_FILT_LEN); // NOTE: 12 bit ADC (0-4095)
	int steeringPosition = AverageAdc(analogRead(PIN_STR), AdcBufferThrottle, ADC_FILT_LEN); // NOTE: 12 bit ADC (0-4095)
	int throttleTrimPosition = AverageAdc(analogRead(PIN_THT_TRM), AdcBufferSteeringTrim, ADC_FILT_LEN); // NOTE: 12 bit ADC (0-4095)
	int steeringTrimPosition = AverageAdc(analogRead(PIN_STR_TRM), AdcBufferThrottleTrim, ADC_FILT_LEN); // NOTE: 12 bit ADC (0-4095)

	if(ThrottleMin > throttlePosition) { ThrottleMin = throttlePosition; }
	if(ThrottleMax < throttlePosition) { ThrottleMax = throttlePosition; }
	if(SteeringMin > steeringPosition) { SteeringMin = steeringPosition; }
	if(SteeringMax < steeringPosition) { SteeringMax = steeringPosition; }
	if(ThrottleTrimMin > throttleTrimPosition) { ThrottleTrimMin = throttleTrimPosition; }
	if(ThrottleTrimMax < throttleTrimPosition) { ThrottleTrimMax = throttleTrimPosition; }
	if(SteeringTrimMin > steeringTrimPosition) { SteeringTrimMin = steeringTrimPosition; }
	if(SteeringTrimMax < steeringTrimPosition) { SteeringTrimMax = steeringTrimPosition; }

	// +==============================+
	// |      Analog Adjustments      |
	// +==============================+
	int adjustedThrottle = mapRange(throttlePosition, ThrottleMin, ThrottleMax, 0, ADC_MAX);
	int adjustedSteering = mapRange(steeringPosition, SteeringMin, SteeringMax, 0, ADC_MAX);

	// +==============================+
	// |        Throttle Trim         |
	// +==============================+
	if(throttleTrimPosition < TRIM_MIN) { throttleTrimPosition = TRIM_MIN; }
	else if(throttleTrimPosition > TRIM_MAX) { throttleTrimPosition = TRIM_MAX; }
	int adjustedThrottleTrim = mapRange(throttleTrimPosition, TRIM_MIN, TRIM_MAX, 0, ADC_MAX); // max out range
	int trimmedCenter = mapRange(adjustedThrottleTrim, 0, ADC_MAX, (ADC_HALF - ((ADC_MAX * TRIM_PERCENT)/2)), (ADC_HALF + ((ADC_MAX * TRIM_PERCENT)/2)));
	int trimmedThrottle = adjustedThrottle;
	if(adjustedThrottle > trimmedCenter) // Above trimmed center
	{
		trimmedThrottle = mapRange(adjustedThrottle, trimmedCenter, ADC_MAX, ADC_HALF, ADC_MAX); // map from 50-100%
	}
	else if (adjustedThrottle < trimmedCenter) // Below trimmed center
	{
		trimmedThrottle = mapRange(adjustedThrottle, 0, trimmedCenter, 0, ADC_HALF); // map from 0-50%
	}

	// +==============================+
	// |        Steering Trim         |
	// +==============================+
	if(steeringTrimPosition < TRIM_MIN) { steeringTrimPosition = TRIM_MIN; }
	else if(steeringTrimPosition > TRIM_MAX) { steeringTrimPosition = TRIM_MAX; }
	int adjustedSteeringTrim = ADC_MAX - mapRange(steeringTrimPosition, TRIM_MIN, TRIM_MAX, 0, ADC_MAX); // max out range
	trimmedCenter = mapRange(adjustedSteeringTrim, 0, ADC_MAX, (ADC_HALF - ((ADC_MAX * TRIM_PERCENT)/2)), (ADC_HALF + ((ADC_MAX * TRIM_PERCENT)/2)));
	int trimmedSteering = adjustedSteering;
	if(adjustedSteering > trimmedCenter) // Above trimmed center
	{
		trimmedSteering = mapRange(adjustedSteering, trimmedCenter, ADC_MAX, ADC_HALF, ADC_MAX); // map from 50-100%
	}
	else if (adjustedSteering < trimmedCenter) // Below trimmed center
	{
		trimmedSteering = mapRange(adjustedSteering, 0, trimmedCenter, 0, ADC_HALF); // map from 0-50%
	}

	// +==============================+
	// |             LEDS             |
	// +==============================+
	if(!digitalRead(PIN_SET_BTN))
	{
		analogWrite(PIN_LED_RED, PWM_LED_MIN);
		analogWrite(PIN_LED_GREEN, PWM_LED_MIN);
	}
	else
	{
		#if 0 // LEDs based off Trim
		analogWrite(PIN_LED_RED,   mapRange(adjustedThrottleTrim, 0, ADC_MAX, PWM_LED_MIN, PWM_LED_MAX));
		analogWrite(PIN_LED_GREEN, mapRange(adjustedSteeringTrim, 0, ADC_MAX, PWM_LED_MIN, PWM_LED_MAX));
		#else // LEDs based off throttle/stering
		analogWrite(PIN_LED_RED,   mapRange(adjustedThrottle, 0, ADC_MAX, PWM_LED_MIN, PWM_LED_MAX));
		analogWrite(PIN_LED_GREEN, mapRange(adjustedSteering, 0, ADC_MAX, PWM_LED_MIN, PWM_LED_MAX));
		#endif
	}

	// +==============================+
	// |           Vibrator           |
	// +==============================+
	if(!digitalRead(PIN_THMB_BTN) && VibCountdown == 0 && !VibActivated)
	{
		VibCountdown = VIB_DURATION;
		digitalWrite(PIN_VIBRATOR, HIGH); // turn on vibrator
		VibActivated = true;
	}
	else if(digitalRead(PIN_THMB_BTN) && VibCountdown == 0 && VibActivated)
	{
		VibActivated = false;
	}
	else if(digitalRead(PIN_VIBRATOR) && VibCountdown == 0)
	{
		digitalWrite(PIN_VIBRATOR, LOW); // turn off vibrator
	}

	// +--------------------------------------------------------------+
	// |                            XInput                            |
	// +--------------------------------------------------------------+

	// +==============================+
	// |           TRIGGERS           |
	// +==============================+
	// // Left Trigger - 0
	// XInput.setTrigger(TRIGGER_LEFT, digitalRead(0) * -255 + 255);
	// // Right Trigger - 7
	// XInput.setTrigger(TRIGGER_RIGHT, digitalRead(7) * -255 + 255);
 
	// +==============================+
	// |           Buttons            |
	// +==============================+

	// #if 0 // FINISHED CONTROLLER
	// // Left Button - 1
	// XInput.setButton(REVOLT_ACCEPT_ITEM, !digitalRead(PIN_THMB_BTN));
	// // Left Button - 1
	// XInput.setButton(REVOLT_FLIP_CAR, !digitalRead(PIN_BTM_BTN));
	// // Back Button - 2
	// XInput.setButton(REVOLT_RESET_CAR, !digitalRead(PIN_SET_BTN));
	// // Back Button - 2
	// XInput.setButton(REVOLT_BACK_PAUSE, !digitalRead(PIN_MENU_BTN));
	// // Back Button - 2
	// XInput.setButton(REVOLT_LOOK_BACK, !digitalRead(PIN_TOP_BTN));
	// // Back Button - 2
	// XInput.setButton(REVOLT_HORN, !digitalRead(PIN_MID_BTN));
	// #else // NO Nose buttons CONTROLLER
	// // Left Button - 1
	// XInput.setButton(REVOLT_ACCEPT_ITEM, !digitalRead(PIN_THMB_BTN));
	// // Left Button - 1
	// XInput.setButton(REVOLT_FLIP_CAR, !digitalRead(PIN_SET_BTN));
	// // Back Button - 2
	// XInput.setButton(REVOLT_BACK_PAUSE, !digitalRead(PIN_MENU_BTN));
	// #endif
	
	// // Up DPad - 3 Down DPad - 5 Left DPad - 4 Right DPad - 6
	// XInput.setDpad(!digitalRead(3), !digitalRead(5), !digitalRead(4), !digitalRead(6));

	// +==============================+
	// |          Joysticks           |
	// +==============================+
	// A0, A2 LEFT
	// RAW
	// int leftXAxis = (steeringPosition - ADC_HALF) * CONV_MULTI;
	// int leftYAxis = (XINPUT_MAX - (throttlePosition - ADC_HALF) * CONV_MULTI);
	// ADJUSTED
	// int leftXAxis = (adjustedSteering - ADC_HALF) * (XINPUT_MAX/(ADC_MAX+1));
	// int leftYAxis = (XINPUT_MAX - (adjustedThrottle - ADC_HALF) * CONV_MULTI);
	// TRIMMED
	int leftXAxis = (trimmedSteering - ADC_HALF) * (XINPUT_MAX/(ADC_MAX+1));
	int leftYAxis = (XINPUT_MAX - (trimmedThrottle - ADC_HALF) * CONV_MULTI);
	// XInput.setJoystick(JOY_LEFT, leftXAxis, leftYAxis);

	// A1, A3 RIGHT
	// These are trims TODO: implment them to trim digitally (this is just for POC)
	// int rightXAxis = (steeringTrimPosition - ADC_HALF) * CONV_MULTI;
	// int rightYAxis = (XINPUT_MAX - (throttleTrimPosition - ADC_HALF) * CONV_MULTI);

	int rightXAxis = mapRange(adjustedSteeringTrim, 0, ADC_MAX, 0, 0x7FFF);
	int rightYAxis = mapRange(adjustedThrottleTrim, 0, ADC_MAX, 0, 0x7FFF);
	// XInput.setJoystick(JOY_RIGHT, rightXAxis, rightYAxis);

	// XInput.send();
	
	// +--------------------------------------------------------------+
	// |                         BLE Gamepad                          |
	// +--------------------------------------------------------------+
    if (bleGamepad.isConnected())
    {

        // Serial.println("Press buttons 5, 16 and start. Move all enabled axes to max. Set DPAD (hat 1) to down right.");
        if     (!bleGamepad.isPressed(XIN_THMB_BTN) && !digitalRead(PIN_THMB_BTN)){ bleGamepad.press  (XIN_THMB_BTN); }
        else if( bleGamepad.isPressed(XIN_THMB_BTN) &&  digitalRead(PIN_THMB_BTN)){ bleGamepad.release(XIN_THMB_BTN); }
        if     (!bleGamepad.isPressed(XIN_SET_BTN)  && !digitalRead(PIN_SET_BTN)) { bleGamepad.press  (XIN_SET_BTN);  }
        else if( bleGamepad.isPressed(XIN_SET_BTN)  &&  digitalRead(PIN_SET_BTN)) { bleGamepad.release(XIN_SET_BTN);  }
        if     (!bleGamepad.isPressed(XIN_MENU_BTN) && !digitalRead(PIN_MENU_BTN)){ bleGamepad.press  (XIN_MENU_BTN); }
        else if( bleGamepad.isPressed(XIN_MENU_BTN) &&  digitalRead(PIN_MENU_BTN)){ bleGamepad.release(XIN_MENU_BTN); }
        // if     (!bleGamepad.isPressed(XIN_TOP_BTN)  && !digitalRead(PIN_TOP_BTN)) { bleGamepad.press  (XIN_TOP_BTN);  }
        // else if( bleGamepad.isPressed(XIN_TOP_BTN)  &&  digitalRead(PIN_TOP_BTN)) { bleGamepad.release(XIN_TOP_BTN);  }
        // if     (!bleGamepad.isPressed(XIN_MID_BTN)  && !digitalRead(PIN_MID_BTN)) { bleGamepad.press  (XIN_MID_BTN);  }
        // else if( bleGamepad.isPressed(XIN_MID_BTN)  &&  digitalRead(PIN_MID_BTN)) { bleGamepad.release(XIN_MID_BTN);  }
        // if     (!bleGamepad.isPressed(XIN_BTM_BTN)  && !digitalRead(PIN_BTM_BTN)) { bleGamepad.press  (XIN_BTM_BTN);  }
        // else if( bleGamepad.isPressed(XIN_BTM_BTN)  &&  digitalRead(PIN_BTM_BTN)) { bleGamepad.release(XIN_BTM_BTN);  }

		#if 0
        bleGamepad.setAxes(AXIS_CENTER, AXIS_CENTER, 0, rightXAxis, rightYAxis, 0);       //(X, Y, Z, RX, RY, RZ)
        #else
        bleGamepad.setX(mapRange(trimmedSteering, 0, ADC_MAX, AXIS_MIN, AXIS_MAX));
	    bleGamepad.setY(mapRange(trimmedThrottle, 0, ADC_MAX, AXIS_MIN, AXIS_MAX));
	    bleGamepad.setRX(mapRange(adjustedSteeringTrim, 0, ADC_MAX, AXIS_MIN, AXIS_MAX));
	    bleGamepad.setRY(mapRange(adjustedThrottleTrim, 0, ADC_MAX, AXIS_MIN, AXIS_MAX));
        #endif
        // bleGamepad.setAxes(leftXAxis, leftYAxis, 0, rightXAxis, rightYAxis, 0);       //(X, Y, Z, RX, RY, RZ)
		
		#if SIM_CONTROLS
		// +==============================+
		// |         Sim Controls         |
		// +==============================+
	    bleGamepad.setSteering(mapRange(trimmedSteering, 0, ADC_MAX, SIM_MIN, SIM_MAX));
	    bleGamepad.setBrake(mapRange(trimmedThrottle, ADC_HALF, ADC_MAX, SIM_MIN, SIM_MAX)); // TODO: verify this is mapped correctly
	    bleGamepad.setAccelerator(mapRange(trimmedThrottle, 0, ADC_HALF, SIM_MIN, SIM_MAX)); // TODO: verify this is mapped correctly
        #endif

        bleGamepad.sendReport();
    }
    else
    {
    	// +--------------------------------------------------------------+
    	// |                         USB Gamepad                          |
    	// +--------------------------------------------------------------+

		#if 0 // FINISHED CONTROLLER
		// Left Button - 1
		usbGamepad.setButton(REVOLT_ACCEPT_ITEM, !digitalRead(PIN_THMB_BTN));
		// Left Button - 1
		usbGamepad.setButton(REVOLT_FLIP_CAR, !digitalRead(PIN_BTM_BTN));
		// Back Button - 2
		usbGamepad.setButton(REVOLT_RESET_CAR, !digitalRead(PIN_SET_BTN));
		// Back Button - 2
		usbGamepad.setButton(REVOLT_BACK_PAUSE, !digitalRead(PIN_MENU_BTN));
		// Back Button - 2
		usbGamepad.setButton(REVOLT_LOOK_BACK, !digitalRead(PIN_TOP_BTN));
		// Back Button - 2
		usbGamepad.setButton(REVOLT_HORN, !digitalRead(PIN_MID_BTN));
		#else // NO Nose buttons CONTROLLER
		// Left Button - 1
		usbGamepad.setButton(REVOLT_ACCEPT_ITEM, !digitalRead(PIN_THMB_BTN));
		// Left Button - 1
		usbGamepad.setButton(REVOLT_FLIP_CAR, !digitalRead(PIN_SET_BTN));
		// Back Button - 2
		usbGamepad.setButton(REVOLT_BACK_PAUSE, !digitalRead(PIN_MENU_BTN));
		#endif

		usbGamepad.setXAxis(leftXAxis);
		usbGamepad.setYAxis(leftYAxis);
		usbGamepad.setRxAxis(rightXAxis);
		usbGamepad.setRyAxis(rightYAxis);
		usbGamepad.sendState();
    }
}