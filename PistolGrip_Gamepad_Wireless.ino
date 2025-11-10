/*
Pistol Grip Gamepad
By: Colby R.
Date: 10/03/2025
	V3: redesigned for ESP32-S3 modules
	V2: designed for TQi 2 channel contrller (the black 2.4GHz ones)
	V1: designed and working for my old traxxas controller (the grey ones)
*/

// #include <XInput.h>
#include <Arduino.h>
#include <BleGamepad.h>

#define VERSION_MAJOR 3

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
/*
	BUTTON_LOGO = 0,
	BUTTON_A = 1,
	BUTTON_B = 2,
	BUTTON_X = 3,
	BUTTON_Y = 4,
	BUTTON_LB = 5,
	BUTTON_RB = 6,
	BUTTON_BACK = 7,
	BUTTON_START = 8,
	BUTTON_L3 = 9,
	BUTTON_R3 = 10,
	DPAD_UP = 11,
	DPAD_DOWN = 12,
	DPAD_LEFT = 13,
	DPAD_RIGHT = 14,
	TRIGGER_LEFT = 15,
	TRIGGER_RIGHT = 16,
	JOY_LEFT,
	JOY_RIGHT,
*/

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
#define PIN_STR_TRM   (3)  // GPIO3  // ADC1_3
#define PIN_THT_TRM   (2)  // GPIO2  // ADC1_2
#define PIN_STR       (1)  // GPIO1  // ADC1_1
#define PIN_THT       (0)  // GPIO0  // ADC1_0
#define PIN_THMB_BTN  (4)  // GPIO4  // (2) // D2
#define PIN_MENU_BTN  (5)  // GPIO5  // (3) // D3
#define PIN_SET_BTN   (6)  // GPIO6  // (4) // D4
#define PIN_LED_RED   (7)  // GPIO7  // (5) // D5 NOTE: PWM!
#define PIN_LED_GREEN (10) // GPIO10 // (6) // D6 NOTE: PWM!
// #define PIN_TOP_BTN   (19) // GPIO19 // (7) // D7 NOTE: USB Uses this?
#define PIN_MID_BTN   (20) // GPIO20 // (8) // D8
#define PIN_BTM_BTN   (21) // GPIO21 // (9) // D9

// +--------------------------------------------------------------+
// |                          Constants                           |
// +--------------------------------------------------------------+
#define ADC_MAX  1023 // For Pro Micro 10-Bit (1024 values, ie. 0-1023)
#define ADC_HALF ((ADC_MAX+1)/2)
#define XINPUT_MAX 65536 // 16 bit resolution
#define XINPUT_MIN -65536 // 16 bit resolution
#define CONV_MULTI (XINPUT_MAX/(ADC_MAX+1)) // Conversion multiplier from ADC to XINPUT resolutions
#define STARTING_LIMITS 100 // every potentiometer is different so we'll ring the values in a bit to start

#define PWM_LED_MAX 255
#define PWM_LED_MIN 0 // LEDs don't turn on till this?
#define PWM_CONV_MULTI (PWM_LED_MAX - PWM_LED_MIN) // Conversion multiplier from ADC to XINPUT resolutions

#define TRIM_PERCENT 0.20 //% amount of range trim can adjust center point
#define TRIM_MIN 25 // trim pots get crazy around the edges
#define TRIM_MAX (ADC_MAX - TRIM_MIN)

#define NUM_BUTTONS      7
#define NUM_HAT_SWITCHES 1

// +--------------------------------------------------------------+
// |                           Globals                            |
// +--------------------------------------------------------------+

BleGamepad bleGamepad;
BleGamepadConfiguration bleGamepadConfig;

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

void setup()
{
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
	// pinMode(PIN_TOP_BTN,   INPUT_PULLUP);
	pinMode(PIN_MID_BTN,   INPUT_PULLUP);
	pinMode(PIN_BTM_BTN,   INPUT_PULLUP);

	// +==============================+
	// |            Serial            |
	// +==============================+
	// Serial.begin(9600); // TODO: needed?
	Serial.begin(115200);
	Serial.println("Starting BLE work!");
	
	// +==============================+
	// |          Bluetooth           |
	// +==============================+
	// bleGamepad.begin();

	bleGamepadConfig.setAutoReport(false);
	bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD); // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
	bleGamepadConfig.setButtonCount(NUM_BUTTONS);
	bleGamepadConfig.setHatSwitchCount(NUM_HAT_SWITCHES);
	bleGamepadConfig.setVid(0xe502);
	bleGamepadConfig.setPid(0xabcd);
	// Some non-Windows operating systems and web based gamepad testers don't like min axis set below 0, so 0 is set by default
	bleGamepadConfig.setAxesMin(0x8001); // -32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
	// bleGamepadConfig.setAxesMin(0x0000); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
	bleGamepadConfig.setAxesMax(0x7FFF); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal 
	bleGamepad.begin(&bleGamepadConfig); // Simulation controls, special buttons and hats 2/3/4 are disabled by default

	// Changing bleGamepadConfig after the begin function has no effect, unless you call the begin function again


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


void loop()
{

	int throttlePosition = analogRead(PIN_THT); // NOTE: 10 bit ADC (0-1023)
	int steeringPosition = analogRead(PIN_STR); // NOTE: 10 bit ADC (0-1023)
	int throttleTrimPosition = analogRead(PIN_THT_TRM); // NOTE: 10 bit ADC (0-1023)
	int steeringTrimPosition = analogRead(PIN_STR_TRM); // NOTE: 10 bit ADC (0-1023)

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
		analogWrite(PIN_LED_RED,   mapRange(adjustedThrottle, 0, ADC_MAX, PWM_LED_MIN, PWM_LED_MAX));
		analogWrite(PIN_LED_GREEN, mapRange(adjustedSteering, 0, ADC_MAX, PWM_LED_MIN, PWM_LED_MAX));
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
	int rightXAxis = (steeringTrimPosition - ADC_HALF) * CONV_MULTI;
	int rightYAxis = (XINPUT_MAX - (throttleTrimPosition - ADC_HALF) * CONV_MULTI);
	// XInput.setJoystick(JOY_RIGHT, rightXAxis, rightYAxis);

	// XInput.send();

	// +--------------------------------------------------------------+
	// |                         BLE Gamepad                          |
	// +--------------------------------------------------------------+
    if (bleGamepad.isConnected())
    {
        // Serial.println("Press buttons 5, 16 and start. Move all enabled axes to max. Set DPAD (hat 1) to down right.");
        if     (!LastThumbBtn && !digitalRead(PIN_THMB_BTN)){ bleGamepad.press(BUTTON_5); }
        else if( LastThumbBtn &&  digitalRead(PIN_THMB_BTN)){ bleGamepad.release(BUTTON_5); }

        if     (!LastMenuBtn && !digitalRead(PIN_MENU_BTN)){ bleGamepad.pressStart(); }
        else if( LastMenuBtn &&  digitalRead(PIN_MENU_BTN)){ bleGamepad.releaseStart(); }

        if     (!LastSetBtn && !digitalRead(PIN_SET_BTN)){ bleGamepad.press(BUTTON_16); }
        else if( LastSetBtn &&  digitalRead(PIN_SET_BTN)){ bleGamepad.release(BUTTON_16); }

        bleGamepad.setAxes(leftXAxis, leftYAxis, 0, rightXAxis, rightYAxis, 0);       //(X, Y, Z, RX, RY, RZ)
        bleGamepad.sendReport();
        delay(10);
        //bleGamepad.setHIDAxes(32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767);  //(X, Y, Z, RZ, RX, RY)
        // bleGamepad.setHat1(HAT_CENTERED);
        // All axes, sliders, hats etc can also be set independently. See the IndividualAxes.ino example
        // delay(1000);

        // // Serial.println("Release button 5 and start. Move all axes to min. Set DPAD (hat 1) to centred.");
        // bleGamepad.release(BUTTON_5);
        // bleGamepad.releaseStart();
        // bleGamepad.setHat1(HAT_CENTERED);
        // bleGamepad.setAxes(0, 0, 0, 0, 0, 0, 0, 0);           //(X, Y, Z, RX, RY, RZ)
        // //bleGamepad.setHIDAxes(0, 0, 0, 0, 0, 0, 0, 0);      //(X, Y, Z, RZ, RX, RY)
        // delay(1000);
    }
}
