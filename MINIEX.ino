#include <Arduino.h>
#include "Adafruit_MCP23X17.h"
#include <ESP32Servo.h> // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define clawServoPin 5
#define auxServoPin 18
#define cabLights 32
#define auxLights 33

#define LEFT_MOTOR 7
#define RIGHT_MOTOR 4

#define STOP 0
#define RUP 1
#define RDOWN 2
#define LUP 3
#define LDOWN 4
#define BOOMUP 5
#define BOOMDOWN 6
#define STICKUP 7
#define STICKDOWN 8
#define TILTUP 9
#define TILTDOWN 10
#define THUMBUP 11
#define THUMBDOWN 12
#define AUXUP 13
#define AUXDOWN 14
#define SWINGLEFT 15
#define SWINGRIGHT 16

#define pivot0 15       // Used for controlling pivot movement (e.g., for controlling tilt)
#define pivot1 14       // Used for controlling pivot movement (e.g., for controlling tilt)
#define mainBoom0 9     // Used for controlling the main boom movement
#define mainBoom1 8     // Used for controlling the main boom movement
#define secondBoom0 0   // Used for controlling the second boom movement
#define secondBoom1 1   // Used for controlling the second boom movement
#define tiltAttach0 3   // Used for controlling attachment tilt movement
#define tiltAttach1 2   // Used for controlling attachment tilt movement
#define thumb0 11       // Used for controlling the thumb movement
#define thumb1 10       // Used for controlling the thumb movement
#define auxAttach0 12   // Used for controlling auxiliary attachment movement
#define auxAttach1 13   // Used for controlling auxiliary attachment movement
#define leftMotor0 7    // Used for controlling the left motor movement
#define leftMotor1 6    // Used for controlling the left motor movement
#define rightMotor0 4   // Used for controlling the right motor movement
#define rightMotor1 5   // Used for controlling the right motor movement


#define FORWARD 1
#define BACKWARD -1

Adafruit_MCP23X17 mcp;
Servo clawServo;
Servo auxServo;
int dly = 250;
int clawServoValue = 90;
int auxServoValue = 90;
int player = 0;
int battery = 0;
int servoDelay = 0;

bool cabLightsOn = false;
bool auxLightsOn = false;
bool moveClawServoUp = false;
bool moveClawServoDown = false;
bool moveAuxServoUp = false;
bool moveAuxServoDown = false;

struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
};

MOTOR_PINS motorPins[] = {
    {4, 5},    // RIGHT_MOTOR Pins (IN1, IN2)
    {6, 7},    // LEFT_MOTOR Pins
    {15, 14},  // SWING_MOTOR pins
    {9, 8},    // BOOM Pins (IN1, IN2)
    {0, 1},    // STICK Pins
    {3, 2},    // TILT pins
    {11, 10},  // THUMB Pins (IN1, IN2)
    {12, 13}   // AUX Pins
};


void moveCar(ControllerPtr ctl, int inputValue)
{
    switch (inputValue)
    {
    case RUP:
        mcp.digitalWrite(rightMotor0, HIGH);
        mcp.digitalWrite(rightMotor1, LOW);
        Serial.println("Moving R forward");
        break;

    case RDOWN:            
        mcp.digitalWrite(rightMotor0, LOW);
        mcp.digitalWrite(rightMotor1, HIGH);
        Serial.println("Moving R backward");
        break;

    case LUP:
    mcp.digitalWrite(leftMotor0, HIGH);
    mcp.digitalWrite(leftMotor1, LOW);
        Serial.println("Moving L forward");
        break;

    case LDOWN:
    mcp.digitalWrite(leftMotor0, LOW);
    mcp.digitalWrite(leftMotor1, HIGH);
        Serial.println("Moving L backward");
        break;


    case BOOMUP:
      mcp.digitalWrite(mainBoom0, HIGH);
      mcp.digitalWrite(mainBoom1, LOW);
        Serial.println("Moving BOOM up");
        break;

    case BOOMDOWN:
      mcp.digitalWrite(mainBoom0, LOW);
      mcp.digitalWrite(mainBoom1, HIGH);
        Serial.println("Moving BOOM down");
        break;
        
    case STICKUP:
      mcp.digitalWrite(secondBoom0, HIGH);
      mcp.digitalWrite(secondBoom1, LOW);
        Serial.println("Moving STICK up");
        break;

    case STICKDOWN:
      mcp.digitalWrite(secondBoom0, LOW);
      mcp.digitalWrite(secondBoom1, HIGH);
        Serial.println("Moving STICK down");
        break;

    case TILTUP:
      mcp.digitalWrite(tiltAttach0, HIGH);
      mcp.digitalWrite(tiltAttach1, LOW);
        Serial.println("Moving TILT up");
        break;

    case TILTDOWN:
      mcp.digitalWrite(tiltAttach0, LOW);
      mcp.digitalWrite(tiltAttach1, HIGH);
        Serial.println("Moving TILT down");
        break;

    case THUMBUP:
    mcp.digitalWrite(thumb0, HIGH);
    mcp.digitalWrite(thumb1, LOW);
        Serial.println("Moving THUMB up");
        break;

    case THUMBDOWN:
    mcp.digitalWrite(thumb0, LOW);
    mcp.digitalWrite(thumb1, HIGH);
        Serial.println("Moving THUMB down");
        break;

        
    case AUXUP:
    mcp.digitalWrite(auxAttach0, LOW);
    mcp.digitalWrite(auxAttach1, HIGH);
        Serial.println("Moving AUX up");
        break;

    case AUXDOWN:
    mcp.digitalWrite(auxAttach0, HIGH);
    mcp.digitalWrite(auxAttach1, LOW);
        Serial.println("Moving AUX down");
        break;


    case SWINGLEFT:
      mcp.digitalWrite(pivot0, HIGH);
      mcp.digitalWrite(pivot1, LOW);
        Serial.println("Moving SWING_MOTOR LEFT");
        break;

    case SWINGRIGHT:
      mcp.digitalWrite(pivot0, LOW);
      mcp.digitalWrite(pivot1, HIGH);
        Serial.println("Moving SWING_MOTOR RIGHT");
        break;

    default:
    mcp.digitalWrite(rightMotor0, LOW);
    mcp.digitalWrite(rightMotor1, LOW);
    mcp.digitalWrite(leftMotor0, LOW);
    mcp.digitalWrite(leftMotor1, LOW);
    mcp.digitalWrite(thumb0, LOW);
    mcp.digitalWrite(thumb1, LOW);    
    mcp.digitalWrite(auxAttach0, LOW);
    mcp.digitalWrite(auxAttach1, LOW);      
    mcp.digitalWrite(pivot0, LOW);
    mcp.digitalWrite(pivot1, LOW);      
    mcp.digitalWrite(secondBoom0, LOW);
    mcp.digitalWrite(secondBoom1, LOW);      
    mcp.digitalWrite(tiltAttach0, LOW);
    mcp.digitalWrite(tiltAttach1, LOW);      
    mcp.digitalWrite(mainBoom0, LOW);
    mcp.digitalWrite(mainBoom1, LOW);
        break;
    }
}



//void handleRoot(AsyncWebServerRequest *request)
//{
//  request->send_P(200, "text/html", htmlHomePage);
//}

//void handleNotFound(AsyncWebServerRequest *request)
//{
//  request->send(404, "text/plain", "File Not Found");
//}



void setUpPinModes()
{

  for (int i = 0; i < sizeof(motorPins) / sizeof(motorPins[0]); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
  }
  for (int i = 0; i <= 15; i++) {
    mcp.pinMode(i, OUTPUT);
}

  //moveCar(STOP);

}


void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}



void processGamepad(ControllerPtr ctl) {
    int move_command = STOP;

    int deadzone = 300; // Adjust as needed
    int minimal_control_input_stick = 300;
    int right_x = ctl->axisRX();       // (-511 - 512) right X axis
    int right_y = ctl->axisRY();       // (-511 - 512) right Y axis
    int left_x = ctl->axisX();       // (-511 - 512) right X axis
    int left_y = ctl->axisY();       // (-511 - 512) right Y axis

    // Check only if move_command is STOP
    if (move_command == STOP) {
        if (abs(right_y) >= minimal_control_input_stick) {
            if (right_y > 0) move_command = STICKUP;
            else if (right_y < 0) move_command = STICKDOWN;
        } else if (abs(right_y) < deadzone) {
            move_command = STOP; // No need to set move_command again
        }

        if (abs(right_x) >= minimal_control_input_stick) {
            if (right_x > 0) move_command = TILTUP;
            else if (right_x < 0) move_command = TILTDOWN;
        } else if (abs(right_x) < deadzone) {
            move_command = STOP; // No need to set move_command again
        }

        if (abs(left_x) >= minimal_control_input_stick) {
            if (left_x > 0) move_command = SWINGLEFT;
            else if (left_x < 0) move_command = SWINGRIGHT;
        } else if (abs(left_x) < deadzone) {
            move_command = STOP; // No need to set move_command again
        }

        if (abs(left_y) >= minimal_control_input_stick) {
            if (left_y > 0) move_command = BOOMUP;

            else if (left_y < 0) move_command = BOOMDOWN;
        } else if (abs(left_y) < deadzone) {
            move_command = STOP; // No need to set move_command again
        }
    }
        int l1_l2_command = ctl->buttons();
    if(l1_l2_command ==0x0010) move_command = RUP;
    Serial.println("Moving r up");
    if(l1_l2_command ==0x0040) move_command = RDOWN;
    Serial.println("Moving r down");
    if(l1_l2_command ==0x0020) move_command = LUP;
    Serial.println("Moving l up");
    if(l1_l2_command ==0x0080) move_command = LDOWN;
    Serial.println("Moving l down");
    // Pass the move command to the moveCar function
    moveCar(ctl, move_command);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    mcp.begin_I2C();
  //   put your setup code here, to run once:
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);
    // You could add additional error handling here,
    // such as logging the error or attempting to recover.
    // For example, you might attempt to reset the MCP23X17
    // and retry initialization before giving up completely.
    // Then, you could gracefully exit the program or continue
    // running with limited functionality.
    mcp.pinMode(leftMotor0, OUTPUT);
    mcp.pinMode(leftMotor1, OUTPUT);
    mcp.pinMode(rightMotor0, OUTPUT);
    mcp.pinMode(rightMotor1, OUTPUT);
    mcp.pinMode(pivot0, OUTPUT);
    mcp.pinMode(pivot1, OUTPUT);
    mcp.pinMode(mainBoom0, OUTPUT);
    mcp.pinMode(mainBoom1, OUTPUT);
    mcp.pinMode(secondBoom0, OUTPUT);
    mcp.pinMode(secondBoom1, OUTPUT);
    mcp.pinMode(tiltAttach0, OUTPUT);
    mcp.pinMode(tiltAttach1, OUTPUT);
    mcp.pinMode(thumb0, OUTPUT);
    mcp.pinMode(thumb1, OUTPUT);
    mcp.pinMode(auxAttach0, OUTPUT);
    mcp.pinMode(auxAttach1, OUTPUT);

     pinMode(clawServoPin, OUTPUT);
  pinMode(auxServoPin, OUTPUT);

  pinMode(cabLights, OUTPUT);
  pinMode(auxLights, OUTPUT);

  clawServo.attach(clawServoPin);
  auxServo.attach(auxServoPin);
  clawServo.write(clawServoValue);
  auxServo.write(auxServoValue);
  }



// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
    {
        processControllers();
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    else {vTaskDelay(1);}
}

