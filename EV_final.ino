//libraries

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//screen

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//pins

Encoder myEnc(2, 3);
const int button_pin = 6;
const int voltagePin = A0;

//encoder pins

const int forward_motor_pin = 11;
const int backward_motor_pin = 9;

//constants

const int ticks_per_revolution = 204;  //amount of encoder "ticks" per revolution
const int tolerance = 10; //amount of encoder "ticks" you are okay being off. This being to low may cause it to never settel into a state.
const int minimumPWM = 200; //the minimum pwm where the motor stalls. Add some safety margin
const float wheel_circumfrance = 5; //in cm
const int car_length = 46; //in cm; from the iner mostpoint of the bottle catcher to the mp

const int bottle_distance = 2; //in meters
const float end_distance = 5; //in meters; from the start point
const float bottle_line_distance = 1; //in meters; from the end point
const float bottle_extra = 10; //how much you wnat to overshoot the bottle line in cm

const float run_time = 15;
const int max_wait_time = 3000;
const int wiggle_time = 718;
unsigned long start;
unsigned long elapsedTime;

//PID stuff

double Kp = 0.4;
double Ki = 0.02;
double Kd = 0.2;

double Input;
double Output;
double Setpoint;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {

    Serial.begin(9600);

    pinMode(button_pin, INPUT_PULLUP);
    pinMode(forward_motor_pin, OUTPUT);
    pinMode(backward_motor_pin, OUTPUT);

    myPID.SetOutputLimits(-255, 255);
    myPID.SetSampleTime(1); //in ms
    myPID.SetMode(AUTOMATIC);

    myEnc.write(0);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;); // Don't proceed, loop forever if allocation fails
    }

    // Clear the buffer
    display.clearDisplay();

    // Set text size and color
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(WHITE); // Draw white text
    display.setCursor(0,0);     // Start at top-left corner

    while (digitalRead(button_pin) == HIGH) {}

    delay(1000);

    start = millis();

    motorDistance((bottle_distance*100)-car_length);
    motorDistance((end_distance*100)-(bottle_distance*100)+(bottle_line_distance*100)+bottle_extra);
    motorDistance((-1.0*((bottle_line_distance*100)+bottle_extra))+car_length);
    motorDrive(0);

    wait(((run_time * 1000) - (millis() - start))-wiggle_time);
    wiggle();

    elapsedTime = millis() - start;

    int sensorValue = analogRead(voltagePin);
    float voltage = ((sensorValue * (3.3 / 1023.0))*4.6);
    display.println(voltage);

    display.println(elapsedTime);
    display.display();
}
void loop(){
}
void motorDrive(int pwmvalue) {

    if(abs(pwmvalue) < minimumPWM && abs(Input-Setpoint) > tolerance) {

        pwmvalue = copysign(minimumPWM, pwmvalue);

    }

    if (pwmvalue >= 0) {

        analogWrite(forward_motor_pin, pwmvalue);
        analogWrite(backward_motor_pin, 0); }

        else { analogWrite(forward_motor_pin, 0);

            analogWrite(backward_motor_pin, abs(pwmvalue));

        }
}
void motorDistance(float revolution) {

    Setpoint = (revolution / (wheel_circumfrance * PI))* ticks_per_revolution;

    myEnc.write(0); Output = 0;

    myPID.SetMode(MANUAL);
    myPID.SetMode(AUTOMATIC);

    while ((abs(Input - Setpoint) > tolerance)) {

        Input = myEnc.read();

        if (myPID.Compute()) {

            motorDrive((int)Output);

        }
    }

    analogWrite(forward_motor_pin, HIGH);
    analogWrite(backward_motor_pin, HIGH);
}

void wiggle() {
    motorDistance(5);
    motorDistance(-5);
}

int wait(int wait_time) {
    if (wait_time < max_wait_time) {
        delay(wait_time);
    }
    else if (wait_time < (2 * max_wait_time)) {
        delay(wait_time / 2);
        wiggle();
        delay(((run_time * 1000) - (millis() - start)) - wiggle_time);
    }
    else {
        delay(max_wait_time);
        wiggle();
        wait(((run_time * 1000) - (millis() - start)) - wiggle_time);
    }
}
