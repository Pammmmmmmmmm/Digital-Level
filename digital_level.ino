#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define trigPin 3    // Pin 12 trigger output
#define echoPin 2    // Pin 2 Echo input
#define buzzerPin 7  // Pin 7 Buzzer output
#define echo_int 0   // Interrupt id for echo pulse

#define TIMER_US 50       // 50 uS timer duration
#define TICK_COUNTS 4000  // 200 mS worth of timer ticks

volatile long echo_start = 0;             // Records start of echo pulse
volatile long echo_end = 0;               // Records end of echo pulse
volatile long echo_duration = 0;          // Duration - difference between end and start
volatile int trigger_time_count = 0;      // Count down counter to trigger pulse time
volatile long range_flasher_counter = 0;  // Count down counter for flashing distance LED

// Level LEDs
const int levelLED_neg1 = 8;
const int levelLED_neg0 = 9;
const int levelLED_level = 10;
const int levelLED_pos0 = 11;
const int levelLED_pos1 = 12;

// Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
uint32_t mpu_loop_timer;
uint32_t ultra_loop_timer;
int temp;

// Display counter
int displaycount = 0;

void setup() {
    pinMode(trigPin, OUTPUT);      // Trigger pin set to output
    pinMode(echoPin, INPUT);       // Echo pin set to input
    pinMode(buzzerPin, OUTPUT);    // Buzzer pin set to output
    pinMode(LED_BUILTIN, OUTPUT);  // Onboard LED pin set to output

    Timer1.initialize(TIMER_US);                        // Initialise timer 1
    Timer1.attachInterrupt(timerISR);                   // Attach interrupt to the timer service routine
    attachInterrupt(echo_int, echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input

    // Start I2C
    Wire.begin();

    lcd.init();
    lcd.backlight();
    lcd.clear();

    // Set Level LEDs as outputs
    pinMode(levelLED_neg1, OUTPUT);
    pinMode(levelLED_neg0, OUTPUT);
    pinMode(levelLED_level, OUTPUT);
    pinMode(levelLED_pos0, OUTPUT);
    pinMode(levelLED_pos1, OUTPUT);

    // Setup the registers of the MPU-6050
    setup_mpu_6050_registers();

    // Read the raw acc and gyro data from the MPU-6050 1000 times
    for (int cal_int = 0; cal_int < 1000; cal_int++) {
        read_mpu_6050_data();
        // Add the gyro x offset to the gyro_x_cal variable
        gyro_x_cal += gyro_x;
        // Add the gyro y offset to the gyro_y_cal variable
        gyro_y_cal += gyro_y;
        // Add the gyro z offset to the gyro_z_cal variable
        gyro_z_cal += gyro_z;
        // Delay 3us to have 250Hz for-loop
        delay(3);
    }

    // Divide all results by 1000 to get average offset
    gyro_x_cal /= 1000;
    gyro_y_cal /= 1000;
    gyro_z_cal /= 1000;

    // Start Serial Monitor
    Serial.begin(115200);

    // Init Timer
    mpu_loop_timer = micros();
    ultra_loop_timer = millis();
}

void loop() {
    if (micros() - mpu_loop_timer >= 2000) {
        mpu_loop_timer = micros();

        // Get data from MPU-6050
        read_mpu_6050_data();

        // Subtract the offset values from the raw gyro values
        gyro_x -= gyro_x_cal;
        gyro_y -= gyro_y_cal;
        gyro_z -= gyro_z_cal;

        // Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)

        // Calculate the traveled pitch angle and add this to the angle_pitch variable
        angle_pitch += gyro_x * 0.0000611;
        // Calculate the traveled roll angle and add this to the angle_roll variable
        // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
        angle_roll += gyro_y * 0.0000611;

        // If the IMU has yawed transfer the roll angle to the pitch angle
        angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
        // If the IMU has yawed transfer the pitch angle to the roll angle
        angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);

        // Accelerometer angle calculations

        // Calculate the total accelerometer vector
        acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

        // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
        // Calculate the pitch angle
        angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
        // Calculate the roll angle
        angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;

        // Accelerometer calibration value for pitch
        angle_pitch_acc -= 0.0;
        // Accelerometer calibration value for roll
        angle_roll_acc -= 0.0;

        if (set_gyro_angles) {
            // If the IMU has been running
            // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
            angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
            // Correct the drift of the gyro roll angle with the accelerometer roll angle
            angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
        } else {
            // IMU has just started
            // Set the gyro pitch angle equal to the accelerometer pitch angle
            angle_pitch = angle_pitch_acc;
            // Set the gyro roll angle equal to the accelerometer roll angle
            angle_roll = angle_roll_acc;
            // Set the IMU started flag
            set_gyro_angles = true;
        }

        // To dampen the pitch and roll angles a complementary filter is used
        // Take 90% of the output pitch value and add 10% of the raw pitch value
        angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
        // Take 90% of the output roll value and add 10% of the raw roll value
        angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
        // Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop

        // Print to Serial Monitor
        // Serial.print(" | Angle  = "); Serial.println(angle_pitch_output);

        // Increment the display counter
        displaycount = displaycount + 1;

        if (displaycount > 100) {
            // Print on first row of LCD
            lcd.setCursor(0, 0);
            lcd.print("P: ");
            lcd.print(angle_pitch_output);
            lcd.print("  R: ");
            lcd.print(angle_roll_output);

            // Check Angle for Level LEDs
            digitalWrite(buzzerPin, LOW);
            if (angle_pitch_output < -2.01) {
                // Turn on Level LED
                digitalWrite(levelLED_neg1, HIGH);
                digitalWrite(levelLED_neg0, LOW);
                digitalWrite(levelLED_level, LOW);
                digitalWrite(levelLED_pos0, LOW);
                digitalWrite(levelLED_pos1, LOW);
            } else if ((angle_pitch_output > -2.00) && (angle_pitch_output < -1.01)) {
                // Turn on Level LED
                digitalWrite(levelLED_neg1, LOW);
                digitalWrite(levelLED_neg0, HIGH);
                digitalWrite(levelLED_level, LOW);
                digitalWrite(levelLED_pos0, LOW);
                digitalWrite(levelLED_pos1, LOW);
            } else if ((angle_pitch_output < 1.00) && (angle_pitch_output > -1.00)) {
                // Turn on Level LED
                digitalWrite(levelLED_neg1, LOW);
                digitalWrite(levelLED_neg0, LOW);
                digitalWrite(levelLED_level, HIGH);
                digitalWrite(levelLED_pos0, LOW);
                digitalWrite(levelLED_pos1, LOW);
                flash_buzzer();
            } else if ((angle_pitch_output > 1.01) && (angle_pitch_output < 2.00)) {
                // Turn on Level LED
                digitalWrite(levelLED_neg1, LOW);
                digitalWrite(levelLED_neg0, LOW);
                digitalWrite(levelLED_level, LOW);
                digitalWrite(levelLED_pos0, HIGH);
                digitalWrite(levelLED_pos1, LOW);
            } else if (angle_pitch_output > 2.01) {
                // Turn on Level LED
                digitalWrite(levelLED_neg1, LOW);
                digitalWrite(levelLED_neg0, LOW);
                digitalWrite(levelLED_level, LOW);
                digitalWrite(levelLED_pos0, LOW);
                digitalWrite(levelLED_pos1, HIGH);
            }

            displaycount = 0;
        }
    }

    if (millis() - ultra_loop_timer >= 500) {
        ultra_loop_timer = millis();

        float distance = echo_duration * 0.034 / 2;
        Serial.println(distance);  // Print the distance in centimeters
        lcd.setCursor(0, 1);
        lcd.print("Jarak: ");
        lcd.print(distance, 1);
        lcd.print(" cm   ");
    }
}

void setup_mpu_6050_registers() {
    // Activate the MPU-6050

    // Start communicating with the MPU-6050
    Wire.beginTransmission(0x68);
    // Send the requested starting register
    Wire.write(0x6B);
    // Set the requested starting register
    Wire.write(0x00);
    // End the transmission
    Wire.endTransmission();

    // Configure the accelerometer (+/-8g)

    // Start communicating with the MPU-6050
    Wire.beginTransmission(0x68);
    // Send the requested starting register
    Wire.write(0x1C);
    // Set the requested starting register
    Wire.write(0x10);
    // End the transmission
    Wire.endTransmission();

    // Configure the gyro (500dps full scale)

    // Start communicating with the MPU-6050
    Wire.beginTransmission(0x68);
    // Send the requested starting register
    Wire.write(0x1B);
    // Set the requested starting register
    Wire.write(0x08);
    // End the transmission
    Wire.endTransmission();
}

void read_mpu_6050_data() {
    // Read the raw gyro and accelerometer data

    // Start communicating with the MPU-6050
    Wire.beginTransmission(0x68);
    // Send the requested starting register
    Wire.write(0x3B);
    // End the transmission
    Wire.endTransmission();
    // Request 14 bytes from the MPU-6050
    Wire.requestFrom(0x68, 14);
    // Wait until all the bytes are received
    while (Wire.available() < 14) {
        ;
    }

    // Following statements left shift 8 bits, then bitwise OR.
    // Turns two 8-bit values into one 16-bit value
    acc_x = Wire.read() << 8 | Wire.read();
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
}

// --------------------------
// timerISR() 50uS second interrupt ISR()
// Called every time the hardware timer 1 times out.
// --------------------------
void timerISR() {
    trigger_pulse();     // Schedule the trigger pulses
    distance_flasher();  // Flash the onboard LED distance indicator
}

// --------------------------
// trigger_pulse() called every 50 uS to schedule trigger pulses.
// Generates a pulse one timer tick long.
// Minimum trigger pulse width for the HC-SR04 is 10 us. This system
// delivers a 50 uS pulse.
// --------------------------
void trigger_pulse() {
    static volatile int state = 0;  // State machine variable

    if (!(--trigger_time_count))           // Count to 200mS
    {                                      // Time out - Initiate trigger pulse
        trigger_time_count = TICK_COUNTS;  // Reload
        state = 1;                         // Changing to state 1 initiates a pulse
    }

    switch (state)  // State machine handles delivery of trigger pulse
    {
        case 0:  // Normal state does nothing
            break;

        case 1:                           // Initiate pulse
            digitalWrite(trigPin, HIGH);  // Set the trigger output high
            state = 2;                    // and set state to 2
            break;

        case 2:  // Complete the pulse
        default:
            digitalWrite(trigPin, LOW);  // Set the trigger output low
            state = 0;                   // and return state to normal 0
            break;
    }
}

// --------------------------
// echo_interrupt() External interrupt from HC-SR04 echo signal.
// Called every time the echo signal changes state.
//
// Note: this routine does not handle the case where the timer
//       counter overflows which will result in the occassional error.
// --------------------------
void echo_interrupt() {
    switch (digitalRead(echoPin))  // Test to see if the signal is high or low
    {
        case HIGH:                  // High so must be the start of the echo pulse
            echo_end = 0;           // Clear the end time
            echo_start = micros();  // Save the start time
            break;

        case LOW:                                   // Low so must be the end of hte echo pulse
            echo_end = micros();                    // Save the end time
            echo_duration = echo_end - echo_start;  // Calculate the pulse duration
            break;
    }
}

// --------------------------
// distance_flasher() Called from the timer 1 timerISR service routine.
// Flashes the onboard LED at a rate inversely proportional
// to distance. The closer it gets the higher the frequency.
// --------------------------
void distance_flasher() {
    if (--range_flasher_counter <= 0)  // Decrement and test the flash timer
    {                                  // Flash timer time out
        if (echo_duration < 25000)     // If the echo duration is within limits
        {
            range_flasher_counter = echo_duration * 2;  // Reload the timer with the current echo duration
        } else {
            range_flasher_counter = 25000;  // If out of range use a default
        }

        digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);  // Toggle the onboard LED
    }
}

void flash_buzzer() {
    static uint32_t buzzer_timer = millis();

    if (millis() - buzzer_timer >= 500) {
        buzzer_timer = millis();

        static bool buzzer_state = false;
        buzzer_state = !buzzer_state;
        digitalWrite(buzzerPin, buzzer_state);
    }
}