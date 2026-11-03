
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

// ============================================================
// TASK 4 - SANDSTORM TRAVERSE
// MOTOR SPEED CONTROL AND ANTI-STALL PROTECTION
//
// Hardware simulation:
//   ESP32 + 2 A4988 drivers + 2 stepper motors
//   2 potentiometers to simulate motor current
//   2 warning LEDs + 1 buzzer
//
// FreeRTOS jobs:
//   Job A: Motor control       -> 100 Hz (every 10 ms)
//   Job B: Communication       -> 20 Hz  (every 50 ms)
// ============================================================


// ============================================================
// 1. PIN CONFIGURATION
// These pin numbers must match your Wokwi wiring.
// ============================================================

// LEFT MOTOR
#define LEFT_STEP 18
#define LEFT_DIR  19
#define LEFT_EN   23
#define LEFT_POT  34
#define LEFT_LED  32

// RIGHT MOTOR
#define RIGHT_STEP 25
#define RIGHT_DIR  26
#define RIGHT_EN   27
#define RIGHT_POT  35
#define RIGHT_LED  33

// BUZZER
#define BUZZER 14


// ============================================================
// 2. SYSTEM PARAMETERS
// These values define the warning, stall and task behaviour.
// ============================================================

// Current above 15 A activates the warning LED and buzzer.
constexpr float WARNING_CURRENT = 15.0;

// Current above 20 A continuously for 350 ms is a stall.
constexpr float STALL_CURRENT = 20.0;
constexpr unsigned long STALL_TIME_MS = 350;

// After a stall, reverse the motor for 200 ms.
constexpr unsigned long RECOVERY_TIME_MS = 200;

// Recovery speed is 20% of the configured maximum step rate.
// This is a simulation of reduced speed, NOT measured motor power.
constexpr float RECOVERY_POWER = 0.20;

// FreeRTOS task periods.
constexpr unsigned long JOB_A_PERIOD_MS = 10;
constexpr unsigned long JOB_B_PERIOD_MS = 50;

// Maximum step pulse rate used by this simulation.
constexpr float MAX_STEP_RATE = 400.0;


// ============================================================
// 3. SHARED DATA AND SYNCHRONIZATION
//
// Both FreeRTOS tasks need to access some common information.
// A mutex-like critical-section lock protects shared variables
// from being read or changed halfway through another operation.
// ============================================================

portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// Movement command received through Serial.
// v     = forward/reverse speed command
// w     = turning command
// width = distance between the two wheels
float commandV = 100.0;
float commandW = 0.0;
float wheelWidth = 300.0;

// Values published by Job A for Job B to display.
float leftCurrent = 0.0;
float rightCurrent = 0.0;

bool leftWarning = false;
bool rightWarning = false;

bool leftRecovering = false;
bool rightRecovering = false;

// Job A increments this counter every time it completes a cycle.
// Job B reads and resets it approximately once per second.
volatile unsigned long jobACount = 0;


// ============================================================
// 4. MOTOR DATA STRUCTURE
//
// Both motors use the same control logic. Instead of writing
// separate functions for each motor, this structure stores
// the pins and operating information for one motor.
// ============================================================

struct Motor {

    // Hardware pins
    int stepPin;
    int dirPin;
    int enPin;
    int potPin;
    int ledPin;

    // Current operating information
    float current = 0.0;
    bool warning = false;
    bool recovering = false;

    // Stall timing information
    bool highCurrentTiming = false;
    unsigned long highCurrentStart = 0;

    // Recovery end time
    unsigned long recoveryEnd = 0;

    // Fractional step calculation
    float stepAccumulator = 0.0;

    // Remember the current direction to avoid unnecessary writes
    int direction = HIGH;
};


// Create one motor object for each side.
Motor leftMotor = {
    LEFT_STEP, LEFT_DIR, LEFT_EN, LEFT_POT, LEFT_LED
};

Motor rightMotor = {
    RIGHT_STEP, RIGHT_DIR, RIGHT_EN, RIGHT_POT, RIGHT_LED
};


// ============================================================
// 5. DIFFERENTIAL STEERING
//
// This function converts one vehicle movement command into
// separate left and right motor speed commands.
//
// Formula:
//   Left speed  = v - (w * width / 2)
//   Right speed = v + (w * width / 2)
//
// A positive or negative result determines motor direction.
// constrain() prevents the requested speed from exceeding
// the configured maximum step rate.
// ============================================================

void calculateWheelSpeeds(
    float v,
    float w,
    float width,
    float &leftSpeed,
    float &rightSpeed
) {

    leftSpeed = v - (w * width / 2.0);
    rightSpeed = v + (w * width / 2.0);

    // Keep both motor commands within the allowed range.
    leftSpeed = constrain(
        leftSpeed, -MAX_STEP_RATE, MAX_STEP_RATE
    );

    rightSpeed = constrain(
        rightSpeed, -MAX_STEP_RATE, MAX_STEP_RATE
    );
}


// ============================================================
// 6. SIMULATED CURRENT READING
//
// The potentiometers do not measure real motor current.
// They simulate a current value between 0 A and 30 A.
//
// ESP32 ADC:
//   0    -> approximately 0 A
//   4095 -> approximately 30 A
//
// This allows us to test warning and stall conditions in Wokwi.
// ============================================================

float readCurrent(int pin) {

    int rawValue = analogRead(pin);

    float simulatedCurrent =
        (rawValue / 4095.0) * 30.0;

    return simulatedCurrent;
}


// ============================================================
// 7. MOTOR CONTROL AND ANTI-STALL PROTECTION
//
// This function controls one motor independently.
//
// It performs five main operations:
//   1. Read the simulated current.
//   2. Activate the warning LED if current exceeds 15 A.
//   3. Detect a stall if current exceeds 20 A for 350 ms.
//   4. Reverse the motor for 200 ms during recovery.
//   5. Generate STEP pulses to control motor speed.
//
// Job A calls this function once for the left motor and once
// for the right motor during every 10 ms control cycle.
// ============================================================

void controlMotor(Motor &m, float targetSpeed) {

    unsigned long now = millis();


    // --------------------------------------------------------
    // STEP 1: Read the current simulated by the potentiometer.
    // --------------------------------------------------------

    m.current = readCurrent(m.potPin);


    // --------------------------------------------------------
    // STEP 2: Overcurrent warning.
    //
    // If current is above 15 A, turn on this motor's warning LED.
    // Otherwise, turn it off.
    // --------------------------------------------------------

    m.warning = (m.current > WARNING_CURRENT);

    digitalWrite(
        m.ledPin,
        m.warning ? HIGH : LOW
    );


    // --------------------------------------------------------
    // STEP 3: Stall detection.
    //
    // A single high-current reading is not enough to declare
    // a stall. Current must remain above 20 A for 350 ms.
    //
    // highCurrentTiming tells us whether the timer has started.
    // --------------------------------------------------------

    if (!m.recovering) {

        if (m.current > STALL_CURRENT) {

            // Start the timer only on the first high-current
            // reading. Do not restart it on every task cycle.
            if (!m.highCurrentTiming) {

                m.highCurrentTiming = true;
                m.highCurrentStart = now;
            }

            // Check whether high current has lasted 350 ms.
            if (now - m.highCurrentStart >= STALL_TIME_MS) {

                // Stall confirmed: begin recovery.
                m.recovering = true;

                // Record when the recovery period should finish.
                m.recoveryEnd = now + RECOVERY_TIME_MS;

            }

        } else {

            // Current returned to a safe level before 350 ms.
            // Cancel the stall timer.
            m.highCurrentTiming = false;
        }
    }


    // --------------------------------------------------------
    // STEP 4: Check whether recovery has finished.
    //
    // The signed subtraction safely handles the millis()
    // counter wrapping around.
    // --------------------------------------------------------

    if (m.recovering &&
        (int32_t)(now - m.recoveryEnd) >= 0) {

        m.recovering = false;

        // Clear the old stall timing information.
        m.highCurrentTiming = false;
    }


    // --------------------------------------------------------
    // STEP 5: Determine the actual motor speed.
    //
    // Normal operation:
    //   Use the requested target speed.
    //
    // Recovery:
    //   Reverse direction at 20% of the configured maximum
    //   step rate. This ensures recovery can move even when
    //   the normal target speed is zero.
    // --------------------------------------------------------

    float actualSpeed = targetSpeed;

    if (m.recovering) {

        float recoverySpeed = MAX_STEP_RATE * RECOVERY_POWER;

        // Negative speed means reverse direction.
        // The original target direction is not used here.
        actualSpeed = -recoverySpeed;
    }


    // --------------------------------------------------------
    // STEP 6: Set the motor direction.
    //
    // Positive speed -> HIGH direction signal.
    // Negative speed -> LOW direction signal.
    //
    // The direction pin is only updated when direction changes.
    // The step accumulator is reset after a direction change.
    // --------------------------------------------------------

    int newDirection =
        (actualSpeed >= 0) ? HIGH : LOW;

    if (newDirection != m.direction) {

        digitalWrite(m.dirPin, newDirection);

        m.direction = newDirection;

        // Avoid carrying fractional steps across a reversal.
        m.stepAccumulator = 0.0;
    }


    // --------------------------------------------------------
    // STEP 7: Generate step pulses.
    //
    // A4988 drivers move the motor by one step for each STEP
    // pulse. The number of pulses determines motor speed.
    //
    // The accumulator stores fractional steps between cycles.
    // For example, 100 steps/second over 10 ms equals one step.
    //
    // Job A runs every 10 ms, so the time factor is 0.010 sec.
    // --------------------------------------------------------

    float stepRate = fabs(actualSpeed);

    m.stepAccumulator +=
        stepRate * (JOB_A_PERIOD_MS / 1000.0);

    // Generate all whole steps accumulated during this cycle.
    while (m.stepAccumulator >= 1.0) {

        digitalWrite(m.stepPin, HIGH);

        // A short HIGH pulse tells the A4988 to take a step.
        delayMicroseconds(3);

        digitalWrite(m.stepPin, LOW);

        m.stepAccumulator -= 1.0;
    }
}


// ============================================================
// 8. JOB A - MOTOR CONTROL TASK
//
// Frequency: 100 Hz
// Period:    10 ms
// Priority:  Higher than Job B
//
// Main responsibility:
//   - Read the latest movement command.
//   - Calculate left and right wheel speeds.
//   - Control both motors.
//   - Update warning and recovery information.
//   - Activate the buzzer when either motor has a warning.
//
// vTaskDelayUntil() keeps the task on a regular 10 ms schedule.
// ============================================================

void motorControlTask(void *parameter) {

    // Save the initial FreeRTOS tick count.
    // vTaskDelayUntil uses it to maintain a fixed task period.
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {

        float v;
        float w;
        float width;


        // ----------------------------------------------------
        // Read shared movement commands safely.
        //
        // The communication task may update these values.
        // The critical section prevents partially updated data
        // from being read by the motor task.
        // ----------------------------------------------------

        portENTER_CRITICAL(&dataMux);

        v = commandV;
        w = commandW;
        width = wheelWidth;

        portEXIT_CRITICAL(&dataMux);


        // ----------------------------------------------------
        // Calculate the left and right motor speed commands.
        // ----------------------------------------------------

        float leftSpeed;
        float rightSpeed;

        calculateWheelSpeeds(
            v,
            w,
            width,
            leftSpeed,
            rightSpeed
        );


        // ----------------------------------------------------
        // Control each motor independently.
        //
        // Each motor has its own current reading, warning,
        // stall timer and recovery state.
        // ----------------------------------------------------

        controlMotor(leftMotor, leftSpeed);

        controlMotor(rightMotor, rightSpeed);


        // ----------------------------------------------------
        // Buzzer warning.
        //
        // The buzzer sounds if either motor current exceeds
        // the 15 A warning threshold.
        // ----------------------------------------------------

        if (leftMotor.warning || rightMotor.warning) {

            tone(BUZZER, 2000);

        } else {

            noTone(BUZZER);
        }


        // ----------------------------------------------------
        // Publish motor information for Job B.
        //
        // Job B reads these shared values for telemetry.
        // The critical section keeps the published snapshot
        // consistent.
        // ----------------------------------------------------

        portENTER_CRITICAL(&dataMux);

        leftCurrent = leftMotor.current;
        rightCurrent = rightMotor.current;

        leftWarning = leftMotor.warning;
        rightWarning = rightMotor.warning;

        leftRecovering = leftMotor.recovering;
        rightRecovering = rightMotor.recovering;

        // Record one completed Job A execution.
        jobACount++;

        portEXIT_CRITICAL(&dataMux);


        // ----------------------------------------------------
        // Maintain the 100 Hz schedule.
        //
        // The task waits until the next 10 ms release time.
        // This is preferable to using delay(10), because
        // vTaskDelayUntil schedules against the previous wake
        // time instead of simply delaying after execution.
        // ----------------------------------------------------

        vTaskDelayUntil(
            &lastWake,
            pdMS_TO_TICKS(JOB_A_PERIOD_MS)
        );
    }
}


// ============================================================
// 9. SERIAL COMMAND BUFFER
//
// Job B receives commands from the Serial Monitor one character
// at a time. The buffer stores characters until a newline arrives.
// ============================================================

char inputBuffer[100];
int inputIndex = 0;


// ============================================================
// 10. COMMAND PROCESSING
//
// Expected command format:
//   v,w,width
//
// Examples:
//   100,0,300     -> straight movement
//   100,0.5,300   -> turn in one direction
//   100,-0.5,300  -> turn in the opposite direction
//   0,0,300       -> stop
//   -100,0,300    -> reverse
//
// The function validates the command before updating the
// shared movement variables.
// ============================================================

void processCommand(char *input) {

    float v;
    float w;
    float width;


    // Convert the comma-separated text into three numbers.
    if (sscanf(input, "%f,%f,%f", &v, &w, &width) != 3) {

        Serial.println("INVALID COMMAND");
        Serial.println("Use format: v,w,width");

        return;
    }


    // Reject commands outside the allowed input ranges.
    if (
        fabs(v) > 400.0 ||
        fabs(w) > 5.0 ||
        width <= 0.0 ||
        width > 1000.0
    ) {

        Serial.println("COMMAND OUT OF RANGE");

        return;
    }


    // Update all three movement variables together.
    // Job A will use the new command in its next cycle.
    portENTER_CRITICAL(&dataMux);

    commandV = v;
    commandW = w;
    wheelWidth = width;

    portEXIT_CRITICAL(&dataMux);

    Serial.println("COMMAND ACCEPTED");
}


// ============================================================
// 11. JOB B - COMMUNICATION AND TELEMETRY TASK
//
// Frequency: 20 Hz
// Period:    50 ms
// Priority:  Lower than Job A
//
// Main responsibility:
//   - Receive Serial Monitor commands.
//   - Validate and process movement commands.
//   - Count Job B executions.
//   - Display motor current, warnings and recovery states.
//
// Telemetry is printed once per second to avoid flooding the
// Serial Monitor. The task itself still runs every 50 ms.
// ============================================================

void communicationTask(void *parameter) {

    TickType_t lastWake = xTaskGetTickCount();

    // Local counter for Job B.
    // Only this task updates this variable.
    unsigned long jobBCount = 0;

    unsigned long lastReport = millis();


    while (true) {


        // ----------------------------------------------------
        // STEP 1: Receive Serial commands.
        //
        // Read every available character without blocking
        // while waiting for a complete command.
        // ----------------------------------------------------

        while (Serial.available() > 0) {

            char ch = Serial.read();

            if (ch == '\n' || ch == '\r') {

                // A newline means the command is complete.
                if (inputIndex > 0) {

                    inputBuffer[inputIndex] = '\0';

                    processCommand(inputBuffer);

                    inputIndex = 0;
                }

            } else if (inputIndex < 99) {

                // Store the character if the buffer has space.
                inputBuffer[inputIndex++] = ch;

            } else {

                // Buffer overflow protection:
                // discard the incomplete command and start again.
                inputIndex = 0;
            }
        }


        // ----------------------------------------------------
        // STEP 2: Count one Job B execution.
        // ----------------------------------------------------

        jobBCount++;


        // ----------------------------------------------------
        // STEP 3: Print telemetry once per second.
        //
        // Job A runs at 100 Hz, so its expected count is
        // approximately 100 executions per second.
        //
        // Job B runs at 20 Hz, so its expected count is
        // approximately 20 executions per second.
        // ----------------------------------------------------

        if (millis() - lastReport >= 1000) {

            lastReport = millis();

            float lc;
            float rc;

            bool lw;
            bool rw;

            bool lr;
            bool rr;

            unsigned long countA;


            // Read a consistent telemetry snapshot.
            portENTER_CRITICAL(&dataMux);

            lc = leftCurrent;
            rc = rightCurrent;

            lw = leftWarning;
            rw = rightWarning;

            lr = leftRecovering;
            rr = rightRecovering;

            // Read and reset the Job A counter.
            countA = jobACount;
            jobACount = 0;

            portEXIT_CRITICAL(&dataMux);


            // ------------------------------------------------
            // Print FreeRTOS execution counters.
            // ------------------------------------------------

            Serial.println();
            Serial.println("================================");
            Serial.println("       TASK 4 FREERTOS MONITOR");
            Serial.println("================================");

            Serial.print("JOB A - Motor Control: ");
            Serial.print(countA);
            Serial.println(" executions/sec");

            Serial.print("JOB B - Communication: ");
            Serial.print(jobBCount);
            Serial.println(" executions/sec");


            // ------------------------------------------------
            // Print LEFT motor status.
            // ------------------------------------------------

            Serial.println("--------------------------------");

            Serial.print("LEFT CURRENT: ");
            Serial.print(lc, 2);

            Serial.print(" A | WARNING: ");
            Serial.print(lw ? "ON" : "OFF");

            Serial.print(" | RECOVERY: ");
            Serial.println(lr ? "ACTIVE" : "OFF");


            // ------------------------------------------------
            // Print RIGHT motor status.
            // ------------------------------------------------

            Serial.print("RIGHT CURRENT: ");
            Serial.print(rc, 2);

            Serial.print(" A | WARNING: ");
            Serial.print(rw ? "ON" : "OFF");

            Serial.print(" | RECOVERY: ");
            Serial.println(rr ? "ACTIVE" : "OFF");

            Serial.println("================================");


            // Reset Job B's local counter for the next report.
            jobBCount = 0;
        }


        // ----------------------------------------------------
        // STEP 4: Maintain the 20 Hz schedule.
        //
        // Job B waits until its next 50 ms release time.
        // ----------------------------------------------------

        vTaskDelayUntil(
            &lastWake,
            pdMS_TO_TICKS(JOB_B_PERIOD_MS)
        );
    }
}


// ============================================================
// 12. SETUP
//
// setup() runs once when the ESP32 starts.
//
// It:
//   - Starts Serial communication.
//   - Configures ADC resolution.
//   - Configures all motor, LED and buzzer pins.
//   - Sets the A4988 drivers to enabled.
//   - Creates the two FreeRTOS tasks.
// ============================================================

void setup() {

    // Start Serial communication at 115200 baud.
    Serial.begin(115200);


    // ESP32 ADC resolution: 12 bits (0 to 4095).
    analogReadResolution(12);


    // --------------------------------------------------------
    // Configure LEFT motor pins as outputs.
    // --------------------------------------------------------

    pinMode(LEFT_STEP, OUTPUT);
    pinMode(LEFT_DIR, OUTPUT);
    pinMode(LEFT_EN, OUTPUT);


    // --------------------------------------------------------
    // Configure RIGHT motor pins as outputs.
    // --------------------------------------------------------

    pinMode(RIGHT_STEP, OUTPUT);
    pinMode(RIGHT_DIR, OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);


    // --------------------------------------------------------
    // Configure warning LEDs.
    // --------------------------------------------------------

    pinMode(LEFT_LED, OUTPUT);
    pinMode(RIGHT_LED, OUTPUT);


    // --------------------------------------------------------
    // Configure buzzer.
    // --------------------------------------------------------

    pinMode(BUZZER, OUTPUT);


    // --------------------------------------------------------
    // Set initial motor output states.
    //
    // STEP LOW: no step pulse is being generated.
    // DIR HIGH: initial direction.
    // EN LOW: A4988 driver enabled (active LOW).
    // --------------------------------------------------------

    digitalWrite(LEFT_STEP, LOW);
    digitalWrite(RIGHT_STEP, LOW);

    digitalWrite(LEFT_DIR, HIGH);
    digitalWrite(RIGHT_DIR, HIGH);

    digitalWrite(LEFT_EN, LOW);
    digitalWrite(RIGHT_EN, LOW);


    // --------------------------------------------------------
    // Initially turn off warning outputs.
    // --------------------------------------------------------

    digitalWrite(LEFT_LED, LOW);
    digitalWrite(RIGHT_LED, LOW);

    noTone(BUZZER);


    // --------------------------------------------------------
    // Print startup information.
    // --------------------------------------------------------

    Serial.println();
    Serial.println("================================");
    Serial.println(" TASK 4 - SANDSTORM TRAVERSE");
    Serial.println(" MOTOR SPEED & ANTI-STALL SYSTEM");
    Serial.println("================================");

    Serial.println("Job A: Motor Control - 100 Hz");
    Serial.println("Job B: Communication - 20 Hz");

    Serial.println("Enter command: v,w,width");


    // --------------------------------------------------------
    // CREATE JOB A.
    //
    // Job A controls both motors and has priority 2.
    // A higher priority allows motor control to take precedence
    // over the lower-priority communication task.
    // --------------------------------------------------------

    xTaskCreate(
        motorControlTask,       // Function executed by Job A
        "Job_A_Motor_Control",  // Task name
        4096,                   // Stack size
        NULL,                   // No task parameter
        2,                      // Priority
        NULL                    // Task handle not required
    );


    // --------------------------------------------------------
    // CREATE JOB B.
    //
    // Job B handles Serial commands and telemetry.
    // It has priority 1, lower than Job A.
    // --------------------------------------------------------

    xTaskCreate(
        communicationTask,       // Function executed by Job B
        "Job_B_Communication",  // Task name
        4096,                    // Stack size
        NULL,                    // No task parameter
        1,                       // Priority
        NULL                     // Task handle not required
    );
}


// ============================================================
// 13. MAIN LOOP
//
// The Arduino loop() is not used for motor control or
// communication because those jobs are handled by FreeRTOS.
//
// vTaskDelay(portMAX_DELAY) blocks the Arduino loop task
// indefinitely, allowing the two FreeRTOS tasks to do the work.
// ============================================================

void loop() {

    vTaskDelay(portMAX_DELAY);
}
