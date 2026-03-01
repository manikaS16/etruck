#include <Servo.h>

// ********************* Pin assignments ********************* //
const int MOTOR_PIN       = A0; // Wheels
const int DUMP_PIN        = A1; // Crane
const int BUCKET_PIN      = A2; // Bucket 
const int STEER_PIN       = A3; // Steer

const int STEER_LEFT_SWITCH  = 5; 
const int STEER_RIGHT_SWITCH = 7;
const int CRANE_LOW_SWITCH  = 4; // Crane lowest
const int CRANE_HIGH_SWITCH = 6; // Crane highest

// ********************* PWM Limits ********************* //
const int ESC_MIN     = 1000; // Full reverse
const int ESC_NEUTRAL = 1500; // Neutral
const int ESC_MAX     = 2000; // Full forward

const int DUMP_STOP   = 1500; // Neutral
const int DUMP_UP     = 1200; // Moves up
const int DUMP_DOWN   = 1800; // Moves down

const int BUCKET_STOP = 1500; // Neutral
const int BUCKET_UP   = 1300; // Moves up
const int BUCKET_DOWN = 1700; // Moves down

const int STEER_STOP  = 1500; // Neutral
const int STEER_LEFT  = 1000; // Turn Left
const int STEER_RIGHT = 2000; // Turn Right

const int STEER_ANGLE_LEFT   = 0;    // Full left
const int STEER_ANGLE_CENTER = 90;   // Center
const int STEER_ANGLE_RIGHT  = 180;  // Full right

// ---------- Steering parameters ----------
const int STEER_CENTER_ANGLE = 90;  // neutral
const int STEER_MAX_DEG = 30;       // ±30 degrees steering range

const unsigned long STEER_FULL_TRAVEL = 2100; // ms (your value) 
const unsigned long STEER_CENTER_TIME = STEER_FULL_TRAVEL / 2;

bool steeringCentered = false;

// ********************* States ********************* //

enum DumperState { DUMPER_STOP, DUMPER_UP, DUMPER_DOWN };
DumperState dumperState = DUMPER_STOP;

enum BucketState { BUCKET_STOP_CMD, BUCKET_UP_CMD, BUCKET_DOWN_CMD };
BucketState bucketState = BUCKET_STOP_CMD;

enum SteerState { STEER_STOP_CMD, STEER_LEFT_CMD, STEER_RIGHT_CMD };
SteerState steerState = STEER_STOP_CMD;

// ********************* Servo objects ********************* //
Servo motor, dump, bucket, steer;

// Optional failsafe
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 2000; // 2 seconds

float steerCommand = 0.0; // -1 left, 0 center, 1 right
        
// ********************* Setup ********************* //
void setup() {
    Serial.begin(115200);
    motor.attach(MOTOR_PIN);
    dump.attach(DUMP_PIN);
    bucket.attach(BUCKET_PIN);
    steer.attach(STEER_PIN);

    stopAll();

    pinMode(CRANE_LOW_SWITCH, INPUT_PULLUP);
    pinMode(CRANE_HIGH_SWITCH, INPUT_PULLUP);
    pinMode(STEER_LEFT_SWITCH, INPUT_PULLUP);
    pinMode(STEER_RIGHT_SWITCH, INPUT_PULLUP);

    Serial.println("Motor, Dump control ready.");
}

// ********************* Main Loop ********************* //
void loop() {
    // Read serial input
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim(); // Remove whitespace

        // Expecting format: "T:-1,D:1,B:-1"
        int tIndex = line.indexOf("T:");
        int dIndex = line.indexOf("D:");
        int bIndex = line.indexOf("B:");
        int sIndex = line.indexOf("S:");

        // --- Handle throttle (motor) ---
        if (tIndex != -1) {
            String tStr = line.substring(tIndex + 2, line.indexOf(',', tIndex) != -1 ? line.indexOf(',', tIndex) : line.length());
            float throttleVal = tStr.toFloat();  // -1.0 to 1.0

            // Constrain input
            throttleVal = constrain(throttleVal, -1.0, 1.0);

            // Map -1 -> ESC_MIN, 0 -> ESC_NEUTRAL, 1 -> ESC_MAX
            int escPWM = ESC_NEUTRAL; // 1000 to 1500
            
            if (throttleVal < 0) {
                // negative: scale from ESC_MIN → ESC_NEUTRAL
                escPWM = ESC_NEUTRAL + throttleVal * (ESC_NEUTRAL - ESC_MIN);
            } else if (throttleVal > 0) {
                // positive: scale from ESC_NEUTRAL → ESC_MAX
                escPWM = ESC_NEUTRAL + throttleVal * (ESC_MAX - ESC_NEUTRAL);
            } else {
                // zero: neutral
                escPWM = ESC_NEUTRAL;
            }

            motor.writeMicroseconds(escPWM);
            Serial.println("Throttle");
        }

        // --- Handle dumper (lift) ---
        if (dIndex != -1) {
            int dumperVal = line.substring(dIndex + 2).toInt();
            if (dumperVal == 1) dumperState = DUMPER_UP;
            else if (dumperVal == -1) dumperState = DUMPER_DOWN;
            else dumperState = DUMPER_STOP;

            Serial.println("Dumper");
        }

        // --- Handle bucket ---
        if (bIndex != -1) {
            int bucketVal = line.substring(bIndex + 2).toInt();
            if (bucketVal == 1) bucketState = BUCKET_UP_CMD;
            else if (bucketVal == -1) bucketState = BUCKET_DOWN_CMD;
            else bucketState = BUCKET_STOP_CMD;

            Serial.println("Bucket");
        }

        if (sIndex != -1) { 
          String sStr = line.substring(sIndex + 2, line.indexOf(',', sIndex) != -1 ? line.indexOf(',', sIndex) : line.length()); steerCommand = sStr.toFloat(); 
          steerCommand = constrain(steerCommand, -1.0, 1.0); 
          Serial.println("Steer");
        }

        lastCommandTime = millis();
    }

    // --- Execute dumper movement ---
    switch (dumperState) {
        case DUMPER_UP:
            while (digitalRead(CRANE_HIGH_SWITCH) == HIGH) {
                dump.writeMicroseconds(DUMP_UP);
                delay(20);
            }
            dump.writeMicroseconds(DUMP_STOP);
            dumperState = DUMPER_STOP;
            break;

        case DUMPER_DOWN:
            while (digitalRead(CRANE_LOW_SWITCH) == HIGH) {
                dump.writeMicroseconds(DUMP_DOWN);
                delay(20);
            }
            dump.writeMicroseconds(DUMP_STOP);
            dumperState = DUMPER_STOP;
            break;

        default:
            dump.writeMicroseconds(DUMP_STOP);
            dumperState = DUMPER_STOP;
            break;
    }

    // --- Execute bucket movement ---
    switch (bucketState) {
        case BUCKET_UP_CMD:
            bucket.writeMicroseconds(BUCKET_UP);
            delay(2000); // safe movement duration
            bucket.writeMicroseconds(BUCKET_STOP);
            bucketState = BUCKET_STOP_CMD;
            break;

        case BUCKET_DOWN_CMD:
            bucket.writeMicroseconds(BUCKET_DOWN);
            delay(2000); // safe movement duration
            bucket.writeMicroseconds(BUCKET_STOP);
            bucketState = BUCKET_STOP_CMD;
            break;

        default:
            bucket.writeMicroseconds(BUCKET_STOP);
            bucketState = BUCKET_STOP_CMD;
            break;
    }

    driveSteerCommand(steerCommand);
    delay(20); // small loop delay
}

// ********************* Helper Functions ********************* //
void stopAll() {
    motor.writeMicroseconds(ESC_NEUTRAL);
    dump.writeMicroseconds(DUMP_STOP);
    bucket.writeMicroseconds(BUCKET_STOP);
}

void driveSteerCommand(float cmd) {
    cmd = constrain(cmd, -1.0, 1.0);

    // Map -1..1 → 1000..2000 with 0 = 1500
    int pwm = map(cmd * 1000, -1000, 1000, STEER_LEFT, STEER_RIGHT);

    // Safety stops using limit switches
    if (cmd < 0 && digitalRead(STEER_LEFT_SWITCH) == HIGH) {
        // allowed to move left
        steer.writeMicroseconds(pwm);
    } 
    else if (cmd > 0 && digitalRead(STEER_RIGHT_SWITCH) == HIGH) {
        // allowed to move right
        steer.writeMicroseconds(pwm);
    }
    else if (cmd == 0.0) {
        // If LEFT limit is triggered, move right to center
        if (digitalRead(STEER_LEFT_SWITCH) == LOW) {
            steer.writeMicroseconds(STEER_RIGHT);
            delay(STEER_CENTER_TIME);
        }
        // If RIGHT limit is triggered, move left to center
        else if (digitalRead(STEER_RIGHT_SWITCH) == LOW) {
            steer.writeMicroseconds(STEER_LEFT);
            delay(STEER_CENTER_TIME);
        }
        // Stop at center
        steer.writeMicroseconds(STEER_STOP);
        return;
    }
    else {
        // Hit limit, stop
        steer.writeMicroseconds(STEER_STOP);
    }
}
