// ============================================================
//                 ROLLING MOTION LAB
//              Arduino UNO Controller
//
//   Hardware:
//      Arduino UNO
//      IR Sensor A -> D2
//      IR Sensor B -> D3
//
//   Purpose:
//      Measure time for a table-tennis ball to travel
//      from Sensor A to Sensor B.
//
//   Sensor spacing = 15 cm
// ============================================================


const byte SENSOR_A_PIN = 2;
const byte SENSOR_B_PIN = 3;


// ------------------------------------------------------------
// Experiment settings
// ------------------------------------------------------------

const float SENSOR_DISTANCE_M = 0.150;

const int MAX_VALID_TRIALS = 10;


// ------------------------------------------------------------
// IR sensor logic
// Most common IR obstacle modules:
// LOW = object detected
// ------------------------------------------------------------

const byte DETECTED = LOW;


// ------------------------------------------------------------
// Timing protection
// ------------------------------------------------------------

// Reject extremely short readings.
// 15 cm / 0.030 s = 5 m/s
const unsigned long MIN_TRAVEL_US = 30000;

// Reject readings taking longer than 5 seconds.
const unsigned long MAX_TRAVEL_US = 5000000;

// Ignore repeated triggering for 10 ms.
const unsigned long DEBOUNCE_US = 10000;


// ------------------------------------------------------------
// State machine
// ------------------------------------------------------------

bool waitingForA = true;
bool waitingForB = false;

bool previousA;
bool previousB;

unsigned long timeA = 0;

unsigned long lastAEvent = 0;
unsigned long lastBEvent = 0;

int trialNumber = 0;


// ------------------------------------------------------------
// Sensor heartbeat
// ------------------------------------------------------------

unsigned long lastHeartbeat = 0;

const unsigned long HEARTBEAT_INTERVAL_MS = 100;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  pinMode(SENSOR_A_PIN, INPUT);
  pinMode(SENSOR_B_PIN, INPUT);

  delay(500);

  previousA = digitalRead(SENSOR_A_PIN);
  previousB = digitalRead(SENSOR_B_PIN);

  Serial.println("SYSTEM,START");

  Serial.print("CONFIG,DISTANCE_M,");
  Serial.println(SENSOR_DISTANCE_M, 3);

  Serial.print("CONFIG,MAX_TRIALS,");
  Serial.println(MAX_VALID_TRIALS);

  Serial.println("READY");
  Serial.println("WAITING_FOR_A");
}


// ============================================================
// RESET EXPERIMENT
// ============================================================

void resetExperiment() {

  trialNumber = 0;

  waitingForA = true;
  waitingForB = false;

  timeA = 0;

  Serial.println("RESET");
  Serial.println("READY");
  Serial.println("WAITING_FOR_A");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  bool currentA = digitalRead(SENSOR_A_PIN);
  bool currentB = digitalRead(SENSOR_B_PIN);


  // ==========================================================
  // RECEIVE COMMANDS FROM PYTHON
  // ==========================================================

  if (Serial.available()) {

    String command = Serial.readStringUntil('\n');

    command.trim();

    if (command == "RESET") {

      resetExperiment();
    }
  }


  // ==========================================================
  // REAL-TIME SENSOR STATUS
  // ==========================================================

  if (
    millis() - lastHeartbeat
    >= HEARTBEAT_INTERVAL_MS
  ) {

    Serial.print("STATE,A,");
    Serial.print(currentA == DETECTED ? 1 : 0);

    Serial.print(",B,");
    Serial.println(currentB == DETECTED ? 1 : 0);

    lastHeartbeat = millis();
  }


  // ==========================================================
  // SENSOR A DETECTION
  // ==========================================================

  bool newAEvent =
    previousA != DETECTED &&
    currentA == DETECTED;


  if (
    waitingForA &&
    newAEvent
  ) {

    unsigned long now = micros();


    // --------------------------------------------------------
    // Debounce
    // --------------------------------------------------------

    if (
      now - lastAEvent
      >= DEBOUNCE_US
    ) {

      // ------------------------------------------------------
      // Crosstalk check
      //
      // If B is detecting at exactly the same time as A,
      // the two sensors might be seeing the same reflection.
      // ------------------------------------------------------

      if (currentB == DETECTED) {

        Serial.println(
          "WARNING,CROSSTALK_A_B"
        );

      } else {

        timeA = now;

        lastAEvent = now;

        trialNumber++;

        waitingForA = false;
        waitingForB = true;


        // ----------------------------------------------------
        // Send A event
        // ----------------------------------------------------

        Serial.print("A,");

        Serial.print(trialNumber);

        Serial.print(",");

        Serial.println(timeA);


        Serial.print("TRIAL_START,");

        Serial.println(trialNumber);
      }
    }
  }


  // ==========================================================
  // SENSOR B DETECTION
  // ==========================================================

  bool newBEvent =
    previousB != DETECTED &&
    currentB == DETECTED;


  if (
    waitingForB &&
    newBEvent
  ) {

    unsigned long now = micros();


    if (
      now - lastBEvent
      >= DEBOUNCE_US
    ) {

      lastBEvent = now;


      unsigned long elapsedUs =
        now - timeA;


      // ======================================================
      // INVALID TOO-FAST READING
      // ======================================================

      if (
        elapsedUs < MIN_TRAVEL_US
      ) {

        Serial.print("REJECT,TOO_FAST,");

        Serial.println(elapsedUs);

        waitingForB = false;
        waitingForA = true;
      }


      // ======================================================
      // INVALID TIMEOUT
      // ======================================================

      else if (
        elapsedUs > MAX_TRAVEL_US
      ) {

        Serial.print("REJECT,TIMEOUT,");

        Serial.println(elapsedUs);

        waitingForB = false;
        waitingForA = true;
      }


      // ======================================================
      // VALID READING
      // ======================================================

      else {

        float timeSeconds =
          elapsedUs / 1000000.0;


        float averageSpeed =
          SENSOR_DISTANCE_M /
          timeSeconds;


        // ----------------------------------------------------
        // Raw B event
        // ----------------------------------------------------

        Serial.print("B,");

        Serial.print(trialNumber);

        Serial.print(",");

        Serial.print(now);

        Serial.print(",");

        Serial.print(elapsedUs);

        Serial.print(",");

        Serial.print(timeSeconds, 6);

        Serial.print(",");

        Serial.println(averageSpeed, 6);


        // ----------------------------------------------------
        // Main result
        // ----------------------------------------------------

        Serial.print("RESULT,");

        Serial.print(trialNumber);

        Serial.print(",");

        Serial.print(timeSeconds, 6);

        Serial.print(",");

        Serial.println(averageSpeed, 6);


        Serial.print("TRIAL_COMPLETE,");

        Serial.println(trialNumber);


        waitingForB = false;


        if (
          trialNumber >= MAX_VALID_TRIALS
        ) {

          Serial.println(
            "EXPERIMENT_COMPLETE"
          );

        } else {

          waitingForA = true;

          Serial.println(
            "WAITING_FOR_A"
          );
        }
      }
    }
  }


  // ==========================================================
  // SAVE PREVIOUS STATES
  // ==========================================================

  previousA = currentA;

  previousB = currentB;
}