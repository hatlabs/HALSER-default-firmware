#include <Arduino.h>

#include "halser_const.h"
#include "nmea_gateway.h"
#include "test_mode.h"

void setup() {
  // Check if test jig is present (GPIO 0 pulled high by RPi)
  pinMode(kTestJigPin, INPUT_PULLDOWN);
  delay(10);
  bool test_jig_present = (digitalRead(kTestJigPin) == HIGH);
  pinMode(kTestJigPin, INPUT);

  if (test_jig_present) {
    run_test_mode();  // Does not return
  } else {
    run_nmea_gateway();  // Does not return
  }
}

void loop() {
  // Not reached — both modes run their own event loops
}
