#include <Arduino.h>
#include <OneButton.h>

// The ESP32 has 16 channels which can generate 16 independent waveforms
// We'll just choose PWM channel 0 here
const int TONE_PWM_CHANNEL = 0;

class Button{
private:
  OneButton button;
  int value;
public:
  explicit Button(uint8_t pin):button(pin) {
    button.attachClick([](void *scope) { ((Button *) scope)->Clicked();}, this);
//    button.attachDoubleClick([](void *scope) { ((Button *) scope)->DoubleClicked();}, this);
    button.attachLongPressStart([](void *scope) { ((Button *) scope)->LongPressed();}, this);
  }

  void Clicked(){
    ledcWrite(0, 0);
    Serial.println("Click then value++");
    value++;
  }

//  void DoubleClicked(){
//
//    Serial.println("DoubleClick");
//  }

  void LongPressed(){
    ledcWriteNote(TONE_PWM_CHANNEL, NOTE_C, 4);
    Serial.println("LongPress and the value is");
    Serial.println(value);
  }

  void handle(){
    button.tick();
  }
};

Button button(12);
#define BUZZER_PIN 13 // ESP32 GIOP13 pin connected to buzzer's pin

void setup() {
  Serial.begin(115200);
  ledcAttachPin(BUZZER_PIN, TONE_PWM_CHANNEL);
}

void loop() {
  button.handle();
}
