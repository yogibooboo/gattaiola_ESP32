#include <Arduino.h>
#include <driver/ledc.h>
#include <U8g2lib.h>

#define PWM_PIN 13
#define PWM_CHANNEL 0

int32_t frequenza = 134200;
int statoconta = 0;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 15, 4, 16);

#define pblack 12
#define pred 14
#define pyellow 27
#define ledverde 21

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void u8g2_prova() {
  char buffer[20]; // Buffer per la frequenza
  itoa(frequenza, buffer, 10);
  u8g2.drawStr(0, 0, "Freq:");
  u8g2.drawStr(50, 0, buffer);

  if (statoconta == 1)
    u8g2.drawStr(0, 40, "COUNT");
  else
    u8g2.drawStr(0, 40, "STOP");

  //Aggiungere qui la visualizzazione dei risultati della decodifica
}

void draw(void) {
  u8g2_prepare();
  u8g2_prova();
}

void setup(void) {
  pinMode(pblack, INPUT_PULLUP);
  pinMode(pred, INPUT_PULLUP);
  pinMode(pyellow, INPUT_PULLUP);
  pinMode(ledverde, OUTPUT);
  Serial.begin(115200);
  ledcSetup(PWM_CHANNEL, frequenza, 4);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 8);
  u8g2.begin();
}

void loop(void) {
  int sblack = digitalRead(pblack);
  int sred = digitalRead(pred);
  int syel = digitalRead(pyellow);

  if (syel == 0) statoconta ^= 1;

  digitalWrite(ledverde, syel ^ 1);

  if (statoconta == 1) {
    frequenza += 1000;
    if (frequenza > 150000) frequenza -= 49000;
  }
  if (sblack == 0) frequenza += 200;
  if (sred == 0) frequenza -= 200;
  if ((sblack == 0) && (sred == 0)) frequenza = 134200;

  ledcSetup(PWM_CHANNEL, frequenza, 4);

  u8g2.clearBuffer();
  draw();
  u8g2.sendBuffer();

  delay(100);
}