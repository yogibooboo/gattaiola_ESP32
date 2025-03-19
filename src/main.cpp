#include <Arduino.h>
#include <driver/ledc.h>
#include <U8g2lib.h>
#include <driver/i2s.h>  // Per il campionamento ADC tramite I2S

#define PWM_PIN 13
#define PWM_CHANNEL 0

int32_t frequenza = 134200;
int statoconta = 0;
int statoacq=0;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 15, 4, 16);

#define pblack 12
#define pred 14
#define pyellow 27
#define pblue 22
#define ledverde 21

// Configurazione ADC e I2S
#define BUFFER_SIZE 10000
uint16_t adc_buffer[BUFFER_SIZE];
#define ADC_CHANNEL ADC1_CHANNEL_3  // Per GPIO39, modifica se usi un altro pin
uint16_t first_adc = 0;  // Per memorizzare il primo valore ADC da visualizzare

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void u8g2_prova() {
  char buffer[20];  // Buffer per la frequenza
  itoa(frequenza, buffer, 10);
  u8g2.drawStr(0, 0, "Freq:");
  u8g2.drawStr(50, 0, buffer);

  if (statoconta == 1)
    u8g2.drawStr(0, 40, "COUNT");
  else
    u8g2.drawStr(0, 40, "STOP");

  // Visualizzazione del primo valore ADC invece del massimo
  char adc_buf[10];
  sprintf(adc_buf, "ADC: %d", first_adc);
  u8g2.drawStr(0, 20, adc_buf);
}

void draw(void) {
  u8g2_prepare();
  u8g2_prova();
}

void setup(void) {
  pinMode(pblack, INPUT_PULLUP);
  pinMode(pred, INPUT_PULLUP);
  pinMode(pyellow, INPUT_PULLUP);
  pinMode(pblue, INPUT_PULLUP);
  pinMode(ledverde, OUTPUT);
  Serial.begin(115200);
  ledcSetup(PWM_CHANNEL, frequenza, 4);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 8);
  u8g2.begin();

  // Configurazione I2S per il campionamento ADC
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),  // Master mode, receive only
      .sample_rate = 134200,                                // Frequenza di campionamento
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,         // 16-bit samples
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,          // Solo canale sinistro
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,    // Formato I2S standard
      .intr_alloc_flags = 0,                                // Nessun flag di interrupt
      .dma_buf_count = 8,                                   // Numero di buffer DMA
      .dma_buf_len = 64,                                    // Lunghezza di ogni buffer
      .use_apll = false,                                    // No APLL clock
      .tx_desc_auto_clear = false,                          // No auto-clear per TX
      .fixed_mclk = 0                                       // MCLK di default
  };

  // Installa il driver I2S
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  // Configura ADC per I2S su GPIO39
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  i2s_adc_enable(I2S_NUM_0);
}

void loop(void) {
  int sblack = digitalRead(pblack);
  int sred = digitalRead(pred);
  int syel = digitalRead(pyellow);
  int sblue = digitalRead(pblue);

  if (syel == 0) statoconta ^= 1;
  if (sblue == 0) statoacq ^= 1;

  digitalWrite(ledverde, syel ^ 1);

  if (statoconta == 1) {
    frequenza += 1000;
    if (frequenza > 150000) frequenza -= 49000;
  }
  if (sblack == 0) frequenza += 200;
  if (sred == 0) frequenza -= 200;
  if ((sblack == 0) && (sred == 0)) frequenza = 134200;

  ledcSetup(PWM_CHANNEL, frequenza, 4);

  // Campionamento ADC quando statoconta == 1
  if (statoacq == 1) {
    i2s_start(I2S_NUM_0);
    size_t bytes_read;
    i2s_read(I2S_NUM_0, adc_buffer, BUFFER_SIZE * 2, &bytes_read, 80 / portTICK_PERIOD_MS);
    i2s_stop(I2S_NUM_0);
    Serial.print("ADC Value: ");
    Serial.println(adc_buffer[0]);
    if (bytes_read == BUFFER_SIZE * 2) {
      // Usa il primo valore del buffer invece del massimo
      first_adc = adc_buffer[0] >> 4;  // Correzione per ADC a 12 bit in campioni a 16 bit
    }
  }

  u8g2.clearBuffer();
  draw();
  u8g2.sendBuffer();

  delay(100);
}