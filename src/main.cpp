#include <Arduino.h>
#include <driver/ledc.h>
#include <U8g2lib.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// Configurazione invariata
const char *ssid = "VodafoneRibes";
const char *password = "scheggia2000";
#define PWM_PIN 13
#define PWM_CHANNEL 0
int32_t frequenza = 134200;
int statoconta = 0;
int statoacq = 0;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 15, 4, 16);
#define pblack 12
#define pred 14
#define pyellow 27
#define pblue 22
#define ledverde 21

// Configurazione ADC e I2S
#define BUFFER_SIZE 10000  // Torniamo a 10000
#define CIRCULAR_SIZE 256  // Buffer circolari da 256
#define MAX_PEAKS 1000
uint16_t adc_buffer[BUFFER_SIZE];
int32_t segnale_filtrato32[CIRCULAR_SIZE];
int32_t correlazione32[CIRCULAR_SIZE];
int32_t picchi32[MAX_PEAKS];
int32_t distanze32[MAX_PEAKS];
struct Bit { int value; int32_t pos; };
Bit bits32[MAX_PEAKS];
uint8_t bytes32[10];
int32_t num_picchi = 0, num_distanze = 0, num_bits = 0;
uint16_t country_code = 0;
uint64_t device_code = 0;
bool crc_ok = false;

#define ADC_CHANNEL ADC1_CHANNEL_3
uint16_t first_adc = 0;

// Funzione di analisi con indici mascherati
void media_correlazione_32(uint16_t* segnale, int32_t* filt, int32_t* corr, int32_t* peaks, int32_t* dists, Bit* bits, uint8_t* bytes,
                          int32_t& n_peaks, int32_t& n_dists, int32_t& n_bits, uint16_t& country, uint64_t& device, bool& crc_valid) {
    const int N = BUFFER_SIZE;
    const int larghezza_finestra = 8;
    const int lunghezza_correlazione = 32;
    const int soglia_mezzo_bit = 24;
    int32_t stato_decodifica = 0, contatore_zeri = 0, contatore_bytes = 0, contatore_bits = 0, stato_decobytes = 0;
    int32_t ultima_distanza = 0, newbit = 0, numbit = 0;
    bool newpeak = false;

    n_peaks = n_dists = n_bits = 0;
    for (int i = 0; i < 10; i++) bytes[i] = 0;
    for (int i = 0; i < CIRCULAR_SIZE; i++) filt[i] = corr[i] = 0;

    int i = 32;
    if (i < N && i + 3 < N) {
        int32_t somma_media = 0;
        for (int j = i - 4; j < i + 4; j++) somma_media += segnale[j];
        filt[i & (CIRCULAR_SIZE - 1)] = somma_media / larghezza_finestra;
    }
    if (i >= lunghezza_correlazione) {
        corr[16 & (CIRCULAR_SIZE - 1)] = 0;
        for (int j = 0; j < 16; j++) corr[16 & (CIRCULAR_SIZE - 1)] += filt[j & (CIRCULAR_SIZE - 1)];
        for (int j = 16; j < 32; j++) corr[16 & (CIRCULAR_SIZE - 1)] -= filt[j & (CIRCULAR_SIZE - 1)];
    }

    int32_t max_i = corr[16 & (CIRCULAR_SIZE - 1)], min_i = corr[16 & (CIRCULAR_SIZE - 1)];
    int32_t max_i8 = corr[8 & (CIRCULAR_SIZE - 1)], min_i8 = corr[8 & (CIRCULAR_SIZE - 1)];
    int32_t stato = 1;

    for (i = 33; i < N - 4; i++) {
        filt[i & (CIRCULAR_SIZE - 1)] = filt[(i-1) & (CIRCULAR_SIZE - 1)] - (segnale[i-4] / larghezza_finestra) + (segnale[i+3] / larghezza_finestra);
        corr[(i-16) & (CIRCULAR_SIZE - 1)] = corr[(i-17) & (CIRCULAR_SIZE - 1)] - filt[(i-32) & (CIRCULAR_SIZE - 1)] + 2 * filt[(i-16) & (CIRCULAR_SIZE - 1)] - filt[i & (CIRCULAR_SIZE - 1)];

        newbit = 2; numbit = 0; newpeak = false;

        if (stato == 1) {
            max_i = max(corr[(i-16) & (CIRCULAR_SIZE - 1)], max_i);
            max_i8 = max(corr[(i-24) & (CIRCULAR_SIZE - 1)], max_i8);
            if (max_i == max_i8 && n_peaks < MAX_PEAKS) {
                peaks[n_peaks++] = i - 24;
                stato = -1;
                min_i = corr[(i-16) & (CIRCULAR_SIZE - 1)];
                min_i8 = corr[(i-24) & (CIRCULAR_SIZE - 1)];
                newpeak = true;
            }
        } else {
            min_i = min(corr[(i-16) & (CIRCULAR_SIZE - 1)], min_i);
            min_i8 = min(corr[(i-24) & (CIRCULAR_SIZE - 1)], min_i8);
            if (min_i == min_i8 && n_peaks < MAX_PEAKS) {
                peaks[n_peaks++] = i - 24;
                stato = 1;
                max_i = corr[(i-16) & (CIRCULAR_SIZE - 1)];
                max_i8 = corr[(i-24) & (CIRCULAR_SIZE - 1)];
                newpeak = true;
            }
        }

        if (n_peaks > 1 && newpeak) {
            int32_t nuova_distanza = peaks[n_peaks-1] - peaks[n_peaks-2];
            if (n_dists < MAX_PEAKS) dists[n_dists++] = nuova_distanza;

            if (stato_decodifica == 0) {
                if (nuova_distanza >= soglia_mezzo_bit) {
                    if (n_bits < MAX_PEAKS) { bits[n_bits].value = 1; bits[n_bits++].pos = i - 24; }
                    newbit = 1; numbit = 1;
                } else {
                    ultima_distanza = nuova_distanza;
                    stato_decodifica = 1;
                }
            } else {
                if (nuova_distanza < soglia_mezzo_bit) {
                    if (n_bits < MAX_PEAKS) { bits[n_bits].value = 0; bits[n_bits++].pos = i - 24; }
                    newbit = 0; numbit = 1;
                } else {
                    if (n_bits + 1 < MAX_PEAKS) {
                        bits[n_bits].value = 1; bits[n_bits++].pos = i - 24 - nuova_distanza;
                        bits[n_bits].value = 1; bits[n_bits++].pos = i - 24;
                    }
                    newbit = 1; numbit = 2;
                }
                stato_decodifica = 0;
            }
        }

        while (numbit > 0) {
            switch (stato_decobytes) {
                case 0:
                    if (newbit == 0) contatore_zeri++;
                    else if (contatore_zeri >= 10) {
                        stato_decobytes = 1;
                        contatore_bytes = contatore_bits = 0;
                        for (int j = 0; j < 10; j++) bytes[j] = 0;
                        Serial.printf("Sequenza sync at: %d\n", i);
                    } else contatore_zeri = 0;
                    break;

                case 1:
                    if (contatore_bits < 8) {
                        bytes[contatore_bytes] >>= 1;
                        if (newbit == 1) bytes[contatore_bytes] |= 0x80;
                        contatore_bits++;
                    } else if (newbit == 1) {
                        contatore_bytes++;
                        contatore_bits = 0;
                        if (contatore_bytes >= 10) {
                            Serial.print("Byte estratti: [");
                            for (int j = 0; j < 10; j++) Serial.printf("%02X%s", bytes[j], j < 9 ? ", " : "]\n");
                            contatore_zeri = contatore_bytes = 0;
                            stato_decobytes = 0;

                            uint16_t crc = 0x0;
                            const uint16_t polynomial = 0x1021;
                            for (int b = 0; b < 10; b++) {
                                uint8_t byte = bytes[b];
                                for (int j = 0; j < 8; j++) {
                                    bool bit = ((byte >> j) & 1) == 1;
                                    bool c15 = ((crc >> 15) & 1) == 1;
                                    crc <<= 1;
                                    if (c15 ^ bit) crc ^= polynomial;
                                    crc &= 0xffff;
                                }
                            }
                            crc_valid = (crc == 0);
                            Serial.printf("CRC: %s\n", crc_valid ? "OK" : "KO");
                            if (crc_valid) {
                                country = (bytes[5] << 2) | (bytes[4] >> 6);
                                device = ((uint64_t)(bytes[4] & 0x3F) << 32) | ((uint64_t)bytes[3] << 24) |
                                         ((uint64_t)bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
                                Serial.printf("Country Code: %u\n", country);
                                Serial.printf("Device Code: %llu\n", device);
                            }
                        }
                    } else {
                        Serial.printf("Perso sync at: %d\n", i);
                        contatore_zeri = contatore_bits = contatore_bytes = 0;
                        stato_decobytes = 0;
                    }
                    break;
            }
            numbit--;
        }
    }
}

void u8g2_prepare(void) { /* invariato */ }

void u8g2_prova() { /* invariato */ }

void draw(void) { /* invariato */ }

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.println("Client WebSocket connesso");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("Client WebSocket disconnesso");
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            if (strcmp((char*)data, "get_buffer") == 0) {
                i2s_start(I2S_NUM_0);
                size_t bytes_read;
                i2s_read(I2S_NUM_0, adc_buffer, BUFFER_SIZE * 2, &bytes_read, 80 / portTICK_PERIOD_MS);
                i2s_stop(I2S_NUM_0);
                if (bytes_read == BUFFER_SIZE * 2) {
                    first_adc = adc_buffer[0] >> 4;
                    client->binary((uint8_t*)adc_buffer, BUFFER_SIZE * 2);
                    Serial.println("Buffer inviato al client");

                    media_correlazione_32(adc_buffer, segnale_filtrato32, correlazione32, picchi32, distanze32, bits32, bytes32,
                                         num_picchi, num_distanze, num_bits, country_code, device_code, crc_ok);
                }
            }
        }
    }
}

void setup(void) {
    pinMode(pblack, INPUT_PULLUP);
    pinMode(pred, INPUT_PULLUP);
    pinMode(pyellow, INPUT_PULLUP);
    pinMode(pblue, INPUT_PULLUP);
    pinMode(ledverde, OUTPUT);
    Serial.begin(115200);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnesso al Wi-Fi");
    Serial.println(WiFi.localIP());

    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    server.begin();

    ledcSetup(PWM_CHANNEL, frequenza, 4);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 8);
    u8g2.begin();

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = 134200,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
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

    if (statoacq == 1) {
        i2s_start(I2S_NUM_0);
        size_t bytes_read;
        i2s_read(I2S_NUM_0, adc_buffer, BUFFER_SIZE * 2, &bytes_read, 80 / portTICK_PERIOD_MS);
        i2s_stop(I2S_NUM_0);
        Serial.print("ADC Value: ");
        Serial.println(adc_buffer[0]);
        if (bytes_read == BUFFER_SIZE * 2) {
            first_adc = adc_buffer[0] >> 4;
        }
    }

    u8g2.clearBuffer();
    draw();
    u8g2.sendBuffer();

    delay(100);
}