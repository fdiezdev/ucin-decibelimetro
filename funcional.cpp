#include <driver/i2s.h>
#include <Arduino.h>

#define I2S_NUM           (I2S_NUM_0)
#define I2S_MIC_CHANNEL   I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_32 // SCK
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_25 // WS / LRCLK
#define I2S_MIC_SERIAL_DATA GPIO_NUM_33 // SD

#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 16000  // probá 16000 o 48000

// I2S config
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_MIC_CHANNEL,
  .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = 256, // menos largo suele ser más estable en prototipo
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA
};

int32_t raw_samples[SAMPLE_BUFFER_SIZE];

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("I2S INMP441 init...");

  // instalar driver
  esp_err_t err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("i2s_driver_install error: %d\n", err);
    while (1) delay(1000);
  }
  i2s_set_pin(I2S_NUM, &i2s_mic_pins);
  // opcional: ajustar clock explícitamente
  i2s_set_clk(I2S_NUM, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  i2s_zero_dma_buffer(I2S_NUM);
  Serial.println("I2S listo.");
}

void loop() {
  size_t bytes_read = 0;
  // leemos SAMPLE_BUFFER_SIZE palabras de 32 bits
  esp_err_t res = i2s_read(I2S_NUM, raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);
  if (res != ESP_OK) {
    Serial.printf("i2s_read err %d\n", res);
    return;
  }
  int samples_read = bytes_read / sizeof(int32_t);

  // procesamos: convertir left-aligned 32bit -> 24bit signed (desplazar 8 bits)
  // calculamos RMS para estabilizar la medida
  uint64_t acc_sq = 0;
  for (int i = 0; i < samples_read; ++i) {
    // raw_samples[i] viene como 32-bit left-aligned, los 24 MSB son datos
    int32_t sample32 = raw_samples[i];
    // Si el mic entrega MSB-aligned, hacemos shift >> 8 para obtener 24-bit en LSB
    int32_t sample24 = sample32 >> 8; // preserve sign
    acc_sq += (uint64_t)((int64_t)sample24 * (int64_t)sample24);
  }
  double rms = 0.0;
  if (samples_read > 0) {
    rms = sqrt((double)acc_sq / (double)samples_read);
  }

  // imprimimos un valor representativo: RMS y primer sample para debug
  int32_t first24 = raw_samples[0] >> 8;
  //Serial.printf("rms=%0.2f\n", samples_read, (long)first24, rms);
  Serial.println(rms);
  // pequeño delay para no saturar el serial (ajustalo según necesidad)
  delay(100);
}
