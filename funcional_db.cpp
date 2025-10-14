#include <driver/i2s.h>
#include <Arduino.h>
#include <math.h>

#define I2S_NUM           (I2S_NUM_0)
#define I2S_MIC_CHANNEL   I2S_CHANNEL_FMT_ONLY_LEFT
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_32 // SCK
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_25 // WS / LRCLK
#define I2S_MIC_SERIAL_DATA GPIO_NUM_33 // SD

#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 16000  // probá 16000 o 48000

// ---------- CONFIG DE CALIBRACION ----------
// Si no querés calibrar ahora, dejalo en 0 y vas a ver solo dBFS.
// Para obtener dB SPL: medir con calibrador (por ejemplo 94 dB) ->
// medir dBFS_reportado -> calibration_offset_db = known_dB - measured_dBFS.
const double calibration_offset_db = 104.5; // <-- reemplazá con tu valor después de calibrar
// -------------------------------------------

// I2S config
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_MIC_CHANNEL,
  .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = 256,
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

  esp_err_t err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("i2s_driver_install error: %d\n", err);
    while (1) delay(1000);
  }
  i2s_set_pin(I2S_NUM, &i2s_mic_pins);
  i2s_set_clk(I2S_NUM, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  i2s_zero_dma_buffer(I2S_NUM);
  Serial.println("I2S listo.");
  Serial.println("Formato salida: RMS | dBFS | dB_SPL (si calibrado)");
}

void loop() {
  size_t bytes_read = 0;
  esp_err_t res = i2s_read(I2S_NUM, raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);
  if (res != ESP_OK) {
    Serial.printf("i2s_read err %d\n", res);
    return;
  }
  int samples_read = bytes_read / sizeof(int32_t);

  // acumulador para RMS
  uint64_t acc_sq = 0;
  int32_t max_abs_sample = 0;
  for (int i = 0; i < samples_read; ++i) {
    int32_t sample32 = raw_samples[i];
    int32_t sample24 = sample32 >> 8; // conservar signo
    int32_t abs_s = (sample24 < 0) ? -sample24 : sample24;
    if (abs_s > max_abs_sample) max_abs_sample = abs_s;
    acc_sq += (uint64_t)((int64_t)sample24 * (int64_t)sample24);
  }

  double rms = 0.0;
  if (samples_read > 0) {
    rms = sqrt((double)acc_sq / (double)samples_read);
  }

  // MAX para 24-bit signed (2^23 - 1)
  const double full_scale_24 = 8388607.0; // 2^23 - 1

  // Evitar log(0)
  const double epsilon = 1e-12;
  double rms_clamped = (rms < epsilon) ? epsilon : rms;

  // dBFS = 20 * log10( rms / full_scale )
  double rms_norm = rms_clamped / full_scale_24;
  if (rms_norm < epsilon) rms_norm = epsilon;
  double dbfs = 20.0 * log10(rms_norm);

  // dB SPL estimado (requiere calibración previa)
  double db_spl = dbfs + calibration_offset_db; // si calibration_offset_db == 0 => no calibrado

  // Imprimir: RMS (cuentas), dBFS y dB SPL (si calibrado)
  Serial.printf("RMS=%.2f  dBFS=%.2f  dB_SPL=%.2f\n", rms, dbfs, db_spl);

  delay(100); // ajustar según necesidad
}
