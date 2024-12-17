#include <driver/i2s.h>
#include "sos-iir-filter.h"

// 설정 부분
#define LEQ_PERIOD        1           // Leq 측정 주기 (1초 단위)
#define WEIGHTING         C_weighting // 사용되는 가중치 필터 (C-가중치 또는 Z-가중치)
#define LEQ_UNITS         "LAeq"      // 측정 단위 (가중치에 따라 설정)
#define DB_UNITS          "dBA"       // dB 단위

#define MIC_EQUALIZER     INMP441     // 마이크 이퀄라이저 필터 설정
#define MIC_OFFSET_DB     3.0103      // 오프셋 보정 값 (사인파 RMS 기준값)

// 마이크 감도 설정 (데이터시트 기반)
#define MIC_SENSITIVITY   -26         // 마이크 감도 (dBFS)
#define MIC_REF_DB        94.0        // 기준 데시벨 (데이터시트 감도 기준)
#define MIC_OVERLOAD_DB   116.0       // 마이크의 음향 과부하 지점
#define MIC_NOISE_DB      29          // 노이즈 플로어 (데시벨)
#define MIC_BITS          24          // 유효 비트 수 (24비트 마이크 데이터 사용)
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS)) // 샘플 변환
#define MIC_TIMING_SHIFT  0           // 일부 마이크의 타이밍 문제 보정

// 참조 진폭 계산
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

// I2S 핀 정의
#define I2S_WS            42  // 워드 셀렉트 핀
#define I2S_SCK           41  // 비트 클럭 핀
#define I2S_SD            38  // 데이터 입력 핀

#define I2S_PORT          I2S_NUM_0   // 사용할 I2S 포트

// OLED 디스플레이 설정
#include <SSD1306Wire.h>
#define OLED_GEOMETRY GEOMETRY_128_64 // OLED 해상도 설정
#define OLED_FLIP_V       1           // OLED 화면 뒤집기 설정
SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY); // 디스플레이 객체 초기화

// 마이크 이퀄라이저 필터 설정 (INMP441 마이크에 맞춤)
SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696,
  sos: {
    {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
};

// 가중치 필터 (C-가중치 필터 설정)
SOS_IIR_Filter C_weighting = {
  gain: -0.491647169337140,
  sos: { 
    {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
    {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
    {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
  }
};

// I2S 초기화 함수
void mic_i2s_init() {
  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // 마스터 및 RX 모드
    sample_rate: SAMPLE_RATE,                       // 샘플레이트: 48kHz
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS), // 비트 수: 32비트
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,      // 왼쪽 채널만 사용
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,         // 인터럽트 플래그
    dma_buf_count: DMA_BANKS,                       // DMA 버퍼 개수
    dma_buf_len: DMA_BANK_SIZE,                     // 각 DMA 버퍼 길이
    use_apll: true,                                 // APLL 사용
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  const i2s_pin_config_t pin_config = {
    bck_io_num: I2S_SCK,  // 비트 클럭 핀
    ws_io_num: I2S_WS,    // 워드 셀렉트 핀
    data_out_num: -1,     // 출력 없음
    data_in_num: I2S_SD   // 데이터 입력 핀
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

// I2S 리더 작업 함수
void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init(); // I2S 초기화
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;

    for (int i = 0; i < SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

void setup() {
  setCpuFrequencyMhz(80); // CPU 주파수를 낮춰 전력 소모를 줄임
  Serial.begin(115200);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);

  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);
}

void loop() {}
