#include <driver/i2s.h>
#include "sos-iir-filter.h"

// Configuration
#define LEQ_PERIOD        1           // 초단위
#define WEIGHTING         C_weighting // 또한 사용 가능: 'C_가중치' 또는 '없음'(Z_가중치)
#define LEQ_UNITS         "LAeq"      // 사용 된 위의 가중치를 기반으로 사용자 지정
#define DB_UNITS          "dBA"       // 사용 된 위의 가중치를 기반으로 사용자 지정
#define USE_DISPLAY       1

// 참고: 일부 마이크에는 최소 DC 블로커 필터가 필요합니다.
#define MIC_EQUALIZER     INMP441    // 정의된 IIR 필터는 아래를 참조하거나 비활성화하려면 '없음'으로 설정하세요.
#define MIC_OFFSET_DB     3.0103      // 기본 오프셋(사인파 RMS 대 dBFS). 선형 보정을 위해 이 값을 수정합니다.

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY   -26         // MIC_REF_DB에서 예상되는 dBFS 값(데이터시트의 감도 값)
#define MIC_REF_DB        94.0        // 데이터시트에서 포인트 감도가 지정되는 값(dB)
#define MIC_OVERLOAD_DB   116.0       // dB - 음향 과부하 지점
#define MIC_NOISE_DB      29          // dB - 노이즈 플로어
#define MIC_BITS          24          //I2S 데이터의 유효한 비트 수
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT  0           // 1로 설정하면 일부 마이크(예: SPH0645LM4H-x)의 MSB 타이밍을 수정할 수 있습니다.

// 컴파일 시 기준 진폭 값 계산
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

// I2S 핀 - 거의 모든 (사용하지 않는) ESP32 핀으로 라우팅할 수 있습니다.
// SD는 입력 전용 핀(36-39)을 포함한 모든 핀을 사용할 수 있습니다.
// SCK(예: BCLK) 및 WS(예: L/R CLK)는 출력 가능 핀이어야 합니다.
//
// 아래는 제 보드 레이아웃의 예시일 뿐이며, 여기에 사용할 핀을 넣으세요.
#define I2S_WS            45
#define I2S_SCK           18
#define I2S_SD            1 

// 사용할 I2S 주변 장치(0 또는 1)
#define I2S_PORT          I2S_NUM_0

// 여기에서 디스플레이 라이브러리(및 지오메트리)를 설정하세요.
  #include <SSD1306Wire.h>
  #define OLED_GEOMETRY GEOMETRY_128_64
  #define OLED_FLIP_V       1
  SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY);

// IIR 필터
SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696, 
  sos: { // 두 번째 주문 섹션 {b1, b2, -a1, -a2}
    {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
};

// 가중치 필터

// C-가중치 IIR 필터, Fs = 48KHz
// 인프레크즈 커브 피팅으로 설계, 각 .m 파일 참조
// B = [-0.49164716933714026, 0.14844753846498662, 0.74117815661529129, -0.03281878334039314, -0.29709276192593875, -0.06442545322197900, -0.00364152725482682]
// A = [1.0, -1.0325358998928318, -0.9524000181023488, 0.8936404694728326   0.2256286147169398  -0.1499917107550188, 0.0156718181681081]
SOS_IIR_Filter C_weighting = {
  gain: -0.491647169337140,
  sos: { 
    {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
    {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
    {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
  }
};

// 플링
#define SAMPLE_RATE       48000 // Hz, IIR 필터 설계에 고정됨
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

//'samples_queue'로 푸시하는 데이터
struct sum_queue_t {
  // 이퀄라이저 필터 후 마이크 샘플의 제곱 합계
  float sum_sqr_SPL;
  // 가중 마이크 샘플의 제곱 합계
  float sum_sqr_weighted;
  // Debug only, FreeRTOS ticks we spent processing the I2S data
  uint32_t proc_ticks;
};
QueueHandle_t samples_queue;

// 샘플 블록을 위한 정적 버퍼
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

// I2S 마이크 샘플링 설정

void mic_i2s_init() {
  // 샘플 속도 * 샘플 비트에 대한 모노 채널을 샘플링하도록 I2S 설정
  // 참고: 최근 Arduino_esp32 업데이트(1.0.2 -> 1.0.3)
  // 에서 ONLY_LEFT 및 ONLY_RIGHT 채널이 바뀐 것 같습니다.
  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate: SAMPLE_RATE,
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
    dma_buf_count: DMA_BANKS,
    dma_buf_len: DMA_BANK_SIZE,
    use_apll: true,
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  // I2S 핀 매핑
  const i2s_pin_config_t pin_config = {
    bck_io_num:   I2S_SCK,  
    ws_io_num:    I2S_WS,    
    data_out_num: -1, // 사용안함
    data_in_num:  I2S_SD   
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  #if (MIC_TIMING_SHIFT > 0) 
    // 문서화되지 않은(?!) I2S 주변 장치 레지스터 조작
    // 일부 I2S 마이크의 MSB 타이밍 문제 수정
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  
  #endif
  
  i2s_set_pin(I2S_PORT, &pin_config);

  //FIXME: esp-idf 및 샘플링 속도에 알려진 문제가 있습니다:
  //       https://github.com/espressif/esp-idf/issues/2634
  //       한편, 아래 라인은 샘플링 속도를 ~47999.992Hz로 설정한 것으로 보입니다.
  //       fifs_req=24576000, sdm0=149, sdm1=212, sdm2=5, odir=2 -> fifs_reached=24575996  
  //NOTE:  이 문제는 ESP32 아두이노 1.0.4, esp-idf 3.2에서 수정된 것으로 보입니다.  제거해도 안전할 것입니다...
  //#include <soc/rtc.h>
  //rtc_clk_apll_enable(1, 149, 212, 5, 2);
}

//
// I2S 리더 작업
//
/*별도의 태스크 읽기 I2S에 대한 아토니널은 IIR 필터
처리 캠을 ESP32의 다른 코어로 스케줄링하는 반면 주 작업은 업데이트와 같은 다른 작업을 수행할 수 있습니다. 
 예제에서 표시

 이 작업은 별도의 우선 순위가 높은 작업으로 실행되도록 설계되었습니다, 
 I2S 데이터에 필요한 최소한의 작업만 수행합니다.
 제곱의 합으로 '압축'될 때까지 필요한 최소한의 작업만 수행합니다. */

#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048
//
void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  // 첫 번째 블록을 버리면 마이크에 시작 시간이 있을 수 있습니다(예: INMP441 최대 83ms).
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
  /* I2S에서 마이크 값을 차단하고 기다립니다.
  드라이버 ISR에 의해 데이터가 DMA 버퍼에서 '샘플' 버퍼로 이동되고 
  요청된 데이터 양이 있으면 작업이 차단 해제됩니다.
  참고: i2s_read는 float[] 버퍼에 쓰는 것을 신경쓰지 않고, 지정된 값에
  정수 값을 하드웨어 주변 장치에서 받은 대로 주어진 주소에 씁니다. */
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();
    
    /* 정수형 마이크 값을 실수로 변환(시프트 포함)합니다, 
    동일한 버퍼를 사용합니다(샘플 크기가 실수 크기와 같다고 가정),
    약간의 메모리를 절약하기 위해*/
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for(int i=0; i<SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
    // 이퀄라이제이션을 적용하고 Z-가중 제곱합을 계산합니다,  필터링된 샘플을 동일한 버퍼에 다시 씁니다.
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

    // 가중치를 적용하고 제곱의 가중 합계를 계산합니다.
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);
    // 합계를 FreeRTOS 대기열로 보내면 주 작업이 합계를 가져옵니다. 및 추가 데시벨 값 계산(나누기, 로그 등...)
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

// Setup and main loop 
//
// 참고: 여기서는 플로트가 아닌 복식을 사용하세요. 작업을 현재 실행 중인 코어에 고정하고 싶지 않다면 두 배를 사용하십시오.
void setup() {

  // 필요한 경우 이제 실제로 CPU 주파수를 낮출 수 있습니다, 즉, ESP32 전력 소비를 (약간) 줄이려는 경우
  setCpuFrequencyMhz(80); // 최저 80MHz로 실행되어야 합니다.
  
  Serial.begin(115200);
  delay(1000); // 안전
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);

  // 대기열 생성
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  
  /* I2S 리더 작업 만들기
 참고: 현재 버전의 ESP-IDF는 태스크를 고정합니다. 
 작업을 실행되는 첫 번째 코어에 자동으로 고정합니다.
 (하드웨어 FPU 명령어 사용으로 인해) 작업을 자동으로 고정합니다.
  수동 제어는 다음을 참조하세요: xTaskCreatePinnedToCore*/
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);

  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  double Leq_dB = 0;

  // 'i2s_reader_task'에 의해 계산된 샘플의 읽기 합계
  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

    // MIC_REF_AMPL을 기준으로 dB 값을 계산하고 마이크 레퍼런스에 맞게 조정합니다.
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // 음향 과부하 또는 노이즈 플로어 측정값 이하인 경우, 무한대 Leq 값을 보고합니다.
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Leq 합계 누적
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // 충분한 샘플을 수집하면 새로운 Leq 값을 계산합니다.
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;
      
      // 직렬 출력, 필요에 따라 사용자 지정(또는 제거)
      Serial.printf("%.1f\n", Leq_dB);
    }
      display.clear();
      // 데이브가 범위를 벗어났을 때 어떻게든 알려주는 것이 중요합니다. 계산된 값에 큰 오류가 있을 가능성이 매우 높기 때문입니다.
      if (Leq_dB > MIC_OVERLOAD_DB) {
        // dB 값이 AOP를 초과하면 '과부하' 표시
        display.drawString(0, 24, "Overload");
      } else if (isnan(Leq_dB) || (Leq_dB < MIC_NOISE_DB)) {
        // dB 값이 노이즈 플로어 미만인 경우 '낮음' 표시
        display.drawString(0, 24, "Low");
      }
      // '짧은' Leq 라인
      double short_Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(double(q.sum_sqr_weighted) / SAMPLES_SHORT) / MIC_REF_AMPL);
      uint16_t len = min(max(0, int(((short_Leq_dB - MIC_NOISE_DB) / MIC_OVERLOAD_DB) * (display.getWidth()-1))), display.getWidth()-1);
      display.drawHorizontalLine(0, 0, len);
      display.drawHorizontalLine(0, 1, len);
      display.drawHorizontalLine(0, 2, len);
      
      // Leq 숫자 데시벨
      display.drawString(0, 4, String(Leq_dB, 1) + " " + DB_UNITS);
      display.display(); 
  }
}

void loop() {
  
}
