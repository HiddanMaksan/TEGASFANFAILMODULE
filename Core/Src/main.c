/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : I2C scan + AT24CM01 self-test, ADS1115 NTC readout,
  *                   and a simple blocking WS2812B PWM+DMA driver.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "at24cm01.h"
#include <string.h>
#include <stdbool.h>
#define NTC_VDD        3.3f
#define NTC_R_FIXED    10000.0f
#define NTC_R0         10000.0f
#define NTC_BETA       3950.0f
#define NTC_ON_BOTTOM  1        // 1 = NTC to GND, 0 = NTC to 3.3V

volatile float ntc_voltage[8];
volatile float ntc_temp[8];






/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Scanner configuration ---- */
#define I2C_SCAN_ADDR_FIRST        0x08u
#define I2C_SCAN_ADDR_LAST         0x77u
#define I2C_SCAN_ADDR_COUNT        (I2C_SCAN_ADDR_LAST - I2C_SCAN_ADDR_FIRST + 1u)  /* 112 */
#define I2C_SCAN_PROBE_TIMEOUT_MS  5u
#define I2C_SCAN_RETRIES           1u

/* ---- Write/verify configuration ---- */
#define TEST_STRING_ADDR           0x001000u

/* ---- WS2812B ----
 * TIM1 clock = 100 MHz (SYSCLK 100 MHz, APB2 /1 -> timer clock 100 MHz).
 * Prescaler = 0, Period = 124 -> 125 ticks -> 800 kHz bit rate.
 * T0H = 35 ticks = 350 ns, T1H = 70 ticks = 700 ns.
 * If you change the clock or Period, recompute these (~28% / ~56% of ARR+1). */
#define WS2812_BIT_0_CCR            35u
#define WS2812_BIT_1_CCR            70u
#define WS2812_BITS_PER_LED         24u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ---- NTC / ADS1115 ---- */
volatile int16_t ntc1raw = 0;
volatile int16_t ntc2raw = 0;
volatile int16_t ntc3raw = 0;
volatile int16_t ntc4raw = 0;
volatile int16_t ntc5raw = 0;
volatile int16_t ntc6raw = 0;
volatile int16_t ntc7raw = 0;
volatile int16_t ntc8raw = 0;




volatile int16_t fan1raw = 0;
volatile int16_t fan2raw = 0;
volatile int16_t fan3raw = 0;
volatile int16_t fan4raw = 0;
volatile int16_t fan5raw = 0;
volatile int16_t fan6raw = 0;
volatile int16_t fan7raw = 0;
volatile int16_t fan8raw = 0;

volatile int g_status0 = -1;
volatile int g_status1 = -1;
volatile int g_status2 = -1;
volatile int g_status3 = -1;
/* ---- Scanner results ---- */
volatile uint32_t g_i2c1_found_bitmap[4] = {0};
volatile uint32_t g_i2c2_found_bitmap[4] = {0};

volatile uint8_t  g_i2c1_found_list[I2C_SCAN_ADDR_COUNT] = {0};
volatile uint8_t  g_i2c1_found_count = 0;
volatile uint8_t  g_i2c2_found_list[I2C_SCAN_ADDR_COUNT] = {0};
volatile uint8_t  g_i2c2_found_count = 0;

volatile uint32_t g_i2c1_probe_total  = 0;
volatile uint32_t g_i2c1_probe_errors = 0;
volatile uint32_t g_i2c2_probe_total  = 0;
volatile uint32_t g_i2c2_probe_errors = 0;

volatile bool     g_any_device_found  = false;

/* ---- Write / verify results ---- */
static const char g_write_string[] = "Hello AT24CM01!";

volatile char        g_readback[64]        = {0};
volatile uint16_t    g_readback_len        = 0;

volatile bool        g_write_ok            = false;
volatile bool        g_read_ok             = false;
volatile bool        g_match_ok            = false;
volatile bool        g_eeprom_overall_ok   = false;

volatile int         g_write_status        = -1;
volatile int         g_read_status         = -1;
volatile uint32_t    g_test_addr           = TEST_STRING_ADDR;
volatile uint16_t    g_test_len            = 0;

volatile bool        g_all_ok              = false;

/* ---- WS2812B state ---- */
static uint16_t g_ws2812_pwm_data[WS2812_BITS_PER_LED];
static volatile uint8_t g_ws2812_data_sent_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static float raw_to_volts(int16_t raw)
{
    return raw * (4.096f / 32768.0f);
}

static float volts_to_celsius(float v)
{
    if (v <= 0.0f || v >= NTC_VDD) return -273.15f;   // invalid reading

#if NTC_ON_BOTTOM
    float r = NTC_R_FIXED * v / (NTC_VDD - v);
#else
    float r = NTC_R_FIXED * (NTC_VDD - v) / v;
#endif

    float inv_t = (1.0f / 298.15f) + (1.0f / NTC_BETA) * logf(r / NTC_R0);
    return (1.0f / inv_t) - 273.15f;
}
/* ======================================================================== */
/*  SCANNER                                                                 */
/* ======================================================================== */

static HAL_StatusTypeDef I2C_ProbeAddress(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    HAL_StatusTypeDef st = HAL_ERROR;

    for (uint8_t attempt = 0; attempt < I2C_SCAN_RETRIES; attempt++)
    {
        st = HAL_I2C_IsDeviceReady(hi2c,
                                   (uint16_t)(addr7 << 1),
                                   1,
                                   I2C_SCAN_PROBE_TIMEOUT_MS);
        if (st == HAL_OK)
        {
            break;
        }
    }

    return st;
}

static void I2C_ScanBus(I2C_HandleTypeDef *hi2c,
                        volatile uint32_t bitmap[4],
                        volatile uint8_t  list[I2C_SCAN_ADDR_COUNT],
                        volatile uint8_t *count,
                        volatile uint32_t *errors)
{
    uint32_t local_bitmap[4] = {0};
    uint8_t  local_count     = 0;
    uint32_t local_errors    = 0;

    for (uint8_t addr = I2C_SCAN_ADDR_FIRST; addr <= I2C_SCAN_ADDR_LAST; addr++)
    {
        HAL_StatusTypeDef st = I2C_ProbeAddress(hi2c, addr);

        if (st == HAL_OK)
        {
            uint8_t idx = (uint8_t)(addr - I2C_SCAN_ADDR_FIRST);
            local_bitmap[idx >> 5] |= (1u << (idx & 0x1Fu));

            if (local_count < I2C_SCAN_ADDR_COUNT)
            {
                list[local_count++] = addr;
            }
        }
        else if (st != HAL_ERROR)
        {
            local_errors++;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        bitmap[i] = local_bitmap[i];
    }
    *count  = local_count;
    *errors = local_errors;
}

static void I2C_ScanAll(void)
{
    I2C_ScanBus(&hi2c1,
                g_i2c1_found_bitmap,
                g_i2c1_found_list,
                &g_i2c1_found_count,
                &g_i2c1_probe_errors);

    I2C_ScanBus(&hi2c2,
                g_i2c2_found_bitmap,
                g_i2c2_found_list,
                &g_i2c2_found_count,
                &g_i2c2_probe_errors);

    g_i2c1_probe_total = I2C_SCAN_ADDR_COUNT;
    g_i2c2_probe_total = I2C_SCAN_ADDR_COUNT;

    g_any_device_found = (g_i2c1_found_count > 0) || (g_i2c2_found_count > 0);
}

/* ======================================================================== */
/*  WRITE / VERIFY                                                          */
/* ======================================================================== */

static void AT24CM01_WriteAndVerify(void)
{
    g_test_len = (uint16_t)(sizeof(g_write_string));

    g_write_status = (int)AT24CM01_Write(&hi2c1,
                                         g_test_addr,
                                         (const uint8_t *)g_write_string,
                                         g_test_len);
    g_write_ok = (g_write_status == (int)HAL_OK);

    if (!g_write_ok)
    {
        return;
    }

    memset((void *)g_readback, 0, sizeof(g_readback));

    g_read_status = (int)AT24CM01_Read(&hi2c1,
                                       g_test_addr,
                                       (uint8_t *)g_readback,
                                       g_test_len);
    g_read_ok = (g_read_status == (int)HAL_OK);

    if (!g_read_ok)
    {
        return;
    }

    g_readback_len = g_test_len;
    g_match_ok = (memcmp((const void *)g_readback,
                         g_write_string,
                         g_test_len) == 0);

    g_eeprom_overall_ok = g_write_ok && g_read_ok && g_match_ok;
}

/* ======================================================================== */
/*  WS2812B PWM + DMA DRIVER -- single-shot, blocking                       */
/* ======================================================================== */

/**
  * @brief  Send one 24-bit colour word (wire order G R B, MSB first).
  * @note   Blocks until the DMA transfer completes (with a timeout so a
  *         missing DMA setup can't hang the firmware forever).
  */
void WS2812_Send(uint32_t color)
{
    for (int i = 0; i < (int)WS2812_BITS_PER_LED; i++)
    {
        uint32_t bit = (color >> (23 - i)) & 1u;
        g_ws2812_pwm_data[i] = bit ? WS2812_BIT_1_CCR : WS2812_BIT_0_CCR;
    }

    g_ws2812_data_sent_flag = 0;
    if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
                              (uint32_t *)g_ws2812_pwm_data,
                              WS2812_BITS_PER_LED) != HAL_OK)
    {
        Error_Handler();
    }

    uint32_t start = HAL_GetTick();
    while (!g_ws2812_data_sent_flag)
    {
        if ((HAL_GetTick() - start) > 10u)
        {
            HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
            break;
        }
    }
    g_ws2812_data_sent_flag = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI4_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef ads_ready = HAL_I2C_IsDeviceReady(&hi2c2,
                                                       (0x4A << 1),
                                                       3,
                                                       100);
  volatile int ads_found = (ads_ready == HAL_OK) ? 1 : 0;
  (void)ads_found;

  /* 10k fixed resistor, 3.3 V divider, 10k NTC, beta 3950 */

  /* Park WP high before anything else touches the bus. */
  AT24CM01_WP_Init();

  /* Phase 1: scan both I2C buses */
  I2C_ScanAll();

  /* Phase 2: write/read/verify the AT24CM01 */
  AT24CM01_WriteAndVerify();

  g_all_ok = (g_i2c1_probe_errors == 0) &&
             (g_i2c2_probe_errors == 0) &&
             g_eeprom_overall_ok;

  /* Phase 3: WS2812B -- full red (G=0x00, R=0xFF, B=0x00) */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		  uint8_t config[2];
		     uint8_t rx[2];

		     config[1] = 0x83;   /* 128 SPS, comparator off */





		     /* ---- AIN0 ---- */
		     config[0] = 0xC3;
		     HAL_I2C_Mem_Write(&hi2c2, (0x4A << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		     HAL_Delay(20);
		     HAL_I2C_Mem_Read(&hi2c2, (0x4A << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		     ntc4raw = (int16_t)((rx[0] << 8) | rx[1]);

		     /* ---- AIN1 ---- */
		     config[0] = 0xD3;
		     HAL_I2C_Mem_Write(&hi2c2, (0x4A << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		     HAL_Delay(20);
		     HAL_I2C_Mem_Read(&hi2c2, (0x4A << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		     ntc3raw = (int16_t)((rx[0] << 8) | rx[1]);

		     /* ---- AIN2 ---- */
		     config[0] = 0xE3;
		     HAL_I2C_Mem_Write(&hi2c2, (0x4A << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		     HAL_Delay(20);
		     HAL_I2C_Mem_Read(&hi2c2, (0x4A << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		     ntc2raw = (int16_t)((rx[0] << 8) | rx[1]);

		     /* ---- AIN3 ---- */
		     config[0] = 0xF3;
		     HAL_I2C_Mem_Write(&hi2c2, (0x4A << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		     HAL_Delay(20);
		     HAL_I2C_Mem_Read(&hi2c2, (0x4A << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		     ntc1raw = (int16_t)((rx[0] << 8) | rx[1]);



		     /* ---- AIN0 ---- */
		     config[0] = 0xC3;
		     HAL_I2C_Mem_Write(&hi2c1, (0x48 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		     HAL_Delay(20);
		     HAL_I2C_Mem_Read(&hi2c1, (0x48 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		     ntc8raw = (int16_t)((rx[0] << 8) | rx[1]);

		     /* ---- AIN1 ---- */
		     config[0] = 0xD3;
		     HAL_I2C_Mem_Write(&hi2c1, (0x48 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		     HAL_Delay(20);
		     HAL_I2C_Mem_Read(&hi2c1, (0x48 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		     ntc7raw = (int16_t)((rx[0] << 8) | rx[1]);

		     /* ---- AIN2 ---- */
		     config[0] = 0xE3;
		     HAL_I2C_Mem_Write(&hi2c1, (0x48 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		     HAL_Delay(20);
		     HAL_I2C_Mem_Read(&hi2c1, (0x48 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		     ntc6raw = (int16_t)((rx[0] << 8) | rx[1]);

		     /* ---- AIN3 ---- */
		     config[0] = 0xF3;
		     HAL_I2C_Mem_Write(&hi2c1, (0x48 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		     HAL_Delay(10);
		     HAL_I2C_Mem_Read(&hi2c1, (0x48 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		     ntc5raw = (int16_t)((rx[0] << 8) | rx[1]);



		  ////////////////////////////fan raw read
		     /* ---- AIN0 ---- */
		   		     config[0] = 0xC3;
		   		     HAL_I2C_Mem_Write(&hi2c2, (0x48 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		   		     HAL_Delay(20);
		   		     HAL_I2C_Mem_Read(&hi2c2, (0x48 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		   		     fan1raw = (int16_t)((rx[0] << 8) | rx[1]);

		   		     /* ---- AIN1 ---- */
		   		     config[0] = 0xD3;
		   		     HAL_I2C_Mem_Write(&hi2c2, (0x48 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		   		     HAL_Delay(20);
		   		     HAL_I2C_Mem_Read(&hi2c2, (0x48 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		   		     fan2raw = (int16_t)((rx[0] << 8) | rx[1]);

		   		     /* ---- AIN2 ---- */
		   		     config[0] = 0xE3;
		   		     HAL_I2C_Mem_Write(&hi2c2, (0x48 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		   		     HAL_Delay(20);
		   		     HAL_I2C_Mem_Read(&hi2c2, (0x48 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		   		     fan3raw = (int16_t)((rx[0] << 8) | rx[1]);

		   		     /* ---- AIN3 ---- */
		   		     config[0] = 0xF3;
		   		     HAL_I2C_Mem_Write(&hi2c2, (0x48 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		   		     HAL_Delay(20);
		   		     HAL_I2C_Mem_Read(&hi2c2, (0x48 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		   		     fan4raw = (int16_t)((rx[0] << 8) | rx[1]);



		   		     /* ---- AIN0 ---- */
		   		     config[0] = 0xC3;
		   		     HAL_I2C_Mem_Write(&hi2c2, (0x49 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		   		     HAL_Delay(20);
		   		     HAL_I2C_Mem_Read(&hi2c2, (0x49 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		   		     fan5raw = (int16_t)((rx[0] << 8) | rx[1]);

		   		     /* ---- AIN1 ---- */
		   		     config[0] = 0xD3;
		   		     HAL_I2C_Mem_Write(&hi2c2, (0x49 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		   		     HAL_Delay(20);
		   		     HAL_I2C_Mem_Read(&hi2c2, (0x49 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		   		     fan6raw = (int16_t)((rx[0] << 8) | rx[1]);

		   		     /* ---- AIN2 ---- */
		   		     config[0] = 0xE3;
		   		     HAL_I2C_Mem_Write(&hi2c2, (0x49 << 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		   		     HAL_Delay(20);
		   		     HAL_I2C_Mem_Read(&hi2c2, (0x49 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		   		     fan7raw = (int16_t)((rx[0] << 8) | rx[1]);

		   		     /* ---- AIN3 ---- */
		   		     config[0] = 0xF3;
		   		     HAL_I2C_Mem_Write(&hi2c2, (0x49<< 1), 0x01, I2C_MEMADD_SIZE_8BIT, config, 2, 100);
		   		     HAL_Delay(10);
		   		     HAL_I2C_Mem_Read(&hi2c2, (0x49 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, rx, 2, 100);
		   		     fan8raw = (int16_t)((rx[0] << 8) | rx[1]);





		     ////////////////////////




		     /* ---- Voltage and temperature ---- */
		     ntc_voltage[0] = raw_to_volts(ntc1raw);
		     ntc_voltage[1] = raw_to_volts(ntc2raw);
		     ntc_voltage[2] = raw_to_volts(ntc3raw);
		     ntc_voltage[3] = raw_to_volts(ntc4raw);


		     /* ---- Voltage and temperature ---- */
		      ntc_voltage[4] = raw_to_volts(ntc5raw);
		      ntc_voltage[5] = raw_to_volts(ntc6raw);
		      ntc_voltage[6] = raw_to_volts(ntc7raw);
		      ntc_voltage[7] = raw_to_volts(ntc8raw);

		      ntc_temp[0] = volts_to_celsius( ntc_voltage[0]);
		      ntc_temp[1] = volts_to_celsius(ntc_voltage[1]);
		      ntc_temp[2] = volts_to_celsius(ntc_voltage[2]);
		      ntc_temp[3] = volts_to_celsius(ntc_voltage[3]);
		      ntc_temp[4] = volts_to_celsius(ntc_voltage[4]);
		      ntc_temp[5] = volts_to_celsius(ntc_voltage[5]);
		      ntc_temp[6] = volts_to_celsius(ntc_voltage[6]);
		      ntc_temp[7] = volts_to_celsius(ntc_voltage[7]);





  }

    HAL_Delay(1000);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROTARYPUSHBUTTON_Pin|ROTARYCLK_Pin|LED11_Pin|LED10_Pin
                          |PA15_TFT_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|LED17_Pin|LED9_Pin|TFTRS_Pin
                          |TFTRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ROTARYPUSHBUTTON_Pin ROTARYCLK_Pin LED11_Pin LED10_Pin
                           PA15_TFT_NSS_Pin */
  GPIO_InitStruct.Pin = ROTARYPUSHBUTTON_Pin|ROTARYCLK_Pin|LED11_Pin|LED10_Pin
                          |PA15_TFT_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INTCAN_Pin */
  GPIO_InitStruct.Pin = INTCAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INTCAN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 LED17_Pin LED9_Pin TFTRS_Pin
                           TFTRESET_Pin WP_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LED17_Pin|LED9_Pin|TFTRS_Pin
                          |TFTRESET_Pin|WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  DMA transfer complete for TIM1 CH1: stop PWM (line idles low)
  *         and release WS2812_Send().
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
        g_ws2812_data_sent_flag = 1;
    }
}

/* USART2 is used for Modbus in this project -- no printf retarget. */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
    __NOP();
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
