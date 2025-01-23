/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// **[레지스터 주소 및 비트 정의 (매크로 상수)]**
#define GPIOA_BASE        (0x40020000UL)  // GPIOA 기준 주소 (LED LD2 연결)
#define GPIOA_MODER_OFFSET  (0x00UL)      // MODER 레지스터 offset (Mode 설정)
#define GPIOA_ODR_OFFSET    (0x14UL)      // ODR 레지스터 offset (Output Data)
#define GPIOC_BASE        (0x40020800UL)  // GPIOC 기준 주소 (버튼 B1 연결)
#define GPIOC_IDR_OFFSET    (0x10UL)      // IDR 레지스터 offset (Input Data)
#define RCC_AHB1ENR_OFFSET  (0x30UL)      // RCC AHB1ENR 레지스터 offset (AHB1 클럭 제어)
#define RCC_BASE          (0x40023800UL)  // RCC 기준 주소

#define RCC_AHB1ENR       (*(unsigned int*)(RCC_BASE + RCC_AHB1ENR_OFFSET)) // RCC AHB1 클럭 Enable 레지스터
#define GPIOA_MODER       (*(unsigned int*)(GPIOA_BASE + GPIOA_MODER_OFFSET)) // GPIOA Mode 설정 레지스터
#define GPIOA_ODR         (*(unsigned int*)(GPIOA_BASE + GPIOA_ODR_OFFSET))   // GPIOA Output Data 레지스터
#define GPIOC_IDR         (*(unsigned int*)(GPIOC_BASE + GPIOC_IDR_OFFSET))   // GPIOC Input Data 레지스터

#define LED_PIN           (5)           // LD2 LED 핀 번호 (PA5)
#define BUTTON_PIN        (13)          // B1 버튼 핀 번호 (PC13)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned int temp; // (optional) 디버깅용 변수
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void delay_ms(uint32_t ms); // 간단한 딜레이 함수 선언 (HAL_Delay 대신)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // **[GPIO 레지스터 초기화 (Init 부분)]**

  // 1. GPIOA 클럭 활성화 (RCC_AHB1ENR 레지스터 설정)
  RCC_AHB1ENR |= (1 << 0); // GPIOAEN 비트 (0번 비트) 설정

  // 2. PA5 핀 출력 모드로 설정 (GPIOA_MODER 레지스터 설정)
  GPIOA_MODER &= ~(3 << (LED_PIN * 2)); // PA5 핀 관련 비트(10, 11번 비트) 초기화 (00: Input)
  GPIOA_MODER |= (1 << (LED_PIN * 2));  // PA5 핀 관련 비트(10번 비트) 1로 설정 (01: Output)

  // 3. GPIOC 클럭 활성화 (RCC_AHB1ENR 레지스터 설정) - 버튼 B1 (PC13) 사용
  RCC_AHB1ENR |= (1 << 2); // GPIOCEN 비트 (2번 비트) 설정

  // 4. PC13 핀 입력 모드로 설정 (GPIOA_MODER 레지스터 설정) - 풀업 저항 사용 (풀업 저항은 외부 풀업 사용 or 내장 풀업 사용)
  // 여기서는 외부 풀업 저항 없이, STM32 내장 풀업 저항 사용
  GPIO_InitTypeDef GPIO_InitStruct = {0}; // (HAL 라이브러리 구조체, 레지스터 직접 제어와 무관) - 풀업 설정을 위해 HAL 함수 사용
  GPIO_InitStruct.Pin = GPIO_PIN_13;      // PC13 핀
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // 입력 모드
  GPIO_InitStruct.Pull = GPIO_PULLUP;    // 풀업 저항 사용
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); // (HAL 함수, 레지스터 직접 제어와 무관) - 풀업 설정을 위해 HAL 함수 사용


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // **[버튼 입력 확인 및 LED 제어 코드]**

    // 1. B1 버튼 (파란 버튼) 상태 읽기 (GPIOC_IDR 레지스터 사용)
    if (!(GPIOC_IDR & (1 << BUTTON_PIN)))  // B1 버튼 (PC13) 눌렸는지 확인 (풀업 저항 사용 가정)
    {
      delay_ms(10); // Debouncing (채터링 방지)
      if (!(GPIOC_IDR & (1 << BUTTON_PIN))) // 버튼 다시 눌렸는지 확인 (Debouncing)
      {
        // 2. LED 상태 반전 (GPIOA_ODR 레지스터 사용)
        GPIOA_ODR ^= (1 << LED_PIN); // XOR 연산으로 LED 상태 토글 (켜짐 <-> 꺼짐)

        while (!(GPIOC_IDR & (1 << BUTTON_PIN))); // 버튼에서 손 뗄 때까지 대기 (Debouncing)
      }
    }
  }
  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // ... (이전 코드와 동일)
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  // ... (이전 코드와 동일)
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  // ... (이전 코드와 동일, GPIOA, GPIOC 설정 부분은 main() 함수 Init 부분으로 옮겼으므로 여기서는 삭제)
}

/* USER CODE BEGIN 4 */

void delay_ms(uint32_t ms) // 간단한 딜레이 함수 구현 (HAL_Delay 대신)
{
  volatile uint32_t count = ms * 10000;
  while(count--);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN 7 */

/* USER CODE END 7 */
