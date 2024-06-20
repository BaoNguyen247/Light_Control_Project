#include "main.h"
#include "cg9a01.h"
#include <stdio.h>
RTC_HandleTypeDef hrtc;
ADC_HandleTypeDef hadc1;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
RTC_TimeTypeDef sTime;						//Khởi Tạo các biến cấu truc cần thiết cho chương trình
RTC_DateTypeDef sDate;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_ch3;
#define usTIM	TIM4
char time[15];
char date[15];
char sensor[5];								//Khai báo các biến dùng để lưu các kí tự để xuất ra màn hình
char timelately[3];
char restminute[2];
char latelyoff[2];
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);		//Khai Báo các hàm khởi tạo
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
void usDelay(uint32_t uSec);
uint8_t nextFiveMinutes;
const float Speedofsound = 0.0343/2;			//Tốc độ âm thanh chia 2
float distance = 26;				//Thời gian hiện tại
float distance1;					//Thời gian quá khứ
float distance2 = 0;				//Thời gian hiện tại
char human;
uint16_t lux = 0;
char light[10];
char control[5];
int main(void)
{
  uint32_t numTicks;
  HAL_Init();

  SystemClock_Config();
  MX_ADC1_Init();
  MX_GPIO_Init();
  MX_DMA_Init();			//Gọi các ham khởi tạo cần thiết
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  GC9A01_Initial();
  ClearScreen2(WHITE);
  show_picture2a(16,0,208,60);
  show_picture2b(16,180,208,60);
  show_picture2a(16,0,208,60);
  show_picture2b(16,180,208,60);						//Xuất hình ảnh, chữ của background cho lcd
  show_picture2c(20,60,40,40);
  show_picture2c(20,60,40,40);
  showzifustr(60, 73, "NHOM 2", BLACK, WHITE);
  LCD_DrawLine(60, 88, 100, 82, RED);
  for(int i = 130; i<210; i++){
	  LCD_DrawLine(i, 82, i, 170, CBLUE);
  }


  showzifustr(142, 90, "Human", BLACK, CPINK);
  showzifustr(140, 100,"Indoor?", BLACK, CPINK);

  LCD_DrawRectangle(20, 100, 127 ,170, GREEN);
	showzifustr(22, 105, "Date:", BLACK, WHITE);
	showzifustr(22, 120, "Time:", BLACK, WHITE);
    showzifustr(22, 135, "Onto:", BLACK, WHITE);
	showzifustr(22, 150, "LastOff:", BLACK, WHITE);

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);					//Đọc dữ liệu
	TIM1->CCR3 = 0;
    while (1)
    {

     	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);			//Hàm đọc giá trị ngày va giờ
    	  sprintf(date, "%02d/%02d/20%02d",sDate.Date,sDate.Month,sDate.Year);    //Chuyển ngày thắng năm thành kiểu dữ liệu kí tự dạng ngày/tháng/năm và lưu vào biến date
    	  showzifustr(55, 105, date, BLACK, WHITE);					//xuất biến ra màn hình
    	  sprintf(time, "%02dh%02d'",sTime.Hours,sTime.Minutes);		//Chuyển giờ phút thành kiểu dữ liệu kí tự dạng giờ/phút và lưu vào biến time
    	  showzifustr(55, 120, time, BLACK, WHITE);					//xuất biến time ra màn hình
    	  distance1 = distance;										//lưu giá trị thời gian hiện tại vào biến distance1
    	  //1.Khối đọc dữ liệu từ cảm biến siêu âm HC-SR04
    	  //a.Phát 1 xung 10us vào chấn Trigger để kích hoạt cảm biến siêu âm
    	  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
    	  usDelay(3);
    	  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
    	  usDelay(10);
    	  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
    	  //b.Miễn là chân Echo vẫn ở mức HIGH, cho biết xung Echo vẫn đang diễn ra thì vòng lặp này tiếp tục đếm và đặt lại biến numTicks = 0
    	  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin)==GPIO_PIN_RESET);
    	  numTicks = 0;
    	  //Trong mỗi vòng lặp  thì numTicks tăng lên 1 và delay 2us
    	  while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin)==GPIO_PIN_SET)
    	  {
    		numTicks++;
    		usDelay(2);
    	  };

    	  distance = (numTicks + 0.0f)*2.8*Speedofsound; //Tính khoảng cách của vật thể bằng công thức: Khoảng cách = thời gian(1 numTicks = 2.8s) * vận tốc

    	  //2.Khối xác định có người hoạt động hay không
    	  distance2 = distance;

    	  if(((distance2-distance1)>=5)||(distance2<20)) //Nếu độ chênh lệch khoảng cách bây giờ so với quá khứ > 5 hoặc có một vật thể quá gần < 20 thi vào hàm if
    	  {
    		  if (nextFiveMinutes >= 60) {   //biến nextFiveMinutes thể hiện thời gian tương lai sau 5p
    		      nextFiveMinutes -= 55 ;    //nếu số phút >= 60 thì đặt lại về 5p
    		  };
    		  human = 1;                     //Xác định có người bằng cách set biến human = 1
    		  nextFiveMinutes = sTime.Minutes;
    		  nextFiveMinutes += 0b0001;     //đặt biến nextFiveMinutes thành thời gian hiện tại + 5;
    		  sprintf(restminute, "%02d'",nextFiveMinutes);				//Tạo biến "thời gian khi đèn tắt" và xuất ra màn hình
        	  showzifustr(60, 135, restminute , BLACK, WHITE);
    	  }else{										//Nếu ngược lại độ chênh lệch nhỏ và không có ngươi thi vao hàm else

    		  if(sTime.Minutes == nextFiveMinutes){		//Nếu "thời gian hiện tại" bằng "thời gian khi đèn tắt"
    		  human = 0;									//Xác định không có người băng cách set biến human = 0
    		  sprintf(timelately, "%02dh'",sTime.Hours);		//Xuất thời gian lúc đèn tắt ra màn hinh
        	  showzifustr(80, 150, timelately , BLACK, WHITE);
    		  sprintf(latelyoff, "%02d'",nextFiveMinutes);
        	  showzifustr(100, 150, latelyoff , BLACK, WHITE);
    		  };

    	  }
    	  //3.Khối điều khiền đèn dựa trên biến human
    	  if(human == 1){	//Nếu có người vào hàm if
    		  if(lux < 2900) //kiểm tra độ sáng trong phòng là biến lux có nhỏ hơn 2900 không
    		  {
    			  if(TIM1->CCR3 <= 100){
        			  TIM1->CCR3 += 5;				//Nếu có thì tăng dần thanh ghi xung điều khiển đèn dần dần từ 1 -> 100%
    			  	  }
    		  }
    		  if(lux>2950){							//Nếu độ sáng trong phòng quá sáng > 2950 sẽ giảm độ sáng bằng cách giảm thanh ghi
    			  if(TIM1->CCR3 > 0){
        			  TIM1->CCR3 -= 5;
    			  }
    			  if(TIM1->CCR3 == 0){				//Nếu thanh ghi = 0 thì giữ nguyên = 0 để tranh bị lỗi giá trị
        			  TIM1->CCR3 = 0;
    			  }
    		  }
    		  if((lux>=2900)&&(lux<=2950))
    		  {
    			  TIM1->CCR3 = TIM1->CCR3;			//Nếu độ sáng nằm trong khoảng  2900-2950 thì giữ nguyên độ sáng
    		  }
    		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    	  }
    	  else{
    		  TIM1->CCR3 = 0;
    		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	  //Nếu biến human = 0 -> không có người và đèn tắt

    	  }
    	      sprintf(sensor, "%.2f", distance);
              showzifustr(150, 120, sensor , BLACK, WHITE);		//Xuất giá trị của cảm biến siêu âm ra màn hình
              //KHối đọc giá tri ánh sáng lux
        	  HAL_ADC_Start(&hadc1);
        	  HAL_ADC_PollForConversion(&hadc1, 20);
        	  lux = HAL_ADC_GetValue(&hadc1); 					//Lấy giá trị ánh sáng từ cảm biến và lưu và biến lux
        	  sprintf(light, "%d (lux)", lux);
        	  showzifustr(150, 140,light, BLACK, WHITE);		//Xuất giái trị ánh sáng ra màn hình
        	  sprintf(control, "%d", TIM1->CCR3);
        	  showzifustr(150, 155, control, BLACK, WHITE);		//Xuất giá trị của thanh ghi xung ra màn hình
        	  HAL_Delay(1000);
  }
}
void SystemClock_Config(void)   //Hàm Khởi tạo và cấp xung hệ thống
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
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
  sTime.Hours = 0x8;
  sTime.Minutes = 0x00;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_JUNE;
  sDate.Date = 0x20;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  HAL_TIM_MspPostInit(&htim1);

}
static void MX_TIM4_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
static void MX_DMA_Init(void)
{

  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, Trigger_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_RST_Pin|LCD_DC_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif
