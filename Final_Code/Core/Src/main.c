#include "main.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

// Defines the starting frequency for the motor ramp-up. This is the frequency at which the motor will start spinning.
#define START_FREQ  400
// Defines the default end frequency for the motor ramp-up. This is used when no frequency is specified in the UART command.
#define DEFAULT_END_FREQ 1700
// Defines the minimum frequency that can be set for the motor. This is to prevent the motor from stalling.
#define MIN_FREQ 400
// Defines the maximum frequency that can be set for the motor. This is to prevent the motor from over-speeding.
#define MAX_FREQ 2500
// Defines the step size for increasing the frequency during ramp-up. A smaller value will result in a smoother ramp-up.
#define FREQ_STEP_UP  10
// Defines the step size for decreasing the frequency during ramp-down. A larger value will result in a faster ramp-down.
#define FREQ_STEP_DOWN  50

// Defines the interval in milliseconds between PWM updates during ramp-up. A smaller value will result in a smoother ramp-up.
#define PWM_UPDATE_INTERVAL_MS 10
// Defines the interval in milliseconds between PWM updates during ramp-down.
#define PWM_UPDATE_INTERVAL_DOWN_MS 30


// Timer handles for the different motors. Motor 1 uses TIM2, Motor 3 uses TIM9, and Motors 2 and 4 share TIM5.
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

// UART handle for communication with the ESP32.
UART_HandleTypeDef huart6;
// DMA handle for UART receive. DMA is used to offload the UART receive process from the CPU.
DMA_HandleTypeDef hdma_usart6_rx;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);

// Defines the possible states for a motor. This state machine is used to control the motor's behavior.
typedef enum {
    MOTOR_STATE_OFF, // The motor is off.
    MOTOR_STATE_RAMP_UP, // The motor is ramping up to the target speed.
    MOTOR_STATE_ON, // The motor is running at the target speed.
    MOTOR_STATE_RAMP_DOWN, // The motor is ramping down to a stop.
    MOTOR_STATE_CALIBRATE_RAMP_UP, // The motor is ramping up to the target speed for calibration.
    MOTOR_STATE_CALIBRATING // The motor is running at the target speed for calibration.
} MotorState;

// Current state of each motor.
MotorState motor1_state = MOTOR_STATE_OFF;
MotorState motor2_state = MOTOR_STATE_OFF;
MotorState motor3_state = MOTOR_STATE_OFF;
MotorState motor4_state = MOTOR_STATE_OFF;

// Current frequency of motor 1.
uint32_t motor1_freq = START_FREQ;
// Last time the PWM for motor 1 was updated. This is used to control the ramp-up and ramp-down speed.
uint32_t motor1_last_update = 0;
// Target end frequency for motor 1.
uint32_t motor1_end_freq = DEFAULT_END_FREQ;
// Start time of the calibration for motor 1.
uint32_t motor1_calibration_start_time = 0;

// Current frequency of motor 3.
uint32_t motor3_freq = START_FREQ;
// Last time the PWM for motor 3 was updated.
uint32_t motor3_last_update = 0;
// Target end frequency for motor 3.
uint32_t motor3_end_freq = DEFAULT_END_FREQ;
// Start time of the calibration for motor 3.
uint32_t motor3_calibration_start_time = 0;

// Current frequency for the shared timer (motors 2 and 4).
uint32_t shared_freq = START_FREQ;
// Last time the PWM for the shared timer was updated.
uint32_t shared_last_update = 0;
// Target end frequency for motor 2.
uint32_t motor2_end_freq = DEFAULT_END_FREQ;
// Target end frequency for motor 4.
uint32_t motor4_end_freq = DEFAULT_END_FREQ;
// Start time of the calibration for motor 2.
uint32_t motor2_calibration_start_time = 0;
// Start time of the calibration for motor 4.
uint32_t motor4_calibration_start_time = 0;

// Buffer to store incoming UART data.
uint8_t RxData[2000];
// Firmware version string. The '\n' at the end is important for the ESP32 to correctly parse the string.
uint8_t FirmwareVer[] = "FW:5.1.3\n";
// Buffer to store the current command being parsed.
char cmdBuffer[150];
// Current index in the command buffer.
uint8_t cmdIndex = 0;

// Sets the PWM frequency for motor 1.
void set_motor1_pwm(uint32_t freq) {
    uint32_t period = (1000000UL / freq) - 1;
    uint32_t pulse  = period / 2;
    __HAL_TIM_SET_AUTORELOAD(&htim2, period);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}
// Sets the PWM frequency for motor 3.
void set_motor3_pwm(uint32_t freq)
{
    uint32_t period = (1000000UL / freq) - 1;
    uint32_t pulse = period / 2;

    __HAL_TIM_SET_AUTORELOAD(&htim9, period);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse);
}


// Sets the PWM frequency for the shared timer (motors 2 and 4).
void set_shared_pwm(uint32_t freq) {
    uint32_t period = (1000000UL / freq) - 1;
    uint32_t pulse  = period / 2;
    __HAL_TIM_SET_AUTORELOAD(&htim5, period);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pulse);
}

// Processes the incoming UART data and executes the corresponding commands.
void ProcessRxBuffer(uint8_t* RxData, uint16_t Size) {
    for (uint16_t i = 0; i < Size; i++) {
        if (RxData[i] == '\n') {
            cmdBuffer[cmdIndex] = '\0';
            int motor_num, freq;
            if (sscanf(cmdBuffer, "Motor %d ON %d", &motor_num, &freq) == 2) {
                if (freq < MIN_FREQ || freq > MAX_FREQ) {
                    freq = DEFAULT_END_FREQ;
                }
                switch (motor_num) {
                    case 1:
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
                        motor1_end_freq = freq;
                        motor1_state = MOTOR_STATE_RAMP_UP;
                        motor1_freq = START_FREQ;
                        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
                        break;
                    case 2:
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
                        motor2_end_freq = freq;
                        motor2_state = MOTOR_STATE_RAMP_UP;
                        shared_freq = START_FREQ;
                        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
                        break;
                    case 3:
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
                        motor3_end_freq = freq;
                        motor3_state = MOTOR_STATE_RAMP_UP;
                        motor3_freq = START_FREQ;
                        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
                        break;
                    case 4:
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
                        motor4_end_freq = freq;
                        motor4_state = MOTOR_STATE_RAMP_UP;
                        shared_freq = START_FREQ;
                        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
                        break;
                }
            } else if (strcmp(cmdBuffer, "Motor 1 ON") == 0) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
                motor1_end_freq = DEFAULT_END_FREQ;
                motor1_state = MOTOR_STATE_RAMP_UP;
                motor1_freq = START_FREQ;
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            } else if (strcmp(cmdBuffer, "Motor 1 OFF") == 0) {
                motor1_state = MOTOR_STATE_RAMP_DOWN;
            } else if (strcmp(cmdBuffer, "Motor 1 REV") == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
            } else if (strcmp(cmdBuffer, "Motor 1 FWD") == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
            } else if (strcmp(cmdBuffer, "Motor 2 ON") == 0) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
                motor2_end_freq = DEFAULT_END_FREQ;
                motor2_state = MOTOR_STATE_RAMP_UP;
                shared_freq = START_FREQ;
                HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
            } else if (strcmp(cmdBuffer, "Motor 2 OFF") == 0) {
                motor2_state = MOTOR_STATE_RAMP_DOWN;
            } else if (strcmp(cmdBuffer, "Motor 2 REV") == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
            } else if (strcmp(cmdBuffer, "Motor 2 FWD") == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
            } else if (strcmp(cmdBuffer, "Motor 3 ON") == 0) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
                motor3_end_freq = DEFAULT_END_FREQ;
                motor3_state = MOTOR_STATE_RAMP_UP;
                motor3_freq = START_FREQ;
                HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
            } else if (strcmp(cmdBuffer, "Motor 3 OFF") == 0) {
                motor3_state = MOTOR_STATE_RAMP_DOWN;
            } else if (strcmp(cmdBuffer, "Motor 3 REV") == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
            } else if (strcmp(cmdBuffer, "Motor 3 FWD") == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
            } else if (strcmp(cmdBuffer, "Motor 4 ON") == 0) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
                motor4_end_freq = DEFAULT_END_FREQ;
                motor4_state = MOTOR_STATE_RAMP_UP;
                shared_freq = START_FREQ;
                HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
            } else if (strcmp(cmdBuffer, "Motor 4 OFF") == 0) {
                motor4_state = MOTOR_STATE_RAMP_DOWN;
            } else if (strcmp(cmdBuffer, "Motor 4 REV") == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
            } else if (strcmp(cmdBuffer, "Motor 4 FWD") == 0) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
            } else if (sscanf(cmdBuffer, "Motor %d CALIBRATE %d", &motor_num, &freq) == 2) {
                if (freq < MIN_FREQ || freq > MAX_FREQ) {
                    freq = DEFAULT_END_FREQ;
                }
                switch (motor_num) {
                    case 1:
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
                        motor1_end_freq = freq;
                        motor1_state = MOTOR_STATE_CALIBRATE_RAMP_UP;
                        motor1_calibration_start_time = HAL_GetTick();
                        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
                        break;
                    case 2:
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
                        motor2_end_freq = freq;
                        motor2_state = MOTOR_STATE_CALIBRATE_RAMP_UP;
                        motor2_calibration_start_time = HAL_GetTick();
                        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
                        break;
                    case 3:
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
                        motor3_end_freq = freq;
                        motor3_state = MOTOR_STATE_CALIBRATE_RAMP_UP;
                        motor3_calibration_start_time = HAL_GetTick();
                        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
                        break;
                    case 4:
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
                        motor4_end_freq = freq;
                        motor4_state = MOTOR_STATE_CALIBRATE_RAMP_UP;
                        motor4_calibration_start_time = HAL_GetTick();
                        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
                        break;
                }
            }
            cmdIndex = 0;
        } else if (cmdIndex < sizeof(cmdBuffer) - 1) {
            cmdBuffer[cmdIndex++] = RxData[i];
        } else {
            memset(cmdBuffer, 0, sizeof(cmdBuffer));
            cmdIndex = 0;
        }
    }
}

// This function is called when a UART receive event occurs.
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART6) {
        ProcessRxBuffer(RxData, Size);
        memset(RxData, 0, sizeof(RxData));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RxData, sizeof(RxData));
    }
}




int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();

  HAL_Delay(2000);

  HAL_UART_Transmit(&huart6, FirmwareVer , strlen((char*)FirmwareVer), 100);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6,RxData,sizeof(RxData));
  HAL_Delay(2000);


  while (1)
  {
      uint32_t now = HAL_GetTick();

      // Motor 1 state machine
      if (now - motor1_last_update >= (motor1_state == MOTOR_STATE_RAMP_UP || motor1_state == MOTOR_STATE_CALIBRATE_RAMP_UP ? PWM_UPDATE_INTERVAL_MS : PWM_UPDATE_INTERVAL_DOWN_MS)) {
          if (motor1_state == MOTOR_STATE_RAMP_UP) {
              set_motor1_pwm(motor1_freq);
              motor1_freq += FREQ_STEP_UP;
              if (motor1_freq >= motor1_end_freq) {
                  motor1_freq = motor1_end_freq;
                  motor1_state = MOTOR_STATE_ON;
              }
          } else if (motor1_state == MOTOR_STATE_CALIBRATE_RAMP_UP) {
              set_motor1_pwm(motor1_freq);
              motor1_freq += FREQ_STEP_UP;
              if (motor1_freq >= motor1_end_freq) {
                  motor1_freq = motor1_end_freq;
                  motor1_state = MOTOR_STATE_CALIBRATING;
              }
          } else if (motor1_state == MOTOR_STATE_CALIBRATING) {
        	  if (now - motor1_calibration_start_time >= 60000) {
        		  motor1_state = MOTOR_STATE_RAMP_DOWN;
        		  char msg[] = "Calibration for Motor 1 is done\n";
        		  HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 100);
        	  }
          } else if (motor1_state == MOTOR_STATE_RAMP_DOWN) {
              set_motor1_pwm(motor1_freq);
              motor1_freq -= FREQ_STEP_DOWN;
              if (motor1_freq <= START_FREQ) {
                  motor1_freq = START_FREQ;
                  motor1_state = MOTOR_STATE_OFF;
                  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
              }
          }
          motor1_last_update = now;
      }

      // Motor 3 state machine
      if (now - motor3_last_update >= (motor3_state == MOTOR_STATE_RAMP_UP || motor3_state == MOTOR_STATE_CALIBRATE_RAMP_UP ? PWM_UPDATE_INTERVAL_MS : PWM_UPDATE_INTERVAL_DOWN_MS)) {
          if (motor3_state == MOTOR_STATE_RAMP_UP) {
              set_motor3_pwm(motor3_freq);
              motor3_freq += FREQ_STEP_UP;
              if (motor3_freq >= motor3_end_freq) {
                  motor3_freq = motor3_end_freq;
                  motor3_state = MOTOR_STATE_ON;
              }
          } else if (motor3_state == MOTOR_STATE_CALIBRATE_RAMP_UP) {
              set_motor3_pwm(motor3_freq);
              motor3_freq += FREQ_STEP_UP;
              if (motor3_freq >= motor3_end_freq) {
                  motor3_freq = motor3_end_freq;
                  motor3_state = MOTOR_STATE_CALIBRATING;
              }
          } else if (motor3_state == MOTOR_STATE_CALIBRATING) {
        	  if (now - motor3_calibration_start_time >= 60000) {
        		  motor3_state = MOTOR_STATE_RAMP_DOWN;
        		  char msg[] = "Calibration for Motor 3 is done\n";
        		  HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 100);
        	  }
          } else if (motor3_state == MOTOR_STATE_RAMP_DOWN) {
              set_motor3_pwm(motor3_freq);
              motor3_freq -= FREQ_STEP_DOWN;
              if (motor3_freq <= START_FREQ) {
                  motor3_freq = START_FREQ;
                  motor3_state = MOTOR_STATE_OFF;
                  HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
              }
          }
          motor3_last_update = now;
      }

      // Shared timer state machine (motors 2 and 4)
      if (now - shared_last_update >= ((motor2_state == MOTOR_STATE_RAMP_UP || motor4_state == MOTOR_STATE_RAMP_UP || motor2_state == MOTOR_STATE_CALIBRATE_RAMP_UP || motor4_state == MOTOR_STATE_CALIBRATE_RAMP_UP) ? PWM_UPDATE_INTERVAL_MS : PWM_UPDATE_INTERVAL_DOWN_MS)) {
          if (motor2_state == MOTOR_STATE_RAMP_UP || motor4_state == MOTOR_STATE_RAMP_UP) {
              set_shared_pwm(shared_freq);
              shared_freq += FREQ_STEP_UP;
              if (shared_freq >= motor2_end_freq && motor2_state == MOTOR_STATE_RAMP_UP) {
                  shared_freq = motor2_end_freq;
                  motor2_state = MOTOR_STATE_ON;
              }
              if (shared_freq >= motor4_end_freq && motor4_state == MOTOR_STATE_RAMP_UP) {
                  shared_freq = motor4_end_freq;
                  motor4_state = MOTOR_STATE_ON;
              }
          } else if (motor2_state == MOTOR_STATE_CALIBRATE_RAMP_UP || motor4_state == MOTOR_STATE_CALIBRATE_RAMP_UP) {
              set_shared_pwm(shared_freq);
              shared_freq += FREQ_STEP_UP;
              if (shared_freq >= motor2_end_freq && motor2_state == MOTOR_STATE_CALIBRATE_RAMP_UP) {
                  shared_freq = motor2_end_freq;
                  motor2_state = MOTOR_STATE_CALIBRATING;
              }
              if (shared_freq >= motor4_end_freq && motor4_state == MOTOR_STATE_CALIBRATE_RAMP_UP) {
                  shared_freq = motor4_end_freq;
                  motor4_state = MOTOR_STATE_CALIBRATING;
              }
          } else if (motor2_state == MOTOR_STATE_CALIBRATING) {
        	  if (now - motor2_calibration_start_time >= 60000) {
        		  motor2_state = MOTOR_STATE_RAMP_DOWN;
        		  char msg[] = "Calibration for Motor 2 is done\n";
        		  HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 100);
        	  }
          } else if (motor4_state == MOTOR_STATE_CALIBRATING) {
        	  if (now - motor4_calibration_start_time >= 60000) {
        		  motor4_state = MOTOR_STATE_RAMP_DOWN;
        		  char msg[] = "Calibration for Motor 4 is done\n";
        		  HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 100);
        	  }
          } else if (motor2_state == MOTOR_STATE_RAMP_DOWN || motor4_state == MOTOR_STATE_RAMP_DOWN) {
              set_shared_pwm(shared_freq);
              shared_freq -= FREQ_STEP_DOWN;
              if (shared_freq <= START_FREQ) {
                  shared_freq = START_FREQ;
                  if(motor2_state == MOTOR_STATE_RAMP_DOWN) {
                      motor2_state = MOTOR_STATE_OFF;
                      HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
                  }
                  if(motor4_state == MOTOR_STATE_RAMP_DOWN) {
                      motor4_state = MOTOR_STATE_OFF;
                      HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_4);
                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
                  }
              }
          }
          shared_last_update = now;
      }


      HAL_Delay(1);
  }


  }



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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


static void MX_TIM2_Init(void)
{



  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};


  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

static void MX_TIM5_Init(void)
{


  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};


  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);

}


static void MX_TIM9_Init(void)
{



  TIM_OC_InitTypeDef sConfigOC = {0};

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 83;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim9);

}


static void MX_USART6_UART_Init(void)
{


  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

}


static void MX_DMA_Init(void)
{

  __HAL_RCC_DMA2_CLK_ENABLE();


  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


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
#endif
