/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    PULSE_RESULT_NONE,      // Still waiting for a pulse or timeout
    PULSE_RESULT_DETECTED,  // Pulse was detected
    PULSE_RESULT_TIMEOUT    // Timeout without a pulse
} PulseResult;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PulseResult detectPulse(void);
// static bool isPulseDetectionActive = false;
// static uint32_t pulseStartTime = 0;
#define PULSE_DURATION 200        // ms
#define DELAY_LOGIC 200           // ms
#define PULSE_DURATION_LOGIC 200  // ms
#define MESSAGE_BIT_DURATION 200  // ms

typedef enum {
    STATE_IDLE,
    STATE_WAITING_ACK,
    STATE_RECEIVING,
    STATE_TRANSMITTING,
    STATE_COMPLETED
} STM_State;

char *messages[3][3] = {{"Hello", "Good Morning", "Good Night"},
                        {"Breakfast", "Lunch", "Dinner"},
                        {"Yes", "No", "Thanks"}};

STM_State currentState = STATE_IDLE;
bool messageReceived = false;
uint8_t receivedMessage = 0;
static uint8_t messageToSend = 0b1010;  // Sample message for right now to send
#define LCD_RS_PORT GPIOB
#define LCD_RS_PIN GPIO_PIN_5

#define LCD_E_PORT GPIOB
#define LCD_E_PIN GPIO_PIN_4

#define LCD_D4_PORT GPIOB
#define LCD_D4_PIN GPIO_PIN_10

#define LCD_D5_PORT GPIOA
#define LCD_D5_PIN GPIO_PIN_8

#define LCD_D6_PORT GPIOB
#define LCD_D6_PIN GPIO_PIN_9

#define LCD_D7_PORT GPIOB
#define LCD_D7_PIN GPIO_PIN_8

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendNibble(uint8_t nibble);
void LCD_PulseEnable(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
char readKeypad(void);
void println(const char *format, ...);
void sendPulse(uint32_t duration_ms);
bool waitForPulse(uint32_t timeout_ms);
void startMessageTransmission();
void listenForMessage();
void transmitMessage(uint8_t message);
uint8_t receiveMessage();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/**
 * @brief Transmits a formatted string or GPIO pin value over UART, followed by
 * a newline.
 * @param format: A format string (like printf).
 * @param ...: Variadic arguments for the format string.
 */
void println(const char *format, ...) {
    char msg[256];  // Buffer for the message
    va_list args;

    // Prepare the formatted string
    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args);
    va_end(args);

    // Append a newline sequence
    strncat(msg, "\r\n", sizeof(msg) - strlen(msg) - 1);

    // Transmit the message over UART
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
 * @brief Asynchronously detects a pulse on the input pin.
 * @note Call this function repeatedly in the main loop.
 * @retval PulseResult: The current state of pulse detection.
 */
PulseResult detectPulse(void) {
    if (HAL_GPIO_ReadPin(Receiver_GPIO_Port, Receiver_Pin) != GPIO_PIN_RESET) {
        //		println("No");
        HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    } else {
        // println("Yes");
        HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
    }

    // Still waiting for a pulse or timeout
    return PULSE_RESULT_NONE;
}

char readKeypad(void) {
    // Keypad Layout
    char keys[4][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {'*', '0', '#'}};

    // Row and Column Pins
    GPIO_TypeDef *rowPorts[4] = {GPIOA, GPIOA, GPIOA, GPIOB};
    uint16_t rowPins[4] = {R1_Pin, R2_Pin, R3_Pin, R4_Pin};

    GPIO_TypeDef *colPorts[3] = {GPIOC, GPIOC, GPIOA};
    uint16_t colPins[3] = {C1_Pin, C2_Pin, C3_Pin};

    // Scan Columns and Rows
    for (int col = 0; col < 3; col++) {
        // Set all columns HIGH
        for (int i = 0; i < 3; i++) {
            HAL_GPIO_WritePin(colPorts[i], colPins[i], GPIO_PIN_SET);
        }

        // Set current column LOW
        HAL_GPIO_WritePin(colPorts[col], colPins[col], GPIO_PIN_RESET);

        // Check rows
        for (int row = 0; row < 4; row++) {
            if (HAL_GPIO_ReadPin(rowPorts[row], rowPins[row]) ==
                GPIO_PIN_RESET) {
                // Debounce
                HAL_Delay(20);
                while (HAL_GPIO_ReadPin(rowPorts[row], rowPins[row]) ==
                       GPIO_PIN_RESET);
                HAL_Delay(20);

                // Return the pressed key
                return keys[row][col];
            }
        }
    }
    return 'x';
}

/**
 * @brief Sends a high pulse for the specified duration.
 * @param duration_ms Duration of the pulse in milliseconds.
 */
void sendPulse(uint32_t duration_ms) {
    HAL_GPIO_WritePin(Transmitter_GPIO_Port, Transmitter_Pin, GPIO_PIN_SET);
    HAL_Delay(duration_ms);
    HAL_GPIO_WritePin(Transmitter_GPIO_Port, Transmitter_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Waits for a pulse on the receiver pin.
 * @param timeout_ms Timeout duration in milliseconds.
 * @retval true if pulse detected, false otherwise.
 */
bool waitForPulse(uint32_t timeout_ms) {
    uint32_t startTime = HAL_GetTick();
    while ((HAL_GetTick() - startTime) < timeout_ms) {
        if (HAL_GPIO_ReadPin(Receiver_GPIO_Port, Receiver_Pin) ==
            GPIO_PIN_RESET) {
            // HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
            // HAL_Delay(10);
            // // println("Pulse detected");
            // HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
            return true;
        }
    }
    return false;
}
// Tracks the last pulse state received
static bool pulseReceived = false;
static bool lastPulseReceived = false;

/**
 * @brief Waits for a pulse with a timeout.
 * @param timeout Duration to wait for a pulse in milliseconds.
 * @retval true if a pulse was detected, false if timeout occurred.
 */
bool waitForPulseWithTimeout(uint32_t timeout) {
    uint32_t startTime = HAL_GetTick();
    while (HAL_GetTick() - startTime < timeout) {
        if (waitForPulse(1)) return true;
    }
    return false;
}

/**
 * @brief Starts the message transmission sequence.
 */
void startMessageTransmission() {
    sendPulse(MESSAGE_BIT_DURATION);
    HAL_Delay(DELAY_LOGIC);

    for (int i = 0; i < 4; i++) {
        if (messageToSend & (1 << i)) {
            sendPulse(MESSAGE_BIT_DURATION);
        } else {
            HAL_Delay(DELAY_LOGIC);
        }
        HAL_Delay(DELAY_LOGIC);
    }
}

/**
 * @brief Listens for a start pulse and processes a received message.
 */
void listenForMessage() {
    bool pulseReceived = waitForPulse(1);
    if (pulseReceived && !lastPulseReceived) {
        HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SignalLed_GPIO_Port, SignalLed_Pin, GPIO_PIN_SET);
    }
    if (!pulseReceived && lastPulseReceived) {
        uint8_t message = 0;
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_Print("Receiving...");
        for (int i = 0; i < 4; i++) {
            if (waitForPulse(MESSAGE_BIT_DURATION)) {
                message |= (1 << i);
            }
            HAL_Delay(DELAY_LOGIC);
        }
        receivedMessage = message;
        messageReceived = true;
    }
    lastPulseReceived = pulseReceived;
}

/**
 * @brief Initializes the LCD with in 4-bit mode.
 */
void LCD_Init(void) {
    HAL_Delay(20);  // Wait for LCD to power up

    // Function Set: 8-bit mode (3 times) for initialization
    LCD_SendNibble(0x3);
    LCD_PulseEnable();
    HAL_Delay(5);

    LCD_SendNibble(0x3);
    LCD_PulseEnable();
    HAL_Delay(1);

    LCD_SendNibble(0x3);
    LCD_PulseEnable();
    HAL_Delay(1);

    // Function Set: 4-bit mode
    LCD_SendNibble(0x2);
    LCD_PulseEnable();
    HAL_Delay(1);

    // Function Set: 4-bit, 2 lines, 5x8 dots
    LCD_SendCommand(0x28);

    // Display ON/OFF Control: Display on, cursor off, blink off
    LCD_SendCommand(0x0C);

    // Clear Display
    LCD_SendCommand(0x01);
    HAL_Delay(2);

    // Entry Mode Set: Increment cursor
    LCD_SendCommand(0x06);
}

/**
 * @brief Sends a command to the LCD.
 */
void LCD_SendCommand(uint8_t cmd) {
    // RS = 0 for command
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);

    // Send higher nibble
    LCD_SendNibble((cmd >> 4) & 0x0F);
    LCD_PulseEnable();

    // Send lower nibble
    LCD_SendNibble(cmd & 0x0F);
    LCD_PulseEnable();

    HAL_Delay(2);  // Wait for command to execute
}

/**
 * @brief Sends data to the LCD.
 */
void LCD_SendData(uint8_t data) {
    // RS = 1 for data
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);

    // Send higher nibble
    LCD_SendNibble((data >> 4) & 0x0F);
    LCD_PulseEnable();

    // Send lower nibble
    LCD_SendNibble(data & 0x0F);
    LCD_PulseEnable();

    HAL_Delay(2);  // Wait for data to be written
}

/**
 * @brief Sends a nibble to the LCD.
 */
void LCD_SendNibble(uint8_t nibble) {
    // Clear all data pins first
    HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, GPIO_PIN_RESET);

    // Set D4 (PB10) - Bit 0
    if (nibble & 0x1) HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, GPIO_PIN_SET);

    // Set D5 (PA8) - Bit 1
    if (nibble & 0x2) HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, GPIO_PIN_SET);

    // Set D6 (PB9) - Bit 2
    if (nibble & 0x4) HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, GPIO_PIN_SET);

    // Set D7 (PB8) - Bit 3
    if (nibble & 0x8) HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, GPIO_PIN_SET);
}

/**
 * @brief Generates a pulse on the E pin of the LCD.
 */
void LCD_PulseEnable(void) {
    // Generate a pulse on the E pin
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);  // Enable pulse must be > 450ns
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);  // Commands need > 37us to settle
}

/**
 * @brief Clears the LCD display.
 */
void LCD_Clear(void) {
    LCD_SendCommand(0x01);  // Clear display command
    HAL_Delay(2);           // Wait for clear to execute
}

/**
 * @brief Sets the cursor position on the LCD.
 */
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address;

    // Calculate DDRAM address based on row and column
    if (row == 0)
        address = 0x00 + col;
    else
        address = 0x40 + col;

    LCD_SendCommand(0x80 | address);
}

/**
 * @brief Prints a character on the LCD.
 */
void LCD_Print(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
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
    LCD_Init();

    // Clear the LCD
    LCD_Clear();

    // Set cursor to first line, first position
    LCD_SetCursor(0, 0);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        int key = readKeypad() - 1;
        if (key != 'x' - 1) {
            if (key >= 0 && key < 9) {
                LCD_Clear();
                LCD_SetCursor(0, 0);
                LCD_Print("Sending: ");
                LCD_SetCursor(1, 0);
                LCD_Print(messages[key / 3][key % 3]);

                messageToSend = key;
                startMessageTransmission();
            }
        }

        // Continuously check for incoming messages
        listenForMessage();
        if (messageReceived) {
            // Display the received message on the LCD
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Received: ");
            LCD_SetCursor(1, 0);
            LCD_Print(messages[receivedMessage / 3][receivedMessage % 3]);
            messageReceived = false;  // Reset the flag
            HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SignalLed_GPIO_Port, SignalLed_Pin,
                              GPIO_PIN_RESET);
        }
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
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
    if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, C2_Pin | C1_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin | D5_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, Transmitter_Pin | C3_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(
        GPIOB,
        D4_Pin | LED_Pin | E_Pin | RS_Pin | SignalLed_Pin | D7_Pin | D6_Pin,
        GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : C2_Pin C1_Pin */
    GPIO_InitStruct.Pin = C2_Pin | C1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin */
    GPIO_InitStruct.Pin = R1_Pin | R2_Pin | R3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LD2_Pin C3_Pin */
    GPIO_InitStruct.Pin = LD2_Pin | C3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : Transmitter_Pin */
    GPIO_InitStruct.Pin = Transmitter_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(Transmitter_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : Receiver_Pin */
    GPIO_InitStruct.Pin = Receiver_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Receiver_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : R4_Pin */
    GPIO_InitStruct.Pin = R4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(R4_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : D4_Pin RS_Pin D7_Pin D6_Pin */
    GPIO_InitStruct.Pin = D4_Pin | RS_Pin | D7_Pin | D6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : Buzzer_Pin */
    GPIO_InitStruct.Pin = Buzzer_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : D5_Pin */
    GPIO_InitStruct.Pin = D5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(D5_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_Pin E_Pin SignalLed_Pin */
    GPIO_InitStruct.Pin = LED_Pin | E_Pin | SignalLed_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
