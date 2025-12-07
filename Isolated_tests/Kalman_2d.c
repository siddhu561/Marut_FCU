/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - Altitude Estimation with 2D Kalman Filter
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "mpu6050.h"
#include "bmp280.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Kalman filter 2D parameters
float TS = 0.004f;           // Sampling time (250 Hz)
float sd_baro = 0.20f;       // Barometer standard deviation (m)
float sd_accZ = 2.0f;        // AccZ standard deviation (m/s²)

// Calibration offsets
float accZ_offset = 0.0f;    // AccZInertial calibration offset
float baro_offset = 0.0f;    // Barometer ground level offset
float ax_offset = 0.0f;      // Accelerometer X offset
float ay_offset = 0.0f;      // Accelerometer Y offset
float az_offset = 0.0f;      // Accelerometer Z offset

// Kalman outputs
float AltitudeKalman = 0.0f;
float VelocityVerticalKalman = 0.0f;

// Internal state
float S_alt = 0.0f;
float S_vel = 0.0f;

// Covariance matrix
float P11 = 10.0f;
float P12 = 0.0f;
float P21 = 0.0f;
float P22 = 10.0f;

// Process noise covariance
float Q11, Q12, Q21, Q22;

// Measurement noise covariance
float R;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
    return ch;
}

/**
 * @brief  Transform accelerometer to inertial frame and remove gravity
 * @param  AccX: X-axis acceleration (g)
 * @param  AccY: Y-axis acceleration (g)
 * @param  AccZ: Z-axis acceleration (g)
 * @param  AngleRoll: Roll angle (degrees)
 * @param  AnglePitch: Pitch angle (degrees)
 * @retval Vertical acceleration in inertial frame (m/s²)
 */
float getAccZInertial(float AccX, float AccY, float AccZ, float AngleRoll,
                     float AnglePitch) {
    float roll = AngleRoll * 0.0174533f;
    float pitch = AnglePitch * 0.0174533f;

    // Rotation matrix: body frame to inertial frame
    float AccZ_world = -sinf(pitch) * AccX
                     + cosf(pitch) * sinf(roll) * AccY
                     + cosf(pitch) * cosf(roll) * AccZ;

    // Remove gravity (1g) and convert to m/s²
    float AccZInertial = (AccZ_world - 1.0f) * 9.81f;

    return AccZInertial;
}

/**
 * @brief  Initialize Kalman filter and calibrate all sensors
 * @retval None
 */
void initKalman2D(void) {
    printf("\r\n=== Altitude Estimation System ===\r\n");
    printf("Calibrating sensors... Keep sensor stationary!\r\n");
    HAL_Delay(1000);

    // Calibrate accelerometer offsets
    printf("Calibrating accelerometer...\r\n");
    ax_offset = mpu_accel_calibration(0);
    ay_offset = mpu_accel_calibration(1);
    az_offset = mpu_accel_calibration(2);

    printf("Accel offsets: Ax=%.3fg, Ay=%.3fg, Az=%.3fg\r\n",
           ax_offset, ay_offset, az_offset);

    // Calibrate barometer offset (set ground level as reference)
    printf("Calibrating barometer (ground level)...\r\n");
    float baro_sum = 0.0f;
    int baro_samples = 100;

    for (int i = 0; i < baro_samples; i++) {
        float alt = altitude_calc();
        baro_sum += alt;
        HAL_Delay(10);
    }

    baro_offset = baro_sum / (float)baro_samples;
    printf("Barometer ground level: %.3f m (offset applied)\r\n", baro_offset);

    // Calibrate AccZInertial offset
    printf("Calibrating AccZInertial offset...\r\n");
    float accz_sum = 0.0f;
    int calib_samples = 200;

    for (int i = 0; i < calib_samples; i++) {
        float Ax = mpu_accel_read_raw(0) - ax_offset;
        float Ay = mpu_accel_read_raw(1) - ay_offset;
        float Az = mpu_accel_read_raw(2) - az_offset;
        float roll = mpu_accel_read(0);
        float pitch = mpu_accel_read(1);

        float acc = getAccZInertial(Ax, Ay, Az, roll, pitch);
        accz_sum += acc;
        HAL_Delay(5);
    }

    accZ_offset = accz_sum / (float)calib_samples;
    printf("AccZInertial offset: %.3f m/s²\r\n", accZ_offset);

    // Initialize Kalman filter noise matrices
    float factor = sd_accZ * sd_accZ;  // (m/s²)²
    float G0 = 0.5f * TS * TS;         // s²
    float G1 = TS;                     // s

    Q11 = G0 * G0 * factor;  // m²
    Q12 = G0 * G1 * factor;  // m²
    Q21 = G1 * G0 * factor;  // m²
    Q22 = G1 * G1 * factor;  // (m/s)²

    R = sd_baro * sd_baro;   // m²

    printf("Calibration complete! Starting altitude estimation...\r\n\r\n");
    HAL_Delay(500);
}

/**
 * @brief  Execute one Kalman filter update step
 * @param  acc: Vertical acceleration (m/s²)
 * @param  meas: Altitude measurement from barometer (m)
 * @retval None
 */
void kalman_2d_step(float acc, float meas) {
    // Prediction step: predict current state
    float vel_pred = S_vel + acc * TS;
    float alt_pred = S_alt + S_vel * TS + 0.5f * TS * TS * acc;

    // Covariance prediction: P = F*P*F' + Q
    float P11p = P11 + TS * (P12 + P21) + TS * TS * P22 + Q11;
    float P12p = P12 + TS * P22 + Q12;
    float P21p = P21 + TS * P22 + Q21;
    float P22p = P22 + Q22;

    // Innovation covariance: L = H*P*H' + R
    float L = P11p + R;
    if (L <= 1e-12f)
        L = 1e-12f;  // Guard against division by zero

    // Kalman gains: K = P*H' / L
    float K0 = P11p / L;   // Gain for altitude
    float K1 = P21p / L;   // Gain for velocity

    // Update step: innovation and state correction
    float innov = meas - alt_pred;
    S_alt = alt_pred + K0 * innov;
    S_vel = vel_pred + K1 * innov;

    // Covariance update: P = (I - K*H) * P
    // Where (I - K*H) = [[1-K0, 0], [-K1, 1]]
    float newP11 = (1.0f - K0) * P11p;
    float newP12 = (1.0f - K0) * P12p;
    float newP21 = -K1 * P11p + P21p;
    float newP22 = -K1 * P12p + P22p;

    P11 = newP11;
    P12 = newP12;
    P21 = newP21;
    P22 = newP22;

    // Zero-velocity update when stationary
    static float prev_alt = 0.0f;
    float alt_change = fabsf(S_alt - prev_alt);

    if (alt_change < 0.02f && fabsf(acc) < 0.5f) {  // Stationary threshold
        S_vel *= 0.95f;  // Gradually dampen velocity to zero
    }

    prev_alt = S_alt;

    // Store outputs
    AltitudeKalman = S_alt;
    VelocityVerticalKalman = S_vel;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
    MX_USART1_UART_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */

    // Initialize sensors
    mpu_init();
    bmp_i2c_setup();

    HAL_Delay(500);  // Let sensors stabilize

    // Initialize Kalman filter (includes all calibrations)
    initKalman2D();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
    	int i = 0;
        // Read sensor data
        float Ax = mpu_accel_read_raw(0) - ax_offset;
        float Ay = mpu_accel_read_raw(1) - ay_offset;
        float Az = mpu_accel_read_raw(2) - az_offset;
        float roll = mpu_accel_read(0);
        float pitch = mpu_accel_read(1);
        float altitude = altitude_calc() - baro_offset;  // Apply barometer offset

        // Transform to inertial frame and apply offset calibration
        float acc = getAccZInertial(Ax, Ay, Az, roll, pitch) - accZ_offset;

        // Update Kalman filter
        kalman_2d_step(acc, altitude);

        // Output results
        printf("Alt: %.2fm | Vel: %.2fm/s \r\n",
               AltitudeKalman, VelocityVerticalKalman);
        i++;
        if (i>100){
			i=0;
			AltitudeKalman -=20.0f;
			printf("\r\n");
		}

        HAL_Delay(4);  // 250 Hz update rate (TS = 0.004s)

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

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
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
    /* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
