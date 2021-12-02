/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * POPUP Robot Main Controller
  * @Author: Francesco Gambino
  * @Year: 2021
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <retarget.h> // Fprintf sulla seriale
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// KINEMATIC VARIABLES
#define L1 0.745
#define L2 0.685
#define OFF 0.07
#define L1square 0.555025
#define L2square 0.469225
#define OFFsquare 0.0049

//ACTUATOR LIMITS
#define P_MAX 12.5 // posizione
#define V_MAX 6 // velocità
#define T_MAX 48 // coppia
#define Kp_MAX 500 // non vengono utilizzati, sono posti a zero. Sono relativi al controllo del firmware del driver dei motori
#define Kd_MAX 1000
// CAN ID
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR_FB 0 // Nel pacchetto di feedback, ID unico per tutti i motori, è presente una stringa che dichiara da quale motore proviene il feedback

#define LINK_BOARD_1 10
#define LINK_BOARD_2 11
#define LINK_BOARD_FB_MSG_ID 0x6 // ID definito in esadecimale
#define LINK_BOARD_STATUS_MSG_ID 0x14 // ID definito in esadecimale
#define LINK_BOARD_CALIBRATION_CHECK_MSG_ID 0x28 // ID definito in esadecimale
// Gli status, feedback, calibration hanno un unico ID poichè all'interno una stringa mi indica a quale Board mi riferisco




#define MOTOR_FB_DEBUG 0
int LINK_FB_DEBUG = 0;
#define TRAJ_PLAN_DEBUG 0
#define POS_CONTROLLER_DEBUG 0
#define SPEED_CONTROLLER_DEBUG 0


//CONTROLLER DEFINEs
#define MAX_INTEGRAL_ERROR 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000 // Creato dal software (non toccare)
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
// Fatto da Francesco!
// CAN PARAMETERS AND BUFFERS
FDCAN_TxHeaderTypeDef pTxHeader; //Nome della variabile,dettagliata in seguito
FDCAN_FilterTypeDef sFilterConfig; //Nome della variabile, dettagliata in seguito
FDCAN_RxHeaderTypeDef pRxHeader; //Nome della variabile, dettagliata in seguito

uint8_t CAN_tx_buffer[8]; // Buffer per la trasmissione, 8 elementi ( da 0 a 7)
uint8_t CAN_rx_buffer[8]; // Buffer per la ricezione


//TARGET VARIABLES
float Cartesian_target[3]; // Vettore composto da 3 elementi
float Joint_target[3];
float Joint_speed_target[3];

// ACTUATORS FEEDBACK VARIABLES
float actualPos[3];
float actualSpeed[3];
float actualCurr[3]; 

// TRAJECTORY PLANNER BUFFER
float Joint_target_plan[2000][3]; //matrix of Ns rows and 3 column
float Joint_speed_target_plan[2000][3];
int plan_counter = 0;
int flag_traj_compl = 0;


// POSITION CONTROLLER VARIABLES
float Joint_target_planned[3]; //here the trajector planner will update the ref point for the position controllers every Ts
float pos_KP[3];
float pos_KI[3];
float pos_KD[3];
float pos_error[3];
float pos_integral[3];
float pos_derivative[3];
float pos_previous_error[3];
float pos_command[3];
volatile int flag_pos_controller = 0;

//  SPEED CONTROLLER VARIABLES
float speed_KP[3];
float speed_KI[3];
float speed_KD[3];
float speed_error[3];
float speed_integral[3];
float speed_derivative[3];
float speed_previous_error[3];
float speed_command[3];
volatile int flag_speed_controller = 0;

// LINK BENDING FEEDBACK VARAIBLES
float actual_horiz_bend = 0; //In futuro occorre creare un vettore di 2 elementi essendoci due 2 link
float actual_vert_bend = 0;

// STATUS VARIABLES
int motor_status_flag[3];
int link_status_flag[2]; //Status: verifico la presenza dell'elemento (assume valori di 0 o 1)
int link_cal_check[2]; // Variabile che uso per verificare la calibrazione (assume valori di 0 o 1)

// VESC CONTROL VARIABLE
float Joint_speed_target_planned[3]; //here the trajector planner will update the ref point for the position controllers every Ts



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
int float_to_uint(float x, float x_min , float x_max, unsigned int bits);
float uint_to_float( int x_int , float x_min , float x_max , int bits);
// Float sono i numeri a virgola mobile

void CAN_RxFilter_Config();
void CAN_TxHeader_Config();

void ActivateMotor(int id);
void SendTorque(int id, float u);

void unpack_motor_FB();
void unpack_link_FB();
void unpack_link_STATUS();
void unpack_link_CAL_CHECK();

void InverseKinematic(float EE_target[3], int Mode);
void TrajectorPlanner(float q0[3], float qf[3], float t);
void BendingCorrection();

void PositionController();
void SpeedController();


//veryfy system connectivity (link + sensors)
void POPUP_system_check();
//verify link status (pressure reading)
void POPUP_link_status_check();
//activate motors
void POPUP_activate_motors();
//start controllers
void POPUP_start_controllers();

void POPUP_start_plan();

//homing
void POPUP_homing();
//send calibration command to link boards
void POPUP_calibrate_link_sensors();

void CAN_TX_vesc_speed(float speed);
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
  MX_ETH_Init();
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart3);

  printf("POPUP Robot Main Controller V0.1\n");


  printf("CAN Register Configuring... \n");

   CAN_RxFilter_Config();
   CAN_TxHeader_Config();

   if (   (HAL_FDCAN_Start(& hfdcan1)) == HAL_OK)   // l'ultimo argomento non ci interessa se usiamo la FIFO
 	  printf("CAN PHY started\n");
   else {
	  printf("CAN PHY initialization error\n");
	  //while(1);

   }
   //HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
   if ( HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK)   // l'ultimo argomento non ci interessa se usiamo la FIFO
 	  printf("CAN Configuring: DONE\n\n\n");


   HAL_Delay(2000);
   ////////////////////////////////////////////////////
   //assign controllers parameter

   //joint 1 position PID controller gain
	 pos_KP[0] = 5;
	 pos_KI[0] = 0;
	 pos_KD[0] = 0;

	//joint 2 position PID controller gain
	 pos_KP[1] = 12;
	 pos_KI[1] = 0;
	 pos_KD[1] = 0;

	//joint 3 position PID controller gain
	 pos_KP[2] = 12;
	 pos_KI[2] = 0;
	 pos_KD[2] = 0;

	//joint 1 speed PID controller gain
	 speed_KP[0] = 0;
	 speed_KI[0] = 0;
	 speed_KD[0] = 0;

	//joint 2 speed PID controller gain
	 speed_KP[1] = 8;
	 speed_KI[1] = 0;
	 speed_KD[1] = 0;
	 //joint 3 speed PID controller gain
	 speed_KP[2] = 8;
	 speed_KI[2] = 0;
	 speed_KD[2] = 0;

	/////////////////////////////////////
  //veryfy system status (link + sensors)
	POPUP_system_check();
 //send calibration command to link boards
	POPUP_calibrate_link_sensors(11);


  //activate motors
	POPUP_activate_motors();
  //start controllers
  //homing
	 //POPUP_
     //POPUP_homing();
  //start loop

	 printf("\nSystem Initialized\n");


  Cartesian_target[0] = 0.7; //X
  Cartesian_target[1] = -0.7; //Y
  Cartesian_target[2] = 0.8; //Z

  printf("Target = [ X: %f   Y: %f   Z: %f ]\n",Cartesian_target[0],Cartesian_target[1],Cartesian_target[2]);

  InverseKinematic(Cartesian_target, 1); //Target[3] , Mode (elbow up,down)

  printf("Joint Target = [ q1: %f   q2: %f   q3: %f ]\n",Joint_target[0],Joint_target[1],Joint_target[2]);
  printf("Actual Joint position = [ q1: %f   q2: %f  q3: %f ]\n",actualPos[0],actualPos[1],actualPos[2]);
  Joint_target[2] = -Joint_target[2];
  printf("\nStarting trajectory planner...\n");

  TrajectorPlanner(actualPos, Joint_target, 10);

  printf("\nStarting trajectory planning completed.\n");


  if(TRAJ_PLAN_DEBUG) {
	  int i,j;
	  for(i = 0; i < 3; i++) {
		  for(j = 0; j < 101; j++) {
			  printf("T: %f q%d: %f  qd%d: %f\n",(j*0.1),i,Joint_target_plan[j][i],i,Joint_speed_target_plan[j][i]); //matrix of Ns rows and 3 column
		  	  }
		  printf("\n\n");
	  	  }
  }

  //assign the first planned variable to the position controller input
  for(int m = 0; m<3; m++) {
	  Joint_target_planned[m] = Joint_target_plan[0][m];
  }


  //start controllers
  POPUP_start_controllers();

  HAL_Delay(1000);

  //start planned movement
  POPUP_start_plan();

  //start printing data from link sensors
  LINK_FB_DEBUG = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if(flag_pos_controller  == 1)
		 PositionController();

	if(flag_speed_controller == 1) {
		SpeedController();
	  	 CAN_TX_vesc_speed(-Joint_speed_target_planned[0]);
	}

	//if first traj is completed start with second pianification
	if(flag_traj_compl == 1) {

	}

	  //printf("VESC target speed: %f\n",Joint_speed_target_planned[0]);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 1;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 1;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 120;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 120;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1200 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000 - 1;
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void CAN_RxFilter_Config(void)
{
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0 ;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;  // mask => allow id from 0x00000150 to 0x0000015F

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  //HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

}

void CAN_TxHeader_Config(void)
{
 pTxHeader.Identifier = 0x00000140;
 pTxHeader.IdType = FDCAN_EXTENDED_ID;                // specifies extended id
 pTxHeader.TxFrameType = FDCAN_DATA_FRAME ;           // frame type
 pTxHeader.DataLength = FDCAN_DLC_BYTES_4;            // specifies frame length
 pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
 pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
 pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
 pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
 pTxHeader.MessageMarker = 0;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0, &pRxHeader, CAN_rx_buffer) == HAL_OK) {
		}

	if((pRxHeader.Identifier == 0)) //feedback from motor (position, speed)
		      {
		unpack_motor_FB();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // yellow led

		      }

	if((pRxHeader.Identifier == LINK_BOARD_FB_MSG_ID)) //feedback from link data board (bending estimate)
		      {
		unpack_link_FB();
		HAL_GPIO_TogglePin(GPIOB, LD1_Pin); // green led

		      }
	if((pRxHeader.Identifier == LINK_BOARD_STATUS_MSG_ID)) //status message with id 20 (DEC)
		      {
		unpack_link_STATUS();
		      }
	if((pRxHeader.Identifier == LINK_BOARD_CALIBRATION_CHECK_MSG_ID)) //status message with id 20 (DEC)
		      {
		unpack_link_CAL_CHECK();
		      }
	if((pRxHeader.Identifier == 2305)) //status message from vesc
		      {
		unpack_vesc_FB();
		      }
	//    for(int i = 0; i < pRxHeader.DataLength; i++)
	//  printf("CAN_BUFF_RX[%d]: %d\n",i,CAN_rx_buffer[i]);

	}
}

void unpack_vesc_FB() {

	  uint32_t rpm = ((CAN_rx_buffer[0] << 24) +(CAN_rx_buffer[1] << 16) + (CAN_rx_buffer[2] << 8) + CAN_rx_buffer[3])/14;

	  float mot1_speed = (float)(rpm/1120.0); //1120 = 14(pole-pair motor)*80 (gearbox reduction)

	  //printf("Speed : %f rad/s \n",mot1_speed*0.10472 );

	  //printf("rpm : %d \n",rpm);

}

void unpack_motor_FB() {
  //if it is the first time execution for each motor i must set the status flag

  int motor_id = CAN_rx_buffer[0];

  int pos_int = CAN_rx_buffer[1] << 8 | CAN_rx_buffer[2];

  int vel_int = CAN_rx_buffer[3] << 4 | CAN_rx_buffer[4] >> 4;

  int current_int = (CAN_rx_buffer[4] & 0xF) << 8 | CAN_rx_buffer[5];


  actualPos[motor_id -1] = uint_to_float(pos_int, -P_MAX, P_MAX, 16);

  actualSpeed[motor_id -1] = uint_to_float(vel_int, -V_MAX, V_MAX, 12);

  actualCurr[motor_id -1] = uint_to_float(current_int, -T_MAX , T_MAX, 12);


  if(motor_id == MOTOR1 && motor_status_flag[0] == 0)
	  motor_status_flag[0] = 1;

  if(motor_id == MOTOR2 && motor_status_flag[1] == 0)
	  motor_status_flag[1] = 1;

  if(motor_id == MOTOR3 && motor_status_flag[2] == 0)
	  motor_status_flag[2] = 1;


  if (MOTOR_FB_DEBUG) {
    printf("CAN_ID: %2d - Pos[rad]: %8f - Vel[rad/s]: %10f - Torque[Nm]: %8f \n",motor_id,actualPos[motor_id],actualSpeed[motor_id],actualCurr[motor_id]);
  	  }
}

void unpack_link_FB() {

  int link_id = CAN_rx_buffer[0];

  int bend_horiz_int = CAN_rx_buffer[1] << 8 | CAN_rx_buffer[2]; //16 bit

  int bend_vert_int = CAN_rx_buffer[3] << 8 | CAN_rx_buffer[4];

actual_horiz_bend = uint_to_float(bend_horiz_int, -90, 90, 16);

actual_vert_bend =  uint_to_float(bend_vert_int, -90, 90, 16);

link_status_flag[link_id - 10] = CAN_rx_buffer[5]; //

  if (LINK_FB_DEBUG) {
    printf("%2.1f\n",link_id - 9,actual_horiz_bend,actual_vert_bend);

  }
}

void unpack_link_STATUS() {

	  int link_id = CAN_rx_buffer[0];

	  int imu_status = CAN_rx_buffer[1];

	  int adc_status = CAN_rx_buffer[2];

	  int pressure_int = CAN_rx_buffer[3] << 8 | CAN_rx_buffer[4];

	  float pressure =  uint_to_float(pressure_int, 0, 2, 16);

	link_status_flag[link_id - 10] = imu_status*adc_status; //

	  if (LINK_FB_DEBUG) {
	    printf("LINK_ID: %d - Link Pressure: %1.2f - IMU_Status: %d - ADC_Status: %d - System Status: &d \n",link_id,pressure,imu_status,adc_status,link_status_flag[link_id - 10]);
	  }

}

void unpack_link_CAL_CHECK() {


	  int link_id = CAN_rx_buffer[0];

	  int cal_check = CAN_rx_buffer[1];

	link_cal_check[link_id - 10] = cal_check; //

	  if (LINK_FB_DEBUG) {
	    printf("LINK_ID: %d - Calibration Status: &d \n",link_id,link_cal_check[link_id - 10]);
	  }

}

void CAN_TX_link_board_status_check(int id) {
	     pTxHeader.Identifier = 0x30;							  //status check message id
		 pTxHeader.IdType = FDCAN_STANDARD_ID;                // specifies extended id
		 pTxHeader.TxFrameType = FDCAN_DATA_FRAME ;           // frame type
		 pTxHeader.DataLength = FDCAN_DLC_BYTES_2;

		 CAN_tx_buffer[0] = id;
		 CAN_tx_buffer[1] = 1;

 		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, CAN_tx_buffer) == HAL_OK) {
 			  //HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
 		  }

}

void CAN_TX_link_board_calibration(int id) {
	     pTxHeader.Identifier = 0x32;							  //status check message id
		 pTxHeader.IdType = FDCAN_STANDARD_ID;                // specifies extended id
		 pTxHeader.TxFrameType = FDCAN_DATA_FRAME ;           // frame type
		 pTxHeader.DataLength = FDCAN_DLC_BYTES_2;

		 CAN_tx_buffer[0] = id;
		 CAN_tx_buffer[1] = 1;

 		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, CAN_tx_buffer) == HAL_OK) {
 			  //HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
 		  }

}

void CAN_TX_vesc_speed(float speed) {
	//vesc id is 1

	float s = speed;

	 pTxHeader.IdType = FDCAN_EXTENDED_ID;                // specifies extended id

    pTxHeader.Identifier = 0x00000301;
    //status check message id
	 pTxHeader.TxFrameType = FDCAN_DATA_FRAME ;           // frame type
	 pTxHeader.DataLength = FDCAN_DLC_BYTES_4;

	 //take speed argument (rad/s)
	 //convert to ERPM
	 int erpm =(int)((s*9.5463*21.0*80.0)); //(rad/s)*(RPM_con)*(pole-pair)*(reduction);


		 CAN_tx_buffer[0] = erpm >> 24;
		 CAN_tx_buffer[1] = erpm >> 16;
		 CAN_tx_buffer[2] = erpm >> 8;
		 CAN_tx_buffer[3] = erpm;


	 if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, CAN_tx_buffer) == HAL_OK) {
		  //HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
	  }
}

void ActivateMotor(int id) {

	pTxHeader.Identifier = id;
	pTxHeader.IdType = FDCAN_STANDARD_ID;                // specifies extended id
	 pTxHeader.TxFrameType = FDCAN_DATA_FRAME ;           // frame type
	 pTxHeader.DataLength = FDCAN_DLC_BYTES_8;

		  		CAN_tx_buffer[0] = 0XFF;
		  		CAN_tx_buffer[1] = 0XFF;
		  		CAN_tx_buffer[2] = 0XFF;
		  		CAN_tx_buffer[3] = 0XFF;
		  		CAN_tx_buffer[4] = 0XFF;
		  		CAN_tx_buffer[5] = 0XFF;
		  		CAN_tx_buffer[6] = 0XFF;
		  		CAN_tx_buffer[7] = 0XFC;

		  		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, CAN_tx_buffer) == HAL_OK) {
		  			  //HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
		  			 // printf("Motor %lu Activated \n",pTxHeader.Identifier);
		  		  }
}

void DeactivateMotor(int id) {

	pTxHeader.Identifier = id;
	pTxHeader.IdType = FDCAN_STANDARD_ID;                // specifies extended id
	 pTxHeader.TxFrameType = FDCAN_DATA_FRAME ;           // frame type
	 pTxHeader.DataLength = FDCAN_DLC_BYTES_8;

		  		CAN_tx_buffer[0] = 0XFF;
		  		CAN_tx_buffer[1] = 0XFF;
		  		CAN_tx_buffer[2] = 0XFF;
		  		CAN_tx_buffer[3] = 0XFF;
		  		CAN_tx_buffer[4] = 0XFF;
		  		CAN_tx_buffer[5] = 0XFF;
		  		CAN_tx_buffer[6] = 0XFF;
		  		CAN_tx_buffer[7] = 0XFD;

		  		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, CAN_tx_buffer) == HAL_OK) {
		  			  //HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
		  			  //printf("Motor %lu Activated \n",pTxHeader.Identifier);
		  		  }
}


void SendTorque(int id, float u) {

	  float p_des = 0;
	  float v_des = 0;
	  float t_ff = u; //Consigliato copiare la variabile in una variabile locale

	  float KP = 0;
	  float KD = 0;

	  // Il motore vuole la posizione in 16 bit e la velocità in 12 bit
	  int p_int = float_to_uint(p_des, -P_MAX, P_MAX, 16);
	  int v_int = float_to_uint(v_des, -V_MAX, V_MAX, 12);

	  int kp_int = float_to_uint(KP, 0, Kp_MAX, 12);
	  int kd_int = float_to_uint(KD, 0, Kd_MAX, 12);

	  int t_int = float_to_uint(t_ff, -T_MAX, T_MAX, 12);

	  pTxHeader.Identifier = id;
	  // Usa il tasto declaration per controllare le varie possibilità
	  pTxHeader.IdType = FDCAN_STANDARD_ID;                // specifies id, usiamo un ID standard
	  pTxHeader.TxFrameType = FDCAN_DATA_FRAME ;           // frame type
	  pTxHeader.DataLength = FDCAN_DLC_BYTES_8;            // 8 byte

	  CAN_tx_buffer[0] = p_int >> 8; //pos 8H
	  CAN_tx_buffer[1] = p_int & 0xFF; //pos 8L FF sono tutti elementi pari a 1
// Maschera
// Vedi datasheet motore
	  CAN_tx_buffer[2] = v_int >> 4; // speed 8H
	  CAN_tx_buffer[3] = ((v_int & 0xF) << 4) | (kp_int >> 8) ; //speed 4L KP 8H

	  CAN_tx_buffer[4] = kp_int & 0xFF; // KP 8L

	  CAN_tx_buffer[5] = kd_int >> 4; // kd 8H

	  CAN_tx_buffer[6] = ((kd_int & 0xF) << 4) | (t_int >> 8) ;   // KP 4L  Torque $H

	  CAN_tx_buffer[7] = t_int & 0xFF; // torque 8L


	  // La & mi serve per puntare all'indirizzo di memoria
	  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, CAN_tx_buffer) == HAL_OK) {

		  }
	  //else
	  //printf("Error sending command to Motor %d\n",id);
}

void InverseKinematic(float EE_target[3], int Mode)
{

  //LOCAL VARIABLE
  float X = EE_target[0];
  float Y = EE_target[1];
  float Z = EE_target[2];

  float Xsquare = pow(X, 2);
  float Ysquare = pow(Y, 2);
  float Zsquare = pow(Z, 2);

  float K = sqrt(Xsquare + Ysquare - OFFsquare);
  float Ksquare = pow(K, 2);
  float c3, s3_p, c1, s1,s3_m;

  if (Z < 0)
  {
    printf("Z must be > 0\n");
        return;
  }

  //compute cosine of joint 3
  c3 = (Xsquare + Ysquare + Zsquare - L1square - L2square - OFFsquare) / (2 * L1 * L2);

  if (c3 < 1 && c3 > -1)
  	  { // if requested point is part of WS

	  if (X >= 0 && Y >= 0 && Mode == 1)
	  	  {
      s3_p = sqrt(1 - pow(c3, 2));

      Joint_target[2] = atan2(s3_p, c3);; //joint 3
      Joint_target[1] = atan2((L1 + L2 * c3) * Z + L2 * s3_p * K, -(L1 + L2 * c3) * K + L2 * s3_p * Z);

      c1 = (X / K + (Y * OFF) / Ksquare) / (1 + (OFFsquare) / Ksquare);
      s1 = sqrt(1 - pow(c1, 2));
      Joint_target[0] = atan2(-s1, -c1);
	  	  }

	  if (X >= 0 && Y >= 0 && Mode == 0)
	  	  {
      s3_p = sqrt(1 - pow(c3, 2));

      Joint_target[2] = atan2(-s3_p, c3); //joint 3
      Joint_target[1] = atan2((L1 + L2 * c3) * Z + L2 * -s3_p * K, -(L1 + L2 * c3) * K + L2 * -s3_p * Z);

      c1 = (X / K + (Y * OFF) / Ksquare) / (1 + (OFFsquare) / Ksquare);
      s1 = sqrt(1 - pow(c1, 2));
      Joint_target[0] = atan2(-s1, -c1);
	  	  }

	  if (X <= 0 && Y >= 0 && Mode == 1)
	  	  {
    

      s3_p = sqrt(1 - pow(c3, 2));

      Joint_target[2] = atan2(s3_p, c3); //joint 3

      Joint_target[1] = atan2((L1 + L2 * c3) * Z + L2 * s3_p * K, -(L1 + L2 * c3) * K + L2 * s3_p * Z);

      c1 = (X / K + (Y * OFF) / Ksquare) / (1 + (OFFsquare) / Ksquare);
      s1 = sqrt(1 - pow(c1, 2));

      Joint_target[0] = atan2(-s1, -c1);
	  	  }

	  if(X <= 0 && Y >= 0 && Mode == 0) {
    
      s3_p = sqrt(1 - pow(c3, 2));

      Joint_target[2] = atan2(-s3_p, c3);

      Joint_target[1] = atan2((L1 + L2 * c3) * Z + L2 * -s3_p * K, -(L1 + L2 * c3) * K + L2 * -s3_p * Z);

      c1 = (X / K + (Y * OFF) / Ksquare) / (1 + (OFFsquare) / Ksquare);
      s1 = sqrt(1 - pow(c1, 2));

      Joint_target[0] = atan2(-s1, -c1);
    }

	  if(X >= 0 && Y <= 0 && Mode == 1)
	  {
		  s3_m = -sqrt(1-(c3*c3));

	  Joint_target[2]= atan2(s3_m,c3);

	  Joint_target[1] = atan2((L1 + L2*c3)*Z - L2*s3_m*K , (L1 + L2*c3)*K + L2*s3_m*Z);

	            float K_m = -Ksquare;

	           c1 = (X/K_m + (Y*OFF)/(K_m*K_m))/(1+(OFFsquare)/(K_m*K_m));

	            s1= sqrt(1-pow(c1,2));

	  Joint_target[0] = atan2(s1,c1) - 3.1415;
	  }
  }
  else
  {
	  printf("Requested target is not part of reachable workspace\n");

  }
}
// joint space trjector planener
void TrajectorPlanner(float q0[3], float qf[3], float t)
{
  // q0 initial pose
  // qf final pose
  // t trajector time (steps)
  float Ts = 0.1; //time division step

  float t_step = 0; //used as counter
  int counter = 0;

  float Ns = t/Ts; //Number of step;

  if(Ns > 2000) //max 20s planning at 0.1 Ts
	  {
	  printf("Ns cannot be higher than 2000");
	  return;
	  }


  float qd0[3];
  float qdf[3];
  float qdd0[3];
  float qddf[3];

  //define poly coefficent

  float a0[3];
  float a1[3];
  float a2[3];
  float a3[3];
  float a4[3];
  float a5[3];

  //from t i must create an array with n elements

  // fifth order polynomial interpolation function
  for (int i = 0; i < 3; i++) // execute for all the joint
  {
    qd0[i] = 0;
    qdd0[i] = 0;

    qdf[i] = 0;
    qddf[i] = 0;
  
    a0[i] = q0[i];
    a1[i] = qd0[i];
    a2[i] = 0.5 * qdd0[i];

    //execute FOR to compute time value of joint
    a3[i] = (1 / (2 * pow(t, 3))) * (20 * (qf[i] - q0[i]) - (8 * qdf[i] + 12 * qd0[i]) * t - (3 * qddf[i] - qdd0[i]) * (t * t));

    a4[i] = (1 / (2 * pow(t, 4))) * (30 * (q0[i] - qf[i]) + (14 * qdf[i] + 16 * qd0[i]) * t + (3 * qddf[i] - 2 * qdd0[i]) * t * t);

    a5[i] = (1 / (2 * pow(t, 5))) * (12 * (qf[i] - q0[i]) - 6 * (qdf[i] + qd0[i]) * t - (qddf[i] - qdd0[i]) * (t * t));

    for(counter = 0; counter < Ns+1; counter++ ) {
    t_step = counter*Ts;

    Joint_target_plan[counter][i] = a0[i] + a1[i] * t_step + a2[i] * t_step * t_step + a3[i] * pow(t_step, 3) + a4[i] * pow(t_step, 4) + a5[i] * pow(t_step, 5);

    Joint_speed_target_plan[counter][i] = a1[i] + 2 * a2[i] * t_step + 3 * a3[i] * t_step * t_step + 4 * a4[i] * pow(t_step, 3) + 5 * a5[i] * pow(t_step, 4);
    }
    //float qd_quintic = a1[i] + 2 * a2[i] * t + 3 * a3[i] * t * t + 4 * a4[i] * pow(t, 3) + 5 * a5[i] * pow(t, 4);

    //float qdd_quintic = 2 * a2[i] + 6 * a3[i] * t + 12 * a4[i] * t * t + 20 * a5[i] * pow(t, 3);


    
  }
}

void plan_step() {
	//every time is called, increment
	plan_counter++;
	if(plan_counter < 101) {
	  for(int m = 0; m<3; m++) {
		  Joint_target_planned[m] = Joint_target_plan[plan_counter][m];
		  //printf("%d -- %f\n",m,Joint_target_planned[m]);
		  Joint_speed_target_planned[0] =  Joint_speed_target_plan[plan_counter][0];
	  	  }
	}
}

void PositionController() {
//executed at 100 Hz
	for(int i = 1; i< 3; i++) {

	  pos_previous_error[i] = pos_error[i];

	  pos_error[i] = Joint_target_planned[i] - actualPos[i];

	  pos_integral[i] = pos_integral[i] + pos_error[i];

	  pos_derivative[i] = pos_error[i] - pos_previous_error[i];

	  //anti windup
	  if (pos_integral[i] > MAX_INTEGRAL_ERROR || pos_integral[i] < -MAX_INTEGRAL_ERROR) pos_integral[i] = 0;

	  pos_command[i] = pos_KP[i] * pos_error[i] + pos_KI[i] * pos_integral[i] + pos_KD[i] * pos_derivative[i];
	  //apply limit
	  if (pos_command[i] < - V_MAX) pos_command[i] = -V_MAX;

	  if (pos_command[i] > V_MAX)   pos_command[i] = V_MAX;

	  if(POS_CONTROLLER_DEBUG) {
		  //printf("MOT_ID: %d - Error: %f - Command: %f\n",i+1,pos_error[i],pos_command[i]);
		 printf("POS\\ MOT_ID: %d - Target: %10f - Actual: %10f - Error: %10f - Command: %10f\n",i+1,Joint_target_planned[i],actualPos[i],pos_error[i],pos_command[i]);

	  }

	}

	flag_pos_controller = 0;
}

void SpeedController() {
//executed at 1000 Hz
	for(int i = 1; i< 3; i++) {
		Joint_speed_target[i] = pos_command[i];

	  speed_previous_error[i] = speed_error[i];

	  speed_error[i] = Joint_speed_target[i] - actualSpeed[i];

	  speed_integral[i] = speed_integral[i] + speed_error[i];

	  speed_derivative[i] = speed_error[i] - speed_previous_error[i];

	  //anti windup
	  if (speed_integral[i] > MAX_INTEGRAL_ERROR || speed_integral[i] < -MAX_INTEGRAL_ERROR) speed_integral[i] = 0;

	  speed_command[i] = speed_KP[i] * speed_error[i] + speed_KI[i] * speed_integral[i] + speed_KD[i] * speed_derivative[i];
	  //apply limit
	  if (speed_command[i] < - T_MAX) speed_command[i] = -T_MAX;

	  if (speed_command[i] > T_MAX)   speed_command[i] = T_MAX;

	  if(SPEED_CONTROLLER_DEBUG) {
		//printf("SPEED\\ MOT_ID: %d - Target: %f - Actual: %f - Error: %f - Command: %f\n",i+1,Joint_speed_target[i],actualSpeed[i],speed_error[i],speed_command[i]);

	  }

	  SendTorque(i+1, speed_command[i]); //i+1 perche i motori sono 1,2,3

	  HAL_Delay(1);
	}
	flag_speed_controller = 0;

}

/*
 * veryfy system connectivity (motor,link,sensors)
 * 1 - send packet to motor and check response (3)
 * -- motor answer with its id in frame 0
 * 2 - send system_status request to link board (x2)
 * -- link board answer with its id in frame 0 and the status of sensors in other frame
 *
 *
 */
void POPUP_system_check() {
	//start procedure
	//moto
	printf("System check started..\n");


	HAL_GPIO_WritePin(GPIOB,LD3_Pin, SET); // red led on

	int i = 0;

//	for(i = 1; i<3; i++) {
//		DeactivateMotor(i+1);
//		while(motor_status_flag[i] == 0)
//			{
//				printf("Motor %d not found\n",i+1);
//				HAL_Delay(1000);
//				DeactivateMotor(i+1);
//			}
//		printf("Motor %d found\n",i+1);
//	}


	//link board status check
	// 2 link board should be found

	for(i = 1; i < 2; i++) {
	while(link_status_flag[i] == 0) {
		printf("Link Board %d not found\n",i+1);
		HAL_Delay(1000);
	}
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, RESET);

	if (link_status_flag[i] == 1) printf("Link %d Sensor OK\n",i+1);
    else if (link_status_flag[i] == 4) printf("Link Board %d IMUs error\n",i+1);
    else if (link_status_flag[i] == 2) printf("Link Board %d RBS_ADC error\n",i+1);
	}
	//sent status check command
	if(link_status_flag[0] == 1) {
	CAN_TX_link_board_status_check(10); //send status check to link board 1
	printf("sent status check to link board 1\n");
	}

	if(link_status_flag[1] == 1) {
	CAN_TX_link_board_status_check(11); //send status check to link board 1
	printf("sent status check to link board 2\n");
	}

	printf("\nSystem check completed\n\n\n");

}

void POPUP_activate_motors() {
	int i = 0;
		for(i = 0; i<3; i++) {
		ActivateMotor(i+1);
		printf("Motor %d activated\n",i+1);
		HAL_Delay(100);

	}
}

void POPUP_start_controllers() {
	//start timers base
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

}

void POPUP_start_plan() {
	//start timers base
	HAL_TIM_Base_Start_IT(&htim4);

}

void POPUP_homing() {

}

void POPUP_calibrate_link_sensors(int id) {
	HAL_Delay(1000);
	CAN_TX_link_board_calibration(id); //send status check to link board 1
	printf("sent calibration command to link board %d\n",(id - 9));
	//wait calibration check from link board
    while(link_cal_check[id - 10] == 0) {
    	printf("Waiting calibration check from link board\n");
    	HAL_Delay(1000);
    }
    printf("Calibration check received from link board %d\n",(id - 9));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Check which version of the timer triggered this callback and toggle LED
	if (htim == &htim2) {
		flag_speed_controller = 1;
	}
	if (htim == &htim3) {
		flag_pos_controller = 1;
	}

	if (htim == &htim4) {
		plan_step();
	}

}

float uint_to_float( int x_int , float x_min , float x_max , int bits) {

  float span = x_max - x_min;
  float offset = x_min;

  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;

}

int float_to_uint(float x, float x_min , float x_max, unsigned int bits) {

  float span = x_max - x_min;
  if (x < x_min) x = x_min;
  else if (x > x_max ) x = x_max;

  return (int)((x - x_min) * ((float)((1 << bits) - 1) / span));

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
