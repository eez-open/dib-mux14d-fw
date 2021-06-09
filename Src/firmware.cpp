#include <math.h>
#include <memory.h>
#include <stdlib.h>

#include "main.h"

#include "firmware.h"
#include "utils.h"

/*

There is two "ports":  P1 and P2.

P1 outputs are: PA0 to PA6 (in total 7 relays).
P2 outputs are: PB0 to PB6 (in total 7 relays).

There are also 3 "special" relays:
  - P2_EXT on PA15 which connects P1 and P2 ports into one 14-bit.
  - P1 relay (PB10) connects selected input to P1 output,
  - P2 relay (PB7) connects selected P2 input to P2 output.

SCPI channels:
	- (@n01) is for P1 "activation" relay
	- (@n02) is for P1 "activation" relay
	- in future (@n03) is for P2_EXT (PA15)
	- (@n11:n17) for P1 port
	- (@n21:n27) for P2 port,

Temperature sensor (Cold Junction Temperature): ADC on F0
*/

static const uint32_t CONF_RELAY_DEBOUNCE_TIME_MS = 1;

// master-slave communication
extern "C" void SPI1_Init(void);
extern SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef *hspiMaster = &hspi1; // SPI for MASTER-SLAVE communication
volatile enum {
	TRANSFER_STATE_WAIT,
	TRANSFER_STATE_SUCCESS,
	TRANSFER_STATE_ERROR
} transferState;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
	transferState = TRANSFER_STATE_SUCCESS;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
	transferState = TRANSFER_STATE_ERROR;
}

// relays
static const int NUM_RELAYS = 7;

static GPIO_TypeDef *P1_RELAY_PORTS[NUM_RELAYS] = { P1_0_GPIO_Port, P1_1_GPIO_Port, P1_2_GPIO_Port, P1_3_GPIO_Port, P1_4_GPIO_Port, P1_5_GPIO_Port, P1_6_GPIO_Port };
static uint16_t P1_RELAY_PINS[NUM_RELAYS] = { P1_0_Pin, P1_1_Pin, P1_2_Pin, P1_3_Pin, P1_4_Pin, P1_5_Pin, P1_6_Pin };
uint8_t g_p1RelayStates = 0; // use this to store the latest state of relays, at the beginning all are off

static GPIO_TypeDef *P2_RELAY_PORTS[NUM_RELAYS] = { P2_0_GPIO_Port, P2_1_GPIO_Port, P2_2_GPIO_Port, P2_3_GPIO_Port, P2_4_GPIO_Port, P2_5_GPIO_Port, P2_6_GPIO_Port };
static uint16_t P2_RELAY_PINS[NUM_RELAYS] = { P2_0_Pin, P2_1_Pin, P2_2_Pin, P2_3_Pin, P2_4_Pin, P2_5_Pin, P2_6_Pin };
uint8_t g_p2RelayStates = 0; // use this to store the latest state of relays, at the beginning all are off

uint8_t g_adib1RelayState = 0;
uint8_t g_adib2RelayState = 0;

uint8_t g_extRelayState = 0;

extern ADC_HandleTypeDef hadc;

#define TS_CAL1_TEMP 30.0f
#define TS_CAL1 *((uint16_t*) 0x1FFFF7B8)
#define TS_CAL2_TEMP 110.0f
#define TS_CAL2 *((uint16_t*) 0x1FFFF7C2)

float getCjTemp() {
	HAL_ADC_Start(&hadc);

	for (int i = 0; i < 10; i++) {
		if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) {
			uint16_t adc = HAL_ADC_GetValue(&hadc);
			HAL_ADC_Stop(&hadc);
			// // P: (x, y)
			// // P1: (TS_CAL1, TS_CAL1_TEMP)
			// // P2: (TS_CAL2, TS_CAL2_TEMP)
			// // y(x) = (x - x1) * (y2 - y1) / (x2 - x1) + y1
			// return roundf((adc * 3.0f / 3.3f - TS_CAL1) * (TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1) + TS_CAL1_TEMP);

			return roundf(20.0f + ((adc / 4096.0f) * 3.3f -  1.4423f) / 0.0535f);
		}

		HAL_Delay(1);
	}

	HAL_ADC_Stop(&hadc);

	return -273.0f;
}


// setup is called once at the beginning from the main.c
extern "C" void setup() {
	// setup relays, i. e. turn all off

	for (int i = 0; i < NUM_RELAYS; i++) {
		RESET_PIN(P1_RELAY_PORTS[i], P1_RELAY_PINS[i]);
	}

	for (int i = 0; i < NUM_RELAYS; i++) {
		RESET_PIN(P2_RELAY_PORTS[i], P2_RELAY_PINS[i]);
	}

	RESET_PIN(P2_EXT_GPIO_Port, P2_EXT_Pin);

	HAL_Delay(CONF_RELAY_DEBOUNCE_TIME_MS); // prevent debounce

	HAL_ADCEx_Calibration_Start(&hadc);
}

// loop is called, of course, inside the loop from the main.c
extern "C" void loop() {
	// start SPI transfer
	static const size_t BUFFER_SIZE = 20;
	uint32_t input[(BUFFER_SIZE + 3) / 4 + 1];
	uint32_t output[(BUFFER_SIZE + 3) / 4];
    transferState = TRANSFER_STATE_WAIT;
    HAL_SPI_TransmitReceive_DMA(hspiMaster, (uint8_t *)output, (uint8_t *)input, BUFFER_SIZE);
    RESET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin); // inform master that module is ready for the SPI communication

    // wait for the transfer to finish
//	uint32_t startTick = HAL_GetTick();
	while (transferState == TRANSFER_STATE_WAIT) {
//		if (HAL_GetTick() - startTick > CONF_SPI_TRANSFER_TIMEOUT_MS) {
//			// transfer is taking too long to finish, maybe something is stuck, abort it
//			__disable_irq();
//			HAL_SPI_Abort(hspiMaster);
//			SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
//			transferState = TRANSFER_STATE_ERROR;
//			__enable_irq();
//			break;
//		}
	}

	// Request and Response are defined in firmware.h
    Request &request = *(Request *)input;
    Response &response = *(Response *)output;

    if (transferState == TRANSFER_STATE_SUCCESS) {
    	// a way to tell the master that command was handled
    	response.command = 0x8000 | request.command;

		if (request.command == COMMAND_GET_INFO) {
			// return back to the master firmware version and MCU id
			static const uint16_t MODULE_TYPE_DIB_MUX14D = 14;
			response.getInfo.moduleType = MODULE_TYPE_DIB_MUX14D;
			response.getInfo.firmwareMajorVersion = FIRMWARE_VERSION_MAJOR;
			response.getInfo.firmwareMinorVersion = FIRMWARE_VERSION_MINOR;
			response.getInfo.idw0 = HAL_GetUIDw0();
			response.getInfo.idw1 = HAL_GetUIDw1();
			response.getInfo.idw2 = HAL_GetUIDw2();
		}

		else if (request.command == COMMAND_GET_STATE) {
			// master periodically asks for the state of the module,

			response.getState.cjTemp = getCjTemp();
		}

		else if (request.command == COMMAND_SET_PARAMS) {
			// turn on/off relays as instructed

			// P1
			for (int i = 0; i < NUM_RELAYS; i++) {
				auto currentRelayState = g_p1RelayStates & (1 << i);
				auto newRelayState = request.setParams.p1RelayStates & (1 << i);
				if (currentRelayState != newRelayState) {
					WRITE_PIN(P1_RELAY_PORTS[i], P1_RELAY_PINS[i], newRelayState);
				}
			}
			g_p1RelayStates = request.setParams.p1RelayStates;

			// P2
			for (int i = 0; i < NUM_RELAYS; i++) {
				auto currentRelayState = g_p2RelayStates & (1 << i);
				auto newRelayState = request.setParams.p2RelayStates & (1 << i);
				if (currentRelayState != newRelayState) {
					WRITE_PIN(P2_RELAY_PORTS[i], P2_RELAY_PINS[i], newRelayState);
				}
			}
			g_p2RelayStates = request.setParams.p2RelayStates;

			// ADIB1
			if (g_adib1RelayState != request.setParams.adib1RelayState) {
				WRITE_PIN(P1_GPIO_Port, P1_Pin, request.setParams.adib1RelayState);
				g_adib1RelayState = request.setParams.adib1RelayState;
			}

			// ADIB2
			if (g_adib2RelayState != request.setParams.adib2RelayState) {
				WRITE_PIN(P2_GPIO_Port, P2_Pin, request.setParams.adib2RelayState);
				g_adib2RelayState = request.setParams.adib2RelayState;
			}

			// EXT
			if (g_extRelayState != request.setParams.extRelayState) {
				WRITE_PIN(P2_EXT_GPIO_Port, P2_EXT_Pin, request.setParams.extRelayState);
				g_extRelayState = request.setParams.extRelayState;
			}

			HAL_Delay(CONF_RELAY_DEBOUNCE_TIME_MS); // prevent debounce

			response.setParams.result = 1; // success
		}

		else {
			// unknown command received, tell the master that no command was handled
			response.command = COMMAND_NONE;
		}
    } else {
    	// invalid transfer, reinitialize SPI just in case
    	HAL_SPI_DeInit(hspiMaster);
		SPI1_Init();

		// tell the master that no command was handled
		response.command = COMMAND_NONE;
    }
}
