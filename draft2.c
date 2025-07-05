// ensure CAN filtering is set up correctly

#include <stdio.h>
#include <string.h>
#include <math.h>

// MPPT variables

#define MPPT_STATE_ALL_ON 0
#define MPPT_STATE_TWO_ON 1
#define MPPT_STATE_ALL_OFF 2

#define MPPT_MOSFET_THRESHOLD_TEMP 65.0f
#define MPPT_TEMP_HYSTERESIS 5.0f

#define MPPT_SOC_THRESHOLD_HIGH 99.9f
#define MPPT_SOC_THRESHOLD_MID 99.0f
#define MPPT_SOC_THRESHOLD_LOW 98.0f
#define MPPT_SOC_HYSTERESIS 0.5f

#define MPPT_PRECHARGE_DELAY_MS 2000
#define MPPT_OUT_PRECHARGE_PIN GPIO_PIN_14
#define MPPT_OUT_MAIN_PIN GPIO_PIN_15

#define MPPT1_IN_SOLAR_PIN GPIO_PIN_8
#define MPPT2_IN_SOLAR_PIN GPIO_PIN_9
#define MPPT3_IN_SOLAR_PIN GPIO_PIN_10

#define MPPT1_12V_SUPPLY_PIN GPIO_PIN_11
#define MPPT2_12V_SUPPLY_PIN GPIO_PIN_12
#define MPPT3_12V_SUPPLY_PIN GPIO_PIN_13

#define MPPT_PORT GPIOE

// DC DC Converter variables

#define DC_DC_CONVERTER_ADC_PIN GPIO_PIN_0
#define DC_DC_CONVERTER_PORT GPIOA

#define VOLTAGE_SENSING_RESISTANCE_R1 10000.0f // 10k ohm
#define VOLTAGE_SENSING_RESISTANCE_R2 1000.0f // 1k ohm

#define DC_DC_THRESHOLD_VOLTAGE 12.0f // 12V threshold for DC-DC converter operation

// Indicator variables

#define HAZARD_IN_PIN GPIO_PIN_11
#define IND_LEFT_IN_PIN GPIO_PIN_12
#define IND_RIGHT_IN_PIN GPIO_PIN_13

#define IND_LEFT_OUT_PIN GPIO_PIN_14
#define IND_RIGHT_OUT_PIN GPIO_PIN_15

#define IND_PORT GPIOB

#define INDICATOR_BLINK_DELAY 500 // milliseconds

// MPPT variables
volatile float MPPT1_MOSFET_TEMP = 0.0f;
volatile float MPPT2_MOSFET_TEMP = 0.0f;   
volatile float MPPT3_MOSFET_TEMP = 0.0f;

uint8_t SOC_CONTROL_STATE = MPPT_STATE_ALL_ON;

//BMS variables

volatile uint8_t BMS_MODE = 0;
volatile float BATTERY_SOC_PERCENT = 0.0f;

// DC DC Converter variables

volatile uint8_t DC_DC_CONVERTER_OP_VOLTAGE = 0;
volatile uint8_t DC_DC_CONVERTER_OP_ADC = 0;

// Indicator variables

uint8_t LEFT_LED_STATE = 0;
uint8_t RIGHT_LED_STATE = 0;
uint32_t LAST_TIME_BLINK_HAZARD = 0;
uint32_t LAST_TIME_BLINK_LEFT = 0;
uint32_t LAST_TIME_BLINK_RIGHT = 0;

void CAN_FILTER_CONFIG(void);
void MPPT_IN_START(void);
void MPPT_PRECHARGE_START(void);
void MPPT_12V_SUPPLY_START(void);
void ALL_MPPT_OFF(void);
void ALL_MPPT_ON(void);
void COOLEST_TWO_MPPT_ON(void);
void MPPT_TEMP_SHUTDOWN(void);
void MPPT_SOC_CONTROL(void);
void INDICATOR_CONTROL(void);

void HAL_Fifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_Getrx_data(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        Error_Handler(); // Error handling
    }

    switch (rx_header.StdId) {
        case 0x692:
            memcpy(&MPPT1_MOSFET_TEMP, rx_data, sizeof(float)); // MPPT1 MOSFET temperature
            break;

        case 0x6A2:
            memcpy(&MPPT2_MOSFET_TEMP, rx_data, sizeof(float)); // MPPT2 MOSFET temperature
            break;

        case 0x6B2:
            memcpy(&MPPT3_MOSFET_TEMP, rx_data, sizeof(float)); // MPPT3 MOSFET temperature
            break;
        
        case 0x6F7:
            memcpy(&BMS_MODE, rx_data[1], sizeof(uint8_t));
            break;

        case 0x6F4:
            memcpy(&BATTERY_SOC_PERCENT, rx_data[4], sizeof(float)); // Battery SOC percentage
            break;
            
    }
    
}

int main(void)
{
    
    CAN_FILTER_CONFIG();

    HAL_CAN_START(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);


    while(BMS_MODE != 0X05 && DC_DC_CONVERTER_OP_VOLTAGE < DC_DC_THRESHOLD_VOLTAGE){
        DC_DC_CONVERTER_OP_ADC = HAL_ADC_GetValue(&hadc1);
        DC_DC_CONVERTER_OP_VOLTAGE = ((VOLTAGE_SENSING_RESISTANCE_R1 + VOLTAGE_SENSING_RESISTANCE_R2)
                                        * DC_DC_CONVERTER_OP_ADC * 3.3) /
                                         (4095 * VOLTAGE_SENSING_RESISTANCE_R2);
    }

    MPPT_IN_START();
    MPPT_PRECHARGE_START();
    MPPT_12V_SUPPLY_START();

    while(1)
    {
        MPPT_TEMP_SHUTDOWN();
        MPPT_SOC_CONTROL();
        INDICATOR_CONTROL();
    }
}

void MPPT_TEMP_SHUTDOWN(void)
{
    if(MPPT1_MOSFET_TEMP > MPPT_MOSFET_THRESHOLD_TEMP + MPPT_TEMP_HYSTERESIS){
        HAL_GPIO_WritePin(MPPT_PORT, MPPT1_IN_SOLAR_PIN, GPIO_PIN_RESET);
    }
    if(MPPT2_MOSFET_TEMP > MPPT_MOSFET_THRESHOLD_TEMP + MPPT_TEMP_HYSTERESIS){
        HAL_GPIO_WritePin(MPPT_PORT, MPPT2_IN_SOLAR_PIN, GPIO_PIN_RESET);
    }
    if(MPPT3_MOSFET_TEMP > MPPT_MOSFET_THRESHOLD_TEMP + MPPT_TEMP_HYSTERESIS){
        HAL_GPIO_WritePin(MPPT_PORT, MPPT3_IN_SOLAR_PIN, GPIO_PIN_RESET);
    }
}

void MPPT_SOC_CONTROL(void)
{
    if(BATTERY_SOC_PERCENT > MPPT_SOC_THRESHOLD_HIGH){
        SOC_CONTROL_STATE = MPPT_STATE_ALL_OFF;
    }
    else if(BATTERY_SOC_PERCENT < MPPT_SOC_THRESHOLD_MID - MPPT_SOC_HYSTERESIS
            && SOC_CONTROL_STATE == MPPT_STATE_ALL_OFF){
        SOC_CONTROL_STATE = MPPT_STATE_TWO_ON;
    }
    else if(BATTERY_SOC_PERCENT <MPPT_SOC_THRESHOLD_LOW - MPPT_SOC_HYSTERESIS
            && SOC_CONTROL_STATE == MPPT_STATE_TWO_ON){
        SOC_CONTROL_STATE = MPPT_STATE_ALL_ON;
    }
    else if(BATTERY_SOC_PERCENT > MPPT_SOC_THRESHOLD_MID + MPPT_SOC_HYSTERESIS
            && SOC_CONTROL_STATE == MPPT_STATE_ALL_ON){
        SOC_CONTROL_STATE = MPPT_STATE_TWO_ON;
    }

    switch(SOC_CONTROL_STATE){
        case MPPT_STATE_ALL_OFF:
                ALL_MPPT_OFF();
                break;
        case MPPT_STATE_TWO_ON:
                COOLEST_TWO_MPPT_ON();
                break;
        case MPPT_STATE_ALL_ON:
                ALL_MPPT_ON();
                break;        
    }
}

void CAN_FILTER_CONFIG(void)
{
    CAN_FilterTypeDef MPPT_FILTER;

    MPPT_FILTER.FilterActivation = ENABLE;
    MPPT_FILTER.FilterMode = CAN_FILTERMODE_IDMASK;
    MPPT_FILTER.FilterScale = CAN_FILTERSCALE_32BIT;
    MPPT_FILTER.FilterIdHigh = 0X0690 << 5; // MPPT1
    MPPT_FILTER.FilterIdLow = 0X0000;
    MPPT_FILTER.FilterMaskIdHigh = 0XFF0 << 5; // update this neatly
    MPPT_FILTER.FilterMaskIdLow = 0X0000;
    MPPT_FILTER.FilterFIFOAssignment = CAN_RX_FIFO0;
    MPPT_FILTER.FilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &MPPT_FILTER);

    CAN_FilterTypeDef BMS_FILTER;
    BMS_FILTER.FilterActivation = ENABLE;
    BMS_FILTER.FilterMode = CAN_FILTERMODE_IDMASK;
    BMS_FILTER.FilterScale = CAN_FILTERSCALE_32BIT;
    BMS_FILTER.FilterIdHigh = 0X0600 << 5;
    BMS_FILTER.FilterIdLow = 0X0000;
    BMS_FILTER.FilterMaskIdHigh = 0XFF0 << 5; // update this neatly
    BMS_FILTER.FilterMaskIdLow = 0X0000;
    BMS_FILTER.FilterFIFOAssignment = CAN_RX_FIFO0;
    BMS_FILTER.FilterBank = 1;
    HAL_CAN_ConfigFilter(&hcan1, &BMS_FILTER);
}

void MPPT_IN_START(void)
{
    HAL_GPIO_WritePin(MPPT_PORT, MPPT1_IN_SOLAR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPPT_PORT, MPPT2_IN_SOLAR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPPT_PORT, MPPT3_IN_SOLAR_PIN, GPIO_PIN_SET);
}

void MPPT_PRECHARGE_START(void)
{
    HAL_GPIO_WritePin(MPPT_PORT, MPPT_OUT_PRECHARGE_PIN, GPIO_PIN_SET);
    HAL_Delay(MPPT_PRECHARGE_DELAY_MS);
    HAL_GPIO_WritePin(MPPT_PORT, MPPT_OUT_PRECHARGE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MPPT_PORT, MPPT_OUT_MAIN_PIN, GPIO_PIN_SET);
}

void MPPT_12V_SUPPLY_START(void)
{
    HAL_GPIO_WritePin(MPPT_PORT, MPPT1_12V_SUPPLY_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPPT_PORT, MPPT2_12V_SUPPLY_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MPPT_PORT, MPPT3_12V_SUPPLY_PIN, GPIO_PIN_SET);
}

void ALL_MPPT_OFF(void)
{
    HAL_GPIO_WritePin(MPPT_PORT, MPPT1_IN_SOLAR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MPPT_PORT, MPPT2_IN_SOLAR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MPPT_PORT, MPPT3_IN_SOLAR_PIN, GPIO_PIN_RESET);
}
void ALL_MPPT_ON(void)
{
   if(MPPT1_MOSFET_TEMP< MPPT_MOSFET_THRESHOLD_TEMP- MPPT_TEMP_HYSTERESIS){
        HAL_GPIO_WritePin(MPPT_PORT, MPPT1_IN_SOLAR_PIN, GPIO_PIN_SET);
        }
    if(MPPT2_MOSFET_TEMP< MPPT_MOSFET_THRESHOLD_TEMP- MPPT_TEMP_HYSTERESIS){
        HAL_GPIO_WritePin(MPPT_PORT, MPPT2_IN_SOLAR_PIN, GPIO_PIN_SET);
    }
    if(MPPT3_MOSFET_TEMP< MPPT_MOSFET_THRESHOLD_TEMP- MPPT_TEMP_HYSTERESIS){
        HAL_GPIO_WritePin(MPPT_PORT, MPPT3_IN_SOLAR_PIN, GPIO_PIN_SET);
    } 
}

void COOLEST_TWO_MPPT_ON(void)
{
    int mppt_on_count = 0;
    if(HAL_GPIO_ReadPin(MPPT_PORT, MPPT1_IN_SOLAR_PIN) == GPIO_PIN_SET){
        mppt_on_count++;
    }
    if(HAL_GPIO_ReadPin(MPPT_PORT, MPPT2_IN_SOLAR_PIN) == GPIO_PIN_SET){
        mppt_on_count++;
    }
    if(HAL_GPIO_ReadPin(MPPT_PORT, MPPT3_IN_SOLAR_PIN) == GPIO_PIN_SET){
        mppt_on_count++;
    }

  struct{
    uint16_t pin;
    float temp;
    } mppts[3] = {
        {MPPT1_IN_SOLAR_PIN, MPPT1_MOSFET_TEMP},
        {MPPT2_IN_SOLAR_PIN, MPPT2_MOSFET_TEMP},
        {MPPT3_IN_SOLAR_PIN, MPPT3_MOSFET_TEMP}
    };
    
    //disable all MPPTs first
    for(int i= 0; i<3; i++){
        HAL_GPIO_WritePin(MPPT_PORT, mppts[i].pin, GPIO_PIN_RESET); // Turn off all MPPTs
    }
    
    for(int i = 0; i<2; i++){
        for(int j= i+1; j<3; j++){
            if(mppts[j].temp < mppts[i].temp){
                //swap positions
                const uint16_t temporaryPin = mppts[i].pin;
                const float temporaryTemp = mppts[i].temp;
                mppts[i].pin = mppts[j].pin;
                mppts[i].temp = mppts[j].temp;
                mppts[j].pin = temporaryPin;
                mppts[j].temp = temporaryTemp;
            }
        }
    }

    switch(mppt_on_count) {
        // Enable the two coolest MPPTs

    case 3:
        int enabledCount = 0;
        for(int i = 0; i < 3 && enabledCount < 2; i++) {
            if(mppts[i].temp < MPPT_MOSFET_THRESHOLD_TEMP- MPPT_TEMP_HYSTERESIS) {
                HAL_GPIO_WritePin(MPPT_PORT, mppts[i].pin, GPIO_PIN_SET); // Enable the MPPT
                enabledCount++;
            }
        }
    
    
    break;

    case 2:
        if(mppts[2].temp - mppts[1].temp > MPPT_MPPT_TEMP_HYSTERESIS){
            int enabledCount = 0;
        for(int i = 0; i < 3 && enabledCount < 2; i++) {
            if(mppts[i].temp < MPPT_MOSFET_THRESHOLD_TEMP- MPPT_TEMP_HYSTERESIS) {
                HAL_GPIO_WritePin(MPPT_PORT, mppts[i].pin, GPIO_PIN_SET); // Enable the MPPT
                enabledCount++;
            }
        }
        }
        break;
    }    
}

void INDICATOR_CONTROL(void)
{
    if (HAL_GPIO_ReadPin(IND_PORT, HAZARD_IN_PIN) == GPIO_PIN_SET)
	  	      {
	  	        if (HAL_GetTick() - LAST_TIME_BLINK_HAZARD >= INDICATOR_BLINK_DELAY) {
	  	          LAST_TIME_BLINK_HAZARD = HAL_GetTick();
	  	          LEFT_LED_STATE=RIGHT_LED_STATE;
	  	          LEFT_LED_STATE = !LEFT_LED_STATE;
	  	          RIGHT_LED_STATE = LEFT_LED_STATE;

	  	          HAL_GPIO_WritePin(IND_PORT, IND_LEFT_OUT_PIN, LEFT_LED_STATE ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  	          HAL_GPIO_WritePin(IND_PORT, IND_RIGHT_OUT_PIN, RIGHT_LED_STATE ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  	        }
	  	      }
	    else if (HAL_GPIO_ReadPin(IND_PORT, IND_RIGHT_IN_PIN) == GPIO_PIN_SET)
	  	      {
	  	        if (HAL_GetTick() - LAST_TIME_BLINK_RIGHT >= INDICATOR_BLINK_DELAY) {
	  	          LAST_TIME_BLINK_RIGHT = HAL_GetTick();

	  	          RIGHT_LED_STATE = !RIGHT_LED_STATE;
	  	          HAL_GPIO_WritePin(IND_PORT, IND_RIGHT_OUT_PIN, RIGHT_LED_STATE ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  	          HAL_GPIO_WritePin(IND_PORT, IND_LEFT_OUT_PIN, GPIO_PIN_RESET);  // turn off left
	  	        }
	  	      }
	    else if (HAL_GPIO_ReadPin(IND_PORT, IND_LEFT_IN_PIN) == GPIO_PIN_SET)
	  	      {
	  	        if (HAL_GetTick() - LAST_TIME_BLINK_LEFT >= INDICATOR_BLINK_DELAY) {
	  	          LAST_TIME_BLINK_LEFT = HAL_GetTick();

	  	          LEFT_LED_STATE = !LEFT_LED_STATE;
	  	          HAL_GPIO_WritePin(IND_PORT, IND_LEFT_OUT_PIN, LEFT_LED_STATE ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  	          HAL_GPIO_WritePin(IND_PORT, IND_RIGHT_OUT_PIN, GPIO_PIN_RESET);  // turn off right
	  	        }

	  	      }
	    else {

		  	  	  HAL_GPIO_WritePin(IND_PORT, IND_LEFT_OUT_PIN,  GPIO_PIN_RESET);
		  	  	  HAL_GPIO_WritePin(IND_PORT, IND_RIGHT_OUT_PIN, GPIO_PIN_RESET);  // turn off right
		  	}
}
