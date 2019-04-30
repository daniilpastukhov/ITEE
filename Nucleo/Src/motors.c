#include "stdbool.h"
#include "main.h"
#include "motors.h"

void setCorrectDirM1(void){
	if(m1Dir == Forw){
		HAL_GPIO_WritePin(AIN1_PORT,AIN1_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_PORT,AIN1_PIN,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(AIN1_PORT,AIN1_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_PORT,AIN1_PIN,GPIO_PIN_SET);
	}
}

void resetDirM1(void){
	HAL_GPIO_WritePin(AIN1_PORT,AIN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_PORT,AIN1_PIN,GPIO_PIN_RESET);
}

void setCorrectDirM2(void){
	if(m1Dir == Forw){
		HAL_GPIO_WritePin(BIN1_PORT,BIN1_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_PORT,BIN1_PIN,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(BIN1_PORT,BIN1_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_PORT,BIN1_PIN,GPIO_PIN_SET);
	}
}

void resetDirM2(void){
	HAL_GPIO_WritePin(BIN1_PORT,BIN1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_PORT,BIN1_PIN,GPIO_PIN_RESET);
}

void handleSpeedAndDirM1(void){
	if(m1Speed>MAX_SPEED || m1Speed<0){
		HAL_GPIO_WritePin(ERROR_LED_PORT,ERROR_LED_PIN,GPIO_PIN_SET);
		errorMot = true;
	}
	else{
		HAL_GPIO_WritePin(PWMA_PORT,PWMA_PIN,GPIO_PIN_SET);
		setCorrectDirM1();
		osDelay(m1Speed);
		resetDirM1();
		HAL_GPIO_WritePin(PWMA_PORT,PWMA_PIN,GPIO_PIN_RESET);
		osDelay(MAX_SPEED-m1Speed);
	}
}

void handleSpeedAndDirM2(void){
	if(m2Speed>MAX_SPEED || m2Speed<0){
		HAL_GPIO_WritePin(ERROR_LED_PORT,ERROR_LED_PIN,GPIO_PIN_SET);
		errorMot = true;
	}
	else{
		HAL_GPIO_WritePin(PWMB_PORT,PWMB_PIN,GPIO_PIN_SET);
		setCorrectDirM2();
		osDelay(m2Speed);
		resetDirM2();
		HAL_GPIO_WritePin(PWMB_PORT,PWMB_PIN,GPIO_PIN_RESET);
		osDelay(MAX_SPEED-m2Speed);
	}
}
