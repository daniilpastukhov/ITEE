
#ifndef __MOTORS_H
#define __MOTORS_H

#ifndef __MOTOR_PINS
#define __MOTOR_PINS
#define AIN1_PORT	GPIOA
#define AIN1_PIN	GPIO_PIN_8

#define AIN2_PORT	GPIOA
#define AIN2_PIN	GPIO_PIN_9

#define PWMA_PORT	GPIOC
#define PWMA_PIN	GPIO_PIN_9

#define BIN1_PORT	GPIOA
#define BIN1_PIN	GPIO_PIN_11

#define BIN2_PORT	GPIOA
#define BIN2_PIN	GPIO_PIN_12

#define PWMB_PORT	GPIOA
#define PWMB_PIN	GPIO_PIN_10

#define ERROR_LED_PORT	GPIOA
#define ERROR_LED_PIN	GPIO_PIN_5

#define MAX_SPEED 10
#endif

enum State{On,Off};
enum Dir{Forw,Backw};

enum State m1State;
enum State m2State;

enum Dir m1Dir;
enum Dir m2Dir;

int16_t m1Speed;
int16_t m2Speed;
bool errorMot = false;

void setCorrectDirM1(void);
void resetDirM1(void);
void setCorrectDirM2(void);
void resetDirM2(void);

#endif
