#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#define HIGH 1
#define LOW 0

//LED for debugging
#define ONBOARD_LED 25

//Motor pins
#define M11 3
#define M12 4
#define M21 5
#define M22 6

//US sensor pins
#define ECHO 9
#define TRIGGER 10

//Interrupt pins
#define IR_PIN 7

void motion_stop(){

    gpio_put(M11, LOW);
    gpio_put(M12, LOW);
    gpio_put(M21, LOW);
    gpio_put(M22, LOW);
}

void forward(bool hit){

	//block forward movement if near wall
	if(hit){
		motion_stop();
	}

	else{

    	gpio_put(M11, HIGH);
    	gpio_put(M12, LOW);
    	gpio_put(M21, HIGH);
    	gpio_put(M22, LOW);	
	}
}

void backwards(){

    gpio_put(M11, LOW);
    gpio_put(M12, HIGH);
    gpio_put(M21, LOW);
    gpio_put(M22, HIGH);
}

void rotate_left(){

    gpio_put(M11, HIGH);
    gpio_put(M12, LOW);
    gpio_put(M21, LOW);
    gpio_put(M22, HIGH);
}

void rotate_right(){

    gpio_put(M11, LOW);
    gpio_put(M12, HIGH);
    gpio_put(M21, HIGH);
    gpio_put(M22, LOW);
}

double measure_distance(){

	uint64_t timerStart;
	uint64_t timerStop;

	//Set trigger to HIGH for 10us
	gpio_put(TRIGGER,HIGH);
	busy_wait_us_32(10);
	gpio_put(TRIGGER, LOW);

	//Wait until ECHO is high
	while(gpio_get(ECHO) == 0){}

	//Start timing
	timerStart = time_us_64();

	while(gpio_get(ECHO) == 1){

		//If condition to prevent sitting in the loop forever
		if(time_us_64() - timerStart > 10000)
			return 172.4;
	}

	timerStop = time_us_64();

	//Calculate and return distance in cm
	return ((double)timerStop - (double)timerStart) / 58.00;
}

int main(){ // add pwm buzzer
	stdio_init_all();

	//Initialize gpio to be used
	gpio_init(ONBOARD_LED);
	gpio_init(M11);
	gpio_init(M12);
	gpio_init(M21);
	gpio_init(M22);
	gpio_init(ECHO);
	gpio_init(TRIGGER);
	gpio_init(IR_PIN);

	//Set all gpio directions
	gpio_set_dir(ONBOARD_LED, GPIO_OUT);
    gpio_set_dir(M11, GPIO_OUT);
    gpio_set_dir(M12, GPIO_OUT);
    gpio_set_dir(M21, GPIO_OUT);
    gpio_set_dir(M22, GPIO_OUT);
    gpio_set_dir(TRIGGER, GPIO_OUT);
    gpio_set_dir(ECHO, GPIO_IN);
    gpio_set_dir(IR_PIN, GPIO_IN);

    //Set all outputs to LOW initially
    gpio_put(TRIGGER, LOW);
    motion_stop();

	//Define interrupt request
	gpio_set_irq_enabled_with_callback(IR_PIN, 0x08, 1, motion_stop);

	//Initialize uart communication
	uart_init(uart0, 74880);
	gpio_set_function(0, GPIO_FUNC_UART);
	gpio_set_function(1, GPIO_FUNC_UART);

	//Uart buffer
	char* readBuffer = (char*) malloc(4);

	bool hitWall = false;

	double distance = 0.0;

	while(true){

		//Measure distance to the wall
		distance = measure_distance();

		if(distance > 5.0){

			hitWall = true;
		}
		else {

			hitWall = false;
		}

		//Only read uart when data is available
		if(uart_is_readable(uart0)){

			uart_read_blocking(uart0, readBuffer, strlen(readBuffer));
		

			if(strstr(readBuffer, "Stop")){

				motion_stop();
			}

			else if(strstr(readBuffer, "Forw")){

				forward(hitWall);
			}

			else if(strstr(readBuffer, "Back")){

				backwards();
			}

			else if(strstr(readBuffer, "Left")){

				rotate_left();
			}

			else if(strstr(readBuffer, "Righ")){

				rotate_right();
			}

		}

	}

	free(readBuffer);
}