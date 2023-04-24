#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/spi.h"

#define HIGH 1
#define LOW 0

//LED for debugging
#define ONBOARD_LED 25

//UART instance - pins 0, 1
#define UART_INST uart0

//SPI instance - pins 2, 3, 4
#define SPI_INST spi0

//Motor pins
#define M11 5
#define M12 6
#define M21 7
#define M22 8

//US sensor pins
#define ECHO 9
#define TRIGGER 10

//Interrupt pins
#define MANUAL 11
#define AUTOMATIC 12

#define ROTATE_TIME 500000

//Define Map sizes
static const int Map_rows = 3;
static const int Map_columns = 3;

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

void Auto_rotate(int direction){

	if(direction > 0){
		rotate_left();
		busy_wait_us_32(ROTATE_TIME);
		motion_stop();
		return; 
	}

	else if(direction < 0){
		rotate_right();
		busy_wait_us_32(ROTATE_TIME);
		motion_stop();
		return; 
	}

	else return;
}

//Update target values to know where to go next
int search(int x, int y, int* target_x, int* target_y, char Map[Map_rows][Map_columns]){

	for(;y < Map_rows; y++){
			for(; x < sizeof(Map[x]); x++){

				//Search for closest x in the current row
				if(Map[y][x] == 'x'){
					*target_x = x;
					*target_y = y;
					return 1;
				}
				else if(Map[y][x] == '#')
					return 1;
			}
		}
	return 0;	
}

void Auto_forward(){

	//Add hit detection here
	forward(0);
	busy_wait_us_32(1000000);
	motion_stop(); 
}

void navigate(int* x, int* y, int target_x, int target_y, int* direction_x, int* direction_y){

	//If target direction and direction multiplied is negative change direction x
	//After changing direction, rotate car
	if((target_x - x) * (*direction_x) < 0){
		(*direction_x) *= -1;
		Auto_rotate(1);
		Auto_rotate(1);
	}

	int steps = abs(target_x - x);

	//Move car forward on the x axis
	for(int i = 0; i < steps; i++){
		Auto_forward();
		(*x)++;
	}

	//Determine y axis direction
	if((*direction_x) > 0)
		Auto_rotate(target_y - y);
	else if((*direction_x) < 0)
		Auto_rotate(-1 * (target_y - y));

	steps = abs(target_y - y);

	//Move car forward on the y axis
	for(int i = 0; i < steps; i++){
		Auto_forward();
		(*y)++;
	}

	//Update y direction
	*direction_y = target_y - y;

	Auto_rotate(-1 * (*direction_x) * (*direction_y));
}

void return_home(int x, int y, int direction_x, int direction_y){

	//navigate to position 0, 0;
	navigate(x, y, 0, 0, direction_x, direction_y);
}

//Function called by interrupt;
//Puts car into self driving mode
void Auto_mode(){

	//Deinitialize UART and SPI
	uart_deinit(UART_INST);
	spi_deinit(SPI_INST);

	//Car location variables
	int x = 0;
	int y = 0;
	int target_x = 0;
	int target_y = 0;
	int direction_x = 1;
	int direction_y = 1;

	char Map[Map_rows][Map_columns]= {
		"xxx",
		"xxx",
		"xxx"
	};

	int blocks = sizeof(Map);

	while(blocks > 0){
		
		//Decrease if unvisited block is found
		blocks -= search(x, y, &target_x, &target_y, Map);

		//Go to address
		navigate(&x, &y, target_x, target_y, &direction_x, &direction_y);

		//Correct map afterwards
		Map[y][x] == '.';
	}
	
	//Return to starting position
	return_home(x, y, direction_x, direction_y);
}

Manual_mode(){

	//Initialize uart communication
	uart_init(UART_INST, 74880);
	gpio_set_function(0, GPIO_FUNC_UART);
	gpio_set_function(1, GPIO_FUNC_UART);

	//Deinitialize spi
	spi_deinit(SPI_INST);

	//Uart buffer
	char* readBuffer = (char*) malloc(4);

	bool hitWall = false;

	double distance = 0.0;

	while(true){

		//Measure distance to the wall
		distance = measure_distance();

		if(distance < 5.0){

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

int main(){ 
	stdio_init_all();

	//Initialize gpio to be used
	gpio_init(ONBOARD_LED);
	gpio_init(M11);
	gpio_init(M12);
	gpio_init(M21);
	gpio_init(M22);
	gpio_init(ECHO);
	gpio_init(TRIGGER);
	gpio_init(MANUAL);
	gpio_init(AUTOMATIC);

	//Set all gpio directions
	gpio_set_dir(ONBOARD_LED, GPIO_OUT);
    gpio_set_dir(M11, GPIO_OUT);
    gpio_set_dir(M12, GPIO_OUT);
    gpio_set_dir(M21, GPIO_OUT);
    gpio_set_dir(M22, GPIO_OUT);
    gpio_set_dir(TRIGGER, GPIO_OUT);
    gpio_set_dir(ECHO, GPIO_IN);
    gpio_set_dir(MANUAL, GPIO_IN);
    gpio_set_dir(AUTOMATIC, GPIO_IN);

    //Set all outputs to LOW initially
    gpio_put(ONBOARD_LED, LOW);
    gpio_put(TRIGGER, LOW);
    motion_stop();

	//Define interrupt requests
	gpio_set_irq_enabled_with_callback(AUTOMATIC, 0x08, 1, Auto_mode);
	gpio_set_irq_enabled_with_callback(MANUAL, 0x08, 1, Manual_mode);

	//Initalize spi for nRF24l01 - spi0 with 1MHZ baud rate
	spi_init(SPI_INST, 1000000)

	//Format spi - 8 data bits, cpha 0, cpol 0, MSB first
	spi_set_format(SPI_INST, 8, 0, 0, 1);

	//Add control over nRF24l01
	while(1){


	}
	
}