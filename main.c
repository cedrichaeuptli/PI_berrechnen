/*
 * PI_berrechnen.c
 *
 * Created: 20.03.2018 18:32:07
 * Author : Cedric
 */ 

//#include <avr/io.h>
#include <stdio.h> 
#include <stdlib.h>
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "stack_macros.h"

#include "mem_check.h"

#include "init.h"
#include "utils.h"
#include "errorHandler.h"
#include "NHD0420Driver.h"
#include "ButtonHandler.h"
#include "avr_f64.h"
#include "math.h"


extern void vApplicationIdleHook( void );
void vAlgorithmus1(void *pvParameters);
void vAlgorithmus2(void *pvParameters);
void vAlgorithmus3(void *pvParameters);
void vAusgabe(void *pvParameters);

TaskHandle_t Algorytmus1;
TaskHandle_t Algorytmus2;
TaskHandle_t Algorytmus3;
TaskHandle_t Ausgabe;

EventGroupHandle_t xSettings;

double ulPI= 1;
double ulPI4 = 1;
uint64_t i = 0;
uint32_t eMilisec = 0;
char ucPi_berechnet[12];
float usPi_5 = 3.14159;
char ucPi_gegeben[12];
int usZeitstop = 0;

double ula = 1;
double ulA = 0;
double ulb = 1/sqrt(2);
double ulB = 0;
double uls = 0.25;
double ulS = 0;
	
float64_t f64_a = 1;
float64_t f64_A = 0;
float64_t f64_b = 0;
float64_t f64_B = 0;
float64_t f64_s = 0;
float64_t f64_S = 0;

float64_t f64_PI = 0;


void vApplicationIdleHook( void )
{	
	
}

int main(void)
{
    resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
	
	xTaskCreate( vAlgorithmus1, (const char *) "Algorytmus1", configMINIMAL_STACK_SIZE+100, NULL, 1, &Algorytmus1);
	xTaskCreate( vAlgorithmus2, (const char *) "Algorytmus2", configMINIMAL_STACK_SIZE+100, NULL, 1, &Algorytmus2);
	xTaskCreate( vAlgorithmus3, (const char *) "Algorytmus2", configMINIMAL_STACK_SIZE+200, NULL, 1, &Algorytmus3);
	xTaskCreate( vAusgabe, (const char *) "Ausgabe", configMINIMAL_STACK_SIZE+300, NULL, 2, &Ausgabe);
	
	xSettings = xEventGroupCreate();

	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"FreeRTOS 10.0.1");
	vDisplayWriteStringAtPos(1,0,"EDUBoard 1.0");
	vDisplayWriteStringAtPos(2,0,"Template");
	vDisplayWriteStringAtPos(3,0,"ResetReason: %d", reason);
	vTaskStartScheduler();
	return 0;
}

/*--------------------------------------------------------------------------------------*/
/* Algorithmus 1: PI Berechnen mit der Leibniz Reihe */
/*--------------------------------------------------------------------------------------*/

void vAlgorithmus1(void *pvParameters) {
	(void) pvParameters;
	for(i=0;i<9999999;i++) 
	{
		xEventGroupWaitBits(xSettings,1,pdFALSE,pdFALSE,portMAX_DELAY);
		ulPI4 = (ulPI4-(1.0/(3+i*4))+(1.0/(5+i*4)));
		ulPI = ulPI4 *4;	
	}
}

/*--------------------------------------------------------------------------------------*/
/* Algorithmus 2: PI Berechnen mit der Gauss-Legendre Reihe */
/*--------------------------------------------------------------------------------------*/

void vAlgorithmus2(void *pvParameters) {
	(void) pvParameters;

	vTaskSuspend( Algorytmus2 );
	
	for(i=0;i<99999999;i++)
	{
		xEventGroupWaitBits(xSettings,1,pdFALSE,pdFALSE,portMAX_DELAY);
		
		if (i < 25)
		{
			ulA = (ula+ulb)/2;
			ulB = sqrt(ula*ulb);
			ulS = uls-(pow(2,i)*pow((ula-ulA),2));
			ulPI = pow(ulA,2)/uls;
			ula = ulA;
			ulb = ulB;
			uls = ulS;
		}
	}
}
/*--------------------------------------------------------------------------------------*/
/* Algorithmus 3: PI Berechnen mit der Gauss-Legendre Reihe und mit float 64 Bit library */
/*--------------------------------------------------------------------------------------*/

void vAlgorithmus3(void *pvParameters) {
	(void) pvParameters;
	vTaskSuspend( Algorytmus3 );
	f64_b = f_sd(1/sqrt(2));
	f64_s = f_sd(0.25);
	f64_a = f_sd(1);
	
	for(i=0;i<99999999;i++)
	{
		xEventGroupWaitBits(xSettings,1,pdFALSE,pdFALSE,portMAX_DELAY);
		if (i < 25)
		{
			f64_A = f_div((f_add(f64_a,f64_b)),f_sd(2));
			f64_B = f_sqrt(f_mult(f64_a,f64_b));
			f64_S = f_sub(f64_s,(f_mult(f_sd(pow(2,i)),f_mult(f_sub(f64_a,f64_A),f_sub(f64_a,f64_A)))));
			f64_PI = f_div(f_mult(f64_A,f64_A),f64_s);
			f64_a = f64_A;
			f64_b = f64_B;
			f64_s = f64_S;
		}
		if (i > 2)
		{
			usZeitstop = 1;
		}
	}
}

/*--------------------------------------------------------------------------------------*/
/* Der Ausgabe Task händelt die Eingabe durch Buttons und die Ausgabe vom Display */
/*--------------------------------------------------------------------------------------*/

void vAusgabe(void *pvParameters) {
	(void) pvParameters;
	char ucPI[20] ;
	char ucInteration[20] ;
	int eDisplayupdatetimer = 0;
	int usReset = 0;
	int usAlgorithmusmodus = 0;
	sprintf(ucPi_gegeben,"%.5f", usPi_5);
	 
/* Timer initialisieren und auf 1 ms stellen */

	TCD0.CTRLA = TC_CLKSEL_DIV256_gc ;
	TCD0.CTRLB = 0x00;
	TCD0.INTCTRLA = 0x03;
	TCD0.PER = 0x7F;
	
	for(;;) {
		
		updateButtons();
		if (getButtonPress(BUTTON1)== SHORT_PRESSED)
		{
			xEventGroupSetBits(xSettings,1);
		}
		if (getButtonPress(BUTTON2)== SHORT_PRESSED)
		{
			xEventGroupClearBits(xSettings,1);
		}
		if (getButtonPress(BUTTON3)== SHORT_PRESSED)
		{
			usReset = 1;
		}
		if (getButtonPress(BUTTON4)== SHORT_PRESSED)
		{
			if (usAlgorithmusmodus < 2)
			{
				usAlgorithmusmodus++;
			}
			else
			{
				usAlgorithmusmodus = 0;
			}
			usReset = 1;
		}
		
		switch (usAlgorithmusmodus)
		{
			case 0:
				vTaskSuspend( Algorytmus3 );
				vTaskSuspend( Algorytmus2 );
				vTaskResume( Algorytmus1  );
				f64_PI = f_sd(ulPI);
				break;
			case 1:
				vTaskSuspend( Algorytmus3 );
				vTaskSuspend( Algorytmus1 );
				vTaskResume( Algorytmus2  );
				f64_PI = f_sd(ulPI);
				break;
			case 2:
				vTaskSuspend( Algorytmus2 );
				vTaskSuspend( Algorytmus1 );
				vTaskResume( Algorytmus3  );
				break;
			default:break;
			
		}
		
		if (usReset == 1)
		{
			usReset = 0;
			xEventGroupClearBits(xSettings,1);
			vTaskDelay(10 / portTICK_RATE_MS);
			ulPI = 1;
			i = 0;
			eMilisec = 0;
			ulPI4 = 1;
			ula = 1;
			ulb = 1/sqrt(2);
			uls = 0.25;
			usZeitstop = 0;
			f64_b = f_sd(1/sqrt(2));
			f64_s = f_sd(0.25);
			f64_a = f_sd(1);
			f64_PI = f_sd(0);
		}
		
		if (eDisplayupdatetimer > 50)
		{
			eDisplayupdatetimer = 0;
			char* tempResultString = f_to_string(f64_PI, 20, 20);
			sprintf(ucPI, "%s", tempResultString);
			sprintf(ucInteration,"%ld", i);
			vDisplayClear();
			vDisplayWriteStringAtPos(0,0,"alg. %d PI:",usAlgorithmusmodus+1);
			vDisplayWriteStringAtPos(1,0,"%s",ucPI);
			if ((xEventGroupGetBits(xSettings) && 0x01) == 1)
			{
				vDisplayWriteStringAtPos(2,0,"Running");
			}
			else
			{
				vDisplayWriteStringAtPos(2,0,"Stopped");
			}
			vDisplayWriteStringAtPos(3,0,"t:%d ms",eMilisec);
			vDisplayWriteStringAtPos(3,10,"i: %s",ucInteration);
		}
		eDisplayupdatetimer++;
		vTaskDelay(10 / portTICK_RATE_MS);
	}
}

/*--------------------------------------------------------------------------------------*/
/* Der Interrrupt ist zuständig für die Zeitmessung */
/*--------------------------------------------------------------------------------------*/

ISR(TCD0_OVF_vect)
{
	sprintf(ucPi_berechnet,"%.5f", ulPI-0.000005);
	
	if ((xEventGroupGetBitsFromISR(xSettings) && 0x01) == 1) 
	{
		if ((strcmp(ucPi_berechnet,ucPi_gegeben)== 0)|| (usZeitstop == 1))
		{
			
		}
		else 
		{
			eMilisec++;
		}
	}
}