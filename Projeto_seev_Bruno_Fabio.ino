#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "funcoes_barra_leds.h"
#include <FastLED.h>
#include <ESP32_Servo.h>

#define VELOCIDADEPERIOD 1000
#define DISPLAYPERIOD 200
#define ADCRES 12
#define DEBOUNCE_TIME 100

// Definition of ADC resolution
#define ADCRES 12
#define VELOCIDADE 35
#define RPM 32
#define BUTTON 14
#define DRS_MANUAL 34
#define BARRA_LED 5
Servo DRS;

#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    24
#define BARRA_LEDS_RPM_MAX 8000
#define BARRA_LEDS_RPM_MIN 3000
#define BARRA_LEDS_RPM_INICIA_PISCA 7500
#define INTENSIDADE_BARRA_LEDS 64
#define BARRA_LEDS_ESTADO_OFF 0
#define BARRA_LEDS_ESTADO_ON 1

//========================================================================>
/*Definicoes de menu */
//Prototipo dee tarefas
void vVelocidade(void *pvParameters);
void vDisplay(void *pvParameters);
void vDRS(void *pvParameters);
void vRPM(void *pvParameters);
void vBotao(void *pvParameters);
void vLED(void *pvParameters);

void IRAM_ATTR Botao_int();
//========================================================================>

TaskHandle_t TaskHandle_1; //Handle para matar tarefa

//========================================================================>

QueueHandle_t velocidadeQueue;
QueueHandle_t RPMQueue;
QueueHandle_t RPMQueuetodisplay;
QueueHandle_t EstadoQueue;
QueueHandle_t EstadoQueuetodisplay;
QueueHandle_t anguloQueue;
QueueHandle_t velocidadeQueuetodisplay;
//========================================================================>

SemaphoreHandle_t BotaoSemaphore;

//========================================================================>

void setup() {

	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

//========================================================================>
	/*ADC*/
	analogReadResolution(ADCRES);

//========================================================================>

	BotaoSemaphore = xSemaphoreCreateBinary();
//========================================================================>

	/*Inputs*/
	pinMode(RPM, INPUT_PULLUP);
	pinMode(BUTTON, INPUT_PULLUP);
	pinMode(VELOCIDADE, INPUT_PULLUP);
	pinMode(DRS_MANUAL, INPUT_PULLUP);
pinMode(BARRA_LED, OUTPUT);
	DRS.attach(16);

//========================================================================>

	attachInterrupt(digitalPinToInterrupt(BUTTON), Botao_int, RISING);
//========================================================================>

	/*Criacao de Queues*/

	EstadoQueue = xQueueCreate(1, sizeof(bool));
	RPMQueue = xQueueCreate(1, sizeof(int));
	velocidadeQueue = xQueueCreate(1, sizeof(int));
	anguloQueue = xQueueCreate(1, sizeof(int));
	RPMQueuetodisplay = xQueueCreate(1, sizeof(int));
	EstadoQueuetodisplay = xQueueCreate(1, sizeof(int));
	velocidadeQueuetodisplay = xQueueCreate(1, sizeof(int));
//========================================================================>

	/*Criacao de tarefas*/
	xTaskCreatePinnedToCore(vVelocidade, "velocidade", 800, NULL, 11, NULL, 1);
	xTaskCreatePinnedToCore(vDisplay, "Display", 5000, NULL, 13, NULL, 0);
	xTaskCreatePinnedToCore(vDRS, "DRS", 5000, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(vRPM, "TPS", 1024, NULL, 13, NULL, 0);
	xTaskCreatePinnedToCore(vBotao, "Menu button", 800, NULL, 11, NULL, 1);
	xTaskCreatePinnedToCore(vLED, "Menu button", 1024, NULL, 12, NULL, 0);
//========================================================================>
	/*Inicializacao da porta serie*/

	Serial.begin(9600);
	while (!Serial)
		;
//	Serial2.begin(9600, SERIAL_8N1, 16, 17);
//    while (!Serial2);

	while (!Serial)
		;
}

void loop() {
	vTaskDelete(NULL);  //matar a tarefa arduino
}

void vDisplay(void *pvParameters) {
	int RPM_per = 0, RPM_tmp = 0, angulo = 0, ANGULO_tmp = 0, velociadade = 0,
			Velocidade_tmp = 0;
	bool estado_tmp = 0, estado = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		if (xQueueReceive(velocidadeQueuetodisplay, &Velocidade_tmp, 0) == pdTRUE) {
			velociadade = Velocidade_tmp;
		}
		if (xQueueReceive(EstadoQueuetodisplay, &estado_tmp, 0) == pdTRUE) {
			estado = estado_tmp;
		}
		if (xQueueReceive(anguloQueue, &ANGULO_tmp, 0) == pdTRUE) {
			angulo = ANGULO_tmp;
		}

		if (xQueueReceive(RPMQueuetodisplay, &RPM_tmp, 0) == pdTRUE) {
			RPM_per = RPM_tmp;
		}

//		        Serial.println("=================================================");
//			    Serial.print("estado:");
//				Serial.println(estado);
//				Serial.print("angulo:");
//			    Serial.println(angulo);
//				Serial.print("velocidade:");
//				Serial.println(velociadade);
//				Serial.print("rpm:");
//				Serial.println(RPM_per);
//			    Serial.println("=================================================");

		Serial.print("velociadade.val=");
		Serial.print(velociadade);
		Serial.write(0xff);
		Serial.write(0xff);
		Serial.write(0xff);

		Serial.print("angulo.val=");
		Serial.print(angulo);
		Serial.write(0xff);
		Serial.write(0xff);
		Serial.write(0xff);

		Serial.print("rpm.val=");
		Serial.print(RPM_tmp);
		Serial.write(0xff);
		Serial.write(0xff);
		Serial.write(0xff);

		Serial.print("estado.val=");
		Serial.print(estado);
		Serial.write(0xff);
		Serial.write(0xff);
		Serial.write(0xff);

		vTaskDelayUntil(&xLastWakeTime, (DISPLAYPERIOD / portTICK_PERIOD_MS));
	}

}

void vVelocidade(void *pvParamenters) {

	TickType_t xLastWakeTime = xTaskGetTickCount();
	short int velocidadeValue = 0;
	int velocidade = 0;
	for (;;) {
		// Serial.println("estou aquii ");
		velocidadeValue = analogRead(VELOCIDADE);

		velocidade = map(velocidadeValue, 0, 4095, 0, 350);
//		 Serial.print("VELOCIDADE: ");
//		 Serial.println(velocidade);

//	    Serial.print("velociadade.val=");
//		Serial.print(velocidade);
//
//		Serial.write(0xff);
//		Serial.write(0xff);
//		Serial.write(0xff);
		xQueueSendToBack(velocidadeQueue, &velocidade, 0);
		xQueueSendToBack( velocidadeQueuetodisplay, &velocidade, 0);
		vTaskDelayUntil(&xLastWakeTime,
				(VELOCIDADEPERIOD / portTICK_PERIOD_MS));
	}
}

void vDRS(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	bool Estado = 0, Estado_tmp = 0;
	int tpsIndex = 0, velocidadeIndex = 0, angle = 0, velocidade = 0;
	int RPM_per = 0, valor_angulo = 0;
	int velocidade_tmp = 0, RPM_tmp = 0;
	int indiceMedia = 0;
	const int numValoresMedia = 5;
	int valoresAngulo[numValoresMedia] = { 0 };
	const int angleTable[5][8] = {
			{ 0, 0, 0, 10, 20, 30, 40, 50 },
			{10, 20,30, 40, 50, 60, 70, 80 },
			{60, 70, 80, 90, 100, 110, 120, 130 },
			{0, 0, 0, 160, 170, 170, 180, 180 },
			{ 0, 0, 0, 160, 170, 170, 180, 180 } };

	for (;;) {

		if (xQueueReceive(EstadoQueue, &Estado_tmp, 0) == pdTRUE) {
			Estado = Estado_tmp;
		}
		if (xQueueReceive(velocidadeQueue, &velocidade_tmp, 0) == pdTRUE) {
			velocidade = velocidade_tmp;
		}
		if (xQueueReceive(RPMQueue, &RPM_tmp, 0) == pdTRUE) {
			RPM_per = RPM_tmp;
		}

//	    Serial.print("estado:");
//		Serial.println(Estado);
//		Serial.print("velocidade:");
//		Serial.println(velocidade);
//		Serial.print("rpm:");
//		Serial.println(RPM_per);

		if (Estado == 1) {
			tpsIndex = map(velocidade, 0, 350, 0, 4);
			velocidadeIndex = map(RPM_per, 0, 8000, 0, 7);

			angle = angleTable[tpsIndex][velocidadeIndex];
//	  xQueueSendToBack(anguloQueue, &angle, 0);
//	    Serial.println("Automatico");
//		Serial.println("ANGULO0: ");
//		Serial.println(angle);
		}
		if (Estado == 0) {
			valor_angulo = analogRead(DRS_MANUAL);
			angle = map(valor_angulo, 0, 4095, 0, 180);
			//	xQueueSendToBack(anguloQueue, &angle, 0);
//	    Serial.println("manual");
//		Serial.println("ANGULO1: ");
//		Serial.println(angle);
//		    Serial.println(valor_angulo);
			// Armazena o valor atual no array
//			valoresAngulo[indiceMedia] = angle;
//
//			// Calcula a média dos últimos 5 valores
//			int somaAngulos = 0;
//			for (int i = 0; i < numValoresMedia; i++) {
//				somaAngulos += valoresAngulo[i];
//			}
//			angle = somaAngulos / numValoresMedia;
//
//			// Atualiza o índice para o próximo valor
//			indiceMedia = (indiceMedia + 1) % numValoresMedia;
		}

//					Serial.print("angulo.val=");
//					Serial.print(angle);
//
//				    Serial.write(0xff);
//				    Serial.write(0xff);
//					Serial.write(0xff);
//				Serial.println("ANGULO1: ");
//				Serial.println(angle);
		DRS.write(angle);
		xQueueSendToBack(anguloQueue, &angle, 0);

		vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
	}
}

void vRPM(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	short int RPM_adc = 0;
	int RPM_valor = 0;

	for (;;) {
		RPM_adc = analogRead(RPM);
		RPM_valor = map(RPM_adc, 0, 4095, 0, 8000);
//Serial.println("RPM: ");
//Serial.println(RPM_valor);
//            Serial.print("rpm.val=");
//			Serial.print(RPM_valor);
//
//			Serial.write(0xff);
//		    Serial.write(0xff);
//		    Serial.write(0xff);

		xQueueSendToBack(RPMQueue, &RPM_valor, 0);
		xQueueSendToBack(RPMQueuetodisplay, &RPM_valor, 0);
		vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
	}
}

void vBotao(void *pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	short int option = 0;
	unsigned long time1 = 0, time2 = 0;
	bool Estado_drs = 0;

	// xSemaphoreTake(menuSemaphore, 0);
	for (;;) {
		xSemaphoreTake(BotaoSemaphore, portMAX_DELAY);
		//Redundancia
		time2 = millis();
		if ((time2 - time1) > DEBOUNCE_TIME) {
			time1 = time2;

			option++;

			if (option > 2) {
				option = 1;
			}

			switch (option) {

			case 1:

//				Serial.println("***************");
				Estado_drs = 0;

				break;

			case 2:

//				Serial.println("!!!!!!!!!!!!!!!!");
				Estado_drs = 1;

				break;

			}
//			            Serial.print("estado.val=");
//					    Serial.print(Estado_drs);
//
//						Serial.write(0xff);
//					    Serial.write(0xff);
//						Serial.write(0xff);
			xQueueSendToBack(EstadoQueue, &Estado_drs, 0);
			xQueueSendToBack(EstadoQueuetodisplay, &Estado_drs, 0);
		}

	}

}

void vLED(void *pvParameters) {
	char *pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = (char*) pvParameters;
	xLastWakeTime = xTaskGetTickCount();

	int RPM_valor = 0;
	u_int k = (BARRA_LEDS_RPM_MAX - BARRA_LEDS_RPM_MIN) / NUM_LEDS;
	u_int num_leds = 0;
	uint8_t funcao_pisca_barra_leds = 0;
	uint8_t estado_barra_leds_pisca = 0;

	CRGB leds[NUM_LEDS];

	FastLED.addLeds<LED_TYPE, BARRA_LED, COLOR_ORDER>(leds, NUM_LEDS);
	FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
	FastLED.clear();
	FastLED.show();
	FastLED.setBrightness(INTENSIDADE_BARRA_LEDS);

	for (;;) {

		xQueuePeek(RPMQueue, &RPM_valor, 0);

		num_leds = RPM_valor / k;
		if (RPM_valor > BARRA_LEDS_RPM_INICIA_PISCA) {
			xQueuePeek(RPMQueue, &RPM_valor, 0);
			for (int i = 0; i < NUM_LEDS; i++) {
				leds[i] = CRGB(0, 0, 255);
			}

			if (estado_barra_leds_pisca == BARRA_LEDS_ESTADO_ON) {
				FastLED.setBrightness(0);
				FastLED.show();
				estado_barra_leds_pisca = BARRA_LEDS_ESTADO_OFF;
				vTaskDelayUntil(&xLastWakeTime, (75 / portTICK_PERIOD_MS));
			}
			if (estado_barra_leds_pisca == BARRA_LEDS_ESTADO_OFF) {

				FastLED.setBrightness(INTENSIDADE_BARRA_LEDS);
				FastLED.show();
				estado_barra_leds_pisca = BARRA_LEDS_ESTADO_ON;
				vTaskDelayUntil(&xLastWakeTime, (75 / portTICK_PERIOD_MS));

			}

		} else {

			for (int i = 0; i < num_leds; i++) {
				leds[i] = CRGB(calcula_R(i), calcula_G(i), calcula_B(i));
			}
			for (int j = num_leds; j < NUM_LEDS; j++) {
				leds[j] = CRGB(0, 0, 0);
			}
			FastLED.show();
		}
//		Serial.println(media_rpm);

		vTaskDelayUntil(&xLastWakeTime, (200 / portTICK_PERIOD_MS));
	}
}

void Botao_int() {
	xSemaphoreGiveFromISR(BotaoSemaphore, NULL);
}
