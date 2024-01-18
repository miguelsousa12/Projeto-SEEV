/*Nome ALUNO A- Luis Jesus
 Nome ALUNO B- Miguel Sousa
 IPLEIRIA - Instituto Politécnico de Leiria
 ESTG - Escola Superior de Tecnologia e Gestão
 LEAU- Licenciatura em Engenharia Automóvel
 SEEV - Sistemas Elétricos e Eletrónicos de Veículos

 TP1: Pretende-se  neste  trabalho  prático  a  implementação  de um algoritmo para em caso de acidente rodoviário, este sistema possuir a capacidade de acionar um dispositivo de segurança passiva, que são os airbags, bem como, a partilha da localização do acidente, acende os 4 piscas, a buzina e os máximos caso se encontre numa situação de pouca luminosidade.
 LINK: https://youtu.be/PQpY12aMU5c
 */

#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <TFT_eSPI.h>
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <freertos/queue.h>
//Associação de pinos
#define AIRBAG_PIN 13
#define GPSRX_PIN 16
#define GPSTX_PIN 17
#define PISCAS_PIN 25
#define BUZER_PIN 12
#define ldrPin  34
#define MAX_PIN 27
#define botaoPin 33
int encerrarTarefa = 0; // Inicialização da variável
void IRAM_ATTR handleInterrupt() { //Função de interrupção
	pinMode(BUZER_PIN, OUTPUT);
	encerrarTarefa = 1;

	digitalWrite(BUZER_PIN, LOW); // Desliga a buzina
	pinMode(MAX_PIN, OUTPUT); //Declara os máximos como saida
	digitalWrite(MAX_PIN, LOW); // Desliga os máximos

	Serial.println("Interrupção acionada!");
}

// Inicia o acelerómetro
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

TinyGPSPlus gps;  // Inicia o GPS
/* Declaração das tarefas */
void vTask1_Aceleracao(void *pvParameters);
void vTask2_Airbags(void *pvParameters);
void vTask3_Piscas(void *pvParameters);
void vTask4_Buzer(void *pvParameters);
void vTask5_GPS(void *pvParameters);
void vTask6_TFT(void *pvParameters);
void vTask7_Maximos(void *pvParameters);

TFT_eSPI tft; //Inicia o TFT
//Declaração das Handles das tarefas
TaskHandle_t airbagHandle;
TaskHandle_t aceleracaoHandle;
TaskHandle_t piscasHandle;
TaskHandle_t buzerHandle;
#define QUEUE_SIZE 10
//Criação de estrutura para o GPS mandar os dados para a Queue
struct DadosGPS {
	float latitude;
	float longitude;
	float altitude;
};
//Declaração da Queue
QueueHandle_t dadosQueue;

void setup(void) {

	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1); //Define que a função Setup seja a de maior prioridade
	Serial.begin(115200);
	//Configuração do botão
	pinMode(botaoPin, INPUT_PULLUP);

	tft.begin(); // Inicializa o TFT
	attachInterrupt(digitalPinToInterrupt(botaoPin), handleInterrupt, FALLING); // Configurar a interrupção do botão
	//Configuração do TFT
	tft.begin();
	tft.setRotation(1);
	tft.fillScreen(TFT_BLACK);
	tft.setTextSize(2);

	// Cria uma Queue para armazenar dados do GPS
	dadosQueue = xQueueCreate(QUEUE_SIZE, sizeof(DadosGPS));
	//Criação da tarefa que lê o acelerómetro
	xTaskCreatePinnedToCore(vTask1_Aceleracao, "Aceleracao", 4096, NULL, 4, &aceleracaoHandle, 1);

}

void vTask1_Aceleracao(void *pvParameters) {
	float totalSpeed = 0;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount(); //Leitura dos Ticks
	while (!accel.begin()) { // Verificação
		Serial.println(
				"O sensor ADXL345 não foi detectado. Verifique as conexões.");
		delay(1000);
	}
// Configuração do acelerómetro para leitura de 16G em cada eixo
	accel.setRange(ADXL345_RANGE_16_G);

	for (;;) {

		while (totalSpeed < 20) {

			sensors_event_t event;
			accel.getEvent(&event);
			// Imprime os dados no serial monitor
			Serial.print("Aceleracao X: ");
			Serial.print(event.acceleration.x);
			Serial.print(" m/s^2\t");
			Serial.print("Aceleracao Y: ");
			Serial.print(event.acceleration.y);
			Serial.print(" m/s^2\t");
			Serial.print("Aceleracao Z: ");
			Serial.print(event.acceleration.z);
			Serial.println(" m/s^2");

			// Cálculo da velocidade total
			totalSpeed = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2)
							+ pow(event.acceleration.z, 2));

			Serial.print(" Aceleraçao Total: ");
			Serial.println(totalSpeed);
			vTaskDelayUntil(&xLastWakeTime, ( 100 / portTICK_PERIOD_MS ));

		}
		//Criação das restantes tarefas
		xTaskCreatePinnedToCore(vTask2_Airbags, "Airbags", 1024, NULL, 4,
				&airbagHandle, 1);
		xTaskCreatePinnedToCore(vTask3_Piscas, "Piscas", 1024, NULL, 4,
				&piscasHandle, 1);
		xTaskCreatePinnedToCore(vTask4_Buzer, "Buzer", 1024, NULL, 4,
				&buzerHandle, 1);
		xTaskCreatePinnedToCore(vTask5_GPS, "GPS", 4096, NULL, 3, NULL, 1);
		xTaskCreatePinnedToCore(vTask6_TFT, "TFT", 4096, NULL, 3, NULL, 1);
		xTaskCreatePinnedToCore(vTask7_Maximos, "Maximos", 2048, NULL, 3, NULL,
				1);
		vTaskDelete(NULL);
	}
}

/*-----------------------------------------------------------*/

void vTask2_Airbags(void *pvParameters) {

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		pinMode(AIRBAG_PIN, OUTPUT);
		digitalWrite(AIRBAG_PIN, HIGH);
		Serial.println("Abertura do airbag");
		vTaskDelete(NULL);
	}

}

void vTask3_Piscas(void *pvParameters) {

	pinMode(PISCAS_PIN, OUTPUT);
	digitalWrite(PISCAS_PIN, LOW);
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();


	// Inverte o estado dos LED's
	for (;;) {

		pinMode(PISCAS_PIN, OUTPUT);
		digitalWrite(PISCAS_PIN, !digitalRead(PISCAS_PIN));
		vTaskDelayUntil(&xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ));

	}

}

void vTask4_Buzer(void *pvParameters) {

	pinMode(BUZER_PIN, OUTPUT); // Configura o pino do buzzer
	digitalWrite(BUZER_PIN, LOW); // Inicializa o pino do buzzer a 0
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskPrioritySet(piscasHandle, 2); // Muda a prioridade de 4 para 2
	vTaskPrioritySet(buzerHandle, 2);
	for (;;) {

		if (encerrarTarefa) { // Apaga a própria tarefa
			Serial.println("Tarefa do buzzer apagada...");
			vTaskDelete(NULL);
		}
		pinMode(BUZER_PIN, OUTPUT);
		// Inverte o estado do pino do buzzer
		digitalWrite(BUZER_PIN, !digitalRead(BUZER_PIN));
		vTaskDelayUntil(&xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ));

	}

}
void vTask5_GPS(void *pvParameters) {
	TickType_t xLastWakeTime;
	DadosGPS dados;

	xLastWakeTime = xTaskGetTickCount();
	Serial2.begin(9600);
	TinyGPSPlus gps;
	delay(100);
	for (;;) {

		if (Serial2.available() > 0) {
			//
			if (gps.encode(Serial2.read())) {
				if (1)  //gps.location.isValid())
				{
					dados.latitude = gps.location.lat();
					dados.longitude = gps.location.lng();
					dados.altitude = gps.altitude.meters();
					xQueueSend(dadosQueue, &dados, portMAX_DELAY);
					delay(10); // Aguarda um curto período de tempo para evitar a sobrecarga do CPU

					//Apresentação dos dados no serial monitor
					Serial.print(F("- latitude: "));
					Serial.println(gps.location.lat());

					Serial.print(F("- longitude: "));
					Serial.println(gps.location.lng());

					Serial.print(F("- altitude: "));
					if (gps.altitude.isValid())
						Serial.println(gps.altitude.meters());
					else
						Serial.println(F("INVALID"));
				} else {
					Serial.println(F("- location: INVALID"));
				}

				Serial.print(F("- speed: "));
				if (gps.speed.isValid()) {
					Serial.print(gps.speed.kmph());
					Serial.println(F(" km/h"));
				} else {
					Serial.println(F("INVALID"));
				}

				Serial.print(F("- GPS date&time: "));
				if (gps.date.isValid() && gps.time.isValid()) {
					Serial.print(gps.date.year());
					Serial.print(F("-"));
					Serial.print(gps.date.month());
					Serial.print(F("-"));
					Serial.print(gps.date.day());
					Serial.print(F(" "));
					Serial.print(gps.time.hour());
					Serial.print(F(":"));
					Serial.print(gps.time.minute());
					Serial.print(F(":"));
					Serial.println(gps.time.second());
				} else {
					Serial.println(F("INVALID"));
				}

			}
		}

		if (millis() > 5000 && gps.charsProcessed() < 10) // Se não forem os dados do GPS, imprime a seguinte mensagem
			Serial.println(F("No GPS data received: check wiring"));

		vTaskDelayUntil(&xLastWakeTime, ( 500 / portTICK_PERIOD_MS ));

	}

}

void vTask6_TFT(void *pvParameters) {
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	DadosGPS dados;

	for (;;) {

		// Apresenta os dados do GPS no TFT

		if (xQueueReceive(dadosQueue, &dados, portMAX_DELAY)) { //Verifica se existe dados na Queue
			tft.setTextColor(TFT_RED);
			tft.setCursor(20, tft.height() / 2 - 60);
			tft.print("ACIDENTE");
			tft.setCursor(20, tft.height() / 2 - 40);
			tft.setTextColor(TFT_BLUE);
			tft.print("Latitude: ");
			tft.println(dados.latitude, 6);
			tft.setCursor(20, tft.height() / 2);
			tft.print("Longitude: ");
			tft.println(dados.longitude, 6);
			tft.setCursor(20, tft.height() / 2 + 40);
			tft.print("Altitude: ");
			tft.println(dados.altitude, 2);
			Serial.print(dados.altitude);
			delay(10);
			// Aguarda um curto período de tempo antes de limpar o ecrã novamente
			vTaskDelayUntil(&xLastWakeTime, ( 500 / portTICK_PERIOD_MS ));

		}
	}

}
void vTask7_Maximos(void *pvParameters) {
	TickType_t xLastWakeTime;
	//Associa o valor lido no pino da LDR à variável ldrValue
	int ldrValue = analogRead(ldrPin);
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		int ldrValue = analogRead(ldrPin);

		Serial.print("Valor do LDR: ");
		Serial.println(ldrValue);
		if (ldrValue < 400) { //Se o valor da LDR for inferior a 400
			pinMode(MAX_PIN, OUTPUT);
			// Liga o led
			digitalWrite(MAX_PIN, HIGH);

		}
		//apaga a tarefa
		vTaskDelete(NULL);
	}
}

void loop() {
	vTaskDelete(NULL);

}
