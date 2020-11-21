#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "semphr.h"
void Task_ValADC(void *param);
void Task_ButonEmergency(void *param);
void Task_Transmission(void *param);
void Task_Pulse_sensor(void *param);

TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;
TaskHandle_t Task_Handle3;
TaskHandle_t Task_Handle4;
SemaphoreHandle_t xCountingSemaphore;
struct package
{
  int temperature;
  bool buton;  
  int pulsesensor;
};
struct package package;

void setup() 
{
  Serial.begin(9600);
  xTaskCreate(Task_ValADC, "Task1",100, NULL,1, &Task_Handle1);
  xTaskCreate(Task_ButonEmergency, "Task2",100, NULL,1, &Task_Handle2); 
  xTaskCreate(Task_Transmission, "Task3",100, NULL,1, &Task_Handle3); 
  xTaskCreate(Task_Pulse_sensor, "Task4",100, NULL,1, &Task_Handle4);
  xCountingSemaphore = xSemaphoreCreateCounting(3,0);
  xSemaphoreGive(xCountingSemaphore); 
}

void loop() {}

void Task_ValADC(void *param)
{
 (void) param;
 const ADC_VREF_TYPE ((0<<REFS1)|(0<<REFS0)|(0<<ADLAR));
 while(1)
 {   
  xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
  Serial.println("Temperatura");
  float stepADC=0.0048828125;
  DIDR0=(1<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (1<<ADC1D) | (0<<ADC0D);
  ADMUX=ADC_VREF_TYPE;
  ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);        
  ADMUX= 0 | ADC_VREF_TYPE;
  delayMicroseconds(10);
  ADCSRA|=(1<<ADSC);
  while((ADCSRA&(1<<ADIF))==0){}
  ADCSRA |= (1<<ADIF);
  unsigned int sensorValue = ADCW;
  package.temperature = sensorValue; 
  xSemaphoreGive(xCountingSemaphore);
  vTaskDelay(1000/portTICK_PERIOD_MS); 
 }
}

void Task_ButonEmergency(void *param)
{  
  bool ButonState = 0;
  bool PreviousState = 0;
  bool BuzzerState = 0;
  DDRD = 0x00;
  PORTD = 0xFF; 
 (void) param;
 while(1)
 { 
  xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
  Serial.println("Butonul");
  ButonState = (PIND & (1 << PIND2 ));
  if( ButonState != PreviousState)
    {
      BuzzerState =! BuzzerState;
      if(ButonState == 0)
        {
           ButonState =! ButonState;     
        }
    }
  PreviousState = ButonState;   
  package.buton = BuzzerState;   
  xSemaphoreGive(xCountingSemaphore);
  vTaskDelay(1000/portTICK_PERIOD_MS); 
 }
}
void Task_Pulse_sensor(void *param)
{
 (void) param;
 const ADC_VREF_TYPE ((0<<REFS1)|(0<<REFS0)|(0<<ADLAR));
 while(1)
 {    
  xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
  Serial.println("Puls");
  float stepADC=0.0048828125;
  DIDR0=(1<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (1<<ADC1D) | (0<<ADC0D);
  ADMUX=ADC_VREF_TYPE;
  ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);        
  ADMUX= 1 | ADC_VREF_TYPE;
  delayMicroseconds(10);
  ADCSRA|=(1<<ADSC);
  while((ADCSRA&(1<<ADIF))==0){}
  ADCSRA |= (1<<ADIF);
  unsigned int sensorValue = ADCW;
  package.pulsesensor = sensorValue/10; 
  xSemaphoreGive(xCountingSemaphore);
  vTaskDelay(1000/portTICK_PERIOD_MS);  
 }
}
void Task_Transmission(void *param)
{
  (void) param;
  RF24 radio(9, 10);        
  const byte address[6] = "00002";
  radio.begin();                  
  radio.openWritingPipe(address); 
  radio.setPALevel(RF24_PA_MIN);  
  radio.stopListening();
    while(1)
    {
      xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
      Serial.println("Transmisia");
      radio.write(&package, sizeof(package)); 
      xSemaphoreGive(xCountingSemaphore);
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }  
}
