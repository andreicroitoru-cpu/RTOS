#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "semphr.h"
#include "queue.h"
void Task_ValADC(void *param);
void Task_ButonEmergency(void *param);
void Task_Transmission(void *param);
void Task_Pulse_sensor(void *param);
TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;
TaskHandle_t Task_Handle3;
TaskHandle_t Task_Handle4;
QueueHandle_t Global_Queue = 0;
QueueHandle_t Global_Queue2 = 0;
typedef struct package
{
  int temperature; 
  int pulsesensor;
}package;
package package1;

void setup() 
{
  xTaskCreate(Task_ValADC, "Task1",100, NULL,1, &Task_Handle1);
  xTaskCreate(Task_ButonEmergency, "Task2",100, NULL,1, &Task_Handle2); 
  xTaskCreate(Task_Transmission, "Task3",100, NULL,2, &Task_Handle3); 
  xTaskCreate(Task_Pulse_sensor, "Task4",100, NULL,1, &Task_Handle4);
  Global_Queue=xQueueCreate(15, sizeof(package));
  Global_Queue2=xQueueCreate(1, sizeof(bool));
  vTaskStartScheduler();
}

void loop() {}

void Task_ValADC(void *param)
{
 (void) param;
 float stepADC=0.0048828125;
 const ADC_VREF_TYPE ((0<<REFS1)|(0<<REFS0)|(0<<ADLAR));
 while(1)
 {   
  DIDR0=(1<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (1<<ADC1D) | (0<<ADC0D);
  ADMUX=ADC_VREF_TYPE;
  ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);        
  ADMUX= 0 | ADC_VREF_TYPE;
  ADCSRA|=(1<<ADSC);
  while((ADCSRA&(1<<ADIF))==0){}
  ADCSRA |= (1<<ADIF);
  unsigned int sensorValue = ADCW;
  int degreesC = ( sensorValue)/10.0;
  package1.temperature = degreesC; 
  if(!xQueueSend(Global_Queue,&package1,1000))
  {
    Serial.println("FAIL");    
    }
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
  if(!xQueueSend(Global_Queue2,& BuzzerState ,1000))
  {
    Serial.println("FAIL");   
  }
  vTaskDelay(1000/portTICK_PERIOD_MS); 
 }
}
void Task_Pulse_sensor(void *param)
{
 (void) param;
 float stepADC=0.0048828125;
 const ADC_VREF_TYPE ((0<<REFS1)|(0<<REFS0)|(0<<ADLAR));
 int sensorValue;
 double alpha=0.75;
 int period=20;
 double refresh=0.0;
 unsigned int beat; 
 static double oldValue=0;
 static double oldrefresh=0; 
 double value=alpha*oldValue+(0-alpha)*beat;
 
 while(1)
 {    
  DIDR0=(1<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (0<<ADC2D) | (1<<ADC1D) | (0<<ADC0D);
  ADMUX=ADC_VREF_TYPE;
  ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);        
  ADMUX= 1 | ADC_VREF_TYPE;
  delayMicroseconds(10);
  ADCSRA|=(1<<ADSC);
  while((ADCSRA&(1<<ADIF))==0){}
  ADCSRA |= (1<<ADIF);
  beat = ADCW;
  sensorValue = (beat/10);
  package1.pulsesensor = sensorValue; 
  refresh=value-oldValue; 
  oldValue=value;
  oldrefresh=refresh; 
  if(!xQueueSend(Global_Queue,&package1 ,1000))
  {
    Serial.println("FAIL");    
  }
  vTaskDelay(1000/portTICK_PERIOD_MS);  
 }
}
void Task_Transmission(void *param)
{
  (void) param;
  bool buton_local;
  typedef struct package_local
{
  int temperature_local; 
  int pulsesensor_local;
}package_local;
package_local package_local1;

  RF24 radio(9, 10);        
  const byte address[6] = "00002";
  radio.begin();                  
  radio.openWritingPipe(address); 
  radio.setPALevel(RF24_PA_MIN);  
  radio.stopListening();
    while(1)
    {
      if(xQueueReceive(Global_Queue,&package_local1,1000))
      {
          radio.write(&package_local1, sizeof(package_local));
      }
      else
      {
          Serial.println("NEPrimit");   
      }
        if(xQueueReceive(Global_Queue2,&buton_local,1000))
      {
           radio.write(&buton_local, sizeof(bool));
      }
      else
      {
           Serial.println("NEPrimit");   
      }
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }  
}
