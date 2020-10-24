#include <Arduino_FreeRTOS.h>

void Task_ValADC(void *param);
void Task_Print(void *param);

TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;

void setup() 
{
  Serial.begin(9600);
  xTaskCreate(Task_ValADC, "Task1",100, NULL,2, &Task_Handle1);
  xTaskCreate(Task_Print, "Task2",100, NULL,1, &Task_Handle2); 
}

void loop() {}

void Task_ValADC(void *param)
{
 (void) param;
 const ADC_VREF_TYPE ((0<<REFS1)|(0<<REFS0)|(0<<ADLAR));
 while(1)
 {   
  //float stepADC=0.0048828125;
  DIDR0=(0<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (ADC2D) | (0<<ADC1D) | (0<<ADC0D);
  ADMUX=ADC_VREF_TYPE;
  ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);        
  ADMUX= 0 | ADC_VREF_TYPE;
  ADCSRA|=(1<<ADSC);
  while((ADCSRA&(1<<ADIF))==0){}
  ADCSRA |= (1<<ADIF); 
  //unsigned int sensorValue = ADCW;
  Serial.println(ADCW);
  vTaskDelay(1000/portTICK_PERIOD_MS); 
 }
}

void Task_Print(void *param)
{
 (void) param;
 while(1)
 {
  Serial.println("Task2");
  vTaskDelay(1000/portTICK_PERIOD_MS); 
 }
}
