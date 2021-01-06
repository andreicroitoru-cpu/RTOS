#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "semphr.h"
LiquidCrystal_I2C lcd(0x27,20,4);  
void Task_Reception(void *param);
void Task_Display(void *param);
void Task_Buzzer(void *param);
TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;
TaskHandle_t Task_Handle3;
SemaphoreHandle_t xMutex;
bool buton;
struct package
{
  int temperature;
  int pulsesensor;
};

struct package package;
void setup() { 
   lcd.init();                      
   lcd.backlight();
   xTaskCreate(Task_Reception, "Task1",100, NULL,2, &Task_Handle1);
   xTaskCreate(Task_Display, "Task2",100, NULL,1, &Task_Handle2);
   xTaskCreate(Task_Buzzer, "Task3",100, NULL,1, &Task_Handle3);
   xMutex= xSemaphoreCreateMutex();
   vTaskStartScheduler();
}

void loop() {}

void Task_Reception(void *param)
{
  (void) param;
  RF24 radio(9, 10);     
  const byte address[6] = "00002";
  radio.begin();                  
  radio.openReadingPipe(0, address); 
  radio.setPALevel(RF24_PA_MIN);  
  radio.startListening();
  while(1)
    {  
      if (radio.available())             
    {
      radio.read(&package, sizeof(package));    
      radio.read(&buton, sizeof(bool));   
    }
      vTaskDelay(1000/portTICK_PERIOD_MS); 
    }  
}
void Task_Buzzer(void *param)
{
  (void) param;
  DDRD = 0b00000100;
  while(1)
  {
    if(buton == 0)
      {
         PORTD = 0b00000000;       
      }
      else
      {
        xSemaphoreTake(xMutex,portMAX_DELAY);
        PORTD = 0b00000100;
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("URGENTA");
        xSemaphoreGive(xMutex);     
      }   
      vTaskDelay(1000/portTICK_PERIOD_MS); 
    }  
}
void Task_Display(void *param)
{
  (void) param;
  while(1)
  {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    lcd.setCursor(0,0);
    lcd.print("Temperature:");
    lcd.print(package.temperature);
    lcd.setCursor(0,1);
    lcd.print("Pulse:");
    lcd.print(package.pulsesensor);
    xSemaphoreGive(xMutex);
    vTaskDelay(1000/portTICK_PERIOD_MS); 
    }  
}
