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
SemaphoreHandle_t xCountingSemaphore;

struct package
{
  int temperature;
  bool buton;  
  int pulsesensor;
};

struct package package;
void setup() {
  
   Serial.begin(9600);
   xTaskCreate(Task_Reception, "Task1",100, NULL,1, &Task_Handle1);
   xTaskCreate(Task_Display, "Task2",100, NULL,1, &Task_Handle2);
   xTaskCreate(Task_Buzzer, "Task3",100, NULL,1, &Task_Handle3);
   xCountingSemaphore = xSemaphoreCreateCounting(2,0);
   xSemaphoreGive(xCountingSemaphore);
}

void loop() {}

void Task_Reception(void *param)
{
  (void) param;
  RF24 radio(9, 10); // CE, CSN         
  const byte address[6] = "00002";
  radio.begin();                  
  radio.openReadingPipe(0, address); 
  radio.setPALevel(RF24_PA_MIN);  
  radio.startListening();
  while(1)
    {  
      xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
      if (radio.available())              //Looking for the data.
    {
      radio.read(&package, sizeof(package));    //Reading the data
      Serial.print(package.temperature);
      Serial.println(" ");
      Serial.print(package.buton);
      Serial.println(" ");
      Serial.print(package.pulsesensor);
    }
      xSemaphoreGive(xCountingSemaphore);
      vTaskDelay(1000/portTICK_PERIOD_MS); 
    }  
}
void Task_Buzzer(void *param)
{
  (void) param;
  DDRD = 0b00000100;
 
  while(1)
  {
     xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
    if(package.buton == 0)
      {
         PORTD = 0b00000000;
         vTaskResume(Task_Handle2);
      }
      else
      {
        vTaskSuspend(Task_Handle2);
        PORTD = 0b00000100;
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("URGENTA");
        
      }
    
      xSemaphoreGive(xCountingSemaphore);
      vTaskDelay(1000/portTICK_PERIOD_MS); 
    }  
}
void Task_Display(void *param)
{
  (void) param;
  lcd.init();                      
  lcd.backlight();
 
  while(1)
  {
    xSemaphoreTake(xCountingSemaphore, portMAX_DELAY);
    lcd.setCursor(0,0);
    lcd.print("Temperature:");
    lcd.print(package.temperature);
    lcd.setCursor(0,1);
    lcd.print("Pulse:");
    lcd.print(package.pulsesensor);
    xSemaphoreGive(xCountingSemaphore);
    vTaskDelay(1000/portTICK_PERIOD_MS); 
    }  
}
