#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MAX30100_PulseOximeter.h>
#include <Adafruit_MLX90614.h>

#define REPORT_PERIOD_MS 1000

LiquidCrystal_I2C lcd(0x27, 16, 2);
PulseOximeter pox;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

uint32_t lastReport = 0; 

// put function declarations here:
void onBeatDetected();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("Initializing....");

  if(!pox.begin()){
    Serial.println("Inisialisasi MAX30100 Gagal!");
    lcd.setCursor(0,1);
    lcd.print("Error: MAX30100");
    while(1);
  }

  if(!mlx.begin()){
    Serial.println("Inisialisasi MLX90614 Gagal!");
    lcd.setCursor(0,1);
    lcd.print("Error: MLX90614");
    while(1);
  }
  
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  lcd.clear();
}

void loop() {
  // put your main code here, to run repeatedly:
  pox.update();

  if(millis() - lastReport > REPORT_PERIOD_MS){
    lastReport = millis();

    float bpm = pox.getHeartRate();
    float SpO2 = pox.getSpO2();
    float temp = mlx.readObjectTempC();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BPM:");
    lcd.print(bpm, 0);
    lcd.print(" SpO2:");
    lcd.print(SpO2, 0);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("Temp:");
    lcd.print(temp, 1);
    lcd.print((char)223); // simbol derajat
    lcd.print("C");

    // Debug ke serial
    Serial.print("BPM: ");
    Serial.print(bpm);
    Serial.print(" | SpO2: ");
    Serial.print(SpO2);
    Serial.print(" | Temp: ");
    Serial.println(temp);
  }
}

// put function definitions here:
void onBeatDetected(){
  Serial.println("Berdetak!!!");
}