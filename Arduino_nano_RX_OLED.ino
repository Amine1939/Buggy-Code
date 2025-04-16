#include <U8g2lib.h>
#include <Wire.h>

// Initialize U8g2 for SH1106 I2C display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

void setup() {
    Serial.begin(9600);
    u8g2.begin();
    u8g2.clearBuffer();          
    u8g2.setFont(u8g2_font_ncenB08_tr);
}

void loop() {
    if (Serial.available()) {
        int receivedNumber = Serial.parseInt();  // Read integer from Master
        Serial.print("Received: ");
        Serial.println(receivedNumber);

        // Display text based on received number
        if (receivedNumber == 1) 
        {
          u8g2.clearBuffer();
          u8g2.setCursor(45, 20);
          u8g2.print("Status:");
            u8g2.setCursor(30, 40);
            u8g2.print("Waste Sample");
            u8g2.setCursor(35, 60);
            u8g2.print("Collected");
            delay(3000);
        } 
        else if (receivedNumber == 2) 
        {
          u8g2.clearBuffer();
          u8g2.setCursor(45, 20);
          u8g2.print("Status:");
            u8g2.setCursor(30, 40);
            u8g2.print("Waste Sample");
            u8g2.setCursor(25, 60);
            u8g2.print("Unloaded Successfully");
            delay(3000);
        }
        else if (receivedNumber == 3) 
        {
          u8g2.clearBuffer();
          u8g2.setCursor(45, 20);
          u8g2.print("Status:");
            u8g2.setCursor(25, 40);
            u8g2.print("Rerouting To");
            u8g2.setCursor(35, 60);
            u8g2.print("Next Stage");
            delay(3000);
        }
        else if (receivedNumber == 4) 
        {
          u8g2.clearBuffer();
          u8g2.setCursor(45, 20);
          u8g2.print("Status:");
            u8g2.setCursor(30, 40);
            u8g2.print("Power Reactor:");
            u8g2.setCursor(45, 60);
            u8g2.print("0%");
            delay(3000);
        }
        else if (receivedNumber == 5) 
        {
          u8g2.clearBuffer();
          u8g2.setCursor(45, 20);
          u8g2.print("Status:");
            u8g2.setCursor(30, 40);
            u8g2.print("Power Reactor:");
            u8g2.setCursor(45, 60);
            u8g2.print("50%");
            delay(3000);
        }
        else if (receivedNumber == 6)
         {
          u8g2.clearBuffer();
          u8g2.setCursor(45, 20);
          u8g2.print("Status:");
            u8g2.setCursor(30, 40);
            u8g2.print("Power Reactor:");
            u8g2.setCursor(45, 60);
            u8g2.print("100%");
            delay(3000);
        }
        else if (receivedNumber == 7) 
        {
          u8g2.clearBuffer();
          u8g2.setCursor(45, 20);
          u8g2.print("Status:");
            u8g2.setCursor(30, 40);
            u8g2.print("Power Reactor:");
            u8g2.setCursor(45, 60);
            u8g2.print("Online");
            delay(3000);
        }

        u8g2.sendBuffer(); // Update OLED
    }
}
