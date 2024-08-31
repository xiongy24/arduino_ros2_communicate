#include <Arduino.h>

void printMenu() {
  Serial.println("\n--- Arduino Menu ---");
  Serial.println("1. Say Hello");
  Serial.println("2. Get Arduino Uptime");
  Serial.println("3. Blink LED");
  Serial.println("Enter your choice:");
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB port only)
  }
  delay(1000); // 添加一个短暂的延迟
  Serial.println("Arduino ready for communication!");
  pinMode(LED_BUILTIN, OUTPUT);
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char choice = Serial.read();
    
    // Clear the serial buffer
    while(Serial.available() > 0) {
      Serial.read();
    }
    
    switch(choice) {
      case '1':
        Serial.println("Hello from Arduino!");
        break;
      case '2':
        Serial.print("Arduino uptime: ");
        Serial.print(millis() / 1000);
        Serial.println(" seconds");
        break;
      case '3':
        Serial.println("Blinking LED 3 times...");
        for(int i = 0; i < 3; i++) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(500);
          digitalWrite(LED_BUILTIN, LOW);
          delay(500);
        }
        Serial.println("Blinking complete!");
        break;
      default:
        Serial.println("Invalid choice. Please try again.");
    }
    
    printMenu();
  }
}