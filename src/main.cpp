#include <Arduino.h>
#include <Servo.h>

Servo myservo;

void printMenu() {
  Serial.println("\n--- Arduino Menu ---");
  Serial.println("1. Say Hello");
  Serial.println("2. Get Arduino Uptime");
  Serial.println("3. Blink LED");
  Serial.println("4. Set Servo Angle");
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
  myservo.attach(9);  // 将舵机连接到9号引脚
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
      case '4':
        Serial.println("Enter servo angle (0-180):");
        while (!Serial.available()) {}  // 等待输入
        int angle = Serial.parseInt();
        if (angle >= 0 && angle <= 180) {
          myservo.write(angle);
          Serial.print("Servo angle set to: ");
          Serial.println(angle);
        } else {
          Serial.println("Invalid angle. Please enter a value between 0 and 180.");
        }
        break;
      default:
        Serial.println("Invalid choice. Please try again.");
    }
    
    printMenu();
  }
}