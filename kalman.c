#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float BesX, BesY, BesZ; //3 Dimensionen des Beschleunigungssensors
float GyroX, GyroY, GyroZ;//3 Dimensionen von Gyroskop
float BesAngleX, BesAngleY, BesAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ; //Drehwinkel
float rollen, nicken, gierung; //sensorische Mobilität
float BesErrorX, BesErrorY, BesErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ; //Ich habe die Funktion compute_IMU_error ausgeführt und die neuen Fehlerwerte in den Code eingefügt: BesErrorX ~(0.9) BesErrorY ~(-3.1) GyroErrorX ~(-2.18) GyroErrorY ~(1.11) GyroErrorZ ~ (-0,8)
float elapsedTime, currentTime, previousTime;
int c = 0;
void setup() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // MPU6050-MPU=0x68
  Wire.write(0x6B);                  // Kommunikation mit der Registry
  Wire.write(0x00);                  // Schreibe Wert 0 in die 6B register
  Wire.endTransmission(true);        // die Übertragung beenden
  //Da mein Projekt in einer kleinen Umgebung entwickelt wird und seine Bewegung von meiner Hand ausgeführt wird, werde ich aus dem Datenblatt die kleinsten Standardwerte auswählen, um den Beschleunigungssensor und das Gyroskop zu konfigurieren.
  // Sensitivity - Full Scale Range (default +/- 2g) Beschleunigungssensor
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  // ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Sensitivity - Full Scale Range (default +/- 250deg/s) Gyroskope
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  
  // Funktion ,die IMU-Fehlerwerte behandlen muss
  calculate_IMU_error();
  delay(20);
}
void loop() {
  // Beschleunigungssensordaten lesen , wir werden insgesamt 6 Register lesen , jeder Achsenwert wird in 2 Registern gespeichert //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Anfang register 0x3B 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Diese Funktion wird vom Controller verwendet, um Bytes von einem Peripheriegerät anzufordern.(address, quantity, stop)
  // Für einen Bereich von +-2g müssen wir die Rohwerte laut Datasheet durch 16384 dividieren.
  BesX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-Werte
  BesY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-Werte
  BesZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-Werte
  // Berechnung von Rollen und Nicken aus den Daten des Beschleunigungsmessers
  BesAngleX = (atan(BesY / sqrt(pow(BesX, 2) + pow(BesZ, 2))) * 180 / PI) - 0.9; // BesErrorX ~(0.9) http://www.starlino.com/imu_guide.html (Website erklärt die Formel)
  BesAngleY = (atan(-1 * BesX / sqrt(pow(BesY, 2) + pow(BesZ, 2))) * 180 / PI) + 3.1; // BesErrorY ~(-3.1)
  // Gyroskopedaten lesen ,insgesamt 6 Register lesen , jeder Achsenwert wird in 2 Registern gespeichert//
  previousTime = currentTime;        
  currentTime = millis();            
  elapsedTime = (currentTime - previousTime) / 1000; // Teilen durch 1000, um Sekunden zu erhalten
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Anfang register 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // Für einen Bereich von 250deg/s müssen wir die Rohwerte laut Datasheet durch 131 dividieren
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  
  // Korrektur der Ausgabedaten (Winkel in Grad) mit Hilfe der aus der Ausführung der Funktion calculate_IMU_error() erhaltenen Variablen.
  // Anpassen der Outputs mit Hilfe von Fehlervariablen
  GyroX = GyroX + 2.18; // GyroErrorX ~(-2.18)
  GyroY = GyroY - 1.11; // GyroErrorY ~(1.11)
  GyroZ = GyroZ + 0.8; // GyroErrorZ ~ (-0.8)
  
  // Aus der Drehung des Sensors in der Zeiteinheit müssen wir berechnen, wie viel Grad der Winkel beträgt
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  gierung =  gierung + GyroZ * elapsedTime;
  
  
  // Erste Methode : Complementary filter - Kombination aus Beschleunigungsmesser und Winkelwerten https://forum.arduino.cc/t/guide-to-gyro-and-accelerometer-with-arduino-including-kalman-filtering/57971
  rollen = 0.98 * gyroAngleX + 0.02 * BesAngleX;
  nicken = 0.98 * gyroAngleY + 0.02 * BesAngleY;
  
  // Print the values on the serial monitor
  Serial.print(rollen);
  Serial.print("/");
  Serial.print(nicken);
  Serial.print("/");
  Serial.println(gierung);
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    BesX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    BesY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    BesZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    BesErrorX = BesErrorX + ((atan((BesY) / sqrt(pow((BesX), 2) + pow((BesZ), 2))) * 180 / PI));
    BesErrorY = BesErrorY + ((atan(-1 * (BesX) / sqrt(pow((BesY), 2) + pow((BesZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  BesErrorX = BesErrorX / 200;
  BesErrorY = BesErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
