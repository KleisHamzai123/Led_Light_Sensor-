#include <avr/io.h>         // Definitionen der I/O-Register einbinden
#include <util/delay.h>     // für Verzögerungen, z.B. _delay_ms()
#include <math.h>
#include <Wire.h>    //https://www.arduino.cc/reference/en/language/functions/communication/wire/ Ich werde Wire-Library verwenden, weil ich beim Versuch, die Daten vom Mpu6050 auszulesen, immer einen Fehler im Terminal hatte und es unmöglich war, eine Lösung zu finden. Da in der Beschreibung von HA stand, dass der Einsatz von Hilfswerkzeugen erlaubt sei, dachte ich an Wire.

#define FOSC 16000000 // Clock Speed
#define BAUD 115200   
#define MYUBRR 8 //FOSC/16/BAUD-1

const int MPU = 0x68; // MPU6050 I2C address
float BesX, BesY, BesZ; //3 Dimensionen des Beschleunigungssensors
float GyroX, GyroY, GyroZ;//3 Dimensionen von Gyroskop
float BesAngleX, BesAngleY, BesAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ; //Drehwinkel
float rollen, nicken, gierung; //sensorische Mobilität
float BesErrorX, BesErrorY, BesErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ; //Ich habe die Funktion computeIMU_error ausgeführt und die neuen Fehlerwerte in den Code eingefügt: BesErrorX ~(5.86) BesErrorY ~(75.38) GyroErrorX ~(0.62) GyroErrorY ~(0.29) GyroErrorZ ~ (-0,8)
float elapsedTime, currentTime, previousTime;
int c = 0;


// Initialisierung der USART für serielle Kommunikation
void USART_init(unsigned int ubrr) {
  //set baud rate
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  //Enable receiver and transmitter
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  //Set frame format: 8data, 2stop bit
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

// Ausgabe eines einzelnen Zeichens
void put_c(unsigned char data) {
  while (!(UCSR0A & (1<<UDRE0)));
  UDR0 = data;
}

void put_string(char data[]){
  int i = 0;
  while(data[i] != 0){
    put_c(data[i]);
    i++;
  }
}

void put_stringNewLine(char data[]){
  int i = 0;
  while(data[i] != 0){
    put_c(data[i]);
    i++;
  }
  put_c('\n');
  put_c('\r');
}
 

// Ausgabe einer Dezimalzahl
void put_dec(int16_t x) {
    unsigned char buf[8];
    
    if (x<0) {
        put_c('-');
        x = -x;
    }
    if (x==0) {
        put_c('0');
    } else {
        int i=0;
        while (i<8 && x>0) {
            buf[i++] = '0' + (x%10);
            x = x/10;
        }
        i=i-1;
        while (i>= 0) put_c(buf[i--]);
    }
}
// Initialisierung der Mpu 6050 für serielle Kommunikation
void setup() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // MPU6050-MPU=0x68
  Wire.write(0x6B);                  // Kommunikation mit der Registry
  Wire.write(0x00);                  // Schreibe Wert 0 in die 6B register
  Wire.endTransmission(true);        // die Übertragung beenden
  //Da mein Projekt in einer kleinen Umgebung entwickelt wird und seine Bewegung von meiner Hand ausgeführt wird, werde ich aus dem Datenblatt die kleinsten Standardwerte auswählen, um den Beschleunigungssensor und das Gyroskop zu konfigurieren.
  // Sensitivity - Full Scale Range (default +/- 2g) Beschleunigungssensor (Seite 15 , Datasheet)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  // ACCEL_CONFIG Register (1C hex)
  Wire.write(0x10);                  // Schreibe Registerbits als 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Sensitivity - Full Scale Range (default +/- 250deg/s) Gyroskope (Seite 14, Datasheet)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // GYRO_CONFIG Register (1B hex)
  Wire.write(0x10);                   // Schreibe Registerbits als 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  _delay_ms(20);
  
  // Funktion ,die IMU-Fehlerwerte behandlen muss
  computeIMU_error();
  _delay_ms(20);
}

void computeIMU_error() {
  // Wir können diese Funktion im Setup-Schleife aufrufen, um den Beschleunigungsmesser- und Gyroldatenfehler zu berechnen. 
  // Ich habe die IMU platt gestellt um die richtigen Werte zu bekommen.
  // Beschleunigungsensorwerte werden 50 Mal ausgelesen
  while (c < 50) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    BesX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    BesY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    BesZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Fehler rechnung
    BesErrorX = BesErrorX + ((atan((BesY) / sqrt(pow((BesX), 2) + pow((BesZ), 2))) * 180 / PI));
    BesErrorY = BesErrorY + ((atan(-1 * (BesX) / sqrt(pow((BesY), 2) + pow((BesZ), 2))) * 180 / PI));
    c++;
  }
  //Berechnung des Mittelwertes
  BesErrorX = BesErrorX / 50;
  BesErrorY = BesErrorY / 50;
  c = 0;
  // Gyrowerte werden 50 Mal ausgelesen
  while (c < 50) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Fehler rechnung
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Berechnung des Mittelwertes
  GyroErrorX = GyroErrorX / 50;
  GyroErrorY = GyroErrorY / 50;
  GyroErrorZ = GyroErrorZ / 50;

}

// Initialisierung der ADC mithilfe von Datasheet
void initAdc(void){
  PRR &= ~ (1 << PRADC);
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADEN); 
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)
}
 
void printLux(double u){
  double x = u;
  double b = 100; 
  double l = (u*b)-5; //Nachdem ich die ersten Messungen durchgeführt und mit denen einer mobilen Anwendung verglichen hatte, die Lux berechnet, waren die Werte, die ich vom Sensor erhielt, immer 4 oder 5 Lux höher.
  put_dec(l);
  put_string("Lux");
}
void printVoltage(double u){
  int whole = u;
  put_dec(whole);
}

void printrollen(roll){
  put_c(' ');
}

void printnicken(nick){
  put_c(' ');
}

void printgierung(gierung){
  put_c(' ');
}


void printConv(int itterations){
  ADCSRA |= (1 << ADSC); //start conversion
  while((ADCSRA & (1 << ADSC)) != 0);

  //Zuerst muss ADCL gelesen und später ADCH 
   for(int i = 1; i <= itterations; i++){  
    int x = ADCL; 
    x += (ADCH<<8);
    double u2 = (((double)x / 1024) * 5);
    double r1 = u2 * (10000/(5 - u2)); //10000 kOhm reference resistance 
    printResistance(r1); //Nicht relevant für das Projekt, aber ich habe sie als Test durchgeführt, um zu sehen, ob alles richtig funktioniert hat oder nicht
    put_c(' ');
    printVoltage(u2);
    put_c(' ');
    printLux(r1, u2);
    put_c(' ');
  
  }
}

void loop() {
  // Beschleunigungssensordaten lesen , wir werden insgesamt 6 Register lesen , jede Achsenwert wird in 2 Registern gespeichert //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Anfang register 0x3B 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Diese Funktion wird vom Controller verwendet, um Bytes von einem Peripheriegerät anzufordern.(address, anzahl, stop)
  // Für einen Bereich von +-2g müssen wir die Rohwerte laut Datasheet durch 16384 dividieren.
  BesX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-Werte
  BesY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-Werte
  BesZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-Werte
  // Berechnung von Rollen und Nicken aus den Daten des Beschleunigungsmessers
  BesAngleX = (atan(BesY / sqrt(pow(BesX, 2) + pow(BesZ, 2))) * 180 / PI) - 5.86; // BesErrorX ~(5.86) https://www.google.com/search?q=calculate+roll+pitch+yaw+from+mpu6050&source=lmns&bih=656&biw=1292&client=ubuntu&hs=ROb&hl=en&sa=X&ved=2ahUKEwjmpOT4wKD6AhUsmYsKHenjC_EQ_AUoAHoECAEQAA
  BesAngleY = (atan(-1 * BesX / sqrt(pow(BesY, 2) + pow(BesZ, 2))) * 180 / PI) - 75.38; // BesErrorY ~(75.38)
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
  
  // Korrektur der Ausgabedaten (Winkel in Grad) mit Hilfe der aus der Ausführung der Funktion computeIMU_error() erhaltenen Variablen.
  // Anpassen der Outputs mit Hilfe von Fehlervariablen(Filter)
  GyroX = GyroX - 0.62; // GyroErrorX ~(0.62)
  GyroY = GyroY - 0.29; // GyroErrorY ~(0.29)
  GyroZ = GyroZ + 0.8; // GyroErrorZ ~ (-0.8)
  
  // Aus der Drehung des Sensors in der Zeiteinheit müssen wir berechnen, wie viel Grad der Winkel beträgt
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  gierung =  gierung + GyroZ * elapsedTime;
  
  
  // Erste Methode : Complementary filter - Die Idee hinter dem komplementären Filter besteht darin, sich langsam bewegende Signale von einem Beschleunigungsmesser und sich schnell bewegende Signale von einem Gyroskop zu nehmen und sie zu kombinieren. Der Beschleunigungsmesser gibt einen guten Orientierungsindikator unter statischen Bedingungen. Das Gyroskop gibt einen guten Indikator für die Neigung unter dynamischen Bedingungen. Article : An Optimized Complementary Filter For An Inertial Measurement Unit Contain MPU6050 Sensor 
  roll = 0.98 * gyroAngleX + 0.02 * BesAngleX; //Aus Article
  nick = 0.98 * gyroAngleY + 0.02 * BesAngleY;
  
  // Werte von Sensoren zeigen
  printrollen();
  printnicken();
  printgierung();
  _delay_ms(500);
}



// Hauptprogramm
int __attribute__((OS_main)) main(void) {
    USART_init(MYUBRR);   // Initialisierung der seriellen Schnittstelle
    initAdc();
    setup();
    while(1){
    printConv(50);
    loop(50);
    _delay_ms(500);
    put_string("\e[1;1H\e[2J");
  
    }
}
