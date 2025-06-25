#include <ModbusMaster.h>

#define TXD2 17  // UART2 TX
#define RXD2 16  // UART2 RX
#define DE_RE 4  // RS485 direction control pin

HardwareSerial SerialModbus(2);
ModbusMaster node;

void preTransmission() {
  digitalWrite(DE_RE, HIGH);
}
void postTransmission() {
  digitalWrite(DE_RE, LOW);
}

void setup() {
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);
  Serial.begin(115200);
  SerialModbus.begin(9600, SERIAL_8N1, RXD2, TXD2);

  node.begin(1, SerialModbus);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  delay(100);

  node.writeSingleRegister(0x000F, 1);    // Enable motor
  node.writeSingleRegister(0x0191, 20);   // Set 2.0A peak current
  node.writeSingleRegister(0x1801, 0x4001);// JOG clockwise
  delay(2000);
  node.writeSingleRegister(0x1801, 0x2222);// Stop
}

void loop() {
  node.loop();  // Process Modbus communications
}