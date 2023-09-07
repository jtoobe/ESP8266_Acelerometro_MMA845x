/*
Ver las especificaciones de cada placa para los distintos modos de funcionamiento
Enb la serigrafia de la placa tambien figura la sigla HW-616
*/
#include <Wire.h>

#define MMA845x_ADDRESS 0x1C // Dirección I2C del sensor MMA845x (quizas debe cambiarse de acuerdo a la placa)

void setup() {
  Wire.begin(4, 5); // Inicializar la comunicación I2C con pines D2 (SDA) y D1 (SCL) - esto es para el ESP8266 varia para el ESP32
  Serial.begin(115200); // Inicializar la comunicación serial

  // Configurar el sensor MMA845x
  // Modo de alta resolución, frecuencia de muestreo de 100Hz
  writeRegister8(MMA845x_ADDRESS, 0x2A, 0x01);

  // Habilitar las interrupciones (opcional)
  //writeRegister8(MMA845x_ADDRESS, 0x2D, 0x08);
}

void loop() {
  // Leer datos del sensor MMA845x
  int16_t x, y, z;
  readXYZ(MMA845x_ADDRESS, &x, &y, &z);

  // Imprimir los valores de aceleración
  Serial.print("Aceleración X: ");
  Serial.println(x);
  Serial.print("Aceleración Y: ");
  Serial.println(y);
  Serial.print("Aceleración Z: ");
  Serial.println(z);

  delay(1000); // Esperar un segundo antes de la próxima lectura
}

void writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readXYZ(uint8_t address, int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(address);
  Wire.write(0x01); // Registro de datos X LSB
  Wire.endTransmission(false);

  Wire.requestFrom(address, 6); // Leer 6 bytes de datos (X, Y y Z)

  if (Wire.available() >= 6) {
    *x = (int16_t)(Wire.read() | (Wire.read() << 8));
    *y = (int16_t)(Wire.read() | (Wire.read() << 8));
    *z = (int16_t)(Wire.read() | (Wire.read() << 8));
  }
}
