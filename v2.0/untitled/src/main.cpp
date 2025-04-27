#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_BMP3XX.h>

// Dirección I2C correcta del BMP388 (ajusta según tu módulo: 0x76 o 0x77)
#define BMP388_I2CADDR_DEFAULT 0x76

Adafruit_MPL3115A2  mpl3115 = Adafruit_MPL3115A2();
Adafruit_BMP3XX    bmp388  = Adafruit_BMP3XX();

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("Inicializando sensores MPL3115A2 y BMP388...");

    // Inicia I2C (Wire) con pull-ups externas en 3.3 V
    Wire1.setSDA(2);
    Wire1.setSCL(3);
    Wire1.begin();

    // MPL3115A2 (0x60)
    if (!mpl3115.begin(&Wire1)) {
        Serial.println("¡No se encontró el sensor MPL3115A2! Revisa conexiones y direcciones.");
        while (1) delay(10);
    }
    Serial.println("Sensor MPL3115A2 inicializado correctamente.");

    // BMP388 (0x76 ó 0x77)
    if (!bmp388.begin_I2C(BMP388_I2CADDR_DEFAULT, &Wire1)) {
        Serial.println("¡No se encontró el sensor BMP388! Revisa conexiones y direcciones.");
        while (1) delay(10);
    }
    Serial.println("Sensor BMP388 inicializado correctamente.");

    // Configuración del BMP388
    bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp388.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp388.setOutputDataRate(BMP3_ODR_50_HZ);

    // Modo barómetro para MPL3115A2
    mpl3115.setMode(MPL3115A2_BAROMETER);
    delay(100);
}

void loop() {
    // Leer presión y temperatura de MPL3115A2
    float presMPL = mpl3115.getPressure();
    float tempMPL = mpl3115.getTemperature();

    // Leer presión y temperatura de BMP388
    float presBMP = bmp388.readPressure();
    float tempBMP = bmp388.readTemperature();

    Serial.print("MPL3115A2 → ");
    Serial.print("Presión: "); Serial.print(presMPL); Serial.print(" Pa, ");
    Serial.print("Temp: ");    Serial.print(tempMPL); Serial.println(" °C");

    Serial.print("BMP388   → ");
    Serial.print("Presión: "); Serial.print(presBMP); Serial.print(" Pa, ");
    Serial.print("Temp: ");    Serial.print(tempBMP); Serial.println(" °C");

    delay(1000);
}
