#include <Arduino.h>
#include "headers/StateMachine.h"

StateMachine stateMachine;

void setup() {
    Serial.begin(115200);

    stateMachine.setup();
}

void loop() {
    stateMachine.update();
}

//#include <SPI.h>
//#include <RF24.h>
//
//// Definición de pines para ESP32 usando HSPI
//#define CE_PIN 4    // Pin CE (Chip Enable)
//#define CSN_PIN 15   // Pin CSN (Chip Select)
//
//// Creación del objeto RF24 utilizando HSPI
//RF24 radio(CE_PIN, CSN_PIN);
//
//// Dirección del receptor
//byte direccion[6] = "00001";
//
//// Mensaje a enviar
//char mensaje[] = "¡Hola Mundo!";
//
//void setup() {
//    Serial.begin(115200);
//    Serial.println("Iniciando RF24 con HSPI...");
//
//    // Inicializar HSPI para ESP32
//    SPIClass * hspi = new SPIClass(HSPI);
//    hspi->begin();
//
//    // Inicializar el módulo RF24 con HSPI
//    if (!radio.begin(hspi)) {
//        Serial.println("Error al inicializar el módulo RF24!");
//        while (1) {}
//    }
//
//    // Configuración del módulo RF24
//    radio.setPALevel(RF24_PA_LOW);      // Nivel de potencia bajo
//    radio.setDataRate(RF24_250KBPS);    // Velocidad de transmisión
//    radio.setChannel(76);               // Canal de comunicación
//
//    // Abrir una tubería de escritura con la dirección del receptor
//    radio.openWritingPipe(direccion);
//
//    // Detener la escucha para poder enviar
//    radio.stopListening();
//
//    Serial.println("RF24 inicializado correctamente con HSPI");
//}
//
//void loop() {
//    // Enviar el mensaje
//    bool ok = radio.write(&mensaje, sizeof(mensaje));
//
//    // Verificar si el mensaje se envió correctamente
//    if (ok) {
//        Serial.println("Mensaje enviado con éxito");
//    } else {
//        Serial.println("Error al enviar el mensaje");
//    }
//
//    // Esperar 1 segundo antes de enviar otro mensaje
//    delay(1000);
//}