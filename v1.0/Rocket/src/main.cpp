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
//    radio.setPALevel(RF24_PA_HIGH);
//    radio.setDataRate(RF24_1MBPS);
//    radio.setChannel(11);
//    radio.setRetries(3, 5);
//    radio.setCRCLength(RF24_CRC_16);
//
//    // Abrir una tubería de escritura con la dirección del receptor
//    radio.openWritingPipe(00002);
//    radio.openReadingPipe(1, 00001);
//
//    radio.setAutoAck(false);
//
//    // Detener la escucha para poder enviar
//    radio.startListening();
//
//    Serial.println("RF24 inicializado correctamente con HSPI");
//}
//
//void loop() {
//    // Leemos los datos recibidos
//    if (radio.available()) {
//        CommandData commandData;
//        radio.read(&commandData, sizeof(CommandData));
//        Serial.print("Mensaje recibido: ");
//        Serial.println(commandData.command);
//    }
//}