#include "Comm.h"
#include <ArduinoJson.h>

Comm::Comm() {
    // Constructor
}


void Comm::generateMessage(const char* message, char* buffer) {
    // Crear un objeto JSON
    JsonDocument doc;  // Ajusta el tamaño según tus necesidades

    // Crear un objeto JSON dentro del documento
    JsonObject obj = doc.to<JsonObject>();

    // Agregar el mensaje al objeto JSON
    obj["message"] = message;

    size_t bufferSize = measureJson(doc) + 1;

    // Serializar el objeto JSON en el buffer
    serializeJson(doc, buffer, bufferSize);
}