/*========= BIBLIOTECAS =========*/
#include <WiFi.h>  // Biblioteca para generar la conexión a internet a través de WiFi
#include <PubSubClient.h> // Biblioteca para generar la conexión MQTT con un servidor (Ej.: ThingsBoard)
#include <ArduinoJson.h>  // Biblioteca para manejar Json en Arduino
#include <OneWire.h>
#include <DallasTemperature.h>

// Conectamos el pin de datos del sensor al pin digital 2 del arduino
#define sensorAgua 25
#define calentador 33

#define LED_BUILTIN 33

//Configuramos una instancia oneWire para comunicarnos con cualquier dispositivo OneWire 
OneWire oneWire(sensorAgua);

//Pasamos nuestra resferencia oneWire a la instancia de DallasTemperature
DallasTemperature sensors(&oneWire);


/*========= CONSTANTES =========*/

// Credenciales de la red WiFi
const char* ssid = "HUAWEI-IoT";
const char* password = "ORTWiFiIoT";

// Host de ThingsBoard
const char* mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;

// Token del dispositivo en ThingsBoard
const char* token = "eI2MFjeRVWWvRYQuDO6G";

/*========= VARIABLES =========*/

// Objetos de conexión
WiFiClient espClient;             // Objeto de conexión WiFi
PubSubClient client(espClient);   // Objeto de conexión MQTT


// Declaración de variables para los datos a manipular
unsigned long lastMsg = 0;  // Control de tiempo de reporte
int msgPeriod = 2000;       // Actualizar los datos cada 2 segundos
float humidity = 0;
float temperature = 0;
boolean led_state = false;

// Mensajes y buffers
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];

// Objeto Json para recibir mensajes desde el servidor
DynamicJsonDocument incoming_message(256);

/*========= FUNCIONES =========*/

// Inicializar la conexión WiFi
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Conectando a: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA); // Declarar la ESP como STATION
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("¡Conectado!");
  Serial.print("Dirección IP asignada: ");
  Serial.println(WiFi.localIP());
}


// Función de callback para recepción de mensajes MQTT (Tópicos a los que está suscrita la placa)
// Se llama cada vez que arriba un mensaje entrante (En este ejemplo la placa se suscribirá al tópico: v1/devices/me/rpc/request/+)
void callback(char* topic, byte* payload, unsigned int length) {

  // Log en Monitor Serie
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // En el nombre del tópico agrega un identificador del mensaje que queremos extraer para responder solicitudes
  String _topic = String(topic);

  // Detectar de qué tópico viene el "mensaje"
  if (_topic.startsWith("v1/devices/me/rpc/request/")) { // El servidor "me pide que haga algo" (RPC)
    // Obtener el número de solicitud (request number)
    String _request_id = _topic.substring(26);

    // Leer el objeto JSON (Utilizando ArduinoJson)
    deserializeJson(incoming_message, payload); // Interpretar el cuerpo del mensaje como Json
    String metodo = incoming_message["method"]; // Obtener del objeto Json, el método RPC solicitado

    // Ejecutar una acción de acuerdo al método solicitado
    if (metodo == "checkStatus") { // Chequear el estado del dispositivo. Se debe responder utilizando el mismo request_number

      char outTopic[128];
      ("v1/devices/me/rpc/response/" + _request_id).toCharArray(outTopic, 128);

      DynamicJsonDocument resp(256);
      resp["status"] = true;
      char buffer[256];
      serializeJson(resp, buffer);
      client.publish(outTopic, buffer);

    } else if (metodo == "setLedStatus") { // Establecer el estado del led y reflejar en el atributo relacionado

      boolean estado = incoming_message["params"]; // Leer los parámetros del método

      if (!estado) {
        digitalWrite(LED_BUILTIN, LOW); // Encender LED
        Serial.println("Encender LED");
      } else {
        digitalWrite(LED_BUILTIN, HIGH); // Apagar LED
        Serial.println("Apagar LED");
      }

      // Actualizar el atributo relacionado
      DynamicJsonDocument resp(256);
      resp["estado"] = !digitalRead(LED_BUILTIN);
      char buffer[256];
      serializeJson(resp, buffer);
      client.publish("v1/devices/me/attributes", buffer);  //Topico para actualizar atributos
      Serial.print("Publish message [attribute]: ");
      Serial.println(buffer);
    }
  }
}

// Establecer y mantener la conexión con el servidor MQTT (En este caso de ThingsBoard)
void reconnect() {
  // Bucle hasta lograr la conexión
  while (!client.connected()) {
    Serial.print("Intentando conectar MQTT...");
    if (client.connect("ESP8266", token, token)) {  //Nombre del Device y Token para conectarse
      Serial.println("¡Conectado!");

      // Una vez conectado, suscribirse al tópico para recibir solicitudes RPC
      client.subscribe("v1/devices/me/rpc/request/+");

    } else {

      Serial.print("Error, rc = ");
      Serial.print(client.state());
      Serial.println("Reintenar en 5 segundos...");
      // Esperar 5 segundos antes de reintentar
      delay(5000);

    }
  }
}

/*========= SETUP =========*/

void setup() {
  // Conectividad
  Serial.begin(115200);                   // Inicializar conexión Serie para utilizar el Monitor
  setup_wifi();                           // Establecer la conexión WiFi
  client.setServer(mqtt_server, mqtt_port);// Establecer los datos para la conexión MQTT
  client.setCallback(callback);           // Establecer la función del callback para la llegada de mensajes en tópicos suscriptos

  // Sensores y actuadores
  pinMode(LED_BUILTIN, OUTPUT);       // Inicializar el LED como salida
  pinMode(calentador, OUTPUT);
  sensors.begin();

}

/*========= BUCLE PRINCIPAL =========*/

void loop() {

  // === Conexión e intercambio de mensajes MQTT ===
  if (!client.connected()) {  // Controlar en cada ciclo la conexión con el servidor
    reconnect();              // Y recuperarla en caso de desconexión
  }

  client.loop();              // Controlar si hay mensajes entrantes o para enviar al servidor

  // === Realizar las tareas asignadas al dispositivo ===
  // En este caso se medirá temperatura y humedad para reportar periódicamente
  // El control de tiempo se hace con millis para que no sea bloqueante y en "paralelo" completar
  // ciclos del bucle principal

  unsigned long now = millis();
  if (now - lastMsg > msgPeriod) {
    lastMsg = now;

    Serial.print("Solicitando temperaturas...");
    sensors.requestTemperatures(); //Enviamos el comando para obtener la temperatura
    Serial.println("Terminado");
    //Después de haber recibido las temperaturas podemos imprimirlas

    temperature = sensors.getTempCByIndex(0);  // Leer la temperatura

    Serial.print("Temperatura del dispositivo uno (index 0): ");
    Serial.print(temperature);
    Serial.println(" °C");


    delay(500);

    // Publicar los datos en el tópico de telemetría para que el servidor los reciba
    DynamicJsonDocument resp(256);
    resp["temperature"] = temperature; //temperature;  //Agrega el dato al Json, ej: "temperature": 21.5
    char buffer[256];
    serializeJson(resp, buffer);
    client.publish("v1/devices/me/telemetry", buffer);  // Publica el mensaje de telemetría

    Serial.print("Publicar mensaje [telemetry]: ");
    Serial.println(buffer);

  }
}
