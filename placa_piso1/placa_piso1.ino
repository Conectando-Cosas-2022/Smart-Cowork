#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <Ticker.h>
#include <WiFi.h>  // Biblioteca para generar la conexión a internet a través de WiFi
#include <PubSubClient.h> // Biblioteca para generar la conexión MQTT con un servidor (Ej.: ThingsBoard)
#include <ArduinoJson.h>  // Biblioteca para manejar Json en Arduino
#include <MQ135.h>


#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

/**************************************************************/
/* Example how to read DHT sensors from an ESP32 using multi- */
/* tasking.                                                   */
/* This example depends on the Ticker library to wake up      */
/* the task every 20 seconds                                  */
/**************************************************************/

DHTesp dht;

// Credenciales de la red WiFi
const char* ssid = "iPhone de Pilar";
const char* password = "091662244";

// Host de ThingsBoard
const char* mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;

// Token del dispositivo en ThingsBoard
const char* token = "6kP90GSF7F5VJXTEdkU1";

#define sensorAgua 25
#define PIN_MQ135 33


/*========= VARIABLES =========*/

MQ135 mq135_sensor(PIN_MQ135);


// Objetos de conexión
WiFiClient espClient;             // Objeto de conexión WiFi
PubSubClient client(espClient);   // Objeto de conexión MQTT

// Declaración de variables para los datos a manipular
unsigned long lastMsg = 0;  // Control de tiempo de reporte
int msgPeriod = 2000;       // Actualizar los datos cada 2 segundos
float humedad = 0;
float tempAmbiente = 0;
boolean led_state = false;

// Mensajes y buffers
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];

// Objeto Json para recibir mensajes desde el servidor
DynamicJsonDocument incoming_message(256);


void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();

/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Ticker for temperature reading */
Ticker tempTicker;
/** Comfort profile */
ComfortState cf;
/** Flag if task should run */
bool tasksEnabled = false;
/** Pin number for DHT11 data pin */
int dhtPin = 25;


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


/**
   initTemp
   Setup DHT library
   Setup task and timer for repeated measurement
   @return bool
      true if task and timer are started
      false if task or timer couldn't be started
*/
bool initTemp() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT22);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
    tempTask,                       /* Function to implement the task */
    "tempTask ",                    /* Name of the task */
    4000,                           /* Stack size in words */
    NULL,                           /* Task input parameter */
    5,                              /* Priority of the task */
    &tempTaskHandle,                /* Task handle. */
    1);                             /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
    // Start update of environment data every 20 seconds
    tempTicker.attach(20, triggerGetTemp);
  }
  return true;
}

/**
   triggerGetTemp
   Sets flag dhtUpdated to true for handling in loop()
   called by Ticker getTempTimer
*/
void triggerGetTemp() {
  if (tempTaskHandle != NULL) {
    xTaskResumeFromISR(tempTaskHandle);
  }
}

/**
   Task to reads temperature from DHT11 sensor
   @param pvParameters
      pointer to task parameters
*/
void tempTask(void *pvParameters) {
  Serial.println("tempTask loop started");
  while (1) // tempTask loop
  {
    if (tasksEnabled) {
      // Get temperature values
      getTemperature();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}

/**
   getTemperature
   Reads temperature from DHT11 sensor
   @return bool
      true if temperature could be aquired
      false if aquisition failed
*/
bool getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    Serial.println("DHT22 error status: " + String(dht.getStatusString()));
    return false;
  }

  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  String comfortStatus;
  switch (cf) {
    case Comfort_OK:
      comfortStatus = "Comfort_OK";
      break;
    case Comfort_TooHot:
      comfortStatus = "Comfort_TooHot";
      break;
    case Comfort_TooCold:
      comfortStatus = "Comfort_TooCold";
      break;
    case Comfort_TooDry:
      comfortStatus = "Comfort_TooDry";
      break;
    case Comfort_TooHumid:
      comfortStatus = "Comfort_TooHumid";
      break;
    case Comfort_HotAndHumid:
      comfortStatus = "Comfort_HotAndHumid";
      break;
    case Comfort_HotAndDry:
      comfortStatus = "Comfort_HotAndDry";
      break;
    case Comfort_ColdAndHumid:
      comfortStatus = "Comfort_ColdAndHumid";
      break;
    case Comfort_ColdAndDry:
      comfortStatus = "Comfort_ColdAndDry";
      break;
    default:
      comfortStatus = "Unknown:";
      break;
  };

  Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity) + " I:" + String(heatIndex) + " D:" + String(dewPoint) + " " + comfortStatus);
  
  // Publicar los datos en el tópio de telemetría para que el servidor los reciba
    DynamicJsonDocument resp(256);
    resp["tempAmbiente"] =  String(newValues.temperature); //temperature;  //Agrega el dato al Json, ej: "temperature": 21.5
    char buffer[256];
    serializeJson(resp, buffer);
    client.publish("v1/devices/me/telemetry", buffer);  // Publica el mensaje de telemetría
    
    Serial.print("Publicar mensaje [telemetry]: ");
    Serial.println(buffer);
  
  return true;
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

    }
  }
}

// Establecer y mantener la conexión con el servidor MQTT (En este caso de ThingsBoard)
void reconnect() {
  // Bucle hasta lograr la conexión
  while (!client.connected()) {
    Serial.print("Intentando conectar MQTT...");
    if (client.connect("ESP32", token, token)) {  //Nombre del Device y Token para conectarse
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


void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("DHT ESP32 example with tasks");
  initTemp();
  // Signal end of setup() to tasks
  tasksEnabled = true;

  setup_wifi();                           // Establecer la conexión WiFi
  client.setServer(mqtt_server, mqtt_port);// Establecer los datos para la conexión MQTT
  client.setCallback(callback);           // Establecer la función del callback para la llegada de mensajes en tópicos suscriptos

  // Sensores y actuadores
  pinMode(sensorAgua, INPUT);            // Inicializar el DHT como entrada

}

void loop() {
   // === Conexión e intercambio de mensajes MQTT ===
  if (!client.connected()) {  // Controlar en cada ciclo la conexión con el servidor
    reconnect();              // Y recuperarla en caso de desconexión
  }
  
  client.loop();              // Controlar si hay mensajes entrantes o para enviar al servidor

  if (!tasksEnabled) {
    // Wait 2 seconds to let system settle down
    
    
    delay(2000);
    // Enable task that will read values from the DHT sensor
    tasksEnabled = true;
    if (tempTaskHandle != NULL) {
      vTaskResume(tempTaskHandle);
    }
  }
  yield();
}

void getAirCondition{
    float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);

  Serial.print("MQ135 RZero: ");
  Serial.print(rzero);
  Serial.print("\t Corrected RZero: ");
  Serial.print(correctedRZero);
  Serial.print("\t Resistance: ");
  Serial.print(resistance);
  Serial.print("\t PPM: ");
  Serial.print(ppm);
  Serial.print("\t Corrected PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");

  delay(300);
  }
