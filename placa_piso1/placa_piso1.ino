#include "DHTesp.h"  // Click here to get the library: http://librarymanager/All#DHTesp
#include <Ticker.h>
#include <WiFi.h>          // Biblioteca para generar la conexión a internet a través de WiFi
#include <PubSubClient.h>  // Biblioteca para generar la conexión MQTT con un servidor (Ej.: ThingsBoard)
#include <ArduinoJson.h>   // Biblioteca para manejar Json en Arduino
#include <MQ135.h>
#include <SPI.h>  //https://www.arduino.cc/en/reference/SPI
#include <Servo.h>


#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY !)
#error Select ESP32 board.
#endif

/**************************************************************/
/* Example how to read DHT sensors from an ESP32 using multi- */
/* tasking.                                                   */
/* This example depends on the Ticker library to wake up      */
/* the task every 20 seconds                                  */
/**************************************************************/

/*========= PARAMETROS WIFI =========*/
//const char* ssid = "HUAWEI-IoT";
//const char* password = "ORTWiFiIoT";
const char* ssid = "dtpub";
const char* password = "domopublico";

// Host de ThingsBoard
const char* mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;

// Token del dispositivo en ThingsBoard
const char* token = "6kP90GSF7F5VJXTEdkU1";

//Parameters
const int ipaddress[4] = { 103, 97, 67, 25 };

/*========= DEFINICION DE PINES =========*/
#define dhtPin 25
#define PIN_MQ135 34

#define pin_ventilador 33
#define SERVO_PIN1 26  // ESP32 pin GIOP26 connected to servo motor
#define SERVO_PIN2 27  // ESP32 pin GIOP26 connected to servo motor



/*========= VARIABLES =========*/

MQ135 mq135_sensor(PIN_MQ135);

Servo servo1;  // derecha mirando desde afuera --> pos =0 esta cerrado
Servo servo2;  // izquierda mirando desde afuera --> pos = 55 esta cerrado

DHTesp dht;

// Objetos de conexión
WiFiClient espClient;            // Objeto de conexión WiFi
PubSubClient client(espClient);  // Objeto de conexión MQTT

// Declaración de variables para los datos a manipular
unsigned long lastMsg = 0;  // Control de tiempo de reporte
int msgPeriod = 2000;       // Actualizar los datos cada 2 segundos
float humedad = 0;
float tempAmbiente = 0;
boolean led_state = false;

// Mensajes y buffers
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];

// Objeto Json para recibir mensajes desde el servidor
DynamicJsonDocument incoming_message(256);


void tempTask(void* pvParameters);
bool getTemperature();
void getAirQuality();
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


// Inicializar la conexión WiFi
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Conectando a: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);  // Declarar la ESP como STATION
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  servo1.attach(SERVO_PIN1);  // attaches the servo on ESP32 pin
  servo2.attach(SERVO_PIN2);  // attaches the servo on ESP32 pin

  servo1.write(0);
  servo2.write(55);
  delay(20);

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
    tempTask,        /* Function to implement the task */
    "tempTask ",     /* Name of the task */
    4000,            /* Stack size in words */
    NULL,            /* Task input parameter */
    5,               /* Priority of the task */
    &tempTaskHandle, /* Task handle. */
    1);              /* Core where the task should run */

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
void tempTask(void* pvParameters) {
  Serial.println("tempTask loop started");
  while (1)  // tempTask loop
  {
    if (tasksEnabled) {
      // Get temperature values
      getTemperature();
      getAirQuality();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}

void publicarTelemetria(const String& key, int value){
   DynamicJsonDocument resp(256);
    resp[key] = value; //temperature;  //Agrega el dato al Json, ej: "temperature": 21.5
    char buffer[256];
    serializeJson(resp, buffer);
    client.publish("v1/devices/me/telemetry", buffer);  // Publica el mensaje de telemetría

    Serial.print("Publicar mensaje [telemetry]: ");
    Serial.println(buffer);
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
  resp["tempAmbiente"] = String(newValues.temperature);  //temperature;  //Agrega el dato al Json, ej: "temperature": 21.5
  char buffer[256];
  serializeJson(resp, buffer);
  client.publish("v1/devices/me/telemetry", buffer);  // Publica el mensaje de telemetría

  Serial.print("Publicar mensaje [telemetry]: ");
  Serial.println(buffer);

  return true;
}


void getAirQuality() {
    int sensorValue = analogRead(PIN_MQ135);  // read analog input pin 0
    Serial.print("AirQuality=");
    Serial.print(sensorValue, DEC);  // prints the value read
    Serial.println(" PPM");
    delay(100);  // wait 100ms for next reading 

    //publica telemetría de la lectura del sensor MQ135 a Thingsboard
    publicarTelemetria("ppm_Aire", sensorValue);
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
  if (_topic.startsWith("v1/devices/me/rpc/request/")) {  // El servidor "me pide que haga algo" (RPC)
    // Obtener el número de solicitud (request number)
    String _request_id = _topic.substring(26);

    // Leer el objeto JSON (Utilizando ArduinoJson)
    deserializeJson(incoming_message, payload);  // Interpretar el cuerpo del mensaje como Json
    String metodo = incoming_message["method"];  // Obtener del objeto Json, el método RPC solicitado

    // Ejecutar una acción de acuerdo al método solicitado
    if (metodo == "abrirVentanas") {  // Establecer el estado del led y reflejar en el atributo relacionado
      abrirVentana1();
      abrirVentana2();
      publicarAtributo("estadoVentanas", true);

    } else if (metodo == "cerrarVentanas") {
      cerrarVentana1();
      cerrarVentana2();
      publicarAtributo("estadoVentanas", false);

    } else if (metodo == "prenderVentilador") {
      digitalWrite(pin_ventilador, HIGH);
      Serial.println("Ventilador prendido");
      publicarAtributo("estadoVentilador", digitalRead(pin_ventilador));

    } else if (metodo == "apagarVentilador") {
      digitalWrite(pin_ventilador, LOW);
      Serial.println("Ventilador apagado");
      publicarAtributo("estadoVentilador", digitalRead(pin_ventilador));
    }
  }
}

void publicarAtributo(const String& nombreAtributo, int valorAtr) {
  DynamicJsonDocument resp(256);
  resp[nombreAtributo] = valorAtr;  // ojo el not !
  char buffer[256];
  serializeJson(resp, buffer);
  client.publish("v1/devices/me/attributes", buffer);  //Topico para actualizar atributos
  Serial.print("Publish message [attribute]: ");
  Serial.println(buffer);
}

// METODOS PARA ABRIR VENTANAS///
void abrirVentana1() {
  for (int pos = 0; pos <= 55; pos += 1) {
    servo1.write(pos);
    delay(20);  // waits 15ms to reach the position
  }
}

void abrirVentana2() {
  for (int pos = 55; pos >= 0; pos -= 1) {
    servo2.write(pos);
    delay(20);  // waits 15ms to reach the position
  }
}

void cerrarVentana1() {
  for (int pos = 55; pos >= 0; pos -= 1) {
    // in steps of 1 degree
    servo1.write(pos);
    delay(15);  // waits 15ms to reach the position
  }
}
void cerrarVentana2() {
  for (int pos = 0; pos <= 55; pos += 1) {
    servo2.write(pos);
    delay(20);  // waits 15ms to reach the position
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


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("DHT ESP32 example with tasks");
  initTemp();
  // Signal end of setup() to tasks
  tasksEnabled = true;

  setup_wifi();                              // Establecer la conexión WiFi
  client.setServer(mqtt_server, mqtt_port);  // Establecer los datos para la conexión MQTT
  client.setCallback(callback);              // Establecer la función del callback para la llegada de mensajes en tópicos suscriptos

  // Sensores y actuadores
  pinMode(dhtPin, INPUT);  // Inicializar el DHT como entrada
  pinMode(PIN_MQ135, INPUT);
}

void loop() {
  // === Conexión e intercambio de mensajes MQTT ===
  if (!client.connected()) {  // Controlar en cada ciclo la conexión con el servidor
    reconnect();              // Y recuperarla en caso de desconexión
  }
  client.loop();  // Controlar si hay mensajes entrantes o para enviar al servidor
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

