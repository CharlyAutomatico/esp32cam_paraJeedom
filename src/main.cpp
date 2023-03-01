// https : //techtutorialsx.com/2020/06/10/esp32-camera-image-server/

#include "config.h"
#include <Arduino.h>
#include "WiFi.h"
#include <ESP32Ping.h> //ping al GW
#include "ESPAsyncWebServer.h"
#include <WiFiClient.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "soc/soc.h"          //disable brownout problems
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "time.h"             //para el calculo del tiempo NTP
#include <PubSubClient.h>     //PubSub messages
const char *ssid = "MyDeco01";
const char *password = "segundaplanta";

// Calculo del tiempo NTP
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

#ifdef BME_ON
#include <Wire.h>
#include "SparkFunBME280.h"
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15
BME280 bme;
float temperatura, humedad;
float correccion_temp = 0; // ponderado en Jeedom
#endif

// Data Timer para enviar los mensajes PUB
#include <Ticker.h> //Timer
Ticker publicMQ;    // Objeto Timer
boolean sentinelSend = false;
Ticker blinkLED;  // Objeto Timer
Ticker blinkLED1; // Objeto Timer
//-----------------------

// Ping al Gateway
Ticker pinging; // objeto Ticker
const IPAddress remote_ip(192, 168, 1, 1);
int valuePing = 0;

// Datos MQTT
// PUB cambiar test x ubicacion
const char *mqtt_server = "192.168.1.185";
const char *mqtt_topic_mensaje = "camara/garage/mensajeCam";
const char *mqtt_topic_temp = "camara/garage/temperatura";
const char *mqtt_topic_hum = "camara/garage/humedad";
const char *mqtt_topic_pir = "camara/garage/pirMSG";
const char *mqtt_topic_pirON = "camara/garage/pirON";
// SUB
const char *mqtt_subtopic_mensaje = "camara/garage/inESP32CAM";
//------------------------

// PIR parametros para la gestion del PIR mqtt
boolean pirSensor;
char pir[50]; // buffer para almacenar mensajes mqtt
long int valuePir = 0;
char pirON[10];

// Temporizador Detetector PIR
unsigned long ultimaDeteccionPir = 0;
unsigned long tiempoDeteccionPir = 6000;

// WifiReconect = ESP.restart
unsigned long previousMillis = 0;
unsigned long interval = 30000;

// Conexión a la red WiFi +  Broker
WiFiClient espClient;
PubSubClient clientMQ(espClient);

// Parametros para gestion mensajes MQTT PUB
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
long int value = 0;
char tiempoNTP[25];
//-----------------------

// Funcion verifica el acceso al Router, si no, reinicia a los
// 6 intentos de 5 segundos cada uno
void callbackTimerpinger()
{
  if (Ping.ping(remote_ip))
  {
#ifdef _DEBUG_
    Serial.println("Router OK");
#endif
    valuePing = 0;
  }
  else
  {
    ++valuePing;
    Serial.printf("Error Nº %d \n", valuePing);
    if (valuePing == numPing)
    {
      Serial.println("Reinicio ESP");
      ESP.restart();
    }
  }
}

// Funcion calculo del tiempo ( en formato string)
void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  // Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
  strftime(tiempoNTP, sizeof(tiempoNTP), "%Y-%m-%d %H:%M:%S", &timeinfo);
  String asString(tiempoNTP);
}

// // static void IRAM_ATTR detectsMovement(void *arg)
// // {
// //   Serial.println("Interrupt");
// //   pirSensor = true;
// // }

// // void enableInterrupt()
// // {
// //   esp_err_t  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *)13);
// //   if (err != ESP_OK)
// //   {
// //     Serial.printf("handler add failed with error 0x%x \r\n", err);
// //   }

// //   err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
// //   if (err != ESP_OK)
// //   {
// //     Serial.printf("set intr type failed with error 0x%x \r\n", err);
// //   }
// // }

AsyncWebServer server(WSPORT);

bool initCamera()
{
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = QUALITY;
  config.fb_count = 1;
  // if (psramFound())
  // {
  //   config.frame_size = RESOLUTION; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  //   config.jpeg_quality = QUALITY;
  //   config.fb_count = 2;
  // }
  // else
  // {
  //   if (RESOLUTION > FRAMESIZE_SVGA)
  //   {
  //     config.frame_size = FRAMESIZE_VGA;
  //   }
  //   config.jpeg_quality = 12;
  //   config.fb_count = 1;
  // }

  esp_err_t result = esp_camera_init(&config);

  // delay(1000);
  if (result != ESP_OK)
  {
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_brightness(s, BRIGHT);
  s->set_contrast(s, CONTRAST);
  s->set_saturation(s, SATUR);
  // s->set_special_effect(s, SPECIAL);
  // s->set_whitebal(s, WHITEBAL);
  // s->set_awb_gain(s, AWBGAIN);
  // s->set_wb_mode(s, AWBMODE);
  // s->set_exposure_ctrl(s, EXPOS);
  // s->set_aec2(s, AEC2);
  // s->set_ae_level(s, AELEV);
  // s->set_aec_value(s, AECVAL);
  // s->set_gain_ctrl(s, GAINCT);
  // s->set_agc_gain(s, GAINAGC);
  // s->set_gainceiling(s, (gainceiling_t)GAINCELL);
  // s->set_bpc(s, BPC);
  // s->set_wpc(s, WPC);
  // s->set_raw_gma(s, RAWGMA);
  // s->set_lenc(s, LENC);
  // s->set_hmirror(s, MIRROR);
  // s->set_vflip(s, FLIP);
  // s->set_dcw(s, DCW);
  // s->set_colorbar(s, CBAR);

  // enableInterrupt();
  return true;
}

// Funcion TICKER enviar MQTT PUB cada x segundos (seg -> definido en intPublicMQTT)
void callbackTimerMQ()
{
  sentinelSend = true;
}
void callbackTimerLED()
{
  // Cambiar de estado el LED
  byte estado = digitalRead(pinLED);
  digitalWrite(pinLED, !estado);
}

// Funciones MQTT ------------------------
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messageMQ;

  // Mostramos el mensaje en el monitor serie y lo asignamos a un String
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    messageMQ += (char)payload[i];
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if (messageMQ == "flashON")
  {
    digitalWrite(LED_BUILTIN, HIGH); // Flash LED
    Serial.println("flashON");
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (messageMQ == "flashOFF")
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("flashOFF");
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if (messageMQ == "reset")
  {
    Serial.println("Reinicio ESP");
    ESP.restart();
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!clientMQ.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (clientMQ.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      clientMQ.publish(mqtt_topic_mensaje, "reconecto");
      // ... and resubscribe
      clientMQ.subscribe(mqtt_subtopic_mensaje);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(clientMQ.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqtt_mensaje()
{
  // mqtt_topic_mensaje
  ++value;
  printLocalTime();
  snprintf(msg, MSG_BUFFER_SIZE, "hello test esp32Cam #%ld  %s", value, tiempoNTP);

#ifdef _DEBUG_
  Serial.print("Publish message: ");
  Serial.println(msg);
#endif
  clientMQ.publish(mqtt_topic_mensaje, msg);
}

void mqtt_temp()
{
  // mqtt_topic_temp
  // mqtt_topic_hum
#ifdef DHT
  // Read temperature as Celsius (the default) & Humidity
  humedad = dht.getHumidity();
  // humedad = dht.readHumidity();
  temperatura = dht.getTemperature();
  // temperatura = dht.readTemperature();
#endif

#ifdef BME_ON
  temperatura = bme.readTempC();
  temperatura = temperatura - correccion_temp;
  humedad = bme.readFloatHumidity();

  // Check if any reads failed and exit early (to try again).
  if (isnan(temperatura))
  {
#ifdef _DEBUG_
    Serial.println(F("Failed to read from DHT/BME sensor!"));
#endif
    return;
  }

#ifdef DHT
  Serial.print(F("Humedad: "));
  Serial.print(humedad);
#endif
  // Serial.print(F("%  Temperatura: "));
  // Serial.println(temperatura);

  static char temperatureTemp[7];
  dtostrf(temperatura, 6, 2, temperatureTemp);
  static char humidity[7];
  dtostrf(humedad, 6, 0, humidity);

#ifdef DHT
  static char humidity[7];
  dtostrf(humedad, 6, 2, humidity);
#endif
#ifdef _DEBUG_
  Serial.print("Publish message: ");
  Serial.print(temperatureTemp);
  Serial.println(" °C");
  Serial.print("Publish message: ");
  Serial.print(humidity);
  Serial.println(" %");
#endif
  clientMQ.publish(mqtt_topic_temp, temperatureTemp);
  clientMQ.publish(mqtt_topic_hum, humidity);
#ifdef DHT
  clientMQ.publish(mqtt_topic_hum, humidity);
#endif
#endif
}

void mqtt_pir()
{
  if (pirSensor)
  {
    ++valuePir;
    printLocalTime();
    snprintf(pir, sizeof(pir), "Detectado movimiento #%ld  %s", valuePir, tiempoNTP);
    snprintf(pirON, sizeof(pirON), "ON");
    digitalWrite(pinLED, HIGH);
    delay(100);
    digitalWrite(pinLED, LOW);
  }
  else
  {
    printLocalTime();
    snprintf(pir, sizeof(pir), "NO detecto movimiento %s", tiempoNTP);
    snprintf(pirON, sizeof(pirON), "OFF");
  }
  Serial.print("Publish message: ");
  Serial.println(pir);
  clientMQ.publish(mqtt_topic_pir, pir);
  clientMQ.publish(mqtt_topic_pirON, pirON);
}
// Fin Funciones MQTT-----------------------

void setup()
{
#ifdef _DEBUG_
  Serial.begin(115200);
  delay(10);
  // Serial.setDebugOutput(true);
  // Serial.println();
#endif
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  pinMode(pinLED, OUTPUT);        // LED rojo
  pinMode(pinPIR, INPUT_PULLUP);  // PIR pin
  pinMode(LED_BUILTIN, OUTPUT);   // flash LED
  digitalWrite(LED_BUILTIN, LOW); // flash LED

#ifdef BME_ON
#ifdef _DEBUG_
  Serial.println("Reading basic values from BME280");
#endif
  // Init BME280 sensor
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  bme.settings.commInterface = I2C_MODE;
  bme.settings.I2CAddress = 0x76;
  bme.settings.runMode = 3;
  bme.settings.tStandby = 0;
  bme.settings.filter = 0;
  bme.settings.tempOverSample = 1;
  bme.settings.pressOverSample = 1;
  bme.settings.humidOverSample = 1;
  bme.begin();
#endif

  if (!initCamera())
  {
    Serial.printf("Failed to initialize camera...");
    return;
  }
  // Iniciamos conexion WIFI
  WiFi.begin(ssid, password);
  // iniciamos TemporizadorLED
  blinkLED.attach(0.2, callbackTimerLED);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println();
  Serial.print("WiFi connected with ip ");
  Serial.println(WiFi.localIP());
  // Setup Timer
  publicMQ.attach(intPublicMQTT, callbackTimerMQ);
  // pinging.attach(intPing, callbackTimerpinger); //comentado por interferencias con el pir
  blinkLED.detach();
  digitalWrite(pinLED, LOW);
  // init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  // Servidor WEB para descargar fotos
  server.on("/picture", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              camera_fb_t *frame = NULL;
              frame = esp_camera_fb_get();
              request->send_P(200, "image/jpeg", (const uint8_t *)frame->buf, frame->len);
              esp_camera_fb_return(frame); });

  server.begin();

  // Configuración MQTT
  clientMQ.setServer(mqtt_server, 1883);
  clientMQ.setCallback(callback);
  //--------------------------
}

void loop()
{
  RESET = digitalRead(resetPin); // Reset button
  if (RESET == HIGH)
  {
    Serial.print("Boton: ");
    Serial.println(RESET);
    Serial.println("Restarting ...");
    delay(1000);
    ESP.restart();
  }

  pirSensor = digitalRead(pinPIR);
  // Serial.print(pirSensor);

  if (pirSensor)
  {
    // Comprobar si se ha dado la vueltas
    if (millis() < ultimaDeteccionPir)
    {
      // Asignar un nuevo valor
      ultimaDeteccionPir = millis();
    }
    // Comprobar si ha pasado tiempo suficiente desde
    // la última pulsación
    if ((millis() - ultimaDeteccionPir) > tiempoDeteccionPir)
    {
      mqtt_pir();
      // Marca de tiempo para nueva detección
      ultimaDeteccionPir = millis();
    }
  }
  else if ((millis() - ultimaDeteccionPir) == pirNoDetect)
  {
    mqtt_pir();
  }

  // if WiFi is down, try reconnecting restart ESP
  unsigned long currentMillis = millis();
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval))
  {
    Serial.println(millis());
    Serial.println(WiFi.status());
    Serial.println("Reiniciar ESP");
    previousMillis = currentMillis;
    ESP.restart();
  }

  // Comprobación si hemos perdido conexión mqtt
  if (!clientMQ.connected())
  {
    reconnect();
  }
  // Procesamos los mensajes entrantes y  ejecutará la función callback que hayamos configurado
  clientMQ.loop();
  // Si ha pasado el tiempo marcado enviamos la temperatura
  if (sentinelSend)
  {
    mqtt_mensaje();
    mqtt_temp();
    // mqtt_pir();
    sentinelSend = false;
  }
}
// OK //TODO parpadep led con ticker cuaando detecta presencia (sin delay)
// OK //TODO publicar pirON = OFF cuando hayab pasado 30 segundos desde el ultimo pirON = ON
// NOK// TODO subscribirse a un topic para ponderar la temperatura
// OK //TODO corregir el pin para el fkash
// TODO ver por que no medi la humedad
// OK// TODO intPublicMQTT cambiar de 60 seg a 5 minutos
