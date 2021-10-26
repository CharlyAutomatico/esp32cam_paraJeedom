#define _DEBUG_ //debug
//#define mDNS_CAM    // activar Nombre DNS

#define LED_BUILTIN 4 // WILL BE USED TO TURN THE LED ON AND OFF - DO NOT MODIFY THIS VALUE (GPIO4)
#define pinLED 2 //Led rojo
//#define DHT //descomentar si utilizamos el sensor de temperatura DHT22
#ifdef DHT
// Data DHT22
#define DHTPIN 14 // Digital pin connected to the DHT sensor
// #define DHTTYPE DHT22   // DHT 22  (AM2302)
#endif

#define BME_ON //Activa  Sensor de temperatura

//Timer send PUB MQTT
#define intPublicMQTT 60 //60 seg envio mensajes MQTT al Broker
//const byte intPublicMQTT = XX; //execution time in sec

//PIR
#define pinPIR 13
//#define pinLed 33
#define pirNoDetect 30000 // 30 segundos sin deteccion

//PING
#define numPing 6 //contador hasta 6
#define intPing 5 // segundos cada test ping al GW

#define ENABLE_WEBSERVER          // Web Server
#define RESOLUTION FRAMESIZE_SVGA // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
#define QUALITY 10                // JPEG quality 10-63 (lower means better quality)
#define WSPORT 80                 //Web server port

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// PARAMETRES IMAGE
#define BRIGHT 2   // Brightness: -2 to 2
#define CONTRAST 0 // Contrast: -2 to 2
#define SATUR 0    // Color Saturation: -2 to 2
//#define SPECIAL   0           // Special effects: 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
#define WHITEBAL 1 // White Balance: 0 = disable , 1 = enable
#define AWBGAIN 1  // White Balance Gain: 0 = disable , 1 = enable
#define AWBMODE 0  // (3) White Balance Mode: 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
#define EXPOS 1    // Exposure Control: 0 = disable , 1 = enable
#define AEC2 0     // 0 = disable , 1 = enable
#define AELEV 0    // -2 to 2
#define AECVAL 300 // 0 to 1200
#define GAINCT 1   // 0 = disable , 1 = enable
#define GAINAGC 0  // 0 = disable , 1 = enable
#define GAINCELL 0 // 0 = disable , 1 = enable
#define BPC 0      // 0 = disable , 1 = enable
#define WPC 1      // 0 = disable , 1 = enable
#define RAWGMA 1   // 0 = disable , 1 = enable
#define LENC 1     // Lens Correction: 0 = disable , 1 = enable
#define MIRROR 1   // Horizontal Mirror: 0 = disable , 1 = enable // 0 para lentes 160
#define FLIP 0     // Vertical Flip: 0 = disable , 1 = enable // 0 para lentes 160
#define DCW 1      // 0 = disable , 1 = enable
#define CBAR 0     // Set a colorBar: 0 = disable , 1 = enable