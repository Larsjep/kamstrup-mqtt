#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "gcm.h"
#include "mbusparser.h"
#include "secrets.h"
#include <WiFiUdp.h>
#include <string>
#include <user_interface.h>

#define DEBUG_BEGIN // Serial.begin(115200);
#define DEBUG_PRINT(x) Serial.print(x);sendmsg(String(mqtt_topic)+"/status",x);
#define DEBUG_PRINTLN(x) Serial.println(x);sendmsg(String(mqtt_topic)+"/status",x);

const size_t headersize = 11;
const size_t footersize = 3;
uint8_t encryption_key[16];
uint8_t authentication_key[16];
uint8_t receiveBuffer[500];
uint8_t decryptedFrameBuffer[500];
VectorView decryptedFrame(decryptedFrameBuffer, 0);
MbusStreamParser streamParser(receiveBuffer, sizeof(receiveBuffer));
mbedtls_gcm_context m_ctx;

WiFiClient espClient;
PubSubClient client(espClient);

#define UDP_PORT 4210
WiFiUDP Udp;

void printUdp(const char* str)
{
  Udp.beginPacket(IPAddress(0xffffffff), 4210);
  Udp.write(str);
  Udp.endPacket();
}

uint32 lastWake = 0;
uint32 serialCounter = 0;

void setup() {
  //DEBUG_BEGIN
  //DEBUG_PRINTLN("")
  Serial.setRxBufferSize(1024);
  Serial.begin(2400);
  Serial.println(" I can print something");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.flush();

  client.setServer(mqttServer, mqttPort);
  
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    Serial.flush();

    if (client.connect(mqttClientID)) { //, mqttUser, mqttPassword )) {

      Serial.println("connected");
      Serial.flush();
    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }

  }

  Serial.flush();
  //delay(200); // Some delay to ensure all has been sent

  //Serial.begin(2400, SERIAL_8N1);
  //Serial.swap();
  hexStr2bArr(encryption_key, conf_key, sizeof(encryption_key));
  hexStr2bArr(authentication_key, conf_authkey, sizeof(authentication_key));

  Udp.begin(UDP_PORT);
  Serial.println("Setup completed");

  printUdp("Setup completed!!\n");

  // printUdp("Going to sleep\n");
  // delay(200);
  // WiFi.forceSleepBegin();
  // yield();
}

bool firstData = false;
uint32 oldTime = 0;

void loop2() 
{
    client.loop();
    printUdp("Going to sleep\n");
    delay(200);
    // WiFi.mode( WIFI_OFF );
    WiFi.forceSleepBegin();
    delay(10000);
    WiFi.forceSleepWake();
    delay(1);
    // Bring up the WiFi connection
    // WiFi.mode(WIFI_STA);
    // WiFi.begin(ssid, password);

    int wakeTime = 0;    
    uint32 tick = system_get_time();
    uint32 cycleTime = tick - oldTime;
    oldTime = tick;
    while (WiFi.status() != WL_CONNECTED)
    {
      wakeTime += 100;
      delay(100);
    }
    uint32 connectTime = system_get_time() - tick;

    if (!client.connected())
    {
      printUdp("MQTT was disconnected, reconnecting\n");
      client.connect(mqttClientID);
    }

    uint32 mqttTime = system_get_time() - tick;

    printUdp((std::string("Has woken up, sending MQTT wifi: ") + std::to_string(connectTime) + "us, MQTT: " + std::to_string(mqttTime) + "us, cycle: " + std::to_string(cycleTime) + "us\n").c_str());
    client.publish("outtopic", "hello is awake");
    // for (int i = 0;i != 10; i++)
    // {      
    //   client.loop();
    //   delay(100);
    // }
}

void wakeAndPrint(const char* str)
{
  WiFi.forceSleepWake();
  delay(1);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }  
  printUdp(str);

  delay(200);
  WiFi.forceSleepBegin();
  lastWake = system_get_time();
}

void wakeAndSend(const MeterData& md)
{
  WiFi.forceSleepWake();
  delay(1);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }
  if (!client.connected())
  {
    client.connect(mqttClientID);
  }

  printUdp("Frame ok, sending MQTT\n");
  sendData(md);

  delay(200);
  WiFi.forceSleepBegin();
  lastWake = system_get_time();
}

uint32_t frameStartCount = 0;


void loop3() {
  while (Serial.available() > 0) {
    serialCounter++;
    //Serial.println("test");
    //for(int i=0;i<sizeof(input);i++){
    uint8_t data = Serial.read();
    if (data == 0x7E)
    {
      frameStartCount++;
    }
    if (streamParser.pushData(data)) {
      //  if (streamParser.pushData(input[i])) {
      VectorView frame = streamParser.getFrame();
      if (streamParser.getContentType() == MbusStreamParser::COMPLETE_FRAME) {
        if (!decrypt(frame))
        {
          wakeAndPrint("Decryption failed\n");
          return;
        }
        MeterData md = parseMbusFrame(decryptedFrame);
        wakeAndSend(md);
      }
    }
  }
  delay(100);
  if ((system_get_time() - lastWake) > 15000000)
  {
    wakeAndPrint((std::string("No frames received in 15s. Serial counter = ") + std::to_string(serialCounter) + ", frame = " + std::to_string(frameStartCount) + "\n").c_str());  
  }
}

std::vector<uint8_t> buffer;
uint32_t lastData = system_get_time();

template <typename I> std::string n2hexstr(I w, size_t hex_len = sizeof(I)<<1) {
    static const char* digits = "0123456789ABCDEF";
    std::string rc(hex_len,'0');
    for (size_t i=0, j=(hex_len-1)*4 ; i<hex_len; ++i,j-=4)
        rc[i] = digits[(w>>j) & 0x0f];
    return rc;
}

void loop()
{
  while (Serial.available() > 0)
  {
    buffer.push_back(Serial.read());
    lastData = system_get_time();
  }

  delay(100);
  if ((system_get_time() - lastData) > 1000000)
  {
    if (buffer.size() > 0)
    {
      std::string text;
      text.reserve(buffer.size() * 2 + 10);
      text += std::to_string(buffer.size()) + ":";
      for (const auto d : buffer)
      {
        text += n2hexstr(d, 2);
      }
      text += "<\n";
      buffer.clear();
      printUdp(text.c_str());
    }
  }
}


void sendData(MeterData md) {
  if (md.activePowerPlusValid)
    sendmsg(String(mqtt_topic) + "/activePowerPlus", String(md.activePowerPlus));
  if (md.activePowerMinusValid)
    sendmsg(String(mqtt_topic) + "/activePowerMinus", String(md.activePowerMinus));
  if (md.activePowerPlusValidL1)
    sendmsg(String(mqtt_topic) + "/activePowerPlusL1", String(md.activePowerPlusL1));
  if (md.activePowerMinusValidL1)
    sendmsg(String(mqtt_topic) + "/activePowerMinusL1", String(md.activePowerMinusL1));
  if (md.activePowerPlusValidL2)
    sendmsg(String(mqtt_topic) + "/activePowerPlusL2", String(md.activePowerPlusL2));
  if (md.activePowerMinusValidL2)
    sendmsg(String(mqtt_topic) + "/activePowerMinusL2", String(md.activePowerMinusL2));
  if (md.activePowerPlusValidL3)
    sendmsg(String(mqtt_topic) + "/activePowerPlusL3", String(md.activePowerPlusL3));
  if (md.activePowerMinusValidL3)
    sendmsg(String(mqtt_topic) + "/activePowerMinusL3", String(md.activePowerMinusL3));
  if (md.reactivePowerPlusValid)
    sendmsg(String(mqtt_topic) + "/reactivePowerPlus", String(md.reactivePowerPlus));
  if (md.reactivePowerMinusValid)
    sendmsg(String(mqtt_topic) + "/reactivePowerMinus", String(md.reactivePowerMinus));

  if (md.powerFactorValidL1)
    sendmsg(String(mqtt_topic) + "/powerFactorL1", String(md.powerFactorL1));
  if (md.powerFactorValidL2)
    sendmsg(String(mqtt_topic) + "/powerFactorL2", String(md.powerFactorL2));
  if (md.powerFactorValidL3)
    sendmsg(String(mqtt_topic) + "/powerFactorL3", String(md.powerFactorL3));
  if (md.powerFactorTotalValid)
    sendmsg(String(mqtt_topic) + "/powerFactorTotal", String(md.powerFactorTotal));

  if (md.voltageL1Valid)
    sendmsg(String(mqtt_topic) + "/voltageL1", String(md.voltageL1));
  if (md.voltageL2Valid)
    sendmsg(String(mqtt_topic) + "/voltageL2", String(md.voltageL2));
  if (md.voltageL3Valid)
    sendmsg(String(mqtt_topic) + "/voltageL3", String(md.voltageL3));

  if (md.centiAmpereL1Valid)
    sendmsg(String(mqtt_topic) + "/currentL1", String(md.centiAmpereL1 / 100.));
  if (md.centiAmpereL2Valid)
    sendmsg(String(mqtt_topic) + "/currentL2", String(md.centiAmpereL2 / 100.));
  if (md.centiAmpereL3Valid)
    sendmsg(String(mqtt_topic) + "/currentL3", String(md.centiAmpereL3 / 100.));

  if (md.activeImportWhValid)
    sendmsg(String(mqtt_topic) + "/energyActiveImportKWh", String(md.activeImportWh / 1000.));
  if (md.activeExportWhValid)
    sendmsg(String(mqtt_topic) + "/energyActiveExportKWh", String(md.activeExportWh / 1000.));
  if (md.activeImportWhValidL1)
    sendmsg(String(mqtt_topic) + "/energyActiveImportKWhL1", String(md.activeImportWhL1 / 1000.));
  if (md.activeExportWhValidL1)
    sendmsg(String(mqtt_topic) + "/energyActiveExportKWhL1", String(md.activeExportWhL1 / 1000.));
  if (md.activeImportWhValidL2)
    sendmsg(String(mqtt_topic) + "/energyActiveImportKWhL2", String(md.activeImportWhL2 / 1000.));
  if (md.activeExportWhValidL2)
    sendmsg(String(mqtt_topic) + "/energyActiveExportKWhL2", String(md.activeExportWhL2 / 1000.));
  if (md.activeImportWhValidL3)
    sendmsg(String(mqtt_topic) + "/energyActiveImportKWhL3", String(md.activeImportWhL3 / 1000.));
  if (md.activeExportWhValidL3)
    sendmsg(String(mqtt_topic) + "/energyActiveExportKWhL3", String(md.activeExportWhL3 / 1000.));

  if (md.reactiveImportWhValid)
    sendmsg(String(mqtt_topic) + "/energyReactiveImportKWh", String(md.reactiveImportWh / 1000.));
  if (md.reactiveExportWhValid)
    sendmsg(String(mqtt_topic) + "/energyReactiveExportKWh", String(md.reactiveExportWh / 1000.));
}

void printHex(const unsigned char* data, const size_t length) {
  for (int i = 0; i < length; i++) {
    Serial.printf("%02X", data[i]);
  }
}

void printHex(const VectorView& frame) {
  for (int i = 0; i < frame.size(); i++) {
    Serial.printf("%02X", frame[i]);
  }
}

bool decrypt(const VectorView& frame) {

  if (frame.size() < headersize + footersize + 12 + 18) {
    Serial.println("Invalid frame size.");
  }

  memcpy(decryptedFrameBuffer, &frame.front(), frame.size());

  uint8_t system_title[8];
  memcpy(system_title, decryptedFrameBuffer + headersize + 2, 8);

  uint8_t initialization_vector[12];
  memcpy(initialization_vector, system_title, 8);
  memcpy(initialization_vector + 8, decryptedFrameBuffer + headersize + 14, 4);

  uint8_t additional_authenticated_data[17];
  memcpy(additional_authenticated_data, decryptedFrameBuffer + headersize + 13, 1);
  memcpy(additional_authenticated_data + 1, authentication_key, 16);

  uint8_t authentication_tag[12];
  memcpy(authentication_tag, decryptedFrameBuffer + headersize + frame.size() - headersize - footersize - 12, 12);

  uint8_t cipher_text[frame.size() - headersize - footersize - 18 - 12];
  memcpy(cipher_text, decryptedFrameBuffer + headersize + 18, frame.size() - headersize - footersize - 12 - 18);

  uint8_t plaintext[sizeof(cipher_text)];

  mbedtls_gcm_init(&m_ctx);
  int success = mbedtls_gcm_setkey(&m_ctx, MBEDTLS_CIPHER_ID_AES, encryption_key, sizeof(encryption_key) * 8);
  if (0 != success) {
    Serial.println("Setkey failed: " + String(success));
    return false;
  }
  success = mbedtls_gcm_auth_decrypt(&m_ctx, sizeof(cipher_text), initialization_vector, sizeof(initialization_vector),
                                     additional_authenticated_data, sizeof(additional_authenticated_data), authentication_tag, sizeof(authentication_tag),
                                     cipher_text, plaintext);
  if (0 != success) {
    Serial.println("authdecrypt failed: " + String(success));
    return false;
  }
  mbedtls_gcm_free(&m_ctx);

  //copy replace encrypted data with decrypted for mbusparser library. Checksum not updated. Hopefully not needed
  memcpy(decryptedFrameBuffer + headersize + 18, plaintext, sizeof(plaintext));
  decryptedFrame = VectorView(decryptedFrameBuffer, frame.size());

  return true;
}

void hexStr2bArr(uint8_t* dest, const char* source, int bytes_n)
{
  uint8_t* dst = dest;
  uint8_t* end = dest + sizeof(bytes_n);
  unsigned int u;

  while (dest < end && sscanf(source, "%2x", &u) == 1)
  {
    *dst++ = u;
    source += 2;
  }
}


void sendmsg(String topic, String payload) {
  if (client.connected() && WiFi.status() == WL_CONNECTED) {
    // If we are connected to WiFi and MQTT, send. (From Niels Ørbæk)
    client.publish(topic.c_str(), payload.c_str());
    delay(10);
  } else {
    // Otherwise, restart the chip, hoping that the issue resolved itself.
    delay(60*1000);
    ESP.restart();
  }
}
