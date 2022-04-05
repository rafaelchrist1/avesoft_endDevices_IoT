#include <SPI.h>      //Biblioteca para a comunicação SPI entre a antena LoRa e o ESP
#include <LoRa.h>     //Biblioteca para o protocolo LoRa
#include <EEPROM.h>   //Biblioteca para salvar valor na memória não volátil do esp32
#include <WiFi.h>     //Biblioteca para abrir a pagina Web
#include <Wire.h>     //Biblioteca para comunicação I2C
#include "DHT.h"      //Biblioteca para o sensor DHT11 - Umidade e Temperatura
#include <BH1750.h>   //Biblioteca para o sensor BH1750 - Luminosidade
#include <ArduinoJson.h> //Biliboteca para a criacao dos JSON
#include "FS.h"
#include "SPIFFS.h"
#include "SPIFFS_functions.h"
#include <ESP32Encoder.h>

#define FORMAT_SPIFFS_IF_FAILED true

DHT dht(27, DHT22);
DHT dht1(28, DHT22);
DHT dht2(29, DHT22);

BH1750 lightMeter(0x23); //Define endereço no I2C do sensor BH1750
BH1750 lightMeter1(0x23); //Define endereço no I2C do sensor BH1750
BH1750 lightMeter2(0x23); //Define endereço no I2C do sensor BH1750

ESP32Encoder encoder;
ESP32Encoder encoder1;
ESP32Encoder encoder2;

typedef struct Sensores
{
  int id_sensor;
  int num;
};

//typedef struct YFS201
//{
//  int id_sensor;
//  int port;
//  volatile byte pulseCount;
//  byte pulse1Sec;
//  float flowRate;
//  float totalLitros;
//  unsigned int flowMilliLitres;
//};

typedef struct YFS201
{
  int id_sensor;
  int port;
  float totalLitros;
};

const int sensorGasFC22 = 34; //Declara sensor FC-22 na porta 34
const int sensorNivel = 4; //Declara sensor de nivel na porta 5
YFS201 sensoresAguaYFS201[3];
Sensores sensoresDHTTemp[3];
Sensores sensoresDHTUmid[3];
Sensores sensoresLux[3];

int numeroSensoresAgua = 0;
int numeroSensoresDHTTemp = 0;
int numeroSensoresDHTUmid = 0;
int numeroSensoresLux = 0;

//Variveis necessárias para o Sensor 5 de fluxo de agua
float calibrationFactor = 0.330;
int intervalRead = 1000;
unsigned long previousMillisWater = 0;

//void IRAM_ATTR pulseCounter1()
//{
//  sensoresAguaYFS201[0].pulseCount ++;
////  Serial.print("1");
//}
//
//void IRAM_ATTR pulseCounter2()
//{
//  sensoresAguaYFS201[1].pulseCount ++;
////  Serial.print("2");
//}
//
//void IRAM_ATTR pulseCounter3()
//{
//  sensoresAguaYFS201[2].pulseCount ++;
////  Serial.print("3");
//}

#define EEPROM_SIZE 18 // Defini quantidade de bytes da EEPROM

const int botaoConfig = 21;

hw_timer_t *timer = NULL; //Faz o controle do temporizador (interrupção por tempo)

//Defini os pinos de conexao da antena LoRa
const int csPin = 5;         //LoRa radio chip select
const int resetPin = 14;     //LoRa radio reset
const int irqPin = 2;        //Change for your board; must be a hardware interrupt pin

//Variaveis globais
bool debug = false;                //Variavel utilizado para o debug do programa (True = inicia a serial / False = não imprime na serial)
byte numeroSensoresRecebidos = 0;
byte totalSensoresReceber = 0;
volatile byte totalReset = 0;
unsigned long timeOutLoRaCampo = 0;

StaticJsonDocument<1000> doc1;
StaticJsonDocument<1000> doc2;

//Variaveis para o protocolo LoRa
String outgoing;              //outgoing message

byte msgCount = 0;            //count of outgoing messages
byte localAddress = 0x14;     //address of this device
byte destination = 0x0A;      //destination to send to
long lastSendTime = 0;        //last send time
int interval = 20000;         //interval between sends

//Funcao para reiniciar o ESP32
void IRAM_ATTR resetModule() {
  if (debug) Serial.println("(watchdog) reiniciar\n"); //Imprime no log
  ESP.restart(); //Reinicia o chip
}

void setup() {
  if (debug) Serial.begin(115200);                   //Inicia serial

  if (debug) Serial.println("EndDeviceStart");

  pinMode(botaoConfig, INPUT); //Declara pino do botao config como input

  LoRa.setPins(csPin, resetPin, irqPin);  //Set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {               //Inicia radio at 915 MHz
    if (debug) Serial.println("LoRa init failed. Check your connections.");
    while (true);                         //If failed, do nothing
  }

  if (debug) Serial.println("LoRa init succeeded.");
  LoRa.setGain(1);
  LoRa.setTxPower(20);

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    if (debug) Serial.println("SPIFFS Mount Failed");
    return;
  }

  //Defini e inicia timer para o WatchDog
  timer = timerBegin(0, 80, true); //Inicia o contador progressivo
  timerAttachInterrupt(timer, &resetModule, true); //Instancia do timer, função callback, interrupção de borda
  timerAlarmWrite(timer, 10000000, true); //Instancia de timer, tempo (us),10.000.000 us = 10 segundos
  timerAlarmEnable(timer); //Habilita a interrupção

  EEPROM.begin(EEPROM_SIZE); //Inicia EEPROM

  //EEPROM.put(2, calibrationFactor);
  //EEPROM.commit();

  //Recupera valor da memoria EEPROM
  //sensoresAtivos = EEPROM.read(0);

  localAddress = EEPROM.read(1);
  EEPROM.get(2, calibrationFactor);
  EEPROM.get(6, sensoresAguaYFS201[0].totalLitros);
  EEPROM.get(10, sensoresAguaYFS201[1].totalLitros);
  EEPROM.get(14, sensoresAguaYFS201[2].totalLitros);

  iniciaSensores();

  ESP32Encoder::useInternalWeakPullResistors = DOWN;

    for (int i = 0; i < numeroSensoresAgua; i++)
    {
      if (i == 0) {
        encoder.attachSingleEdge1(sensoresAguaYFS201[i].port);
        encoder.setCount(0);
        if (debug) Serial.println("Criou 1");
      }
      if (i == 1) {
        encoder1.attachSingleEdge1(sensoresAguaYFS201[i].port);
        encoder1.setCount(0);
        if (debug) Serial.println("Criou 2");
      }
      if (i == 2) {
        encoder2.attachSingleEdge1(sensoresAguaYFS201[i].port);
        encoder2.setCount(0);
        if (debug) Serial.println("Criou 3");
      }
    }
}

void loop() {
  timerWrite(timer, 0); //Reset o temporizador (alimenta o watchdog) zera o contador

  if (LoRa.isTransmitting())
  {
    if (debug) Serial.println("Não está mais transmitindo");
    if (!LoRa.begin(915E6)) ESP.restart();
  }

  unsigned long currentMillis = millis();

  //Atualiza valor dos sensores de fluxo de agua

  //  if (currentMillis - previousMillisWater > intervalRead)
  //  {
  //    for (int j = 0; j < numeroSensoresAgua; j++)
  //    {
  //      timerWrite(timer, 0); //Reset o temporizador (alimenta o watchdog) zera o contador
  //      if (debug) Serial.print("Aquisitou sensor: "); Serial.println(j);
  //      sensoresAguaYFS201[j].pulse1Sec = sensoresAguaYFS201[j].pulseCount;
  //      sensoresAguaYFS201[j].pulseCount = 0;
  //      sensoresAguaYFS201[j].flowRate = ((1000.0 / (millis() - previousMillisWater)) * sensoresAguaYFS201[j].pulse1Sec) / calibrationFactor;
  //
  //      sensoresAguaYFS201[j].flowMilliLitres = (sensoresAguaYFS201[j].flowRate / 60) * 1000;
  //      sensoresAguaYFS201[j].totalLitros += sensoresAguaYFS201[j].flowMilliLitres / 1000.0;
  //    }
  //    timerWrite(timer, 0); //Reset o temporizador (alimenta o watchdog) zera o contador
  //    EEPROM.put(6, sensoresAguaYFS201[0].totalLitros);
  //    EEPROM.put(10, sensoresAguaYFS201[1].totalLitros);
  //    EEPROM.put(14, sensoresAguaYFS201[2].totalLitros);
  //    EEPROM.commit();
  //    previousMillisWater = millis();
  //  }

  if (currentMillis - previousMillisWater > intervalRead)
  {
    for (int j = 0; j < numeroSensoresAgua; j++)
    {
      timerWrite(timer, 0); //Reset o temporizador (alimenta o watchdog) zera o contador
      if (debug) Serial.print("Aquisitou sensor: ");  if (debug)Serial.println(j);

      if (j == 0)
      {
        Serial.print("Sensor de nivel: "); Serial.println(digitalRead(22));
        float aux = abs((encoder.getCount() * calibrationFactor) / 1000.0);
         if (debug)Serial.print("Numero de pulsos recebidos : ");  if (debug)Serial.println( String((int32_t)encoder.getCount()));
         if (debug)Serial.print("Numero de litros: ");  if (debug)Serial.println(aux);
         sensoresAguaYFS201[j].totalLitros += aux;
         if (debug)Serial.print("Total de litros: ");  if (debug)Serial.println(sensoresAguaYFS201[j].totalLitros);
        encoder.setCount(0);
      }
      if (j == 1)
      {
        float aux = abs((encoder1.getCount() * calibrationFactor) / 1000.0);
         if (debug)Serial.print("Numero de pulsos recebidos : ");  if (debug)Serial.println( String((int32_t)encoder1.getCount()));
         if (debug)Serial.print("Numero de litros: ");  if (debug)Serial.println(aux);
        sensoresAguaYFS201[j].totalLitros += aux;
        encoder1.setCount(0);
      }
      if (j == 2)
      {
        float aux = abs((encoder2.getCount() * calibrationFactor) / 1000.0);
        if (debug)Serial.print("Numero de pulsos recebidos : ");  if (debug)Serial.println( String((int32_t)encoder2.getCount()));
        if (debug)Serial.print("Numero de litros: ");  if (debug)Serial.println(aux);
        sensoresAguaYFS201[j].totalLitros += aux;
        encoder2.setCount(0);
      }
    }
    EEPROM.put(6, sensoresAguaYFS201[0].totalLitros);
    EEPROM.put(10, sensoresAguaYFS201[1].totalLitros);
    EEPROM.put(14, sensoresAguaYFS201[2].totalLitros);
    EEPROM.commit();
    previousMillisWater = millis();
  }

  //Comando do botão config para entrar na configuração do endereço
  if (digitalRead(botaoConfig) == HIGH)
  {
    configEndDevice();
  }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

//Função para enviar mensagem pelo LoRa
void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  if (msgCount == 250) msgCount = 0;
  if (debug) Serial.println("Finalizou de enviar mensagem");
}

//Função para receber mensagem pelo LoRa
void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  if (debug) Serial.println("Recebeu");
  if (debug) Serial.println(millis());
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    if (debug) Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (debug) Serial.print("Endereço Local: ");
  if (debug)Serial.println(localAddress);
  if (debug) Serial.print("Endereço Recebido: ");
  if (debug)Serial.println(recipient);
  if (recipient != localAddress) {
    if (debug) Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  if (debug) Serial.println("Received from: 0x" + String(sender, HEX));
  if (debug) Serial.println("Sent to: 0x" + String(recipient, HEX));
  if (debug) Serial.println("Message ID: " + String(incomingMsgId));
  if (debug) Serial.println("Message length: " + String(incomingLength));
  if (debug) Serial.println("Message: " + incoming);
  if (debug) Serial.println("RSSI: " + String(LoRa.packetRssi()));
  if (debug) Serial.println("Snr: " + String(LoRa.packetSnr()));
  if (debug) Serial.println();

  StaticJsonDocument<500> doc10;

  DeserializationError err = deserializeJson(doc10, incoming); //Quebra o JSON para pegar as variaveis
  //Caso der erro no valor ele desconsidera os valores recebidos
  if (err) {
    if (debug) Serial.print(F("deserializeJson() failed with code "));
    if (debug) Serial.println(err.f_str());
    return;
  }
  else
  {
    if (doc10["status_device"] == 1)
    {
      lerSensor();
    }
    else
    {
      if (doc10["status_device"] == 2 and totalSensoresReceber == 0)
      {
        totalSensoresReceber = doc10["number_sensors"];
        if (debug) Serial.print("Total de sensores a receber"); if (debug) Serial.println(totalSensoresReceber);
        numeroSensoresRecebidos = 0;

        StaticJsonDocument<500> doc11;
        doc11["id_iot_end"] = doc10["id_iot_end"];
        String jsonBuffer = "";
        serializeJson(doc11, jsonBuffer); //Escreve para o client
        int aux = jsonBuffer.length();
        jsonBuffer.remove(aux - 1);
        jsonBuffer += ",\"sensors\": [";
        deleteFile(SPIFFS, "/JSONEndDevice.txt");
        writeFile(SPIFFS, "/JSONEndDevice.txt", jsonBuffer.c_str());
      }
      else if (numeroSensoresRecebidos < totalSensoresReceber)
      {
        if (numeroSensoresRecebidos < totalSensoresReceber - 1) incoming += ",";
        appendFile(SPIFFS, "/JSONEndDevice.txt", incoming.c_str());
        numeroSensoresRecebidos ++;
      }
      if (numeroSensoresRecebidos == totalSensoresReceber)
      {
        if (debug) Serial.println("Recebeu todos os sensores");
        appendFile(SPIFFS, "/JSONEndDevice.txt", "]}");
        listDir(SPIFFS, "/", 0);
        totalSensoresReceber = 0;
        iniciaSensores();
      }
      String message = "Configurou_" + String(localAddress) + "_OK";   //Mensagem ao mestre para confirmar config com sucesso
      sendMessage(message);      //Envia a mensagem
    }
  }
}

void iniciaSensores()
{
  DeserializationError err = deserializeJson(doc1, readFileString(SPIFFS, "/JSONEndDevice.txt")); //Quebra o JSON para pegar as variaveis
  //Caso der erro no valor ele desconsidera os valores recebidos
  if (err) {
    if (debug) Serial.print(F("deserializeJson() failed with code "));
    if (debug) Serial.println(err.f_str());
    return ;
  }
  else
  {
    byte numeroSensores = doc1["sensors"].size();
    numeroSensoresAgua = 0;
    numeroSensoresDHTTemp = 0;
    numeroSensoresDHTUmid = 0;
    numeroSensoresLux = 0;
    for (int i = 0; i < numeroSensores; i++)
    {
      if (doc1["sensors"][i]["type"] == "water-flow-rate")
      {
        sensoresAguaYFS201[numeroSensoresAgua].id_sensor = doc1["sensors"][i]["id_sensor"];
        sensoresAguaYFS201[numeroSensoresAgua].port = doc1["sensors"][i]["ad_paramameters"][0]["port"];

//        if (numeroSensoresAgua == 0) {
//          encoder.attachSingleEdge1(sensoresAguaYFS201[numeroSensoresAgua].port);
//          encoder.setCount(0);
//          if (debug) Serial.println("Criou 1");
//        }
//        if (numeroSensoresAgua == 1) {
//          encoder1.attachSingleEdge1(sensoresAguaYFS201[numeroSensoresAgua].port);
//          encoder1.setCount(0);
//          if (debug) Serial.println("Criou 2");
//        }
//        if (numeroSensoresAgua == 2) {
//          encoder2.attachSingleEdge1(sensoresAguaYFS201[numeroSensoresAgua].port);
//          encoder2.setCount(0);
//          if (debug) Serial.println("Criou 3");
//        }
        //        pinMode(sensoresAguaYFS201[i].port, INPUT_PULLUP);
        //
        //        if (numeroSensoresAgua == 0) attachInterrupt(digitalPinToInterrupt(sensoresAguaYFS201[numeroSensoresAgua].port), pulseCounter1, FALLING);
        //        if (numeroSensoresAgua == 1) attachInterrupt(digitalPinToInterrupt(sensoresAguaYFS201[numeroSensoresAgua].port), pulseCounter2, FALLING);
        //        if (numeroSensoresAgua == 2) attachInterrupt(digitalPinToInterrupt(sensoresAguaYFS201[numeroSensoresAgua].port), pulseCounter3, FALLING);

        numeroSensoresAgua ++;
      }

      if (doc1["sensors"][i]["type"] == "temperature")
      {
        sensoresDHTTemp[numeroSensoresDHTTemp].id_sensor = doc1["sensors"][i]["id_sensor"];
        sensoresDHTTemp[numeroSensoresDHTTemp].num = numeroSensoresDHTTemp;

        if (numeroSensoresDHTTemp == 0) dht.begin();
        if (numeroSensoresDHTTemp == 1) dht1.begin();
        if (numeroSensoresDHTTemp == 2) dht2.begin();

        numeroSensoresDHTTemp ++;
      }

      if (doc1["sensors"][i]["type"] == "humidity")
      {
        sensoresDHTUmid[numeroSensoresDHTUmid].id_sensor = doc1["sensors"][i]["id_sensor"];
        sensoresDHTUmid[numeroSensoresDHTUmid].num = numeroSensoresDHTUmid;

        if (numeroSensoresDHTUmid == 0) dht.begin();
        if (numeroSensoresDHTUmid == 1) dht1.begin();
        if (numeroSensoresDHTUmid == 2) dht2.begin();

        numeroSensoresDHTUmid ++;
      }

      if (doc1["sensors"][i]["type"] == "lux")
      {
        sensoresLux[numeroSensoresLux].id_sensor = doc1["sensors"][i]["id_sensor"];
        sensoresLux[numeroSensoresLux].num = numeroSensoresLux;

        if (numeroSensoresLux == 0)
        {
          int aux = doc1["sensors"][i]["ad_paramameters"][0]["port"];
          aux += 1;
          Wire.begin(aux, doc1["sensors"][i]["ad_paramameters"][0]["port"]);
          if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
            if (debug) Serial.println(F("BH1750 Zero iniciado"));
          }
          else {
            if (debug) Serial.println(F("Erro ao inicializar BH1750 Zero"));
          }
        }
        if (numeroSensoresLux == 1)
        {
          int aux = doc1["sensors"][i]["ad_paramameters"][0]["port"];
          aux += 1;
          Wire.begin(aux, doc1["sensors"][i]["ad_paramameters"][0]["port"]);
          if (lightMeter1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
            if (debug) Serial.println(F("BH1750 Um iniciado"));
          }
          else {
            if (debug) Serial.println(F("Erro ao inicializar BH1750 Um"));
          }
        }
        if (numeroSensoresLux == 2)
        {
          int aux = doc1["sensors"][i]["ad_paramameters"][0]["port"];
          aux += 1;
          Wire.begin(aux, doc1["sensors"][i]["ad_paramameters"][0]["port"]);
          if (lightMeter2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
            if (debug) Serial.println(F("BH1750 Dois iniciado"));
          }
          else {
            if (debug) Serial.println(F("Erro ao inicializar BH1750 Dois"));
          }
        }

        numeroSensoresLux ++;
      }
    }
  }
  if (debug) Serial.print("Sensores de agua: ");
  if (debug) Serial.println(numeroSensoresAgua);
  if (debug) Serial.print("Sensores DHT de Temperatura: "); Serial.println(numeroSensoresDHTTemp);
  if (debug) Serial.print("Sensores DHT de Humidade: ");
  if (debug) Serial.println(numeroSensoresDHTUmid);
  if (debug) Serial.print("Sensores DHT de Lux: ");
  if (debug) Serial.println(numeroSensoresLux);
}

void lerSensor()
{
  unsigned long initialAcquisitionTime = millis();
  String jsonBuffer = "";
  if (debug) Serial.println("Entrou no void de ler os sensores");
  byte numeroSensores = doc1["sensors"].size();
  int  auxIdEndDevice = doc1["id_iot_end"];
  jsonBuffer = "{\"id_iot_end\":" + String(auxIdEndDevice) + ",";
  jsonBuffer = jsonBuffer + "\"sensors\": [";
  for (int i = 0; i < numeroSensores; i++)
  {
    if (i > 0) jsonBuffer = jsonBuffer + ",";
    if (doc1["sensors"][i]["type"] == "temperature")
    {
      for (int j = 0; j < numeroSensoresDHTTemp; j++)
      {
        if (sensoresDHTTemp[j].id_sensor == doc1["sensors"][i]["id_sensor"])
        {
          int aux = doc1["sensors"][i]["id_sensor"];
          jsonBuffer = jsonBuffer + "{\"id_sensor\":" + String(aux) + ",";
          if (sensoresDHTTemp[j].num == 0) jsonBuffer = jsonBuffer + "\"value\":" +  String(dht.readTemperature());
          if (sensoresDHTTemp[j].num == 1) jsonBuffer = jsonBuffer + "\"value\":" +  String(dht1.readTemperature());
          if (sensoresDHTTemp[j].num == 2) jsonBuffer = jsonBuffer + "\"value\":" +  String(dht2.readTemperature());
          jsonBuffer = jsonBuffer + "}";
          if (debug) Serial.println("Temperatura");
        }
      }
    }

    if (doc1["sensors"][i]["type"] == "humidity")
    {
      for (int j = 0; j < numeroSensoresDHTUmid; j++)
      {
        if (sensoresDHTUmid[j].id_sensor == doc1["sensors"][i]["id_sensor"])
        {
          int aux = doc1["sensors"][i]["id_sensor"];
          jsonBuffer = jsonBuffer + "{\"id_sensor\":" +  String(aux) + ",";
          if (sensoresDHTUmid[j].num == 0) jsonBuffer = jsonBuffer + "\"value\":" +  String(dht.readHumidity());
          if (sensoresDHTUmid[j].num == 1) jsonBuffer = jsonBuffer + "\"value\":" +  String(dht1.readHumidity());
          if (sensoresDHTUmid[j].num == 2) jsonBuffer = jsonBuffer + "\"value\":" +  String(dht2.readHumidity());
          jsonBuffer = jsonBuffer + "}";
          if (debug) Serial.println("Humidade");
        }
      }
    }

    if (doc1["sensors"][i]["type"] == "lux")
    {
      for (int j = 0; j < numeroSensoresLux; j++)
      {
        if (sensoresLux[j].id_sensor == doc1["sensors"][i]["id_sensor"])
        {
          int aux = doc1["sensors"][i]["id_sensor"];
          jsonBuffer = jsonBuffer + "{\"id_sensor\":" +  String(aux) + ",";
          if (sensoresLux[j].num == 0) jsonBuffer = jsonBuffer + "\"value\":" +  String(lightMeter.readLightLevel());
          if (sensoresLux[j].num == 1) jsonBuffer = jsonBuffer + "\"value\":" +  String(lightMeter1.readLightLevel());
          if (sensoresLux[j].num == 2) jsonBuffer = jsonBuffer + "\"value\":" +  String(lightMeter2.readLightLevel());
          jsonBuffer = jsonBuffer + "}";
          if (debug) Serial.println("Luminosidade");
        }
      }
    }

    if (doc1["sensors"][i]["type"] == "iqar")
    {
      if (debug) Serial.println("Qualidade de ar");
      pinMode(doc1["sensors"][i]["ad_paramameters"][0]["port"], INPUT); //Define o pino do sensor de gas como entrada
      int aux = doc1["sensors"][i]["id_sensor"];
      jsonBuffer = jsonBuffer + "{\"id_sensor\":" +  String(aux) + ",";
      jsonBuffer = jsonBuffer + "\"value\":" +  String(analogRead(doc1["sensors"][i]["ad_paramameters"][0]["port"]));
      jsonBuffer = jsonBuffer + "}";
    }

    if (doc1["sensors"][i]["type"] == "water-level")
    {
      if (debug) Serial.println("Nivel Agua");
      pinMode(doc1["sensors"][i]["ad_paramameters"][0]["port"], INPUT); //Define o pino do sensor de gas como entrada
      int aux = doc1["sensors"][i]["id_sensor"];
      jsonBuffer = jsonBuffer + "{\"id_sensor\":" + String(aux) + ",";
      jsonBuffer = jsonBuffer + "\"value\":" +  String(digitalRead(doc1["sensors"][i]["ad_paramameters"][0]["port"]));
      jsonBuffer = jsonBuffer + "}";
    }

    if (doc1["sensors"][i]["type"] == "water-flow-rate")
    {
      if (debug) Serial.println("Volume agua");
      for (int l = 0; l < numeroSensoresAgua; l++)
      {
        if (sensoresAguaYFS201[l].id_sensor == doc1["sensors"][i]["id_sensor"])
        {
          int aux = doc1["sensors"][i]["id_sensor"];

          jsonBuffer = jsonBuffer + "{\"id_sensor\":" +  String(aux) + ",";
          jsonBuffer = jsonBuffer + "\"value\":" +  String(sensoresAguaYFS201[l].totalLitros);
          jsonBuffer = jsonBuffer + "}";
        }
      }

    }
    timerWrite(timer, 0);
  }
  jsonBuffer = jsonBuffer + "]}";
  if (debug) Serial.println(jsonBuffer);
  if (millis() - initialAcquisitionTime < 450) sendMessage(jsonBuffer);
}

//Função que abre um access point para configurar endereço LoRa do device
void configEndDevice()
{
  // Replace with your network credentials
  const char* ssid     = "AP_EndDevice";
  const char* password = "123456789";

  byte endereco = EEPROM.read(1);
  if (endereco == 255) endereco = 20;
  if (endereco < 20) endereco = 20;

  // Set web server port number to 80
  WiFiServer server(80);

  // Variable to store the HTTP request
  String header;
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  if (debug) Serial.print("AP IP address: ");
  if (debug) Serial.println(IP);

  server.begin();

  while (1)
  {
    timerWrite(timer, 0); //Reset o temporizador (alimenta o watchdog) zera o contador

    WiFiClient client = server.available();   // Listen for incoming clients

    if (client) {                             // If a new client connects,
      if (debug) Serial.println("New Client.");          // print a message out in the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        timerWrite(timer, 0); //Reset o temporizador (alimenta o watchdog) zera o contador
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          if (debug) Serial.write(c);                    // print it out the serial monitor
          header += c;
          if (c == '\n') {                    // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");

              // CSS Style
              client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
              client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
              client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
              client.println(".button2 {background-color: #555555;}</style></head>");

              // Web Page Heading
              client.println("<body><h1>End Device Config</h1>");

              client.println("<h3>Endereco LoRa<h3>");
              client.println(endereco);
              client.print("\t");
              client.print("<a href=\"/+endereco\"><button  style='width: 60px'>+</button></a>");
              client.print("\t");
              client.println("<a href=\"/-endereco\"><button  style='width: 60px'>-</button></a>");

              client.println("<h3>Ajuste Fluxo de Agua<h3>");
              client.println(calibrationFactor, 3);
              client.print("\t");
              client.print("<a href=\"/+calibration\"><button  style='width: 60px'>+</button></a>");
              client.print("\t");
              client.println("<a href=\"/-calibration\"><button  style='width: 60px'>-</button></a>");

              client.println("<h3>Total de litros do Sensor 1<h3>");
              client.println(sensoresAguaYFS201[0].totalLitros, 2);
              client.print("\t");
              client.print("<a href=\"/ResetLitrosS1\"><button  style='width: 200px'>Reset Litros Sensor 1</button></a>");
              client.println("<h3>Total de litros do Sensor 2<h3>");
              client.println(sensoresAguaYFS201[1].totalLitros, 2);
              client.print("\t");
              client.print("<a href=\"/ResetLitrosS2\"><button  style='width: 200px'>Reset Litros Sensor 2</button></a>");
              client.println("<h3>Total de litros do Sensor 3<h3>");
              client.println(sensoresAguaYFS201[2].totalLitros, 2);
              client.print("\t");
              client.print("<a href=\"/ResetLitrosS3\"><button  style='width: 200px'>Reset Litros Sensor 3</button></a>");

              client.println("<p><a href=\"/salvar\"><button class=\"button button2\">Salvar</button></a></p>");

              client.println("</body></html>");

              client.println();
              break;
            } else { // if you got a newline, then clear currentLine
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
          if (currentLine.endsWith("GET /salvar"))
          {
            //Escreve o valor na memória EEPROM
            EEPROM.write(1, endereco);
            EEPROM.put(2, calibrationFactor);
            EEPROM.put(6, sensoresAguaYFS201[0].totalLitros);
            EEPROM.put(10, sensoresAguaYFS201[1].totalLitros);
            EEPROM.put(14, sensoresAguaYFS201[2].totalLitros);
            EEPROM.commit();
            ESP.restart();
          }
          if (currentLine.endsWith("GET /+endereco"))endereco ++;
          if (currentLine.endsWith("GET /-endereco"))endereco --;
          if (currentLine.endsWith("GET /+calibration"))calibrationFactor += 0.005;
          if (currentLine.endsWith("GET /-calibration"))calibrationFactor -= 0.005;
          if (currentLine.endsWith("GET /ResetLitrosS1"))sensoresAguaYFS201[0].totalLitros = 0;
          if (currentLine.endsWith("GET /ResetLitrosS2"))sensoresAguaYFS201[1].totalLitros = 0;
          if (currentLine.endsWith("GET /ResetLitrosS3"))sensoresAguaYFS201[2].totalLitros = 0;
        }
      }
      // Clear the header variable
      header = "";
      // Close the connection
      client.stop();
      if (debug) Serial.println("Client disconnected.");
      if (debug) Serial.println("");
    }
  }

}
