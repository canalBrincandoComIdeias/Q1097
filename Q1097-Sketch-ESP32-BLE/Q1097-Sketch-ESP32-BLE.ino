/****************************************************************************************
  Video Q1096 - Tutorial Maker - Servo Motor
  
  Desenvolvido pela Fábrica de Programas - Brincando com Ideias (www.brincandocomideias.com)
  www.youtube.com/c/BrincandoComIdeias

  CUIDADOS AO USAR AS SEGUINTES PORTAS NO ESP32:
  -6 a 11: Reservados para a memória Flash SPI (Não usar)
  -34 a 39: Apenas entrada (INPUT)
  -0, 2, 12, 15: Possuem restrições no boot

  Autor Flavio Guimaraes
*****************************************************************************************/
#include "MoveServoESP32.h"
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>

// UUIDs de 128 bits (PARA USO REAL):
#define SERVICE_UUID "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"

#define pinPotA 33    //Potenciometro Esquerdo
#define pinPotB 34    //Potenciometro Central
#define pinPotC 35    //Potenciometro Direito
#define pinServoA 16  //Servo da Garra
#define pinServoB 17  //Servo do Braço
#define pinServoC 18  //Servo da Base
#define pinBotaoA 25  //Botão Liga/Desliga
#define pinBotaoB 26  //Pushbutton Esquerda
#define pinBotaoC 27  //Pushbutton Direita
#define pinLEDVm 14   //Led Vermelho
#define pinLEDVd 13   //Led Verde

//Variaveis de Controle
bool estadoExec = false;  //(false=Pause    true=Play)
bool valBotaoBAnt;
unsigned long delayBotaoB;
bool valBotaoCAnt;
unsigned long delayBotaoC;
bool estadoBraco;
bool estadoBracoAnt;
bool limpaPonteiro = false;
bool ligado = true;
unsigned long delayPosicao;
bool manualA = true;
bool manualB = true;
bool manualC = true;
int valA, valPotA, valPotAAnt;
int valB, valPotB, valPotBAnt;
int valC, valPotC, valPotCAnt;
int moverA = 0;
int moverB = 0;
int moverC = 0;
unsigned long delayMoveRemoto;
unsigned long delayValPot;
#define TEMPO_POSICAO 2000    //tempo de pausa na execução entre cada posição (em milissegundos)
#define VELOCIDADE_PLAY 80    //velocidade dos movimentos em modo play (de 0 zero a 100)
#define VELOCIDADE_GRAVA 95   //velocidade dos movimentos em modo gravação (de 0 zero a 100)
#define VELOCIDADE_REMOTO 80  //velocidade dos movimentos em modo gravação remoto (de 0 zero a 100)

//Variaveis para Controle da Memória
#define QTDE_POSICOES 200  //cada posicão conta com 5 bytes na memória (podem guardar a posiçao de até 5 servomotores)
byte bufPosicao[5];
int ponteiro = 0;
int posicoesGravadas = 0;

//Objetos para os Servos no ESP32
MoveServoESP32 servoA;
MoveServoESP32 servoB;
MoveServoESP32 servoC;

bool conexaoCancelada = false;
bool oKRecebido = false;
bool estadoBotaoA;
bool estadoBotaoAAnt;
unsigned long delayEstado;
unsigned long delayEnvio;
bool enviaEstado = false;
bool byteRecebido = false;
char chrBT;
String txtBT;

// DECLARAÇÃO DA VARIÁVEL deviceConnected *ANTES* das classes
bool deviceConnected = false;

// DECLARAÇÃO DAS CLASSES DE CALLBACK ANTES DE USÁ-LAS
class MyServerCallbacks : public BLEServerCallbacks {
public:
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Conectado");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Desconectado");
    pServer->getAdvertising()->start();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *characteristic) {
    if (characteristic->getValue().length() > 0) {
      std::string rxValue(characteristic->getValue().c_str());
      chrBT = rxValue[0];
      byteRecebido = true;
    }
  }
};

// DECLARAÇÃO DAS VARIÁVEIS GLOBAIS FORA DE QUALQUER FUNÇÃO
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;  // Inicializado com NULL

void setup() {
  pinMode(pinBotaoA, INPUT);
  pinMode(pinBotaoB, INPUT);
  pinMode(pinBotaoC, INPUT);
  pinMode(pinLEDVm, OUTPUT);
  pinMode(pinLEDVd, OUTPUT);

  servoA.attach(pinServoA, VELOCIDADE_PLAY);
  servoB.attach(pinServoB, VELOCIDADE_PLAY);
  servoC.attach(pinServoC, VELOCIDADE_PLAY);

  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Sketch Iniciado!");

#ifdef ARDUINO_ARCH_ESP32
  if (!EEPROM.begin(512)) {
    Serial.println("Erro ao iniciar EEPROM");
    Serial.println("Reiniciando...");
    delay(1000);
    ESP.restart();
  }
#endif

  //Limpa a EEPROM (executar apenas uma vez. Depois, comentar e carregar)
  //EEPROM.put((QTDE_POSICOES + 1) * 5, 0);

  //Lê quantas posições foram gravadas na EEPROM
  EEPROM.get((QTDE_POSICOES + 1) * 5, posicoesGravadas);

  BLEDevice::init("ESP32-BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  uint32_t properties = BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR;

  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, properties);
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Aguardando conexões BLE...");
}

void loop() {
  servoA.ServoLoop();
  servoB.ServoLoop();
  servoC.ServoLoop();

  //Controle do Envio do Estado Atual
  if (enviaEstado) {
    if ((millis() - delayEnvio) > 1000) {
      char byteEnviar;

      //Estado
      if (estadoBraco) {
        byteEnviar = 'X';
      } else {
        byteEnviar = 'x';
      }

      //Serial.print("Enviando estado atual: ");
      //Serial.write(byteEnviar);

      pCharacteristic->setValue((uint8_t *)&byteEnviar, 1);
      pCharacteristic->notify();

      enviaEstado = false;
    }
  }

  //CONTROLE DO BLUETOOTH
  if (byteRecebido) {
    byteRecebido = false;

    txtBT += chrBT;

    //Solicitação do App pelo estado atual
    if (txtBT.charAt(0) == '*') {
      enviaEstado = true;
      delayEnvio = millis();
    }

    if (txtBT.charAt(0) == 'F') {
      //Serial.println("Botao Fecha a Garra Apertado.");
      manualA = false;
      moverA = -1;
    }

    if (txtBT.charAt(0) == 'f') {
      //Serial.println("Botao Fecha a Garra Solto.");
      moverA = 0;
    }

    if (txtBT.charAt(0) == 'A') {
      //Serial.println("Botao Abre a Garra Apertado.");
      manualA = false;
      moverA = 1;
    }

    if (txtBT.charAt(0) == 'a') {
      //Serial.println("Botao Abre a Garra Solto.");
      moverA = 0;
    }

    if (txtBT.charAt(0) == 'D') {
      //Serial.println("Botao Desce o Braco Apertado.");
      manualB = false;
      moverB = 1;
    }

    if (txtBT.charAt(0) == 'd') {
      //Serial.println("Botao Desce o Braco Solto.");
      moverB = 0;
    }

    if (txtBT.charAt(0) == 'S') {
      //Serial.println("Botao Sobe o Braco Apertado.");
      manualB = false;
      moverB = -1;
    }

    if (txtBT.charAt(0) == 's') {
      //Serial.println("Botao Sobe o Braco Solto.");
      moverB = 0;
    }

    if (txtBT.charAt(0) == 'G') {
      //Serial.println("Botao Gira Sentido Anti-horário Apertado.");
      manualC = false;
      moverC = 1;
    }

    if (txtBT.charAt(0) == 'g') {
      //Serial.println("Botao Gira Sentido Anti-horário Braco Solto.");
      moverC = 0;
    }

    if (txtBT.charAt(0) == 'H') {
      //Serial.println("Botao Gira Sentido Horário Apertado.");
      manualC = false;
      moverC = -1;
    }

    if (txtBT.charAt(0) == 'h') {
      //Serial.println("Botao Gira Sentido Horário Braco Solto.");
      moverC = 0;
    }

    if (txtBT.charAt(0) == 'P') {
      //Serial.println("Botao Play Apertado.");
      estadoExec = true;
    }

    if (txtBT.charAt(0) == 'p') {
      //Serial.println("Botao Play Solto.");
    }

    if (txtBT.charAt(0) == 'U') {
      //Serial.println("Botao Pause Apertado.");
      estadoExec = false;
    }

    if (txtBT.charAt(0) == 'u') {
      //Serial.println("Botao Pause Solto.");
    }

    if (txtBT.charAt(0) == 'N') {
      //Serial.println("Botao Next Apertado.");
      moveNext();
    }

    if (txtBT.charAt(0) == 'n') {
      //Serial.println("Botao Next Solto.");
    }

    if (txtBT.charAt(0) == 'X') {
      //Serial.println("Botao Ligado.");
      estadoBraco = true;
    }

    if (txtBT.charAt(0) == 'x') {
      //Serial.println("Botao Desligado.");
      estadoBraco = false;
    }

    if (txtBT.charAt(0) == 'R') {
      //Serial.println("Botao Record Apertado.");
      gravaPos();
    }

    if (txtBT.charAt(0) == 'r') {
      //Serial.println("Botao Record Solto.");
    }

    txtBT = "";
  }


  //Controle do estado de execução
  if ((millis() - delayEstado) > 200) {
    if (digitalRead(pinBotaoA)) {
      estadoBotaoA = !estadoBotaoA;
    }
    if (estadoBotaoA != estadoBotaoAAnt) {
      delayEstado = millis();
      estadoBraco = estadoBotaoA;

      char byteEnviar;

      //Estado
      if (estadoBraco) {
        byteEnviar = 'X';
      } else {
        byteEnviar = 'x';
      }

      pCharacteristic->setValue((uint8_t *)&byteEnviar, 1);
      pCharacteristic->notify();
    }
    estadoBotaoAAnt = estadoBotaoA;
  }

  if (estadoBraco) {
    digitalWrite(pinLEDVm, false);
    servoA.velocidade(VELOCIDADE_PLAY);
    servoB.velocidade(VELOCIDADE_PLAY);
    servoC.velocidade(VELOCIDADE_PLAY);
  } else {
    digitalWrite(pinLEDVm, ligado);
    digitalWrite(pinLEDVd, false);
    servoA.velocidade(VELOCIDADE_GRAVA);
    servoB.velocidade(VELOCIDADE_GRAVA);
    servoC.velocidade(VELOCIDADE_GRAVA);
    estadoExec = false;

    if (ligado) {
      int val = analogRead(pinPotA);        // leitura do potenciometro
      valPotA = map(val, 0, 4095, 0, 180);  // converte o valor de faixa 0-4095 para a feixa dos angulos 0-180

      val = analogRead(pinPotB);
      valPotB = map(val, 0, 4095, 180, 0);

      val = analogRead(pinPotC);
      valPotC = map(val, 0, 4095, 180, 0);

      if (!manualA) {
        if (abs(valPotA - valPotAAnt) >= 8) manualA = true;
      }
      if (!manualB) {
        if (abs(valPotB - valPotBAnt) >= 8) manualB = true;
      }
      if (!manualC) {
        if (abs(valPotC - valPotCAnt) >= 8) manualC = true;
      }

      if ((millis() - delayValPot) > 100) {
        delayValPot = millis();
        valPotAAnt = valPotA;
        valPotBAnt = valPotB;
        valPotCAnt = valPotC;
      }

      if (manualA) {
        valA = valPotA;
      } else {
        //controle do movimento remoto
        if ((millis() - delayMoveRemoto) > (100 - VELOCIDADE_REMOTO)) {
          valA = constrain(valA + moverA, 0, 180);
        }
      }

      if (manualB) {
        valB = valPotB;
      } else {
        //controle do movimento remoto
        if ((millis() - delayMoveRemoto) > (100 - VELOCIDADE_REMOTO)) {
          valB = constrain(valB + moverB, 0, 180);
        }
      }

      if (manualC) {
        valC = valPotC;
      } else {
        //controle do movimento remoto
        if ((millis() - delayMoveRemoto) > (100 - VELOCIDADE_REMOTO)) {
          valC = constrain(valC + moverC, 0, 180);
        }
      }

      if ((millis() - delayMoveRemoto) > (100 - VELOCIDADE_REMOTO)) {
        delayMoveRemoto = millis();
      }

      servoA.move(valA);
      servoB.move(valB);
      servoC.move(valC);
    }
  }

  //Execução em Play
  if (estadoBraco && estadoExec) {
    if (estadoBraco) {
      digitalWrite(pinLEDVd, ligado);

      //Executa movimentos
      if ((millis() - delayPosicao) > TEMPO_POSICAO) {
        delayPosicao = millis();

        EEPROM.get(ponteiro * 5, bufPosicao);
        servoA.move(bufPosicao[0]);
        servoB.move(bufPosicao[1]);
        servoC.move(bufPosicao[2]);
        ponteiro++;

        if (ponteiro >= posicoesGravadas) {
          ponteiro = 0;
        }
      }
    } else {
      digitalWrite(pinLEDVd, LOW);
    }
  }

  //Execução em Pause
  if (estadoBraco && !estadoExec && ligado) {
    digitalWrite(pinLEDVd, bitRead(millis(), 9));
  }

  if (!estadoBraco && estadoBracoAnt) {
    limpaPonteiro = true;
  }

  if (estadoBraco && !estadoBracoAnt) {
    ponteiro = 0;
  }

  //Controle do BotaoB
  bool valBotaoB = digitalRead(pinBotaoB);
  if (valBotaoB && !valBotaoBAnt) {
    if ((millis() - delayBotaoB) > 200) {
      delayBotaoB = millis();

      if (estadoBraco) {  //Em Execução (Play ou Pause)
        estadoExec = !estadoExec;

      } else {  //Em Gravação
        gravaPos();
      }
    }
  }
  estadoBracoAnt = estadoBraco;
  valBotaoBAnt = valBotaoB;

  //Controle do BotaoC
  bool valBotaoC = digitalRead(pinBotaoC);
  if (valBotaoC && !valBotaoCAnt) {
    if ((millis() - delayBotaoC) > 200) {
      delayBotaoC = millis();

      if (estadoBraco && !estadoExec && ligado) {  //Em Execução (Play)
        moveNext();
      }
    }
  }
  valBotaoCAnt = valBotaoC;

  //Coloca o braço em posição para desligar
  if (estadoBraco) {
    if (digitalRead(pinBotaoB) && digitalRead(pinBotaoC)) {
      servoA.park();
      servoB.park();
      servoC.park();
      digitalWrite(pinLEDVm, LOW);
      digitalWrite(pinLEDVd, LOW);
      ligado = false;
    }
  }

  //Dados digitados no monitor serial, repassados para o bluetooth
  if (deviceConnected) {
    if (Serial.available() > 0) {
      size_t len = Serial.available();
      uint8_t sbuf[len];
      Serial.readBytes(sbuf, len);
      pCharacteristic->setValue(sbuf, len);
      pCharacteristic->notify();
      Serial.print("Dados enviados para o celular: ");
      Serial.write(sbuf, len);
      Serial.println();
    }
  }
  delay(1);
}

void moveNext() {
  EEPROM.get(ponteiro * 5, bufPosicao);
  servoA.move(bufPosicao[0]);
  servoB.move(bufPosicao[1]);
  servoC.move(bufPosicao[2]);
  ponteiro++;

  if (ponteiro >= posicoesGravadas) {
    ponteiro = 0;
  }
}

void gravaPos() {
  if (limpaPonteiro) {  //Se mudou para gravação, precisa zerar o ponteiro
    limpaPonteiro = false;
    posicoesGravadas = 0;
    ponteiro = 0;
  }

  if (posicoesGravadas < QTDE_POSICOES) {
    digitalWrite(pinLEDVm, false);
    delay(200);

    //Grava Posição
    bufPosicao[0] = servoA.getAngulo();
    bufPosicao[1] = servoB.getAngulo();
    bufPosicao[2] = servoC.getAngulo();
    EEPROM.put(ponteiro * 5, bufPosicao);

    ponteiro++;
    posicoesGravadas = ponteiro;
    EEPROM.put((QTDE_POSICOES + 1) * 5, posicoesGravadas);
  }
}