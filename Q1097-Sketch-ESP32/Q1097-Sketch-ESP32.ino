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
#include "BluetoothSerial.h"
#include <EEPROM.h>

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

BluetoothSerial serialBT;

String device_name = "ESP32-BT";
String txtBT;

bool conexaoCancelada = false;
bool oKRecebido = false;
bool estadoBotaoA;
bool estadoBotaoAAnt;
unsigned long delayEstado;

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

  serialBT.begin(device_name);
  Serial.printf("O dispositivo de nome \"%s\" foi iniciado.\nO pareamento com Bluetooth esta disponivel!\n", device_name.c_str());
}

void loop() {
  servoA.ServoLoop();
  servoB.ServoLoop();
  servoC.ServoLoop();

  //val = map(val, 0, 4095, 0, 180);  // converte o valor de faixa 0-4095 para a feixa dos angulos 0-180

  //Dados recebidos do bluetooth
  if (serialBT.available()) {
    char byteRecebido = serialBT.read();

    //Retorno ao monitor serial
    //Serial.write(byteRecebido);
    //Serial.println();

    //Pedido de estado
    if (byteRecebido == '*') {
      char byteEnviar;

      //Estado
      if (estadoBraco) {
        byteEnviar = 'X';
      } else {
        byteEnviar = 'x';
      }

      serialBT.write(byteEnviar);
    }

    //Comando de desconexao. Ignora texto automatico do modulo.
    if (byteRecebido == '+') {
      conexaoCancelada = true;
    }

    if (conexaoCancelada) {
      if (byteRecebido == 'O') {
        oKRecebido = true;
      }
      if (oKRecebido) {
        if (byteRecebido == 'K') {
          conexaoCancelada = false;
          oKRecebido = false;
        }
      }
    }

    if (!conexaoCancelada) {
      if (byteRecebido == 'F') {
        //Serial.println("Botao Fecha a Garra Apertado.");
        manualA = false;
        moverA = -1;
      }

      if (byteRecebido == 'f') {
        //Serial.println("Botao Fecha a Garra Solto.");
        moverA = 0;
      }

      if (byteRecebido == 'A') {
        //Serial.println("Botao Abre a Garra Apertado.");
        manualA = false;
        moverA = 1;
      }

      if (byteRecebido == 'a') {
        //Serial.println("Botao Abre a Garra Solto.");
        moverA = 0;
      }

      if (byteRecebido == 'D') {
        //Serial.println("Botao Desce o Braco Apertado.");
        manualB = false;
        moverB = 1;
      }

      if (byteRecebido == 'd') {
        //Serial.println("Botao Desce o Braco Solto.");
        moverB = 0;
      }

      if (byteRecebido == 'S') {
        //Serial.println("Botao Sobe o Braco Apertado.");
        manualB = false;
        moverB = -1;
      }

      if (byteRecebido == 's') {
        //Serial.println("Botao Sobe o Braco Solto.");
        moverB = 0;
      }

      if (byteRecebido == 'G') {
        //Serial.println("Botao Gira Sentido Anti-horário Apertado.");
        manualC = false;
        moverC = 1;
      }

      if (byteRecebido == 'g') {
        //Serial.println("Botao Gira Sentido Anti-horário Braco Solto.");
        moverC = 0;
      }

      if (byteRecebido == 'H') {
        //Serial.println("Botao Gira Sentido Horário Apertado.");
        manualC = false;
        moverC = -1;
      }

      if (byteRecebido == 'h') {
        //Serial.println("Botao Gira Sentido Horário Braco Solto.");
        moverC = 0;
      }

      if (byteRecebido == 'P') {
        //Serial.println("Botao Play Apertado.");
        estadoExec = true;
      }

      if (byteRecebido == 'p') {
        //Serial.println("Botao Play Solto.");
      }

      if (byteRecebido == 'U') {
        //Serial.println("Botao Pause Apertado.");
        estadoExec = false;
      }

      if (byteRecebido == 'u') {
        //Serial.println("Botao Pause Solto.");
      }

      if (byteRecebido == 'N') {
        //Serial.println("Botao Next Apertado.");
        moveNext();
      }

      if (byteRecebido == 'n') {
        //Serial.println("Botao Next Solto.");
      }

      if (byteRecebido == 'X') {
        //Serial.println("Botao Ligado.");
        estadoBraco = true;
      }

      if (byteRecebido == 'x') {
        //Serial.println("Botao Desligado.");
        estadoBraco = false;
      }

      if (byteRecebido == 'R') {
        //Serial.println("Botao Record Apertado.");
        gravaPos();
      }

      if (byteRecebido == 'r') {
        //Serial.println("Botao Record Solto.");
      }
    }
  }

  //Controle do estado de execução
  if ((millis() - delayEstado) > 200) {
    if (digitalRead(pinBotaoA)) {
      estadoBotaoA = !estadoBotaoA;
    }
    if (estadoBotaoA != estadoBotaoAAnt) {
      delayEstado = millis();
      estadoBraco = estadoBotaoA;

      if (estadoBraco) {
        serialBT.write('X');
      } else {
        serialBT.write('x');
      }
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
  if (Serial.available()) {
    serialBT.write(Serial.read());
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