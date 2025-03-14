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

//Objetos para os Servos no ESP32  
MoveServoESP32 servoA;
MoveServoESP32 servoB;
MoveServoESP32 servoC; 

void setup() {
  pinMode(pinBotaoA, INPUT);
  pinMode(pinBotaoB, INPUT);
  pinMode(pinBotaoC, INPUT);
  pinMode(pinLEDVm, OUTPUT);
  pinMode(pinLEDVd, OUTPUT);

  servoA.attach(pinServoA);
  servoB.attach(pinServoB);
  servoC.attach(pinServoC);
}

void loop() {
  servoA.ServoLoop();
  servoB.ServoLoop();
  servoC.ServoLoop();

  //Teste dos Pushbuttons e LEDs
  digitalWrite(pinLEDVm, digitalRead(pinBotaoB));
  digitalWrite(pinLEDVd, digitalRead(pinBotaoC));

  //Teste do Botao Liga/Desliga, Potenciômetros e Servos
  if (digitalRead(pinBotaoA)) {  //Se ligado, deixa os servos em 90 graus (para ajuste das engrenagens)
    servoA.move(90);
    servoB.move(90);
    servoC.move(90);
  } else {
    int val = analogRead(pinPotA);    // leitura do potenciometro
    val = map(val, 0, 4095, 0, 180);  // converte o valor de faixa 0-1023 para a feixa dos angulos 0-180
    servoA.move(val);

    val = analogRead(pinPotB);
    val = map(val, 0, 4095, 0, 180);
    servoB.move(val);

    val = analogRead(pinPotC);
    val = map(val, 0, 4095, 0, 180);
    servoC.move(val);
  }
  delay(15);
}
