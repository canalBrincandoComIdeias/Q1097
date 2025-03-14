/****************************************************************************************
  Video Q1097 - Tutorial Maker - Servo Motor
  
  Desenvolvido pela Fábrica de Programas - Brincando com Ideias (www.brincandocomideias.com)
  www.youtube.com/c/BrincandoComIdeias

  Autor Flavio Guimaraes
*****************************************************************************************/
#include <Servo.h>

#define pinPotA    A1 //Potenciometro Esquerdo
#define pinPotB    A2 //Potenciometro Central
#define pinPotC    A3 //Potenciometro Direito
#define pinServoA  2  //Servo da Garra
#define pinServoB  3  //Servo do Braço
#define pinServoC  4  //Servo da Base
#define pinBotaoA  5  //Botão Liga/Desliga
#define pinBotaoB  6  //Pushbutton Esquerda
#define pinBotaoC  7  //Pushbutton Direita
#define pinLEDVm   8  //Led Vermelho
#define pinLEDVd   9  //Led Verde

//Objetos para os servos
Servo servoA;
Servo servoB;
Servo servoC;

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

  //Teste dos Pushbuttons e LEDs
  digitalWrite(pinLEDVm, digitalRead(pinBotaoB));
  digitalWrite(pinLEDVd, digitalRead(pinBotaoC));

  //Teste do Botao Liga/Desliga, Potenciômetros e Servos
  if (digitalRead(pinBotaoA)) {   //Se ligado, deixa os servos em 90 graus (para ajuste das engrenagens)
    servoA.write(90); 
    servoB.write(90); 
    servoC.write(90); 
  } else {
    int val = analogRead(pinPotA);       // leitura do potenciometro
    val = map(val, 0, 1023, 0, 180);     // converte o valor de faixa 0-1023 para a feixa dos angulos 0-180
    servoA.write(val);

    val = analogRead(pinPotB);          
    val = map(val, 0, 1023, 0, 180);    
    servoB.write(val);

    val = analogRead(pinPotC);          
    val = map(val, 0, 1023, 0, 180);    
    servoC.write(val);
  }
  delay(15);
}
