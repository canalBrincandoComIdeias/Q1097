//Classe para Personalizar a Biblioteca Servo e Permitir Movimentos com Velocidade Configurável
class MoveServo : public Servo {
private:
  int angulo;               // Propriedade privada para armazenar o ângulo desejado
  int anguloAtual;          // Propriedade privada para armazenar o ângulo atual
  int valVeloc;             // Propriedade privada para armazenar a velocidade dos movimentos (0= 1000ms de pausa entre angulos, 100= 0ms de pausa)
  int pausaEntreMov;        // Propriedade privada para armazenar a pausa entre os angulos (em milissegundos)
  unsigned long delayMove;  //Propriedade privada para armazenar o ultimo momento que o movimento foi executado
  bool parked;

public:
  // Construtor padrão
  MoveServo() {
    angulo = 90;
    anguloAtual = 90;
    parked = false;
  }

  uint8_t attach(int pin, int veloc = 100) {
    uint8_t valRet = Servo::attach(pin);
    write(90);
    velocidade(veloc);
    return valRet;
  }

  void write(int value) {
    Servo::write(value);
    angulo = constrain(value, 0, 180);
  }

  // Método para registrar o angulo de destino (de 0 (zero) até 180)
  void move(int novoAngulo) {
    if (!parked) {
      angulo = constrain(novoAngulo, 0, 180);
    }
  }

  // Método para registrar a velocidade dos movimentos (de 0 (zero) até 100)
  void velocidade(int setVelocidade) {
    valVeloc = constrain(setVelocidade, 0, 100);
    pausaEntreMov = map(valVeloc, 0, 100, 100, 0);
  }

  void park() {
    angulo = 90;
    parked = true;
  }

  // Getter para o ângulo atual
  int getAnguloAtual() {
    return anguloAtual;
  }

  // Getter para o ângulo destino
  int getAngulo() {
    return angulo;
  }

  // Método para ser chamado dentro do loop ou por um timer (responsável por executar os movimentos)
  void ServoLoop() {
    if ((millis() - delayMove) > pausaEntreMov) {
      delayMove = millis();
      if (angulo < anguloAtual) {
        anguloAtual--;
        Servo::write(anguloAtual);
      }

      if (angulo > anguloAtual) {
        anguloAtual++;
        Servo::write(anguloAtual);
      }
    }
  }
};