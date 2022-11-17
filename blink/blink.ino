
#define calentador 33

#define LED_BUILTIN 2

void setup() {
pinMode(LED_BUILTIN, OUTPUT);       // Inicializar el LED como salida
  pinMode(calentador, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(calentador, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);
  digitalWrite(calentador, LOW);
  digitalWrite(LED_BUILTIN, LOW);
   delay(500);

}
