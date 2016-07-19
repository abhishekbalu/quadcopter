// the setup function runs once when you press reset or power the board
#define LED 2
int frequency = 10; //in Hz
void setup() {
  pinMode(LED, OUTPUT);
}
void loop() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000 * (1/frequency));              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000 * (1/frequency));              // wait for a second
}
