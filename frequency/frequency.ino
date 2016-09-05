// the setup function runs once when you press reset or power the board
#define LED 2
int frequency = 1; //in Hz
float period = (1.0/frequency)*1000; //Period in ms
float half_period = period/2; 
void setup() {
  pinMode(LED, OUTPUT);
}
void loop() {
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(half_period);              // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(half_period);              // wait for a second
}
