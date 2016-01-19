
int sensorVal = 0;
int sensorPin = A0;
int x = 0;

void setup() {
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  sensorVal = analogRead(sensorPin);  
  
  Serial.print(x);
  Serial.print(" ");
  Serial.print(sensorVal);
  Serial.print("\n");

  x += 1;
  delay(50);
}
