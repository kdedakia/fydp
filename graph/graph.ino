int sensorPin = A0;
int potPin = A1;
int potPin2 = A2;
int sensorVal = 0;
int potVal = 0;
int potVal2 = 0;
int potMax = 663;
int potMax2 = 659;

double kp = 0;
double kp_min = 0;
double kp_max = 10;

double ki = 0;
double ki_min = 5;
double ki_max = 20;
bool graph = true;

// Pot 1 = kp
void set_kp() {
  double m = (kp_max - kp_min) / potMax;
  kp = m * potVal + kp_min;
}

// Pot 2 = ki
void set_ki() {
  double m = (ki_max - ki_min) / potMax2;
  ki = m * potVal2 + ki_min;
}

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(potPin2, INPUT);
  Serial.begin(9600);
}

void loop() {
  sensorVal = analogRead(sensorPin);  
  potVal = analogRead(potPin);
  potVal2 = analogRead(potPin2);
  
  set_kp();
  set_ki();
  
  Serial.println("KP: " + (String)kp);
  Serial.println("KI: " + (String)ki);
  
  if (graph) {
    Serial.print("!"); 
    Serial.print(potVal);
    Serial.print(" ");
    Serial.print(potVal2);
    Serial.print("\n");
  }

  delay(50);
}
