unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    Serial.println(" En sekund har gått");
    Serial.print("ndnfn")
  }
  // put your main code here, to run repeatedly:

}
