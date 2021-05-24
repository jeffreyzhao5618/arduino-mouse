int vrx = A0;
int vry = A1;
int sw = 7;

void setup() {
  pinMode(sw, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  int vrx_val = analogRead(vrx);
  int vry_val = analogRead(vry);
  int sw_val = digitalRead(sw);
  String instr = (String) vrx_val + ',' + (String) vry_val + ',' + (String) sw_val + '\n';
  Serial.print(instr);
  delay(20);
}
