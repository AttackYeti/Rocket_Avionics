
#define LOW_PRESSURE_LOX A1
#define LOW_PRESSURE_PROP A2

#define HIGH_PRESSURE A6

void setup() {
  // put your setup code here, to run once:

  pinMode(LOW_PRESSURE_LOX, INPUT);
  pinMode(LOW_PRESSURE_PROP, INPUT);
  pinMode(HIGH_PRESSURE, INPUT);

  Serial.begin(9600);
}

// 0.88V - 4.4V : ?? - 5000 PSI

int lowpressurelox, lowpressureprop, highpressure;
float converted_lox_low, converted_prop_low, converted_high;

void loop() {
  // put your main code here, to run repeatedly:
  lowpressurelox = analogRead(LOW_PRESSURE_LOX);
  lowpressureprop = analogRead(LOW_PRESSURE_PROP);
  highpressure = analogRead(HIGH_PRESSURE);

  converted_lox_low = lowPressureConversion(lowpressurelox);
  converted_prop_low = lowPressureConversion(lowpressureprop);

  converted_high = highPressureConversion(highpressure);
  
  char buffer[20];
  sprintf(buffer, "%d,%d,%d", converted_lox_low, converted_prop_low, converted_high);
  Serial.println(buffer);
}

float lowPressureConversion(int raw){
  return 1.2258857538273733*raw + -123.89876445934394;
}

float highPressureConversion(int raw){
  return 6.612739309669555*raw - 1237.7612969223858;
}
