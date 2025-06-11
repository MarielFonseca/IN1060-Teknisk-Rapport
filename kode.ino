//Laster inn biblioteker som koden bruker til å kommunisere med sensoren
#include <Wire.h> // For I2C-kommunikasjon mellom Arduinoen og MPU6050
#include <Adafruit_MPU6050.h> // Adafruit-bibliotek for å bruke MPU6050 enklere
#include <Adafruit_Sensor.h>  // Standard sensorbibliotek som gjør databehandling enklere
#define n 50 //Lagrer 50 målinger, brukes for å beregne standardavvik




Adafruit_MPU6050 mpu; //oppretter MPU6050 objekt
const int buzzer = 8; //Buzzer må koble opp til pin 8
const int paLED = 10; //LED til når artefakten er på, pin 10
const int knappPaa = 2; //Knapp til av og på, pin 2
const int alarmLED = 12; //LEDen til aktivert alarm, pin 12
const int alarmLEDTo = 13; //LEDen til aktivert alarm, pin 13
const int alarmLEDTre = 11; //LEDen til aktivert alarm, pin 11
const int knappNull = 4; //Knapp til nullstilling av alarm, pin 4


bool systemPaa = false; //Holder styr på om systemet er aktivt
bool alarmAktiv = false; //Holder styr på om alarmen er utløst


//Variabler for startpunktet til akselerasjon
float nedx = 0; //start-vektor når man resetter
float nedy = 0;
float nedz = 0;
sensors_event_t a, g, temp; //Struktur for akselerasjon, gyroskop og temperatur


int i = 0; //Teller opp helt til den går til n og tilbake igjen, indeks for akselerasjons arrayene


//Arrayer for å lagre akselerasjon over tid i arrayer
float accelx[n] = {}; //Oppretter arrayer
float accely[n] = {};
float accelz[n] = {};


int state = 0; //Brukes for status av fase under mulig fall


long tid = 0; //Tidsmåler for når fall starter
long tid2 = 0; //Tidsmåler for lengden på stillstand


void setup() {
Serial.begin(115200); //starter kommunikasjon for debugging, og angir hvor raskt kommunikasjonen skjer
pinMode(buzzer, OUTPUT); //sett buzzer som output
pinMode(paLED, OUTPUT); //sett LED som output
pinMode(alarmLED, OUTPUT); //sett LED som output
pinMode(alarmLEDTo, OUTPUT); //sett LED som output
pinMode(alarmLEDTre, OUTPUT); //sett LED som output
pinMode(knappPaa, INPUT_PULLUP); //knappen bruker input pullup, aktiv LOW
pinMode(knappNull, INPUT_PULLUP); //knappen bruker input pullup, aktiv LOW


//Skjekker om MPU finnes og er koblet til. Om den ikke finnes vises det en feilmelding og koden stopper i en evig løkke
if(!mpu.begin()){
  Serial.println("MPU6050 ikke funnet!"); //Feilmelding
  while (1); //Evig løkke
}




//Setter opp sensorens følsomhet og filtrering
mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Maks måling på akselerometeret: ±8g
mpu.setGyroRange(MPU6050_RANGE_500_DEG);  // Maks rotasjon på gyroskopet: ±500 grader/sek
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Filter for å redusere støy, stabelisere den innhentede dataen
}


void loop() {
 sjekkPaaKnapp();
 oppdaterData();


 if(!systemPaa){
   deaktiverAlarm();
   return;
 }


 fallSjekk();


 if(alarmAktiv) {
   aktiverAlarm();
   sjekkDeaktivering();
 }
}


void skruAv_Pa(){
 systemPaa = !systemPaa; //Bytter mellom av og på basert på tidligere tilstand
 alarmAktiv = false; //Alarmen er ikke aktiv
 digitalWrite(paLED, systemPaa ? HIGH : LOW); //Endrer lyset basert på om systemet er på eller av


 //Lagrer den nåværende akselerasjonen som nullpunkt for alle aksene
 nedx = a.acceleration.x;
 nedy = a.acceleration.y;
 nedz = a.acceleration.z;


 //Loope over, og sette alle til null
 for (int k = 0; k < n; k++){
   accelx[k] = 0;
   accely[k] = 0;
   accelz[k] = 0;
 }


 state = 0; //Nullstiller fallstatusen
 delay(500); //Debounce for å unngå dobbelttrykk
}


void sjekkPaaKnapp(){
 if (digitalRead(knappPaa) == LOW) { //Sjekker om knappen blir aktivert, skur av/på systemet
   skruAv_Pa();
 }
}


void oppdaterData(){
 mpu.getEvent(&a, &g, &temp); //Leser data fra sensoren og lagrer dem
 i = (i+1)%n; //Gjør at den kan loope tilbake til 0 når n er nådd, roterende indeks
//Lagrer oppdatert akselerasjon, trekker fra nullpunkt verdiene (vi er bare interessert i endring)
 accelx[i] = a.acceleration.x - nedx;
 accely[i] = a.acceleration.y - nedy;
 accelz[i] = a.acceleration.z - nedz;
}


void deaktiverAlarm(){ //Skrur av alarm
 alarmAktiv = false;
 digitalWrite(alarmLED, LOW);
 digitalWrite(alarmLEDTo, LOW);
 digitalWrite(alarmLEDTre, LOW);
 noTone(buzzer);
}


void aktiverAlarm(){ //Skrur på alarm
 digitalWrite(alarmLED, HIGH);
 digitalWrite(alarmLEDTo, HIGH);
 digitalWrite(alarmLEDTre, HIGH);
 tone(buzzer,1000);
}


void sjekkDeaktivering(){ //Sjekker om knappen blir aktivert, skur av alarm
 if(digitalRead(knappNull) == LOW) {
   deaktiverAlarm();
 }
}




// Sjekker om fall forekommer
void fallSjekk(){
float accel[n] = {}; //En midlertidig array for akselerasjon på z-aksen
memcpy(accel, accelz, sizeof accelz); //Kopierer data fra de globale arrayene


Serial.println(accel[i]); //Skriver ut nåværende verdien for feilsøking


//Fase 1: Registrerer rask negativ akselerasjon fra z-aksen, starter et fall
if (accel[i] < -5 && state == 0) {
   tid = millis();
   state = 1;
}


//Hvis det går mer enn tre sekunder før neste fase, nullstilles fallet
if (millis() - tid > 3000 && state == 1){
 state = 0; //Setter status til null
}
//Fase 2: Registerer positiv akselerasjon i z-retning etter fall, når bakken treffes
if (accel[i] > 5 && state == 1) {
  state = 2;
  tid2 = millis();
}


//Dersom personen har ligget stille i 3 sekunder aktiveres alarmen
if (millis()-tid2 > 3000 && state == 2) { // hvis de har ligget stille i 3 sekunder
  state = 0;
}


//Fase 3: Ligger personen stille, beregnes standardavvik i akselerasjon
if (state == 2){
  // regn ut standardavvik
  float sum = 0;
  for (int k = 0; k < n; k++){
    sum += accel[k];
  }
  float mean = sum / n; //Gjennomsnittlig akselerasjon




  float errsq = 0;
  for (int k = 0; k < n; k++){
    float err = (accel[k]-mean);
    errsq += err*err;
  }
  float avgErrSq = errsq/n; //Beregner standardavviket


  float std = sqrt(avgErrSq); //Standardavvik


   //Er standardavviket lavt, under 1 (personen ligger stille), aktiveres alarmen
  if (std < 1.0) {
    startAlarm();
    state = 3;
  }
}
}


//Funksjon som kjører alarmen
void startAlarm(){
while (systemPaa){
  aktiverAlarm();


  //Trykkes nullstillingsknappen slås alarmen av
  if (digitalRead(knappNull) == LOW || digitalRead(knappPaa) == LOW){
   systemPaa = false;
   deaktiverAlarm();
  }
 }
}
