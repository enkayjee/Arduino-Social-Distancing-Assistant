/*
 * includes
 */
#include <Wire.h>

/*
 * Pin addresses
 */
#define ambientTemperatureSensor A1 // using pin A1 to read output from LM35D Temperature Sensor
#define ultrasonicEcho 2 // using pin D2 of Arduino to read Echo value of HC-SR04
#define ultrasonicTrig 3 //using pin D3 of Arduino to write Trig value of HC-SR04
#define buzzerOutput 4 // using pin D4 of Arduino to write to buzzer output
#define D6T_ADDR 0x0A  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02
/*
 * Constants for D6T
 */
#define N_ROW 1
#define N_PIXEL 1
#define N_READ ((N_PIXEL + 1) * 2 + 1)
#define SAMPLE_TIME_0009MS  9
#define SAMPLE_TIME_0010MS  10
#define SAMPLE_TIME_0012MS  12
#define SAMPLE_TIME_0015MS  15
#define SAMPLE_TIME_0020MS  20
#define SAMPLE_TIME_0040MS  40
#define SAMPLE_TIME_0060MS  60
#define SAMPLE_TIME_0100MS  100
#define SAMPLE_TIME_0200MS  200
#define SAMPLE_TIME_0400MS  400
#define SAMPLE_TIME_0800MS  800
#define SAMPLE_TIME_1600MS  1600
#define SAMPLE_TIME_3200MS  3200

#define PARA_0009MS_1 ((uint8_t)0x90)
#define PARA_0009MS_2 ((uint8_t)0xD3)
#define PARA_0009MS_3 ((uint8_t)0x29)
#define PARA_0010MS_1 ((uint8_t)0x90)
#define PARA_0010MS_2 ((uint8_t)0xD4)
#define PARA_0010MS_3 ((uint8_t)0x3C)
#define PARA_0012MS_1 ((uint8_t)0x90)
#define PARA_0012MS_2 ((uint8_t)0xD5)
#define PARA_0012MS_3 ((uint8_t)0x3B)
#define PARA_0015MS_1 ((uint8_t)0x90)
#define PARA_0015MS_2 ((uint8_t)0xD6)
#define PARA_0015MS_3 ((uint8_t)0x32)
#define PARA_0020MS_1 ((uint8_t)0x90)
#define PARA_0020MS_2 ((uint8_t)0xD7)
#define PARA_0020MS_3 ((uint8_t)0x35)
#define PARA_0040MS_1 ((uint8_t)0x90)
#define PARA_0040MS_2 ((uint8_t)0xD8)
#define PARA_0040MS_3 ((uint8_t)0x18)
#define PARA_0060MS_1 ((uint8_t)0x90)
#define PARA_0060MS_2 ((uint8_t)0xD9)
#define PARA_0060MS_3 ((uint8_t)0x1F)
#define PARA_0100MS_1 ((uint8_t)0x90)
#define PARA_0100MS_2 ((uint8_t)0xDA)
#define PARA_0100MS_3 ((uint8_t)0x16)
#define PARA_0200MS_1 ((uint8_t)0x90)
#define PARA_0200MS_2 ((uint8_t)0xDB)
#define PARA_0200MS_3 ((uint8_t)0x11)
#define PARA_0400MS_1 ((uint8_t)0x90)
#define PARA_0400MS_2 ((uint8_t)0xDC)
#define PARA_0400MS_3 ((uint8_t)0x04)
#define PARA_0800MS_1 ((uint8_t)0x90)
#define PARA_0800MS_2 ((uint8_t)0xDD)
#define PARA_0800MS_3 ((uint8_t)0x03)
#define PARA_1600MS_1 ((uint8_t)0x90)
#define PARA_1600MS_2 ((uint8_t)0xDE)
#define PARA_1600MS_3 ((uint8_t)0x0A)
#define PARA_3200MS_1 ((uint8_t)0x90)
#define PARA_3200MS_2 ((uint8_t)0xDF)
#define PARA_3200MS_3 ((uint8_t)0x0D)

/***** Setting Parameters for presence detection*****/
#define comparingNumInc 8 // x tickL ms   (range: 1 to 39)  (example) 16 x 100 ms -> 1.6 sec
#define comparingNumDec 8  // x tickL ms  (range: 1 to 39)  (example) 16 x 100 ms -> 1.6 sec
#define threshHoldInc 10 //  /10 degC   (example) 10 -> 1.0 degC (temperature change > 1.0 degC -> Enable)  
#define threshHoldDec 10 //  /10 degC   (example) 10 -> 1.0 degC (temperature change > 1.0 degC -> Disable)
/*
 * Constants
 */
const boolean debug = false;
const boolean info = true;
const int tickL = SAMPLE_TIME_0100MS;
const int numberOfTicksAfterWhichToUpdateAmbientTemp = 10;
const int numberOfTicksAfterWhichToReadTemperature = 2;

/*
 * Global Variables
 */
int tickCounterAmbientTemperature = 0;
int tickCounterObjectTemperature = 0;
unsigned int tempread = 0;
unsigned int intmp;
float ambientTemperature = 30.0;
uint8_t rbuf[N_READ];
int16_t pix_data = 0;
int16_t seqData[40] = {0};
bool  occuPix = 0;
bool  occuPixFlag = false;
uint8_t  resultOccupancy = 0;
uint16_t  totalCount = 0;

void setup()
{
  pinMode(ambientTemperatureSensor,INPUT); // Configuring pin A1 as input
  pinMode(ultrasonicTrig, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ultrasonicEcho, INPUT); // Sets the echoPin as an INPUT
  pinMode(buzzerOutput, OUTPUT); // Sets buzzer-pin as OUTPUT
  analogReference(INTERNAL);
  digitalWrite(buzzerOutput, HIGH); // Tone OFF
  uint8_t para[3] = {0};
  switch(tickL){
    case SAMPLE_TIME_0009MS:
      para[0] = PARA_0009MS_1;
      para[1] = PARA_0009MS_2;
      para[2] = PARA_0009MS_3;
      break;
    case SAMPLE_TIME_0010MS:
      para[0] = PARA_0010MS_1;
      para[1] = PARA_0010MS_2;
      para[2] = PARA_0010MS_3;
      break;
    case SAMPLE_TIME_0012MS:
      para[0] = PARA_0012MS_1;
      para[1] = PARA_0012MS_2;
      para[2] = PARA_0012MS_3;
      break;
    case SAMPLE_TIME_0015MS:
      para[0] = PARA_0015MS_1;
      para[1] = PARA_0015MS_2;
      para[2] = PARA_0015MS_3;
      break;
    case SAMPLE_TIME_0020MS:
      para[0] = PARA_0020MS_1;
      para[1] = PARA_0020MS_2;
      para[2] = PARA_0020MS_3;
      break;
    case SAMPLE_TIME_0040MS:
      para[0] = PARA_0040MS_1;
      para[1] = PARA_0040MS_2;
      para[2] = PARA_0040MS_3;
      break;
    case SAMPLE_TIME_0060MS:
      para[0] = PARA_0060MS_1;
      para[1] = PARA_0060MS_2;
      para[2] = PARA_0060MS_3;
      break;
    case SAMPLE_TIME_0100MS:
      para[0] = PARA_0100MS_1;
      para[1] = PARA_0100MS_2;
      para[2] = PARA_0100MS_3;
      break;
    case SAMPLE_TIME_0200MS:
      para[0] = PARA_0200MS_1;
      para[1] = PARA_0200MS_2;
      para[2] = PARA_0200MS_3;
      break;
    case SAMPLE_TIME_0400MS:
      para[0] = PARA_0400MS_1;
      para[1] = PARA_0400MS_2;
      para[2] = PARA_0400MS_3;
      break;
    case SAMPLE_TIME_0800MS:
      para[0] = PARA_0800MS_1;
      para[1] = PARA_0800MS_2;
      para[2] = PARA_0800MS_3;
      break;
    case SAMPLE_TIME_1600MS:
      para[0] = PARA_1600MS_1;
      para[1] = PARA_1600MS_2;
      para[2] = PARA_1600MS_3;
      break;
    case SAMPLE_TIME_3200MS:
      para[0] = PARA_3200MS_1;
      para[1] = PARA_3200MS_2;
      para[2] = PARA_3200MS_3;
      break;
    default:
      para[0] = PARA_0100MS_1;
      para[1] = PARA_0100MS_2;
      para[2] = PARA_0100MS_3;
      break;
  }
  if(info){
    Serial.begin(9600);    
  }
  Wire.begin();  // i2c master
  Wire.beginTransmission(D6T_ADDR);  // I2C client address
  Wire.write(0x02);                  // D6T register
  Wire.write(0x00);                  // D6T register
  Wire.write(0x01);                  // D6T register
  Wire.write(0xEE);                  // D6T register
  Wire.endTransmission();            // I2C repeated start for read
  Wire.beginTransmission(D6T_ADDR);  // I2C client address
  Wire.write(0x05);                  // D6T register
  Wire.write(para[0]);                  // D6T register
  Wire.write(para[1]);                  // D6T register
  Wire.write(para[2]);                  // D6T register
  Wire.endTransmission();            // I2C repeated start for read
  Wire.beginTransmission(D6T_ADDR);  // I2C client address
  Wire.write(0x03);                  // D6T register
  Wire.write(0x00);                  // D6T register
  Wire.write(0x03);                  // D6T register
  Wire.write(0x8B);                  // D6T register
  Wire.endTransmission();            // I2C repeated start for read
  Wire.beginTransmission(D6T_ADDR);  // I2C client address
  Wire.write(0x03);                  // D6T register
  Wire.write(0x00);                  // D6T register
  Wire.write(0x07);                  // D6T register
  Wire.write(0x97);                  // D6T register
  Wire.endTransmission();            // I2C repeated start for read
  Wire.beginTransmission(D6T_ADDR);  // I2C client address
  Wire.write(0x02);                  // D6T register
  Wire.write(0x00);                  // D6T register
  Wire.write(0x00);                  // D6T register
  Wire.write(0xE9);                  // D6T register
  Wire.endTransmission();            // I2C repeated start for read 
}
void calculateAmbientTemperature(){
  if(tickCounterAmbientTemperature == 0){
    // first sample seems to fluctuate a lot. Disregard it
    intmp=analogRead(ambientTemperatureSensor);
    if(info && debug){
      Serial.print("Discarding First reading: ");
      Serial.println(intmp); 
    }
    tickCounterAmbientTemperature++;
  }
  else if (tickCounterAmbientTemperature < 4) {
    intmp=analogRead(ambientTemperatureSensor),
    tempread = tempread + intmp;  // read  more samples. better stability.
    if( info && debug){
      Serial.print("Partial reading: ");
      Serial.println(intmp);
    }
    tickCounterAmbientTemperature++;
  }
  else if (tickCounterAmbientTemperature == 4) {
      tempread = tempread>>2;
      // default voltage is 5.0,  analogReference(INTERNAL) sets it to 1.1.
      ambientTemperature = 110 * (float)tempread / 1024; // analogReference(INTERNAL); needed
      if(info && debug){
        Serial.print("Temp is: ");
      Serial.println(ambientTemperature);
      }
        tempread = 0; // reset tempread
        tickCounterAmbientTemperature++;
  }
  else if (tickCounterAmbientTemperature == numberOfTicksAfterWhichToUpdateAmbientTemp -1 ) {
    tickCounterAmbientTemperature = 0; // reset tickCounter
  }
  else {
    tickCounterAmbientTemperature++;
  }
}
long calculateEchoDuration(){
  int i = 0;
  long durationSum = 0;
  long duration;
  while(i<10){
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse: 
    digitalWrite(ultrasonicTrig, LOW); 
    delayMicroseconds(2); 
    digitalWrite(ultrasonicTrig, HIGH); 
    delayMicroseconds(5); 
    digitalWrite(ultrasonicTrig, LOW); 
    durationSum += pulseIn(ultrasonicEcho, HIGH);
    i++;
  }
  duration = durationSum / 10;
  return duration;
}
long calculateDistance(long microseconds, long temp) { 
  return (microseconds * ((331.3 + 0.606 * temp)/10000)) / 2; //Multiplying the speed of sound through a certain temperature of air by the //length of time it takes to reach the object and back, divided by two }
}
/** JUDGE_occupancy: judge occupancy*/
void judge_seatOccupancy(void) { 
  int j = 0; 
  for (j = 0; j < 39; j++){
    seqData[39 - j] = seqData[38 - j];
  }
  seqData[0] = pix_data;            
  if (totalCount <= comparingNumInc){
    totalCount++;
  }
  if (totalCount > comparingNumInc){    
    if (occuPix == false){
      if ((int16_t)(seqData[0] - seqData[comparingNumInc]) >= (int16_t)threshHoldInc){
        occuPix = true;
      }
    }
    else{   //resultOccupancy == true
      if ((int16_t)(seqData[comparingNumDec] - seqData[0]) >= (int16_t)threshHoldDec){
        occuPix = false;
      }
    }
    if (resultOccupancy == 0) {                
        if(occuPix == true){
          resultOccupancy = 1;
        }
    }
    else{
      occuPixFlag = false;
      if (occuPix == true){
        occuPixFlag = true;
      }
      if (occuPixFlag == false){
        resultOccupancy = 0;
      }
    }
  }
}
void loop()
{
  int i, j;
  calculateAmbientTemperature();
  memset(rbuf, 0, N_READ);
  // Wire buffers are enough to read D6T-16L data (33bytes) with
  // MKR-WiFi1010 and Feather ESP32,
  // these have 256 and 128 buffers in their libraries.
  Wire.beginTransmission(D6T_ADDR);  // I2C client address
  Wire.write(D6T_CMD);               // D6T register
  Wire.endTransmission();            // I2C repeated start for read
  Wire.requestFrom(D6T_ADDR, N_READ);
  i = 0;
  while (Wire.available()) {
      rbuf[i++] = Wire.read();
  }

  if (D6T_checkPEC(rbuf, N_READ - 1)) {
      return;
  }
  // 1st data is PTAT measurement (: Proportional To Absolute Temperature)
  int16_t itemp = conv8us_s16_le(rbuf, 0);
  if(tickCounterObjectTemperature < numberOfTicksAfterWhichToReadTemperature){
    if(info && debug){
      Serial.print("Discarded:");
      Serial.print(" PTAT:");
      Serial.print(itemp / 10.0, 1);
      Serial.print(", Temperature:"); 
    }
    else if(info) {
      Serial.println("Initializing....");
    }
  
    // loop temperature pixels of each thrmopiles measurements
    for (i = 0, j = 2; i < N_PIXEL; i++, j += 2) {
        itemp = conv8us_s16_le(rbuf, j);
        if(info && debug){
          Serial.print(itemp / 10.0, 1);  // print PTAT & Temperature
          if ((i % N_ROW) == N_ROW - 1) {
              Serial.println(" [degC]");  // wrap text at ROW end.
          } else {
              Serial.print(", ");   // print delimiter
          } 
        }
    }
    tickCounterObjectTemperature++;
    delay(tickL); 
  }
  else{
    long duration, cm;
    duration = calculateEchoDuration();
    cm = calculateDistance(duration, ambientTemperature);
    if(info){
      Serial.print("Ambient Temperature: ");
      Serial.print(ambientTemperature);
      Serial.print(", Distance: ");
      Serial.print(cm);
      Serial.print(", PTAT:");
      Serial.print(itemp / 10.0, 1);
      Serial.print(", Temperature:");
    }
    // loop temperature pixels of each thrmopiles measurements
    for (i = 0, j = 2; i < N_PIXEL; i++, j += 2) {
        itemp = conv8us_s16_le(rbuf, j);
        pix_data = itemp;
        if(info){
          Serial.print(itemp / 10.0, 1);  // print PTAT & Temperature
          if ((i % N_ROW) == N_ROW - 1) {
              Serial.print(" [degC]");  // wrap text at ROW end.
          } else {
              Serial.print(", ");   // print delimiter
          }
        }
    }
    judge_seatOccupancy();
    if(info){
      Serial.print(", Occupancy:");
      Serial.println(resultOccupancy, 1);
    }
    if(cm < 182 && resultOccupancy == 1){
      digitalWrite(buzzerOutput, LOW); // Tone ON
      delay(tickL-70);
      digitalWrite(buzzerOutput, HIGH); // Tone OFF
     }
     else{
      delay(tickL-70); // 70 ms taken by the echoDuration function   
     }
  }
}
uint8_t calc_crc(uint8_t data) {
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

/** <!-- D6T_checkPEC {{{ 1--> D6T PEC(Packet Error Check) calculation.
 * calculate the data sequence,
 * from an I2C Read client address (8bit) to thermal data end.
 */
bool D6T_checkPEC(uint8_t buf[], int n) {
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1);  // I2C Read address (8bit)
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret) {
      if(info){
        Serial.print("PEC check failed:");
        Serial.print(crc, HEX);
        Serial.print("(cal) vs ");
        Serial.print(buf[n], HEX);
        Serial.println("(get)"); 
      }
    }
    return ret;
}


/** <!-- conv8us_s16_le {{{1 --> convert a 16bit data from the byte stream.
 */
int16_t conv8us_s16_le(uint8_t* buf, int n) {
    int ret;
    ret = buf[n];
    ret += buf[n + 1] << 8;
    return (int16_t)ret;   // and convert negative.
}
