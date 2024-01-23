
// Code for Z_unknown= 90 ohms.
// This version is without commented statements of V6
// V6 with single frequency sweep and calibration fucntion (6_19_22)
// Changed Rout to Z_cal and have updated Z_cal as Z_cal=200+Rout (6_20_22)
// Settling time changed from 0x00,0x64 (100) to 0x01,0xFF (511) (7_5_22)
// Settling time changed from 0x01,0xff (511) to 0x07,0xFF (511*4) (7_6_22)

// Changed the MCKL from 16.67M to 16.766M and then start freq and freq increment values accordingly. (6_23_22)
// Start Freq 80K, Increment 1K and No of Increments 10 i.e. 80-90K
// PGA= X5, Vout=2 P-P
// Fixed CalGain
//===================================
//
#include <Wire.h>
#include <ADXL362.h>
#include <SPI.h>
#include "Firebase_Arduino_WiFiNINA.h"
#include <FirebaseJson.h>

//===================================
// variables
//int real_imp = 0; //This does not need to be a global variable
//int img_imp = 0; //This does not need to be a global variable
//double Imp_Mag = 0.0; //This does not need to be a global variable
//double Phase = 0.0; //This does not need to be a global variable
//double Z_Mag = 0.0; //This does not need to be a global variable
double gainFactor = 0.0;
double Rout = 200.0;
double Z_cal = 1000 + Rout; //1470
double pi = 3.1415926535897932384626433832795;
int N = 0;
int avg_amt = 50; //Number of points to average. Added 6/30/22
double system_phase = 0.0; //System phase calculated at calibration. Added 6/30/22
double Z_fit = 0.0;
double Z = 0.0;

//====================================
const byte A = 1;
//const byte B = 7;
//====================================
// Registers
const PROGMEM byte cont_reg_1 = 0x80;
const PROGMEM byte cont_reg_2 = 0x81;


const PROGMEM byte start_freq_1 = 0x82;
const PROGMEM byte start_freq_2 = 0x83;
const PROGMEM byte start_freq_3 = 0x84;

const PROGMEM byte freq_incre_1 = 0x85;
const PROGMEM byte freq_incre_2 = 0x86;
const PROGMEM byte freq_incre_3 = 0x87;

const PROGMEM byte no_incre_1 = 0x88;
const PROGMEM byte no_incre_2 = 0x89;

const PROGMEM byte settling_time_1 = 0x8A;
const PROGMEM byte settling_time_2 = 0x8B;

const PROGMEM byte status_reg = 0x8F;

const PROGMEM byte real_reg_1 = 0x94;
const PROGMEM byte real_reg_2 = 0x95;
const PROGMEM byte img_reg_1 = 0x96;
const PROGMEM byte img_reg_2 = 0x97;




// Parameters
//const PROGMEM byte increment=0b1100100;
const PROGMEM byte freq_increment1 = 0x00; // (f_i/[16.776e6e6/4])2^27=32002.3195 where f_i=1000Hz
const PROGMEM byte freq_increment2 = 0x7D; // (10/[16.776e6/4])2^27=32002.3195
const PROGMEM byte freq_increment3 = 0x02; // (10/[16.776/4])2^27=32002.3195

const PROGMEM byte start_freqVal_1 = 0x18; // (f/[16.776e6/4])2^27=2560185.56 where f=80KHz 27
const PROGMEM byte start_freqVal_2 = 0x6A; // (f/[16.776e6/4])2^27=2560185.56 where f=80KHz 10
const PROGMEM byte start_freqVal_3 = 0x74; // (f/[16.776e6/4])2^27=2560185.56 where f=80KHz B9

const PROGMEM byte no_increVal_1 = 0x00; // check what does 0x01 represent it should be 00000001
const PROGMEM byte no_increVal_2 = 0x01; //10(0x00,0x0A) (0x01,0xF4 for 500)

const PROGMEM byte Vout = 0x00;
const PROGMEM byte PGA = 0x01;


// Modes
const PROGMEM byte standby = 0xB0 | Vout | PGA;
const PROGMEM byte initialize_mode = 0x10 | Vout | PGA;
const PROGMEM byte startFreq_mode = 0x20 | Vout | PGA;
const PROGMEM byte incFreq_mode = 0x30 | Vout | PGA;
const PROGMEM byte powerDown_mode = 0xA0 | Vout | PGA;
const PROGMEM byte repeat_mode = 0x40 | Vout | PGA;
const PROGMEM byte repeat = 0x40 | Vout | PGA;

const PROGMEM byte reset = 0x10;

// ADXL 362//
ADXL362 xl;
int16_t temp;
int16_t interruptPin = 4;          //Setup ADXL362 interrupt output to Interrupt 0 (digital pin 2)
int16_t interruptStatus = 0;
int test = 0;
int16_t XValue, YValue, ZValue, Temperature; //// MAX 30105
String motion_status = "";

//**Firebase***//
#define FIREBASE_HOST "temptest-ed84e-default-rtdb.firebaseio.com"//"zeebee-ba484-default-rtdb.firebaseio.com"//"hf-wearable-device-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "cjO9NSsVqFyxftHsfEmZb0rDTEIGfJwjRU2HtbSd"//"hTUhYuWzdSeYVx4ozNeog16vqCoHPUkzPggWpp7j"//"8D7uQf5zkbyWJhkkTyFWLGrfqtASJhiH3OeW5aBb"////
#define WIFI_SSID "fau"
#define WIFI_PASSWORD ""
FirebaseData db;
String Count;
int count = 0;
String path = "/Test_test";
//**MAX30105**//
#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
int beatAvg_1=0;
//SPO02//
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[200]; //infrared LED sensor data
uint16_t redBuffer[200];  //red LED sensor data
#else
uint32_t irBuffer[200]; //infrared LED sensor data
uint32_t redBuffer[200];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int32_t spo;
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

void setup() {

  digitalWrite(A, HIGH);
  Wire.begin();
  Serial.begin(9600);
  //while (!Serial);
  Registers();
  //  ChoosePath(1);

  // Calibration();
  gainFactor = 0.00000008614377993206; //0.00000008824502305657;
  //ADXL//
  xl.begin(7);                   // Setup SPI protocol, issue device soft reset
  xl.beginMeasure();              // Switch ADXL362 to measure mode
  xl.setupDCActivityInterrupt(300, 10);   // 300 code activity threshold.  With default ODR = 100Hz, time threshold of 10 results in 0.1 second time threshold
  xl.setupDCInactivityInterrupt(80, 200);   // 80 code inactivity threshold.  With default ODR = 100Hz, time threshold of 30 results in 2 second time threshold
  Serial.println();
  xl.SPIwriteOneRegister(0x2A, 0x40);
  // Setup Activity/Inactivity register
  xl.SPIwriteOneRegister(0x27, 0x3F); // Referenced Activity, Referenced Inactivity, Loop Mode
  // turn on Autosleep bit
  byte POWER_CTL_reg = xl.SPIreadOneRegister(0x2D);
  POWER_CTL_reg = POWER_CTL_reg | (0x04);       // turn on POWER_CTL[2] - Autosleep bit
  xl.SPIwriteOneRegister(0x2D, POWER_CTL_reg);
  xl.beginMeasure();                      // DO LAST! enable measurement mode
  xl.checkAllControlRegs();               // check some setup conditions
  delay(100);
  pinMode(4, INPUT);
  attachInterrupt(0, interruptFunction, RISING);  // A high on output of ADXL interrupt means ADXL is awake, and wake up Arduino


  //MAX30105
  Serial.println("Initializing...");
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  Serial.print("test");
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  //**Firebase**//
  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);
  String jsonStr;
  delay(50);
}

void loop() {


  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);
  Count = "/" + String(count);


  FreqSweep();
  if (Z_fit > 70.0 & Z_fit < 11210)
  {
    Z = Z_fit;
    if (Firebase.setFloat(db, path + Count + "/TI", Z))
    {
      Serial.println("TI Connected");
    }
    else
    {
      Serial.println("TI not connected");
    }
  }
  else
  {
    if (Firebase.setFloat(db, path + Count + "/TI", Z))
    {
      Serial.println("AD5933 Connected");
    }
    else
    {
      Serial.println("AD5933 not connected");
    }
  }
  interruptStatus = digitalRead(interruptPin);
  if (interruptStatus == 0) {
    motion_status = "Inactive";
    Serial.print("\nADXL went to sleep - Put Arduino to sleep now \n");
    // myFile.println("\nADXL went to sleep - Put Arduino to sleep now \n");
    if (Firebase.setString(db, path + Count + "/Activity Status", "Not Active"))
    {
      Serial.println("ADXL Connected");
      // Serial.println(Time);
    }
    else
    {
      Serial.println("ADXL not connected");
    }
    // digitalWrite(7, LOW);    // Turn off LED as visual indicator of asleep
    delay(100);
  }

  // if ADXL362 is awake, report XYZT data to Serial Monitor
  else {

    delay(10);
   // digitalWrite(7, HIGH);    // Turn on LED as visual indicator of awake
    //xl.readXYZTData(XValue, YValue, ZValue, Temperature);
    // myFile.println("\nADXL is awake");
    Serial.println("awake");
    motion_status = "Active";
    if (Firebase.setString(db, path + Count + "/Activity Status", "Active"))
    {
      Serial.println("ADXL Active Connected");
    }
    else
    {
      Serial.println("ADXL Active not connected");
    }
  }
  //delay(20);
  hr();
  if (beatAvg>59 & beatAvg<121)
  {
    beatAvg_1=beatAvg;
  if (Firebase.setFloat(db, path + Count + "/HR", beatAvg_1 ))
  {
    Serial.println("HR CONNECTED");
    // Serial.println(Time);
  }
  else
  {
    Serial.println("HR NOT CONNECTED");
  }
  }
  else
{
    if (Firebase.setFloat(db, path + Count + "/HR", beatAvg_1))
  {
    Serial.println("HR CONNECTED");
    // Serial.println(Time);
  }
  else
  {
    Serial.println("HR NOT CONNECTED");
  }
  }
  delay(2000);
  sp();
  if (validSPO2==1 && spo2>94)
  {
    spo=spo2;
  if (Firebase.setFloat(db, path + Count + "/SPO2", spo))
  {
    Serial.println("ADXL Connected");
    // Serial.println(Time);
  }
  else
  {
    Serial.println("ADXL not connected");
  }
  }
  else
  {
      if (Firebase.setFloat(db, path + Count + "/SPO2", spo))
  {
    Serial.println("SPO2 Connected");
    // Serial.println(Time);
  }
  else
  {
    Serial.println("SPO2 not connected");
  }
    }
  delay (30000);
  count++;


}


void Registers()
{
  TransByte(cont_reg_2, reset);
  //check if this is the right way to code startfrequency
  TransByte(start_freq_1, start_freqVal_1);
  TransByte(start_freq_2, start_freqVal_2);
  TransByte(start_freq_3, start_freqVal_3);

  TransByte(no_incre_1, no_increVal_1);
  TransByte(no_incre_2, no_increVal_2);


  TransByte(freq_incre_1, freq_increment1);
  TransByte(freq_incre_2, freq_increment2);
  TransByte(freq_incre_3, freq_increment3);

  TransByte(settling_time_1, 0x07); 
  TransByte(settling_time_2, 0xFF); //100

  TransByte(cont_reg_1, standby);
  Serial.println("Registers Set");
}

void Calibration()
{
  //ChoosePath(2);

  double Imp_avg = 0.0;
  double Phs_avg = 0.0;
  //double CalImp_Mag = 0.0;

  TransByte(cont_reg_1, initialize_mode);
  delay(250);
  TransByte(cont_reg_1, startFreq_mode);
  byte status_val = RecByte(status_reg); //Data obtained from "Recbyte" command should be byte, not Int. Ensures no extra data
  for (int y = 0; y < avg_amt; y++)
  {
    while ((status_val & 0x02) != 2) // use AND operator
    {
      status_val = RecByte(status_reg);
    }
    int real_imp = RealImp();
    //Serial.print ("Real");
    //Serial.println(real_imp);

    int img_imp = ImgImp();
    //Serial.print ("Img");
    //Serial.println(img_imp);

    Imp_avg += (sqrt(pow(real_imp, 2) + pow(img_imp, 2))) * 1.0; //update for unknown and also check how to do calibration
    Phs_avg += ((atan2(img_imp, real_imp)) * (180.0 / pi));
    TransByte(cont_reg_1, repeat);

  }
  double CalImp_Mag = Imp_avg / avg_amt;
  system_phase = Phs_avg / avg_amt;
  gainFactor = (1.0 / (CalImp_Mag * Z_cal));
  Serial.print("Impedance mag1: ");
  Serial.println(CalImp_Mag);
  Serial.print("Calgain factor1: ");
  Serial.println(gainFactor, 20);
  Serial.print("System Phase: ");
  Serial.println(system_phase, 4);


  Serial.println("Calibration Complete");
  TransByte(cont_reg_1, powerDown_mode);

  //TransByte(cont_reg_1,repeat_mode); // check or update its placement

  //ChoosePath(2);

}


void FreqSweep()
{
  // ChoosePath(2);
  TransByte(cont_reg_1, standby);
  delay(250);
  //  int x = 0;
  TransByte(cont_reg_1, initialize_mode);
  //  Serial.println("Entering Sweep");
  //Delay to let signal pass through sample
  delay(250);

  TransByte(cont_reg_1, startFreq_mode);
  //  Serial.println("After Start Freq Mode");



  byte status_val = RecByte(status_reg);
  //Serial.println(status_val, BIN);
  //  while (Serial.available() == 0)
  //  {

  double Phase = 0.0;
  double Z_Mag = 0.0;
  for (int y = 0; y < avg_amt; y++)
  {
    while ((status_val & 0x02) != 2)
    { // While the sweep is not complete, check for available data
      //      Serial.println("Checking for data");
      status_val = RecByte(status_reg);
    }
    //Once the data is available, this next lines will run (no if statement needed)
    int real_imp = RealImp();
    int img_imp = ImgImp();
    double Imp_val = (sqrt(pow(real_imp, 2) + pow(img_imp, 2))) * 1.0; //update for unknown and also check how to do calibration
    //Serial.println(Imp_val);
    Z_Mag += (1.0 / (gainFactor * Imp_val)) - Rout;
    Phase += (atan2(img_imp, real_imp)) * (180.0 / pi) - system_phase;

    TransByte(cont_reg_1, repeat);
    //The calculation of impedance, including gain factor, was moved into the loop
    //Averaging is the only thing done outside the loop
  }
  Z_Mag = Z_Mag / avg_amt;
  //  Serial.print("Z mag: ");
  //  Serial.println(Z_Mag);

  Phase = Phase / avg_amt;

  Z_fit = 647503.0 * pow(Z_Mag, -0.933);
  //delay(250);
  // }
  //  N = x - 1;
  //Serial.print("Mag: ");
  Serial.println(Z_fit);
  //Serial.print("  Phase: ");
  //Serial.println(Phase);



  //  }
  //Print_Results();
  //TransByte(cont_reg_1, standby);
  TransByte(cont_reg_1, powerDown_mode);

  // write a check to see if all frequencies have been swept and
  //    if (Firebase.setFloat(db, path + Count + "/TI", Z_fit))
  //    {
  //      Serial.println("TI Connected");
  //    }
  //    else
  //    {
  //      Serial.println("TI not connected");
  //    }

}
void TransByte(byte a, byte v)
{
  Wire.beginTransmission(0x0D);
  Wire.write(a);
  Wire.write(v);
  Wire.endTransmission();
}

byte RecByte(byte a)
{
  Wire.beginTransmission(0x0D);
  Wire.write(a);
  Wire.endTransmission();
  Wire.requestFrom(0x0D, 1);
  if (Wire.available() > 0)
  {
    byte x = Wire.read();
    Wire.endTransmission();
    return x;
  }
}

int RealImp()
{
  byte real_1 = RecByte(real_reg_1);
  byte real_2 = RecByte(real_reg_2);
  int real_val = (real_1 << 8) | real_2;
  if (real_val > 0x7FFF) //0x7FFF = 32767
  {
    real_val = real_val - 65536;
    //    Serial.print("Negative Real val");
  }
  else
  {
    //    Serial.print("Positive Real val");
  }
  return real_val;

}

int ImgImp()
{ byte img_1 = RecByte(img_reg_1);
  byte img_2 = RecByte(img_reg_2);
  int img_val = (img_1 << 8) | img_2;
  if (img_val > 0x7FFF) //0x7FFF = 32767
  {
    img_val = img_val - 65536;
    //    Serial.println("   Negative Imag val");
  }
  else
  {
    //Do nothing
    //    Serial.println("   Positive Imag val");
  }
  return img_val;
}



void ChoosePath( byte n) {
  switch (n) {
    case 1:
      digitalWrite(A, LOW);
      //  digitalWrite(B, HIGH);
      //Switched to unknown Z
      Serial.println("Case 1");
      delay(100);
      break;
    case 2:
      digitalWrite(A, HIGH);
      Serial.println("Case 2");

      // digitalWrite(B, LOW);
      delay(100);
      //Switch to Rcal (1.5k)
      break;
  }
}
void interruptFunction() {
  Serial.println("\nArduino is Awake! \n");
}

void hr()
{
  for (int i = 0 ; i < 500 ; i++)
  {
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);

    if (irValue < 50000)
      Serial.print(" No finger?");

    Serial.println();
  }
}
void sp()
{

  bufferLength = 200; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  //while (1)
  //{
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 50; i < 200; i++)
  {
    redBuffer[i - 50] = redBuffer[i];
    irBuffer[i - 50] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 150; i < 200; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //send samples and calculation result to terminal program through UART
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.print(irBuffer[i], DEC);

    Serial.print(F(", HR="));
    Serial.print(heartRate, DEC);

    Serial.print(F(", HRvalid="));
    Serial.print(validHeartRate, DEC);

    Serial.print(F(", SPO2="));
    Serial.print(spo2, DEC);

    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  // }

}
