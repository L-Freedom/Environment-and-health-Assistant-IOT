#include <stdlib.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#define SSID "329A2.4GHz"
#define PASS "8888Fang"
#include "MQ135.h"
const int ANALOGPIN = 2;
MQ135 gasSensor = MQ135(ANALOGPIN);
//#define SSID "UCInet Mobile Access"
//#define PASS ""
#define IP "api.thingspeak.com"
#define trigPin 13
#define echoPin 12
#define led 9
#define led2 8
#define DHTPIN 2      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

/************************mq2sensor************************************/
/************************Hardware Related Macros************************************/
#define MQ2PIN (4)                      //define which analog input channel you are going to use
#define RL_VALUE_MQ2 (1)                //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR_MQ2 (9.577) //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                        //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define CALIBARAION_SAMPLE_TIMES (50)     //define how many samples you are going to take in the calibration phase
#define CALIBRATION_SAMPLE_INTERVAL (500) //define the time interal(in milisecond) between each samples in the \
                                          //cablibration phase
#define READ_SAMPLE_INTERVAL (50)         //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES (5)             //define the time interal(in milisecond) between each samples in \
                                          //normal operation

/**********************Application Related Macros**********************************/
#define GAS_HYDROGEN (0)
#define GAS_LPG (1)
#define GAS_METHANE (2)
#define GAS_CARBON_MONOXIDE (3)
#define GAS_ALCOHOL (4)
#define GAS_SMOKE (5)
#define GAS_PROPANE (6)
#define accuracy (0) //for linearcurves
//#define         accuracy                    (1)   //for nonlinearcurves, un comment this line and comment the above line if calculations
//are to be done using non linear curve equations
/*****************************Globals************************************************/
float Ro = 0; //Ro is initialized to 10 kilo ohms

String GET = "GET /update?key=your_key&field1="; //replace ZZZZZ by your ThingSpeak channel write key
SoftwareSerial monitor(10, 11);                          //Serial communication to ESP8266 module (RX, TX)

DHT dht(DHTPIN, DHTTYPE);

//Variables
int luminancePin = A1;
int uvPin = A0;
unsigned long starttime;
unsigned long sampletime_ms = 1000;
unsigned long delay_time = 60000;
int failNum = 0;

//setup
void setup()
{
    //start serial communications
    Serial.begin(9600);
    monitor.begin(9600);
    //configure Arduino pins
    Serial.println("Initializing...");
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(led, OUTPUT);
    pinMode(led2, OUTPUT);

    //communication with wifi module
    //monitor.flush();
    monitor.println("AT+RESTORE");
    monitor.println("AT");
    monitor.println("AT+CWMODE=4");
    delay(2000);
    monitor.flush();
    //String ip = "AT+CIFSR";
    //monitor.println(ip);
    //Serial.print("ip is ");

    Serial.print("Calibrating MQ2...\n");
    Ro = MQCalibration(MQ2PIN); //Calibrating the sensor. Please make sure the sensor is in clean air
                                //when you perform the calibration
    Serial.print("MQ2 Calibration is done...\n");
    Serial.print("Ro=");
    Serial.print(Ro);
    Serial.print("kohm");
    Serial.print("\n");

    // Loop through all the data returned
    while (monitor.available())
    {
        // The esp has data so display its output to the serial window
        char c = monitor.read(); // read the next character.
        Serial.write(c);
    }
    Serial.println("");

    if (monitor.find("OK"))
    {
        Serial.println("Communication with ESP8266 module: OK");
    }
    else
    {
        Serial.println("ESP8266 module ERROR");
    }

    //connect wifi router
    connectWiFi();

    Serial.print("Sampling (");
    Serial.print(sampletime_ms / 1000);
    Serial.println("s)...");

    //initialize timer
    starttime = millis();

    dht.begin();
}

void loop()
{

    //5 seconds cicle
    if ((millis() - starttime) >= sampletime_ms)
    {
        //read other sensors
        char buffer[10];

        //light sensor
        float luminance = analogRead(luminancePin);

        //UV sensor
        float uv = analogRead(uvPin);
        uv = uv * 0.0049; //convert values to volts
        uv = uv * 307;    //convert to mW/m²
        uv = uv / 200;    //calculate UV index

        //MQ-9
        //float sensor_volt;
        //float RS_gas;
        //float ratio;
        //-Replace the name "R0" with the value of R0 in the demo of First Test -/
        //float R0 = 2.05;
        //int sensorValueGas = analogRead(A2);
        //sensor_volt = ((float)sensorValueGas / 1024) * 5.0;
        //RS_gas = (5.0 - sensor_volt) / sensor_volt; // Depend on RL on yor module
        //ratio = RS_gas / R0;                        // ratio = RS/R0
        //------------------------------------------------------------/
        float ratio = gasSensor.getPPM();

        // Reading temperature or humidity takes about 250 milliseconds!
        // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
        float humidity = dht.readHumidity();
        // Read temperature as Celsius (the default)
        float temperature = dht.readTemperature();
        // Compute heat index in Celsius (isFahreheit = false)
        float heat = dht.computeHeatIndex(temperature, humidity, false);

        // Check if any reads failed and exit early (to try again).
        if (isnan(humidity) || isnan(temperature))
        {
            Serial.println(F("Failed to read from DHT sensor!"));
            return;
        }

        // Compute gases
        float methane = MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_METHANE);
        float co = MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_CARBON_MONOXIDE);
        float alcohol = MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_ALCOHOL);
        
        // Serial.print("HYDROGEN:");
        // Serial.print(MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_HYDROGEN));
        // Serial.print("ppm");
        // Serial.print("    ");
        // Serial.print("LPG:");
        // Serial.print(MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_LPG));
        // Serial.print("ppm");
        // Serial.print("    ");
        // Serial.print("METHANE:");
        // Serial.print(MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_METHANE));
        // Serial.print("ppm");
        // Serial.print("    ");
        // Serial.print("CARBON_MONOXIDE:");
        // Serial.print(MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_CARBON_MONOXIDE));
        // Serial.print("ppm");
        // Serial.print("    ");
        // Serial.print("ALCOHOL:");
        // Serial.print(MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_ALCOHOL));
        // Serial.print("ppm");
        // Serial.print("    ");
        // Serial.print("SMOKE:");
        // Serial.print(MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_SMOKE));
        // Serial.print("ppm");
        // Serial.print("    ");
        // Serial.print("PROPANE:");
        // Serial.print(MQGetGasPercentage(MQRead(MQ2PIN) / Ro, GAS_PROPANE));
        // Serial.print("ppm");
        // Serial.print("\n");

                //distance measure
        long duration;
        float distance = 0.;
        digitalWrite(trigPin, LOW); // Added this line
        //delayMicroseconds(2);       // Added this line
        digitalWrite(trigPin, HIGH);
        //  delayMicroseconds(1000); - Removed this line
        //delayMicroseconds(10); // Added this line
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distance = (duration / 2) / 29.1;
        if (distance < 4 || distance > 15)
        {                            // This is where the LED On/Off happens
            digitalWrite(led, HIGH); // When the Red condition is met, the Green LED should turn off
            digitalWrite(led2, LOW);
        }
        else
        {
            digitalWrite(led, LOW);
            digitalWrite(led2, HIGH);
        }

        //convert sensor values to strings
        String totalStr = "S,D:";
        String distanceStr = dtostrf(distance, 3, 1, buffer);
        totalStr += distanceStr;
        String luminanceStr = dtostrf(luminance, 4, 1, buffer);
        luminanceStr.replace(" ", "");
        totalStr += ",L:";
        totalStr += luminanceStr;
        String uvStr = dtostrf(uv, 4, 1, buffer);
        uvStr.replace(" ", "");
        totalStr += ",U:";
        totalStr += uvStr;
        String humidityStr = dtostrf(humidity, 4, 1, buffer);
        humidityStr.replace(" ", "");
        totalStr += ",H:";
        totalStr += humidityStr;
        String temperatureStr = dtostrf(temperature, 4, 1, buffer);
        temperatureStr.replace(" ", "");
        totalStr += ",T:";
        totalStr += temperatureStr;
        String ratioStr = dtostrf(ratio, 4, 1, buffer);
        ratioStr.replace(" ", "");
        totalStr += ",A:";
        totalStr += ratioStr;
        String coStr = dtostrf(co, 4, 1, buffer);
        totalStr += ",C:";
        totalStr += coStr;
        String meStr = dtostrf(methane, 4, 1, buffer);
        totalStr += ",M:";
        totalStr += meStr;
        String wiStr = dtostrf(alcohol, 4, 1, buffer);
        totalStr += ",W:";
        totalStr += wiStr;
        totalStr += ",E";
        Serial.print(totalStr);

        //send data to ThingSpeak
        updateSensors(luminanceStr, uvStr, humidityStr, temperatureStr, ratioStr, coStr, meStr, wiStr);

        // Serial.print(F("luminance: "));
        // Serial.print(luminance);
        // Serial.print(F("Humidity: "));
        // Serial.print(humidity);
        // Serial.print(F("%  Temperature: "));
        // Serial.print(temperature);
        // Serial.print(F("°C "));
        // Serial.print(F("Heat index: "));
        // Serial.print(heat);
        // Serial.print(F("°C "));

        // Serial.print("uv index = ");
        // Serial.println(uv);

        // Serial.print(" AirQ = ");
        // Serial.println(ratio);



        

        //wait next sampling cycle
        // Serial.print("Wait ");
        // Serial.print(delay_time / 1000);
        // Serial.println("s for next sampling");
        // Serial.println();
        // delay(delay_time);

        //initialize new cycle
        // Serial.println();
        // Serial.print("Sampling (");
        // Serial.print(sampletime_ms / 1000);
        // Serial.println("s)...");
        starttime = millis();
    }
}

//Send data to ThingSpeak
void updateSensors(String luminanceStr, String uvStr, String humidityStr, String temperatureStr, String ratioStr, String coStr, String meStr, String wiStr)
{

    String cmd = "AT+CIPSTART=\"TCP\",\"";
    cmd += IP;
    cmd += "\",80";
    monitor.println(cmd);
    delay(2000);

    cmd = GET;
    cmd += luminanceStr;
    cmd += "&field2=";
    cmd += humidityStr;
    cmd += "&field3=";
    cmd += temperatureStr;
    cmd += "&field4=";
    cmd += uvStr;
    cmd += "&field5=";
    cmd += coStr;
    cmd += "&field6=";
    cmd += ratioStr;
    cmd += "&field7=";
    cmd += meStr;
    cmd += "&field8=";
    cmd += wiStr;
    cmd += "\r\n";
    delay(1000);
    int strsize = cmd.length();
    monitor.println("AT+CIPSEND=" + String(strsize));
    delay(2000);

    monitor.print(cmd);
    if (monitor.find("OK"))
    {
        Serial.println("Transmission completed with success");
    }
    else
    {
        Serial.println("Transmission failed!");
        failNum++;
        if (failNum > 100)
        {
            if (connectWiFi())
            {
                failNum = 0;
            }
        }
    }
}

void sendDebug(String cmd)
{
    Serial.print("SEND: ");
    Serial.println(cmd);
    monitor.println(cmd);
}

boolean connectWiFi()
{
    Serial.println("Connecting wi-fi...");
    String cmd = "AT+CWMODE=3";
    monitor.println(cmd);
    delay(2000);
    monitor.flush(); //clear buffer
    cmd = "AT+CWJAP=\"";
    cmd += SSID;
    cmd += "\",\"";
    cmd += PASS;
    cmd += "\"";
    monitor.println(cmd);
    delay(5000);

    if (monitor.find("OK"))
    {
        Serial.println("Connection succeeded!");
        return true;
    }
    else
    {
        Serial.println("Connection failed!");
        return false;
    }
    Serial.println();
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
    return (((float)RL_VALUE_MQ2 * (1023 - raw_adc) / raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
    int i;
    float RS_AIR_val = 0, r0;

    for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++)
    { //take multiple samples
        RS_AIR_val += MQResistanceCalculation(analogRead(mq_pin));
        delay(CALIBRATION_SAMPLE_INTERVAL);
    }
    RS_AIR_val = RS_AIR_val / CALIBARAION_SAMPLE_TIMES; //calculate the average value

    r0 = RS_AIR_val / RO_CLEAN_AIR_FACTOR_MQ2; //RS_AIR_val divided by RO_CLEAN_AIR_FACTOR yields the Ro
                                               //according to the chart in the datasheet

    return r0;
}

/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
    int i;
    float rs = 0;

    for (i = 0; i < READ_SAMPLE_TIMES; i++)
    {
        rs += MQResistanceCalculation(analogRead(mq_pin));
        delay(READ_SAMPLE_INTERVAL);
    }

    rs = rs / READ_SAMPLE_TIMES;

    return rs;
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function uses different equations representing curves of each gas to 
         calculate the ppm (parts per million) of the target gas.
************************************************************************************/
float MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
    if (accuracy == 0)
    {
        if (gas_id == GAS_HYDROGEN)
        {
            return (pow(10, ((-2.109 * (log10(rs_ro_ratio))) + 2.983)));
        }
        else if (gas_id == GAS_LPG)
        {
            return (pow(10, ((-2.123 * (log10(rs_ro_ratio))) + 2.758)));
        }
        else if (gas_id == GAS_METHANE)
        {
            return (pow(10, ((-2.622 * (log10(rs_ro_ratio))) + 3.635)));
        }
        else if (gas_id == GAS_CARBON_MONOXIDE)
        {
            return (pow(10, ((-2.955 * (log10(rs_ro_ratio))) + 4.457)));
        }
        else if (gas_id == GAS_ALCOHOL)
        {
            return (pow(10, ((-2.692 * (log10(rs_ro_ratio))) + 3.545)));
        }
        else if (gas_id == GAS_SMOKE)
        {
            return (pow(10, ((-2.331 * (log10(rs_ro_ratio))) + 3.596)));
        }
        else if (gas_id == GAS_PROPANE)
        {
            return (pow(10, ((-2.174 * (log10(rs_ro_ratio))) + 2.799)));
        }
    }

    else if (accuracy == 1)
    {
        if (gas_id == GAS_HYDROGEN)
        {
            return (pow(10, ((-2.109 * (log10(rs_ro_ratio))) + 2.983)));
        }
        else if (gas_id == GAS_LPG)
        {
            return (pow(10, ((-2.123 * (log10(rs_ro_ratio))) + 2.758)));
        }
        else if (gas_id == GAS_METHANE)
        {
            return (pow(10, ((-2.622 * (log10(rs_ro_ratio))) + 3.635)));
        }
        else if (gas_id == GAS_CARBON_MONOXIDE)
        {
            return (pow(10, ((-2.955 * (log10(rs_ro_ratio))) + 4.457)));
        }
        else if (gas_id == GAS_ALCOHOL)
        {
            return (pow(10, ((-2.692 * (log10(rs_ro_ratio))) + 3.545)));
        }
        else if (gas_id == GAS_SMOKE)
        {
            return (pow(10, (-0.976 * pow((log10(rs_ro_ratio)), 2) - 2.018 * (log10(rs_ro_ratio)) + 3.617)));
        }
        else if (gas_id == GAS_PROPANE)
        {
            return (pow(10, ((-2.174 * (log10(rs_ro_ratio))) + 2.799)));
        }
    }
    return 0.;
}
