/* Use the Espressif EEPROM library. Skip otherwise */
#if defined(ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_ESP8266)
#include <EEPROM.h>
#define USE_EEPROM
#endif

/*************************BME688 Setup*******************************************/

#include <bsec2.h>

#include "Trained_Model.h"

#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000) /* 360 minutes - 4 times a day */
#define PANIC_LED LED_BUILTIN
#define ERROR_DUR 1000

//void errLeds(void);
void checkBsecStatus(Bsec2 bsec);
void updateBsecState(Bsec2 bsec);
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
bool loadState(Bsec2 bsec);
bool saveState(Bsec2 bsec);

/* Create an object of the class Bsec2 */
Bsec2 envSensor;
#ifdef USE_EEPROM
static uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
#endif
/* Gas estimate names will be according to the configuration classes used */
const String gasName[] = { "Air", "Coffee"};

/*********************************WiFi Setup******************************************/

#include <WiFi.h>
#include <HTTPClient.h>
 
const char* ssid = "2GIRLS1WIFI";
const char* password =  "jetisbald";

IPAddress ip;

/*******************************GPS Setup********************************************/

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);

#define GPS_UPDATE_DELAY 5000 //One Minute
long lastGPSUpdate = -2000;
String gpsLatitude = "0.000";
String gpsLongitude = "0.000";

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true


void setup() {

  //Setup Serial
  Serial.begin(115200);
  Wire.begin();
  delay(4000);   //Delay needed before calling the WiFi.begin

  //Setup Wifi
  WiFi.begin(ssid, password); 
 
  while (WiFi.status() != WL_CONNECTED) { //Check for the connection
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
  ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  /* Desired subscription list of BSEC2 outputs */

    bsecSensor sensorList[] = {
            BSEC_OUTPUT_RAW_TEMPERATURE,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_RAW_HUMIDITY,
            BSEC_OUTPUT_RAW_GAS,
            BSEC_OUTPUT_RAW_GAS_INDEX,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
            BSEC_OUTPUT_GAS_ESTIMATE_1,
            BSEC_OUTPUT_GAS_ESTIMATE_2
    };

    Serial.begin(115200);
  #ifdef USE_EEPROM
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);
  #endif
    
    pinMode(PANIC_LED, OUTPUT);

   
    while (!Serial) delay(10);
   

    if (!envSensor.begin(0x77, Wire))
    {
        checkBsecStatus(envSensor);
    }

    
    if (!envSensor.setConfig(Trained_Model_config))
    {
        checkBsecStatus (envSensor);
    }

 
    if (!loadState(envSensor))
    {
        checkBsecStatus (envSensor);
    }


    if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_HIGH_PERFORMANCE))
    {
        checkBsecStatus (envSensor);
    }

 
    envSensor.attachCallback(newDataCallback);

    Serial.println("\nBSEC library version " + \
            String(envSensor.version.major) + "." \
            + String(envSensor.version.minor) + "." \
            + String(envSensor.version.major_bugfix) + "." \
            + String(envSensor.version.minor_bugfix));

  if (!GPS.begin(0x10)) {
    Serial.println("Unable to connect to GPS");  // The I2C address to use is 0x10
  } else {
    Serial.println("Connected to GPS module");
  }
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);

  Serial.println(F("Initialized"));
}

void loop() {
  // put your main code here, to run repeatedly:
  updateGPS();

  if (!envSensor.run()) {
    checkBsecStatus (envSensor);
  }

}


//sendDataToServer(sensorHumidity, sensorTemperature, sensorPressure, sensorGas, sensorGasIndex, sensorCompensatedTemp, sensorCompensatedHumidity, sensorProability, sensorClassification);
void sendDataToServer(String sensorHumidity, String sensorTemperature, String sensorPressure, String sensorGas, String sensorGasIndex, String sensorCompensatedTemperature, String sensorCompensatedHumidity, String sensorProbability, String sensorClassification) {

  HTTPClient httpClient;

  String serverPath1 = "https://script.google.com/macros/s/AKfycbwcNoQVO6CNp14xyarB5DzSb3t_J-cuTK_ZDjMyZQe6w9T6KOsAowWzCn-Zarh8PXBoiQ/exec";
  String serverName1 = "script.google.com";
  
  httpClient.begin(serverPath1); //Specify destination for HTTP request

  String bound = "rftdcw34r5tgbv43h53rv34abcacb";
  httpClient.addHeader("Content-Type", "multipart/form-data; boundary=" + bound);
  
  
  String PostData = "--" + bound + "\r\nContent-Disposition: form-data; name=\"Humidity\"" + "\r\n\r\n" + sensorHumidity+ "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"Temperature\"" + "\r\n\r\n" + sensorTemperature + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"Pressure\"" + "\r\n\r\n" + sensorPressure + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"Gas\"" + "\r\n\r\n" + sensorGas + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"GasIndex\"" + "\r\n\r\n" + sensorGasIndex + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"CompensatedTemperature\"" + "\r\n\r\n" + sensorCompensatedTemperature + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"CompensatedHuidity\"" + "\r\n\r\n" + sensorCompensatedHumidity + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"Probability\"" + "\r\n\r\n" + sensorProbability + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"Classification\"" + "\r\n\r\n" + sensorClassification + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"Latitude\"" + "\r\n\r\n" + gpsLatitude + "\r\n";
  PostData += "--" + bound + "\r\nContent-Disposition: form-data; name=\"Longitude\"" + "\r\n\r\n" + gpsLongitude + "\r\n";
  
  String tail = "\r\n--" + bound + "--\r\n";
    
  int httpResponseCode = httpClient.POST(PostData + tail);

  
}

void classifyData() {

  
}

void updateGPS() {

  
  if (millis() - lastGPSUpdate > GPS_UPDATE_DELAY) {
    if (GPS.newNMEAreceived()) {

    Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA())) return;
  }
  
    lastGPSUpdate = millis();
    if (GPS.fix) {
      Serial.println("Got a fix");
      Serial.print("Satellites: ");
      Serial.println(String(GPS.satellites));
      gpsLatitude = String(GPS.latitudeDegrees, 4);
      gpsLongitude = String(GPS.longitudeDegrees, 4);
    } else {
      Serial.println("No fix");
      Serial.print("Satellites: ");
      Serial.println(String(GPS.satellites));
    }
  }
  
}

void errLeds(void)
{
    while(1)
    {
        digitalWrite(PANIC_LED, HIGH);
        delay(ERROR_DUR);
        digitalWrite(PANIC_LED, LOW);
        delay(ERROR_DUR);
    }
}

void updateBsecState(Bsec2 bsec)
{
    static uint16_t stateUpdateCounter = 0;
    bool update = false;

    if (!stateUpdateCounter || (stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
        /* Update every STATE_SAVE_PERIOD minutes */
        update = true;
        stateUpdateCounter++;
    }

    if (update && !saveState(bsec))
        checkBsecStatus(bsec);
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
        return;

    String sensorHumidity = "";
    String sensorTemperature = "";
    String sensorPressure = "";
    String sensorGas = "";
    String sensorGasIndex = "";
    String sensorCompensatedTemp = "";
    String sensorCompensatedHumidity = "";
    String sensorProbability = "";
    String sensorClassification = "";

    Serial.println("BSEC outputs:\n\ttimestamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output  = outputs.output[i];
        switch (output.sensor_id)
        {
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                sensorTemperature = String(output.signal);
                Serial.println("\ttemperature = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
            sensorPressure = String(output.signal);
                Serial.println("\tpressure = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY:
            sensorHumidity = String(output.signal);
                Serial.println("\thumidity = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_GAS:
            sensorGas = String(output.signal);
                Serial.println("\tgas resistance = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_GAS_INDEX:
            sensorGasIndex = String(output.signal);
                Serial.println("\tgas index = " + String(output.signal));
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            sensorCompensatedTemp = String(output.signal);
                Serial.println("\tcompensated temperature = " + String(output.signal));
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            sensorCompensatedHumidity = String(output.signal);
                Serial.println("\tcompensated humidity = " + String(output.signal));
                break;
            case BSEC_OUTPUT_GAS_ESTIMATE_1:
            case BSEC_OUTPUT_GAS_ESTIMATE_2:
                if((int)(output.signal * 10000.0f) > 0) /* Ensure that there is a valid value xx.xx% */
                {
                  sensorClassification = gasName[(int) (output.sensor_id - BSEC_OUTPUT_GAS_ESTIMATE_1)];
                  sensorProbability = String(output.signal * 100);
                  
                    Serial.println("\t" + \
                      gasName[(int) (output.sensor_id - BSEC_OUTPUT_GAS_ESTIMATE_1)] + \
                      String(" probability : ") + String(output.signal * 100) + "%");
                }
                break;
            default:
                break;
        }
    }

    //sendDataToServer(sensorHumidity, sensorTemperature, sensorPressure, sensorGas, sensorGasIndex, sensorCompensatedTemp, sensorCompensatedHumidity, sensorProbability, sensorClassification);
    updateBsecState(envSensor);
}

void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK)
    {
        Serial.println("BSEC error code : " + String(bsec.status));
        errLeds(); /* Halt in case of failure */
    } else if (bsec.status > BSEC_OK)
    {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
        errLeds(); /* Halt in case of failure */
    } else if (bsec.sensor.status > BME68X_OK)
    {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}

bool loadState(Bsec2 bsec)
{
#ifdef USE_EEPROM
    

    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
    {
        /* Existing state in EEPROM */
        Serial.println("Reading state from EEPROM");
        Serial.print("State file: ");
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        {
            bsecState[i] = EEPROM.read(i + 1);
            Serial.print(String(bsecState[i], HEX) + ", ");
        }
        Serial.println();

        if (!bsec.setState(bsecState))
            return false;
    } else
    {
        /* Erase the EEPROM with zeroes */
        Serial.println("Erasing EEPROM");

        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
            EEPROM.write(i, 0);

        EEPROM.commit();
    }
#endif
    return true;
}

bool saveState(Bsec2 bsec)
{
#ifdef USE_EEPROM
    if (!bsec.getState(bsecState))
        return false;

    Serial.println("Writing state to EEPROM");
    Serial.print("State file: ");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
        EEPROM.write(i + 1, bsecState[i]);
        Serial.print(String(bsecState[i], HEX) + ", ");
    }
    Serial.println();

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
#endif
    return true;
}
