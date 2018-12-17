// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       RelayActuatorSAMD21.ino
    Created:	13.12.2018 20:55:18
    Author:     TERRAIG2018\Raig
*/

// Define User Types below here or use a .h file
//


// Define Function Prototypes that use User Types below here or use a .h file
//


// Define Functions below here or use other .ino or cpp files
//
// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_DEBUG_PRINT
#define MY_DEBUG_INCOMING
#include <Boards.h>
#define VERSION "2.0"
#define MyRSSI_Values

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_NEW_DRIVER   // ATC on RFM69 works only with the new driver (not compatible with old=default driver)
#define MY_RFM69_ATC_TARGET_RSSI_DBM (-70)  // target RSSI -70dBm
#define MY_RFM69_MAX_POWER_LEVEL_DBM (10)   // max. TX power 10dBm = 10mW
#define MY_SENSOR_NETWORK
#define MY_NODE_ID 103
// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

#define MY_DALLASTEMPERATURE
#define MY_SHT2XTEMPERATURE

#include <Arduino.h>
//#include <SoftwareSerial.h>
#include <MySensors.h>
#include <MyConfig.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SPI.h>
#include <neotimer.h>
#include <Bounce2.h>
#include <Wire.h>
#include <Sodaq_SHT2x.h>



#define RELAY_1  4  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define RELAY_2  5  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay
#define CONTACT1 6
#define CONTACT2 7
#define KEY_INCLUSION A2
#define KEY_OPEN  A1
#define KEY_CLOSE A0

//#define BATT_OUTPUT_PIN A1
//int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point
#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 3 // Pin where dallas sensor is connected ->PD3
#define MAX_ATTACHED_DS18B20 4
#define TEMPERATURE_PRECISION 12 // bit resolution
#define NUM_BUTTONS 5
const uint8_t BUTTON_PINS[NUM_BUTTONS] = { CONTACT1, CONTACT2, KEY_INCLUSION, KEY_OPEN, KEY_CLOSE };

#define CHILD_ID_RSSI            (1)
#define CHILD_ID_TEMP_SENSOR     (10)
#define CHILD_ID_SHT2X_TEMP    (CHILD_ID_TEMP_SENSOR + MAX_ATTACHED_DS18B20)
#define CHILD_ID_SHT2X_HUM     (CHILD_ID_TEMP_SENSOR + MAX_ATTACHED_DS18B20 + 1)
#define CHILD_ID_SHT2X_DEW     (CHILD_ID_TEMP_SENSOR + MAX_ATTACHED_DS18B20 + 2)
#define CHILD_ID_RELAY1          (20)
#define CHILD_ID_RELAY2          (21)
#define CHILD_ID_RELAY_DELAY     (30)
#define CHILD_ID_RELAY_DELAY     (30)
#define CHILD_ID_CONTACT1        (40)
#define CHILD_ID_CONTACT2        (41)



Neotimer mytimer = Neotimer(15000); // x second timer
Neotimer relaytimer1 = Neotimer(); // x relay hold timer
Neotimer relaytimer2 = Neotimer(); // x relay hold timer
Bounce * buttons = new Bounce[NUM_BUTTONS];
int oldValue = -1;

MyMessage msgDelay1(CHILD_ID_RELAY_DELAY, V_VAR1);
MyMessage msgDelay2(CHILD_ID_RELAY_DELAY, V_VAR2);
MyMessage msgDelay3(CHILD_ID_RELAY_DELAY, V_VAR3);
MyMessage msgDelay4(CHILD_ID_RELAY_DELAY, V_VAR4);
MyMessage msgDelay5(CHILD_ID_RELAY_DELAY, V_VAR5);

MyMessage msgTemp(CHILD_ID_TEMP_SENSOR, V_TEMP);
MyMessage msgRelay1(CHILD_ID_RELAY1, V_STATUS);
MyMessage msgRelay2(CHILD_ID_RELAY2, V_STATUS);
MyMessage msgContact1(CHILD_ID_CONTACT1, V_TRIPPED);
MyMessage msgContact2(CHILD_ID_CONTACT2, V_TRIPPED);

#ifdef MY_SHT2XTEMPERATURE
MyMessage msgSHT_TEMP(CHILD_ID_SHT2X_TEMP,	V_TEMP);
MyMessage msgSHT_HUM(CHILD_ID_SHT2X_HUM,	V_HUM);
MyMessage msgSHT_DEW(CHILD_ID_SHT2X_DEW,	V_HUM);
#endif

#ifdef MyRSSI_Values
MyMessage msgRSSI_BAT(CHILD_ID_RSSI, V_VAR1);
MyMessage msgRSSI_TX(CHILD_ID_RSSI, V_VAR2);
MyMessage msgRSSI_RX(CHILD_ID_RSSI, V_VAR3);
MyMessage msgRSSI_TX_LEVEL(CHILD_ID_RSSI, V_VAR4);
MyMessage msgRSSI_TX_PERC(CHILD_ID_RSSI, V_VAR5);
#endif


const char *Scetch_Info = { "Relay Timer SAMD21" };

#ifdef MY_DALLASTEMPERATURE
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
int numSensors = 0;
bool metric = true;
DeviceAddress tempDeviceAddress;
#define TEMPERATURE_PRECISION 12
//int resolution = 12;
//int sensorValue = 0;
#endif

uint32_t relay_delay[2];
uint8_t relayCountDown[2];

int EEPROM_Write(int pos, char *zeichen, int size);
int EEPROM_Read(int pos, char *character, int size);
void printAddress(DeviceAddress deviceAddress);
void writeAddress(DeviceAddress deviceAddress, uint8_t index);
void readAddress(DeviceAddress deviceAddress, uint8_t index);
void setRelay(uint8_t RelayID, bool relaystate);
void setRelayDelay(uint8_t RelayID, uint8_t relaydelay);
void checkRelayTimer(void);
void setup(void);
void loop(void);

typedef struct EEProm_Struct {
	byte listDeviceAddress[8][4];
	bool Relais1;
	bool Relais2;
	uint16_t Delay1;
	uint16_t Delay2;
}EEProm_Struct;

EEProm_Struct EE;

void before()
{
	// Then set relay pins in output mode
	pinMode(RELAY_1, OUTPUT);
	pinMode(RELAY_2, OUTPUT);
	// Set relay to last known state (using eeprom storage)
	EEPROM_Read(0, reinterpret_cast<char*>(&EE), sizeof(EE));
	relay_delay[0] = EE.Delay1 * 1000;
	relay_delay[1] = EE.Delay2 * 1000;

	if (relay_delay[0] > 0) {
		digitalWrite(RELAY_1, RELAY_OFF);
	}
	else {
		digitalWrite(RELAY_1, EE.Relais1 ? RELAY_ON : RELAY_OFF);
	}
	if (relay_delay[1] > 0) {
		digitalWrite(RELAY_2, RELAY_OFF);
	}
	else {
		digitalWrite(RELAY_2, EE.Relais2 ? RELAY_ON : RELAY_OFF);
	}

#ifdef MY_DALLASTEMPERATURE
	// Startup up the OneWire library
//sensors.setResolution(12);
//sensors.begin();
#endif
}

void setup()
{
	//start I2C
#ifdef MY_SHT2XTEMPERATURE
	Wire.begin();
#endif
	// start serial port
	SerialUSB.begin(115200);
	while (!SerialUSB); // Wait for Serial monitor to open


#ifdef MY_DALLASTEMPERATURE
	sensors.setWaitForConversion(true);
	// second adc read to get a correct value
	//sensorValue = 0;//analogRead(BATTERY_SENSE_PIN);
#endif

// Setup the Contacts and Keyboard
	for (int i = 0; i < NUM_BUTTONS; i++) {
		buttons[i].attach(BUTTON_PINS[i], INPUT_PULLUP);       //setup the bounce instance for the current button
		buttons[i].interval(25);              // debouncer interval in ms
	}

	mytimer.start();
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(Scetch_Info, VERSION);
	SerialUSB.print(Scetch_Info);
	SerialUSB.println(VERSION);
	wait(1000);

	// Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID_RELAY1, S_BINARY, ("Relay"), false);
	wait(1000);
	present(CHILD_ID_RELAY2, S_BINARY, ("Relay"), false);
	wait(1000);
	present(CHILD_ID_RELAY_DELAY, S_CUSTOM, "OnOff relay_delay");
	wait(1000);
	present(CHILD_ID_CONTACT1, S_DOOR, "Contact 1");
	wait(1000);
	present(CHILD_ID_CONTACT2, S_DOOR, "Contact 2");
	wait(1000);

#ifdef MyRSSI_Values
	//	Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID_RSSI, S_CUSTOM, "Batt Volt, TX UPLINK QUALITY RSSI");
#endif

#ifdef MY_DALLASTEMPERATURE
	// Fetch the number of attached temperature sensors
	sensors.setResolution(12);
	sensors.begin();
	numSensors = sensors.getDeviceCount();
	//int i;
	// Present all sensors to controller
	for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
		present(CHILD_ID_TEMP_SENSOR + i, S_TEMP, ("DEG C"), true);
		// Search the wire for address
		if (sensors.getAddress(tempDeviceAddress, i))
		{
			SerialUSB.print("device ");
			SerialUSB.print(i, DEC);
			SerialUSB.print(" address: ");
			printAddress(tempDeviceAddress);
			SerialUSB.println();
			writeAddress(tempDeviceAddress, i);
			//EE.listDeviceAddress[8][i] = tempDeviceAddress;
			// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
			sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

			SerialUSB.print("Resolution is: ");
			SerialUSB.print(sensors.getResolution(tempDeviceAddress), DEC);
			SerialUSB.println();
		}
		else {
			SerialUSB.print("ghost device at ");
			SerialUSB.print(i, DEC);
			//SerialUSB.print(" but could not detect address. Check power and cabling");
		}
		EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE)); //save the NV structure to EEProm
		wait(1000);
	}
#endif

#ifdef MY_SHT2XTEMPERATURE
	present(CHILD_ID_SHT2X_TEMP, S_TEMP, "Temperature_SHT");
	wait(1000);
	present(CHILD_ID_SHT2X_HUM , S_HUM, "Humitidy_SHT");
	wait(1000);
	present(CHILD_ID_SHT2X_DEW , S_HUM, "Dew_Point_SHT");
	wait(1000);
#endif
}


void loop()
{
	float temperature;
	//int batteryPcnt = 100;//sensorValue / 10;
	//float batteryV = sensorValue * 0.003613281;
	DeviceAddress deviceAddress;
	if (mytimer.repeat()) {
		SerialUSB.println(" Timer Repeat");

#ifdef MyRSSI_Values
		// retrieve RSSI / SNR reports from incoming ACK
#ifdef MY_DEBUG_PRINT
		SerialUSB.print("TX RSSI dBm : ");
		SerialUSB.println(transportGetSignalReport(SR_TX_RSSI));
		SerialUSB.print("RX RSSI dBm : ");
		SerialUSB.println(transportGetSignalReport(SR_RX_RSSI));
		SerialUSB.print("TX Power Level : ");
		SerialUSB.println(transportGetSignalReport(SR_TX_POWER_LEVEL));
		SerialUSB.print("TX Power Percent : ");
		SerialUSB.println(transportGetSignalReport(SR_TX_POWER_PERCENT));
#endif

		send(msgRSSI_BAT.set(3.3, 2));
		send(msgRSSI_TX.set(transportGetSignalReport(SR_TX_RSSI)));
		send(msgRSSI_RX.set(transportGetSignalReport(SR_RX_RSSI)));
		send(msgRSSI_TX_LEVEL.set(transportGetSignalReport(SR_TX_POWER_LEVEL)));
		send(msgRSSI_TX_PERC.set(transportGetSignalReport(SR_TX_POWER_PERCENT)));

#endif
		//switch the battery divder on this takes about 3µA

#ifdef MY_DALLASTEMPERATURE
		// Fetch temperatures from Dallas sensors
		sensors.requestTemperatures();

		// query conversion time and sleep until conversion completed
		int16_t conversionTime = 2000;//sensors.millisToWaitForConversion(sensors.getResolution());

									 // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
		wait(conversionTime);
		int i;
		// Read temperatures and send them to controller
		for (i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
			// Fetch and round temperature to one decimal
			//temperature = getControllerConfig().isMetric ? sensors.getTempCByIndex(i) : sensors.getTempFByIndex(i);
			//temperature = sensors.getTempCByIndex(i);
			readAddress(deviceAddress, i);
			sensors.requestTemperaturesByAddress(deviceAddress);
			temperature = sensors.getTempC(deviceAddress);
			// Send in the new temperature
			send(msgTemp.setSensor(CHILD_ID_TEMP_SENSOR + i).set(temperature, 2));

			//#ifdef MY_DEBUG_PRINT
				  //sensors.getAddress(tempDeviceAddress, i);
			printAddress(deviceAddress);
			SerialUSB.print(".i = ");
			SerialUSB.print(i);
			SerialUSB.print(". T = ");
			SerialUSB.println(temperature, 3);
			//#endif
		}


#endif

#ifdef MY_SHT2XTEMPERATURE

		SerialUSB.print("Humidity(%RH): ");
		send(msgSHT_HUM.setSensor(CHILD_ID_SHT2X_HUM).set(SHT2x.GetHumidity(), 2));
		SerialUSB.print(SHT2x.GetHumidity());
		SerialUSB.print("  Temperature(C): ");
		send(msgSHT_TEMP.setSensor(CHILD_ID_SHT2X_TEMP).set(SHT2x.GetTemperature(), 2));
		SerialUSB.println(SHT2x.GetTemperature());
		SerialUSB.print("  Dewpoint(C): ");
		send(msgSHT_DEW.setSensor(CHILD_ID_SHT2X_DEW).set(SHT2x.GetDewPoint(), 2 ));
		SerialUSB.println(SHT2x.GetDewPoint());

#endif

	}

	for (int i = 0; i < NUM_BUTTONS; i++) {
		// Update the Bounce instance :
		buttons[i].update();
		// If it fell, flag the need to toggle the LED

		if (buttons[0].fell()) {
			send(msgContact1.set(false));
		}
		if (buttons[0].rose()) {
			send(msgContact1.set(true));
		}
		if (buttons[1].fell()) {
			send(msgContact2.set(false));
		}
		if (buttons[1].rose()) {
			send(msgContact2.set(true));
		}
	}

	checkRelayTimer();
}

void receive(const MyMessage &message)
{
#ifdef MY_DEBUG_INCOMING
	SerialUSB.print(", Custom: ");
	SerialUSB.print(message.sensor);
	SerialUSB.print(", Type: ");
	SerialUSB.print(message.type);
	SerialUSB.print(", Data: ");
	SerialUSB.println(message.getInt());
	saveState(message.sensor + message.type, message.getInt());
#endif
	// Message check
	switch (message.sensor) {
	case CHILD_ID_RELAY1:
		if (message.type == V_STATUS) {
			setRelay(RELAY_1, message.getBool());
			SerialUSB.print("RelayID1 = ");
			SerialUSB.println(message.sensor);
		};
		break;
	case CHILD_ID_RELAY2:
		if (message.type == V_STATUS) {
			setRelay(RELAY_2, message.getBool());
			SerialUSB.print("RelayID2 = ");
			SerialUSB.println(message.sensor);
		};
		break;
	case CHILD_ID_RELAY_DELAY: {
		switch (message.type) {
		case V_VAR1: {
			setRelayDelay(RELAY_1, (uint8_t)message.getInt());
			SerialUSB.print("RELAY1_DELAY = ");
			SerialUSB.println((uint8_t)message.getInt());
		}
					 break;
		case  V_VAR2: {
			setRelayDelay(RELAY_2, (uint8_t)message.getInt());
			SerialUSB.print("RELAY2_DELAY = ");
			SerialUSB.println((uint8_t)message.getInt());
		}
					  break;
		}
	}
							   break;
	}

	// Write some debug info
#ifdef MY_DEBUG_INCOMING
	SerialUSB.print("Incoming change for sensor:");
	SerialUSB.print(message.sensor);
	SerialUSB.print(", New status: ");
	SerialUSB.println(message.getBool());
#endif


}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (deviceAddress[i] < 16) SerialUSB.print("0");
		SerialUSB.print(deviceAddress[i], HEX);
	}
}

// function to write a device address
void writeAddress(DeviceAddress deviceAddress, uint8_t index)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		EE.listDeviceAddress[i][index] = deviceAddress[i];
	}
}
// function to read a device address
void readAddress(DeviceAddress deviceAddress, uint8_t index)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		deviceAddress[i] = EE.listDeviceAddress[i][index];
	}
}

void setRelay(uint8_t RelayID, bool relaystate)
{
	// Change relay state
	switch (RelayID) {
	case RELAY_1: {
		digitalWrite(RelayID, relaystate ? RELAY_ON : RELAY_OFF);
		EE.Relais1 = relaystate;
		if (relay_delay[0] > 0) {
			relaytimer1.set(relay_delay[0]);
			relaytimer1.start();
			SerialUSB.print("RelayID = ");
			SerialUSB.println(RelayID);
			break;
		}
	}
	case RELAY_2: {
		digitalWrite(RelayID, relaystate ? RELAY_ON : RELAY_OFF);
		EE.Relais2 = relaystate;
		if (relay_delay[1] > 0) {
			relaytimer2.set(relay_delay[1]);
			relaytimer2.start();
			SerialUSB.print("RelayID = ");
			SerialUSB.println(RelayID);
			break;
		}

	}
	}
	// Store state in eeprom
//	saveState(RelayID, relaystate);
	EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE));

}

void setRelayDelay(uint8_t RelayID, uint8_t relaydelay)
{
	switch (RelayID) {
	case RELAY_1: {
		relay_delay[0] = relaydelay * 1000;
		EE.Delay1 = relaydelay; //save delay to NV structure
		break;
	}

	case RELAY_2: {
		relay_delay[1] = relaydelay * 1000;
		EE.Delay2 = relaydelay; //save delay to NV structure
		break;
	}
	}
	//EE.Delay1 = relaydelay;
	EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE)); //save the NV structure to EEProm
	SerialUSB.print("relaydelay 1 = ");
	SerialUSB.println(relay_delay[0]);
	SerialUSB.print("relaydelay 2 = ");
	SerialUSB.println(relay_delay[1]);
}

void checkRelayTimer() {
	if (relaytimer1.done()) {
		EE.Relais1 = RELAY_OFF;
		relaytimer1.stop();
		digitalWrite(RELAY_1, RELAY_OFF);
		send(msgRelay1.setSensor(CHILD_ID_RELAY1).set(int32_t (RELAY_OFF)));
	}
	if (relaytimer2.done()) {
		EE.Relais2 = RELAY_OFF;
		relaytimer2.stop();
		digitalWrite(RELAY_2, RELAY_OFF);
		send(msgRelay2.setSensor(CHILD_ID_RELAY2).set(int32_t (RELAY_OFF)));
	}
	//	EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE));
		//	SerialUSB.println("Check Relay Timer ");
}

// Write any data structure or variable to EEPROM
int EEPROM_Write(int pos, char *zeichen, int size)
{
	for (int i = 0; i < size; i++)
	{
		saveState(pos + i, *zeichen);
		zeichen++;
	}
	return pos + size;
}

// Read any data structure or variable from EEPROM
int EEPROM_Read(int pos, char *character, int size)
{
	for (int i = 0; i < size; i++)
	{
		*character = loadState(pos + i);
		character++;
	}
	return pos + size;
}
