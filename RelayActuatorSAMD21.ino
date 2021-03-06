
// Visual Micro is in vMicro>General>Tutorial Mode
//
/*
	Name:       RelayActuatorSAMD21.ino
	Created:	15.05.2019 
	Author:     TERRAIG2018\Raig
*/

//#define __SAMD21G18A__

// Define Functions below here or use other .ino or cpp files

// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_DEBUG_PRINT
#define MY_DEBUG_INCOMING
#include <Boards.h>
#define VERSION "2.1"
#define MyRSSI_Values

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_NEW_DRIVER   // ATC on RFM69 works only with the new driver (not compatible with old=default driver)
#define MY_RFM69_ATC_TARGET_RSSI_DBM (-70)  // target RSSI -70dBm
#define MY_RFM69_MAX_POWER_LEVEL_DBM (10)   // max. TX power 10dBm = 10mW
#define MY_SENSOR_NETWORK
//#define MY_NODE_ID 103
// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

#define MY_DALLASTEMPERATURE
#define MY_SHT2XTEMPERATURE

#include <Arduino.h>
#include <MySensors.h>
//#include <MyConfig.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SPI.h>
#include <neotimer.h>
#include <Wire.h>
#include <Sodaq_SHT2x.h>
#include "Adafruit_ZeroTimer.h"
#include <SeqButton.h>
#include <FlashAsEEPROM.h>


#define RELAY_1  4  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define RELAY_2  5  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define RELAY_3  8  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay


//#define BATT_OUTPUT_PIN A1
//int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point
#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 3 // Pin where dallas sensor is connected ->PD3
#define MAX_ATTACHED_DS18B20 4
#define TEMPERATURE_PRECISION 12 // bit resolution
#define NUM_BUTTONS 5

enum BUTTON_F { contact1 = 6, contact2 = 7, key_close = A0, key_open = A1, key_aux = A2 };
enum MSG_STATE { NoCHANGE, CONTACT1_On, CONTACT1_Off, CONTACT2_On, CONTACT2_Off,
	KEY_OPEN_On, KEY_OPEN_Off, KEY_CLOSE_On, KEY_CLOSE_Off, KEY_AUX_On, KEY_AUX_Off
} MSG_State;

SeqButton CONTACT1, CONTACT2, KEY_OPEN, KEY_CLOSE, KEY_AUX;


#define CHILD_ID_RSSI            (1)
#define CHILD_ID_TEMP_SENSOR     (10)
#define CHILD_ID_SHT2X_TEMP    (CHILD_ID_TEMP_SENSOR + MAX_ATTACHED_DS18B20)
#define CHILD_ID_SHT2X_HUM     (CHILD_ID_TEMP_SENSOR + MAX_ATTACHED_DS18B20 + 1)
#define CHILD_ID_SHT2X_DEW     (CHILD_ID_TEMP_SENSOR + MAX_ATTACHED_DS18B20 + 2)
#define CHILD_ID_RELAY_ON        (20)
#define CHILD_ID_RELAY_OFF       (21)
#define CHILD_ID_RELAY_AUX       (22)
#define CHILD_ID_RELAY_DELAY     (30)
#define CHILD_ID_CONTACT1        (40)
#define CHILD_ID_CONTACT2        (41)



Neotimer mytimer = Neotimer(30000); // x second timer
Neotimer relaytimer1 = Neotimer(); // x relay hold timer
Neotimer relaytimer2 = Neotimer(); // x relay hold timer
Neotimer relaytimer3 = Neotimer(); // x relay hold timer

MyMessage msgDelay1(CHILD_ID_RELAY_DELAY, V_VAR1);
MyMessage msgDelay2(CHILD_ID_RELAY_DELAY, V_VAR2);
MyMessage msgDelay3(CHILD_ID_RELAY_DELAY, V_VAR3);
MyMessage msgDelay4(CHILD_ID_RELAY_DELAY, V_VAR4);
MyMessage msgDelay5(CHILD_ID_RELAY_DELAY, V_VAR5);

MyMessage msgTemp(CHILD_ID_TEMP_SENSOR, V_TEMP);
MyMessage msgRelayOn(CHILD_ID_RELAY_ON, V_STATUS);
MyMessage msgRelayOff(CHILD_ID_RELAY_OFF, V_STATUS);
MyMessage msgRelayAux(CHILD_ID_RELAY_AUX, V_STATUS);
MyMessage msgContact1(CHILD_ID_CONTACT1, V_TRIPPED);
MyMessage msgContact2(CHILD_ID_CONTACT2, V_TRIPPED);

#ifdef MY_SHT2XTEMPERATURE
MyMessage msgSHT_TEMP(CHILD_ID_SHT2X_TEMP, V_TEMP);
MyMessage msgSHT_HUM(CHILD_ID_SHT2X_HUM, V_HUM);
MyMessage msgSHT_DEW(CHILD_ID_SHT2X_DEW, V_HUM);
#endif

#ifdef MyRSSI_Values
MyMessage msgRSSI_BAT(CHILD_ID_RSSI, V_VAR1);
MyMessage msgRSSI_TX(CHILD_ID_RSSI, V_VAR2);
MyMessage msgRSSI_RX(CHILD_ID_RSSI, V_VAR3);
MyMessage msgRSSI_TX_LEVEL(CHILD_ID_RSSI, V_VAR4);
MyMessage msgRSSI_TX_PERC(CHILD_ID_RSSI, V_VAR5);
#endif


const char *Scetch_Info = { "Relay Timer SAMD21 " };

#ifdef MY_DALLASTEMPERATURE
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
int numSensors = 0;
bool metric = true;
DeviceAddress tempDeviceAddress;
#define TEMPERATURE_PRECISION 12
struct{
	int id;
	DeviceAddress addr;
} T[MAX_ATTACHED_DS18B20];
#endif

uint32_t relay_delay[3];
uint8_t relayCountDown[3];
bool FirstRun = true;


int EEPROM_Write(int pos, char *zeichen, int size);
int EEPROM_Read(int pos, char *character, int size);
void printAddress(DeviceAddress deviceAddress);
void writeAddress(DeviceAddress deviceAddress, uint8_t index);
void readAddress(DeviceAddress deviceAddress, uint8_t index);
void setRelay(uint8_t RelayID, bool relaystate);
void setRelayDelay(uint8_t RelayID, uint8_t relaydelay);
void checkRelayTimer(void);
void setup(void);
//void loop(void);

//typedef 
struct EEProm_Struct {
	uint8_t MyNodeID;
	byte listDeviceAddress[8][4];
	bool Relais1;
	bool Relais2;
	bool Relais3;
	uint16_t Delay1;
	uint16_t Delay2;
	uint16_t Delay3;
} EE;


Adafruit_ZeroTimer zt3 = Adafruit_ZeroTimer(3);

void before()
{
	SerialUSB.begin(115200);// start serial port
	while (SerialUSB.available() > 0); // Wait for Serial monitor to open
	wait(1000);

	// Then set relay pins in output mode
	pinMode(RELAY_1, OUTPUT);
	pinMode(RELAY_2, OUTPUT);
	pinMode(RELAY_3, OUTPUT);
	// Set relay to last known state (using eeprom storage)
	if (!EEPROM.isValid()) // if Flash reprogrammed then request value from controller
	{
		SerialUSB.println("EEPROM is invalid");
		setRelay(RELAY_1, RELAY_OFF);
		setRelay(RELAY_2, RELAY_OFF);
		setRelay(RELAY_3, RELAY_OFF);
	}
	else {
		EEPROM_Read(0, reinterpret_cast<char*>(&EE), sizeof(EE));
		SerialUSB.println("EEPROM is ok");
		relay_delay[0] = EE.Delay1 * 1000;
		relay_delay[1] = EE.Delay2 * 1000;
		relay_delay[2] = EE.Delay3 * 1000;
		_transportConfig.nodeId = EE.MyNodeID; //work around of missing EEPROM give back the node ID of first negogtiating from FLASH

	}

	
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

	if (relay_delay[2] > 0) {
		digitalWrite(RELAY_3, RELAY_OFF);
	}
	else {
		digitalWrite(RELAY_3, EE.Relais3 ? RELAY_ON : RELAY_OFF);
	}
}

void setup()
{
	CONTACT1.init(contact1, &cb_contact1_true, &cb_contact1_false, false, LOW, 100);
	CONTACT2.init(contact2, &cb_contact2_true, &cb_contact2_false, false, LOW, 100);
	KEY_CLOSE.init(key_close, &cb_key_close_true, &cb_key_close_false, false, LOW, 50);
	KEY_OPEN.init(key_open, &cb_key_open_true, &cb_key_open_false, false, LOW, 50);
	KEY_AUX.init(key_aux, &cb_key_aux_true, &cb_key_aux_false, false, LOW, 50);
	
#ifdef MY_SHT2XTEMPERATURE
	Wire.begin();//start I2C
#endif

#ifdef MY_DALLASTEMPERATURE
	sensors.setWaitForConversion(true);
#endif
	
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
	present(CHILD_ID_RELAY_ON, S_BINARY, ("Relay On"));
	wait(1000);
	present(CHILD_ID_RELAY_OFF, S_BINARY, ("Relay Off"));
	wait(1000);
	present(CHILD_ID_RELAY_AUX, S_BINARY, ("Relay Aux"));
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
	// Present all sensors to controller
	for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
		// Search the wire for address
		if (sensors.getAddress(T[i].addr, i))
		{
			T[i].id = sensors.getUserData(T[i].addr);
			//sprintf(idbuffer, "%d", T[i].id);
			present(CHILD_ID_TEMP_SENSOR + i, S_TEMP, ("DEG C"), true);
			SerialUSB.print("device ");
			SerialUSB.print(T[i].id, DEC);
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
		wait(1000);
	}
#endif
	if (!EEPROM.isValid()) {
		EE.MyNodeID = getNodeId();
		EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE)); //save the NV structure to EEProm
		wait(1000);
	}
	SerialUSB.print("relaydelay 1 = ");
	SerialUSB.println(relay_delay[0]);
	SerialUSB.print("relaydelay 2 = ");
	SerialUSB.println(relay_delay[1]);
	SerialUSB.print("relaydelay 3 = ");
	SerialUSB.println(relay_delay[2]);
	SerialUSB.print("My_NODE_ID  = ");
	SerialUSB.println(EE.MyNodeID);
	
#ifdef MY_SHT2XTEMPERATURE
	present(CHILD_ID_SHT2X_TEMP, S_TEMP, "Temperature_SHT");
	wait(1000);
	present(CHILD_ID_SHT2X_HUM, S_HUM, "Humitidy_SHT");
	wait(1000);
	present(CHILD_ID_SHT2X_DEW, S_HUM, "Dew_Point_SHT");
	wait(1000);
#endif

	/********************* Timer #3, 16 bit, two PWM outs, period = 65535 */
	zt3.configure(
		TC_CLOCK_PRESCALER_DIV64,		// prescaler
		TC_COUNTER_SIZE_16BIT,			// bit width of timer/counter
		TC_WAVE_GENERATION_NORMAL_PWM	// frequency or PWM mode
	);

	zt3.setCompare(0, 0xFFFF / 4);
	zt3.setCompare(1, 0xFFFF / 2);
	zt3.setCallback(true, TC_CALLBACK_CC_CHANNEL0, Timer3Callback0);  
	zt3.setCallback(true, TC_CALLBACK_CC_CHANNEL1, Timer3Callback1);  
	zt3.enable(true);


}


void loop()
{
	float temperature;
	//int batteryPcnt = 100;//sensorValue / 10;
	//float batteryV = sensorValue * 0.003613281;

	DeviceAddress deviceAddress;


	if (mytimer.repeat()) {
		SerialUSB.println("Loop Timer Repeat");

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
			//readAddress(deviceAddress, i);
			//sensors.requestTemperaturesByAddress(deviceAddress);
			//temperature = sensors.getTempC(deviceAddress);
			temperature = sensors.getTempC(T[i].addr);
			// Send in the new temperature
			send(msgTemp.setSensor(CHILD_ID_TEMP_SENSOR + i).set(temperature, 2));
/*
		for (int id = 0; id < MAX_ATTACHED_DS18B20; id++)
		{
			temperature = sensors.getTempCByIndex(id);
			Serial.print("DS1820 id ");
			Serial.print(id);
			Serial.print("Temp C = ");
			Serial.println(temperature);
			send(msgTemp.setSensor(CHILD_ID_TEMP_SENSOR + id).set(temperature, 2));
*/		//}


#ifdef MY_DEBUG_PRINT
			printAddress(deviceAddress);
			SerialUSB.print(".i = ");
			SerialUSB.print(i);
			SerialUSB.print(". T = ");
			SerialUSB.println(temperature, 3);
#endif
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
		send(msgSHT_DEW.setSensor(CHILD_ID_SHT2X_DEW).set(SHT2x.GetDewPoint(), 2));
		SerialUSB.println(SHT2x.GetDewPoint());
#endif
		}
	send_MyMessage();
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
	case CHILD_ID_RELAY_ON:
		if (message.type == V_STATUS) {
			setRelay(RELAY_1, message.getBool());
			SerialUSB.print("RelayID1 = ");
			SerialUSB.println(message.sensor);
			};
		break;
	case CHILD_ID_RELAY_OFF:
		if (message.type == V_STATUS) {
			setRelay(RELAY_2, message.getBool());
			SerialUSB.print("RelayID2 = ");
			SerialUSB.println(message.sensor);
			};
		break;
	case CHILD_ID_RELAY_AUX:
		if (message.type == V_STATUS) {
			setRelay(RELAY_3, message.getBool());
			SerialUSB.print("RelayID3 = ");
			SerialUSB.println(message.sensor);
			};
		break;
	case CHILD_ID_RELAY_DELAY: {
		switch (message.type) {
		case V_VAR1: {
			setRelayDelay(RELAY_1, (uint8_t)message.getInt());
			MSG_State = KEY_OPEN_Off;//switch relay off if delay time is changed
			SerialUSB.print("RELAY1_DELAY = ");
			SerialUSB.println((uint8_t)message.getInt());
			}
		break;
		case  V_VAR2: {
			setRelayDelay(RELAY_2, (uint8_t)message.getInt());
			MSG_State = KEY_CLOSE_Off; //switch relay off if delay time is changed
			SerialUSB.print("RELAY2_DELAY = ");
			SerialUSB.println((uint8_t)message.getInt());
			}
		break;
		case  V_VAR3: {
			setRelayDelay(RELAY_3, (uint8_t)message.getInt());
			MSG_State = KEY_AUX_Off; //switch relay off if delay time is changed
			SerialUSB.print("RELAY3_DELAY = ");
			SerialUSB.println((uint8_t)message.getInt());
			}
		break;
		}
	}
		break;
	}
#ifdef MY_DEBUG_INCOMING
	SerialUSB.print("Incoming change for sensor:");
	SerialUSB.print(message.sensor);
	SerialUSB.print(", New status: ");
	SerialUSB.println(message.getBool());
#endif
}

float getTempByID(int id)
{
	for (uint8_t index = 0; index < MAX_ATTACHED_DS18B20; index++)
	{
		if (T[index].id == id)
		{
			return sensors.getTempC(T[index].addr);
		}
	}
	return -999;
}
void printAddress(DeviceAddress deviceAddress)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (deviceAddress[i] < 16) SerialUSB.print("0");
		SerialUSB.print(deviceAddress[i], HEX);
	}
}

void writeAddress(DeviceAddress deviceAddress, uint8_t index)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		EE.listDeviceAddress[i][index] = deviceAddress[i];
	}
}

void readAddress(DeviceAddress deviceAddress, uint8_t index)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		deviceAddress[i] = EE.listDeviceAddress[i][index];
	}
}

void setRelay(uint8_t RelayID, bool relaystate)
{
static bool relay_state[3] = { RELAY_OFF, RELAY_OFF };
	// Change relay state
	switch (RelayID) {
	case RELAY_1: {
		if (relay_delay[0] > 0) {
			relaytimer1.set(relay_delay[0]);
			if (relaystate) {
				relaytimer1.start();
			}
			//MSG_State = KEY_OPEN_On;
		}
		else {
			relay_state[0] = !relay_state[0];
			relaystate = relay_state[0];
			EE.Relais1 = relaystate;
			EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE));//store state on non monostable operation
		}
		if (relaystate) { 
			MSG_State = KEY_OPEN_On; 
		}
		else {
			MSG_State = KEY_OPEN_Off;
		}
	}   break;
	case RELAY_2: {
		if (relay_delay[1] > 0) {
			relaytimer2.set(relay_delay[1]);
			if (relaystate) {
				relaytimer2.start();
			}
		}
		else {
			relay_state[1] = !relay_state[1];
			relaystate = relay_state[1];
			EE.Relais2 = relaystate;
			EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE));//store state on non monostable operation
		}	
		if (relaystate) {
			MSG_State = KEY_CLOSE_On;
		}
		else {
			MSG_State = KEY_CLOSE_Off;
		}
	}	break;
	case RELAY_3: {
		if (relay_delay[2] > 0) {
			relaytimer3.set(relay_delay[2]);
			if (relaystate) {
				relaytimer3.start();
			}
		}
		else {
			relay_state[2] = !relay_state[2];
			relaystate = relay_state[2];
			EE.Relais3 = relaystate;
			EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE));//store state on non monostable operation
		}
		if (relaystate) {
			MSG_State = KEY_AUX_On;
		}
		else {
			MSG_State = KEY_AUX_Off;
		}
	}	break;
	}
	digitalWrite(RelayID, relaystate ? RELAY_ON : RELAY_OFF);
	SerialUSB.print("Relay State = ");
	SerialUSB.print(relaystate);
	SerialUSB.print(" RelayPinID = ");
	SerialUSB.println(RelayID);
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
	case RELAY_3: {
		relay_delay[2] = relaydelay * 1000;
		EE.Delay3 = relaydelay; //save delay to NV structure
		break;
		}
	}

	EEPROM_Write(0, reinterpret_cast<char*>(&EE), sizeof(EE)); //save the NV structure to EEProm

	SerialUSB.print("relaydelay 1 = ");
	SerialUSB.println(relay_delay[0]);
	SerialUSB.print("relaydelay 2 = ");
	SerialUSB.println(relay_delay[1]);
	SerialUSB.print("relaydelay 3 = ");
	SerialUSB.println(relay_delay[2]);
}

void checkRelayTimer() {
	if (relaytimer1.done()) {
		SerialUSB.println("Timer 1 done...");
		relaytimer1.stop();
		setRelay(RELAY_1, RELAY_OFF);
		MSG_State = KEY_OPEN_Off;
	}
	if (relaytimer2.done()) {
		SerialUSB.println("Timer 2 done...");
		relaytimer2.reset();
		setRelay(RELAY_2, RELAY_OFF);
		MSG_State = KEY_CLOSE_Off;
	}
	if (relaytimer3.done()) {
		SerialUSB.println("Timer 3 done...");
		relaytimer3.reset();
		setRelay(RELAY_3, RELAY_OFF);
		MSG_State = KEY_AUX_Off;
	}
}

// Write any data structure or variable to EEPROM
int EEPROM_Write(int pos, char *zeichen, int size)
{
	for (int i = 0; i < size; i++)
	{
		EEPROM.write(pos + i, *zeichen);
		//	  saveState(pos + i, *zeichen);
		zeichen++;
	}
	EEPROM.commit();
	SerialUSB.print("After commit, calling isValid() returns ");
	SerialUSB.println(EEPROM.isValid());
	return pos + size;
}

// Read any data structure or variable from EEPROM
int EEPROM_Read(int pos, char *character, int size)
{
	for (int i = 0; i < size; i++)
	{
		*character = EEPROM.read(pos + i);
		//	  *character = loadState(pos + i);
		character++;
	}
	return pos + size;
}


//define the interrupt handlers
void TC3_Handler() {
	Adafruit_ZeroTimer::timerHandler(3);
}
// the timer 3 callbacks
void Timer3Callback0()
{
	CONTACT1.handler();
	CONTACT2.handler();
	KEY_CLOSE.handler();
	KEY_OPEN.handler();
	KEY_AUX.handler();
}

void Timer3Callback1()
{
//	send_MyMessage();
//	SerialUSB.println("Callback 1 ");
}

void cb_contact1_false(SeqButton * button) {
	MSG_State = CONTACT1_Off;
	SerialUSB.println("Contact1 False ");
}

void cb_contact1_true(SeqButton * button) {
	MSG_State = CONTACT1_On;
	SerialUSB.println("Contact1 True ");
}

void cb_contact2_false(SeqButton * button) {
	MSG_State = CONTACT2_Off;
	SerialUSB.println("Contact2 False ");
}

void cb_contact2_true(SeqButton * button) {
	MSG_State = CONTACT2_On;
	SerialUSB.println("Contact2 True ");
}

void cb_key_open_false(SeqButton * button) {
	SerialUSB.println("KEY_OPEN False ");
}

void cb_key_open_true(SeqButton * button) {
	setRelay(RELAY_1, RELAY_ON);
	SerialUSB.println("KEY_OPEN True ");
}

void cb_key_close_false(SeqButton * button) {
	SerialUSB.println("KEY CLOSE False ");
}

void cb_key_close_true(SeqButton * button) {
	setRelay(RELAY_2, RELAY_ON);
	SerialUSB.println("KEY CLOSE True ");
}

void cb_key_aux_false(SeqButton* button) {
	SerialUSB.println("KEY CLOSE False ");
}

void cb_key_aux_true(SeqButton* button) {
	setRelay(RELAY_3, RELAY_ON);
	SerialUSB.println("KEY CLOSE True ");
}

void send_MyMessage(/*MSG_STATE MSG_State*/) {
	switch (MSG_State) {
	case NoCHANGE: break;
	case CONTACT1_On:	send(msgContact1.set(true)); break;
	case CONTACT1_Off:	send(msgContact1.set(false)); break;
	case CONTACT2_On:	send(msgContact2.set(true)); break;
	case CONTACT2_Off:	send(msgContact2.set(false)); break;
	case KEY_OPEN_On:	send(msgRelayOn.set(true)); break;
	case KEY_OPEN_Off:	send(msgRelayOn.set(false)); break;
	case KEY_CLOSE_On:	send(msgRelayOff.set(true)); break;
	case KEY_CLOSE_Off:	send(msgRelayOff.set(false)); break;
	case KEY_AUX_On:	send(msgRelayAux.set(true)); break;
	case KEY_AUX_Off:	send(msgRelayAux.set(false)); break;
	}
	MSG_State = NoCHANGE;
}
