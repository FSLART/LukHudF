#include <Arduino.h>
#include <ESP32_CAN.h>
#include <BSONPP.h>
#include "Can-Header-Map/CAN_datadb.h"
#include <WiFi.h>
#include <ESP2SOTA.h>
#include <WebServer.h>



#define PB1_TX_13 9
#define PB0_RX_12 8



#define BSON_RPM "rpm"
#define BSON_VEHICLESPEED "vel"
#define BSON_MOTORTEMPERATURE "mot_t"
#define BSON_LV_BATTERYVOLTAGE "lv_bat_v"
#define SIZE_OF_BSON 40


#ifdef __LART_T14__
  #define BSON_GEARSHIFT "gear"
  #define BSON_OILPRESSURE "oil_p"
  #define BSON_OILTEMPERATURE "oil_t"
  #define BSON_DATALOGGERSTATUS "dtl_s"
  #define BSON_AFR "af_r"
  #define BSON_TCSLIP "tc_s"
  #define BSON_TCLAUNCH "tc_l"
  #undef SIZE_OF_BSON
  #define SIZE_OF_BSON 128
#endif

#ifdef __LART_T24__

  #define BSON_SOC "soc"
  #define BSON_BATTERYTEMPERATURE "bat_t"
  #define BSON_INVERTERTEMPERATURE "inv_t"
  #define BSON_POWER "pow"
  #define BSON_LAPCOUNT "lap_c"
  #define BSON_LAPTIME "lap_t"
  #define BSON_INVERTERVoltage "inv_v"
  #define BSON_HV_BATTERYVOLTAGE "hv_bat_v"
 
  #undef SIZE_OF_BSON
 
  #define SIZE_OF_BSON 128

  
#endif 



typedef union  {
  int32_t encodedValue;
  float decodedValue;
}EncodingUnion;
uint32_t _millis =0; 
uint32_t _millis_target=0;
uint32_t period=100;
char *bsonW="\xFF\xFF\xFF\xFF";

void loop()
{
	
}

TWAI_Interface CAN1(1000, 21, 22); // argument 1 - BaudRate,  argument 2 - CAN_TX PIN,  argument 3 - CAN_RX PIN


const char* ssid = "Volante";
const char* password = "volante2024";
WebServer server(80);

void setup (void) {
	uint32_t power=0;
	uint16_t rpm=0;
	uint16_t motor_temperature=0;
	uint16_t inverter_temperature=0;
	uint16_t mean_battery_temperature=0;
	uint16_t inverter_voltage = 0;

	uint8_t buffer[SIZE_OF_BSON];
	BSONPP bson(buffer, sizeof(buffer));
	// Setup serial port
	//8 bit, Odd parity and 1 bit for stop
	Serial1.begin(115200);
	
	WiFi.mode(WIFI_AP);  
	WiFi.softAP(ssid, password);
	IPAddress IP = IPAddress (06, 14, 22, 24);
	IPAddress NMask = IPAddress (255, 255, 255, 0);
	IPAddress myIP = WiFi.softAPIP();
	WiFi.softAPConfig(IP, IP, NMask);

	ESP2SOTA.begin(&server);
	server.begin();
	
	while (1){
		server.handleClient();
	uint8_t msg[8] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
		_millis=millis();
		
		//int parsedPacketSize = CAN.parsePacket();
		//TODO check if this is the same as packetSize
		//(void) parsedPacketSize;

		int _id= CAN1.RXpacketBegin(); 

		int  packetSize = CAN1.RXgetDLC();
		
		if (_id != -1) {
				for(int i = 0; i<= packetSize; i++){
					msg[i] = CAN1.RXpacketRead(i);
				}
				
				
				if(msg==nullptr){
					Serial.println("Error reading CAN packet");
					continue;
				}
				switch (_id)
				{
				case CAN_VCU_ID_1:
					/* code */
					break;
				case CAN_VCU_ID_2:
					mean_battery_temperature = MAP_DECODE_MOTOR_TEMPERATURE(msg);
					inverter_temperature = MAP_DECODE_INVERTER_TEMPERATURE(msg);
					break;
				case CAN_VCU_ID_3:
					/* code */
					break;
				case CAN_VCU_ID_4:
					inverter_temperature = MAP_DECODE_INVERTER_VOLTAGE(msg);
					rpm = MAP_DECODE_RPM(msg);
					break;
				default:
					continue;
				}
		}

		if(_millis-_millis_target>=period){
			bson.clear(); 
			bson.append(BSON_RPM, (int32_t)rpm);
			bson.append(BSON_LV_BATTERYVOLTAGE, (int32_t)rpm); //float 
			bson.append(BSON_MOTORTEMPERATURE, (int32_t) motor_temperature);
			bson.append(BSON_VEHICLESPEED, (int32_t) rpm);
		#ifdef __LART_T14__
			bson.append(BSON_AFR, (int32_t)rpm);
			bson.append(BSON_GEARSHIFT, (int32_t) rpm);
			bson.append(BSON_DATALOGGERSTATUS, (int32_t) rpm);
			bson.append(BSON_TCSLIP, (int32_t) rpm);
			bson.append(BSON_TCLAUNCH, (int32_t) rpm);
			bson.append(BSON_OILTEMPERATURE,  (int32_t)rpm); //float
			bson.append(BSON_OILPRESSURE,  (int32_t)rpm); //float
		#endif
		#ifdef __LART_T24__
			bson.append(BSON_SOC, (int32_t)rpm); 
			bson.append(BSON_BATTERYTEMPERATURE, (int32_t)mean_battery_temperature);
			bson.append(BSON_INVERTERVoltage, (int32_t)inverter_voltage); 
			bson.append(BSON_POWER, (int32_t)power);//int16_t
			bson.append(BSON_LAPCOUNT , (int32_t)rpm);//int16_t
			bson.append(BSON_LAPTIME,(int32_t)_millis);
			bson.append(BSON_HV_BATTERYVOLTAGE, (int32_t)rpm); //float 
		#endif
			
		
			
			Serial1.write(bsonW);
			Serial1.write(bson.getBuffer(), bson.getSize());
			_millis_target=_millis;

		}	
	}
}

	


