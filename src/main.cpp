#include <SoftwareSerial.h>
#include <SPI.h>
#include <CAN.h>
#include <BSONPP.h>
#include "CAN_db.h"
//#define __LART_T24__
#define PB1_TX_13 9
#define PB0_RX_12 8



#define BSON_RPM "rpm"
#define BSON_VEHICLESPEED "vel"
#define BSON_ENGINETEMPERATURE "eng_t"
#define BSON_BATTERYVOLTAGE "bat_v"
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
 
  #undef SIZE_OF_BSON
 
  #define SIZE_OF_BSON 128

  
#endif 

SoftwareSerial mySerial(PB0_RX_12,PB1_TX_13);

typedef union  {
  int32_t encodedValue;
  float decodedValue;
}EncodingUnion;
uint32_t _millis =0; 
uint32_t _millis_target=0;
uint32_t period=100;
String bsonW="\xFF\xFF\xFF\xFF";



void setup (void) {
	uint16_t rpm=0;
	uint16_t motor_temperature=0;
	uint16_t inverter_temperature=0;

	uint8_t buffer[SIZE_OF_BSON];
	BSONPP bson(buffer, sizeof(buffer));
	// Setup serial port
	//8 bit, Odd parity and 1 bit for stop
	Serial.begin(115200);
	
	mySerial.begin(115200);
	//Serial begin on hardware TX and RX for an arduino nano
	if (!CAN.begin(1000E3)) {
		Serial.println("Starting CAN failed!");
		//while (1);
	}
	while (1){
		_millis=millis();
		
		int parsedPacketSize = CAN.parsePacket();
		//TODO check if this is the same as packetSize
		(void) parsedPacketSize;
		int _id= CAN.packetId(); 

		int packetSize = CAN.packetDlc();
		char msg[8];
		if (_id != -1) {
				//Read msg in one swoop
				memcpy(msg, CAN.peek(), packetSize);
				
				/*for (i = 0; i < (size_t)packetSize; i++){
					msg[i]=CAN.read();
				}*/
				
				// only print packet data for non-RTR packets
				
				switch (_id){
					#ifdef __LART_T24__
						case CAN_TEMPS_DRIVE:
							rpm=MAP_RPM(msg);
							motor_temperature=MAP_MOTOR_TEMPERATURE(msg);
							inverter_temperature=MAP_INVERTER_TEMPERATURE(msg);
							break;
					#endif		
				}
				
		}
		
		
		
		if(_millis-_millis_target>=period){
			bson.clear(); 
			bson.append(BSON_RPM, (int32_t)rpm);
			bson.append(BSON_BATTERYVOLTAGE, (int32_t)rpm); //float 
			bson.append(BSON_ENGINETEMPERATURE, (int32_t) rpm);
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
			bson.append(BSON_BATTERYTEMPERATURE, (int32_t)rpm);
			bson.append(BSON_INVERTERTEMPERATURE, (int32_t)rpm); 
			bson.append(BSON_POWER, (int32_t)rpm);//int16_t
			bson.append(BSON_LAPCOUNT , (int32_t)rpm);//int16_t
			bson.append(BSON_LAPTIME,(int32_t)_millis);
		#endif
			
		
			mySerial.print(bsonW);
			mySerial.write(bson.getBuffer(), bson.getSize());
			_millis_target=_millis;

		}	
	}
}

	

		
