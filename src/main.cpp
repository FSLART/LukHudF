#include <ardubsonStreamParser.h>
#include <ardubsonDocument.h>
#include <ardubsonObject.h>
#include <ardubson.h>
#include <ardubsonConfig.h>
#include <ardubsonTypes.h>
#include <IEEE754tools.h>
#include <ardubsonObjBuilder.h>
#include <ardubsonElement.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include <CAN.h>


#define BSON_RPM "rpm"
#define CAN_RPM 0x21 
#define BSON_VEHICLESPEED "vel"
#define BSON_ENGINETEMPERATURE "eng_t"
#define BSON_BATTERYVOLTAGE "bat_v"
#define PB1_TX_13 9
#define PB0_RX_12 8
#define __LART_T24__

#ifdef __LART_T14__
  #define BSON_GEARSHIFT "gear"
  #define BSON_OILPRESSURE "oil_p"
  #define BSON_OILTEMPERATURE "oil_t"
  #define BSON_DATALOGGERSTATUS "dtl_s"
  #define BSON_AFR "af_r"
  #define BSON_TCSLIP "tc_s"
  #define BSON_TCLAUNCH "tc_l"
#endif
#ifdef __LART_T24__
  #define BSON_SOC "soc"
  #define BSON_BATTERYTEMPERATURE "bat_t"
  #define BSON_INVERTERTEMPERATURE "inv_t"
  #define BSON_POWER "pow"
  #define BSON_LAPCOUNT "lap_c"
  #define BSON_LAPTIME "lap_t"
  #define CAN_MOTOR_TEMPERATURE 0x21
  #define CAN_INVERTER_TEMP 0x21
  #define CAN_INSTANT_POWER 0x20
#endif 

unsigned long millist=0;
unsigned long milliss=0;

typedef union  {
  int32_t encodedValue;
  float decodedValue;
}EncodingUnion;
//Todo
SoftwareSerial mySerial(PB1_TX_13, PB0_RX_12);
EncodingUnion g_OilPressure;
String bsonW="\xFF\xFF\xFF\xFF";
int period=500;
BSONObjBuilder bob;

int32_t timeSinceStart = 0; 
void setup() {
	// Setup serial port
	//8 bit, Odd parity and 1 bit for stop
	Serial.begin(115200,SERIAL_8O1);
	mySerial.begin(115200);
	//Serial begin on hardware TX and RX for an arduino nano

    
	  //delay(2000);
    bob.append(BSON_RPM, (int32_t)0);
    bob.append(BSON_BATTERYVOLTAGE, (int32_t)0); //float 
    bob.append(BSON_ENGINETEMPERATURE, (int32_t) 0);
    bob.append(BSON_VEHICLESPEED, (int32_t) 0);
    
  #ifdef __LART_T14__
    bob.append(BSON_AFR, (int32_t)0);
    bob.append(BSON_GEARSHIFT, (int32_t) 0);
    bob.append(BSON_DATALOGGERSTATUS, (int32_t) 0);
    bob.append(BSON_TCSLIP, (int32_t) 0);
    bob.append(BSON_TCLAUNCH, (int32_t) 0);
    bob.append(BSON_OILTEMPERATURE,  (int32_t)0); //float
    bob.append(BSON_OILPRESSURE,  (int32_t)0); //float
  #endif
  #ifdef __LART_T24__
  	bob.append(BSON_SOC, (int32_t)0); 
    bob.append(BSON_BATTERYTEMPERATURE,(int32_t) 0);
    bob.append(BSON_INVERTERTEMPERATURE ,(int32_t) 0); 
    bob.append(BSON_POWER , (int16_t)0);
    bob.append(BSON_LAPCOUNT, (int16_t)0);
    bob.append(BSON_LAPTIME,(int32_t)timeSinceStart);
  #endif
	//while(1);
	if (!CAN.begin(1000E3)) {
		Serial.println("Starting CAN failed!");
		while (1);
	}
	delay(100);
}
void loop(){
	int32_t rpm; 

	timeSinceStart=millis();
	
	/*BSONObject bo =bob.obj();
	int a =  bo.len();*/
	int parsedPacketSize = CAN.parsePacket();
	int _id= CAN.packetId(); 

	int packetSize = CAN.packetDlc();
	char msg[8];
	if (_id != -1) {
			size_t i;
			for (i = 0; i < packetSize; i++){
				msg[i]=CAN.read();
			}
			
			// only print packet data for non-RTR packets
			
			switch (_id){
				#ifdef __LART_T24__
					case 0x21:
						rpm=msg[1];
						Serial.println(rpm);
						break;
				#endif		
			}
			
	}
	
	
	millist=millis();
	if(millist-milliss>=period){
	
		bo.updateField(BSON_RPM, (int32_t)rpm);
		bo.updateField(BSON_BATTERYVOLTAGE, (int32_t)rpm); //float 
		bo.updateField(BSON_ENGINETEMPERATURE, (int32_t) rpm);
		bo.updateField(BSON_VEHICLESPEED, (int32_t) rpm);
		
	#ifdef __LART_T14__
		bo.updateField(BSON_AFR, (int32_t)rpm);
		bo.updateField(BSON_GEARSHIFT, (int32_t) rpm);
		bo.updateField(BSON_DATALOGGERSTATUS, (int32_t) rpm);
		bo.updateField(BSON_TCSLIP, (int32_t) rpm);
		bo.updateField(BSON_TCLAUNCH, (int32_t) rpm);
		bo.updateField(BSON_OILTEMPERATURE,  (int32_t)rpm); //float
		bo.updateField(BSON_OILPRESSURE,  (int32_t)rpm); //float
	#endif
	#ifdef __LART_T24__
		bo.updateField(BSON_SOC, (int32_t)rpm); 
		bo.updateField(BSON_BATTERYTEMPERATURE, (int32_t)rpm);
		bo.updateField(BSON_INVERTERTEMPERATURE, (int32_t)rpm); 
		bo.updateField(BSON_POWER, (int16_t)rpm);
		bo.updateField(BSON_LAPCOUNT , (int16_t)rpm);
		bo.updateField(BSON_LAPTIME,(int32_t)timeSinceStart);
	#endif
		
	  
		mySerial.print(bsonW);
		mySerial.write(bo.rawData(), a);
		milliss=millist;
	}	

		
}
