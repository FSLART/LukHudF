#include <SoftwareSerial.h>
#include <SPI.h>
#include <CAN.h>
#include <BSONPP.h>
#define __LART_T24__
#define PB1_TX_13 9
#define PB0_RX_12 8

#define CAN_RPM 0x21 


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
  #define GPIO_SERIAL_PARITY GPIO_SERIAL_PARITY_ODD
  #define GPIO_SERIAL_STOP_BITS 1

  #define CAN_MOTOR_TEMPERATURE 0x21
  #define CAN_INVERTER_TEMP 0x21
  #define CAN_INSTANT_POWER 0x20

  #define BSON_SOC "soc"
  #define BSON_BATTERYTEMPERATURE "bat_t"
  #define BSON_INVERTERTEMPERATURE "inv_t"
  #define BSON_POWER "pow"
  #define BSON_LAPCOUNT "lap_c"
  #define BSON_LAPTIME "lap_t"
 
  #undef SIZE_OF_BSON
 
  #define SIZE_OF_BSON 128

  
#endif 

SoftwareSerial mySerial(PB1_TX_13, PB0_RX_12);

typedef union  {
  int32_t encodedValue;
  float decodedValue;
}EncodingUnion;
uint32_t _millis =0; 
uint32_t _millis_target=0;
uint32_t period=100;
EncodingUnion g_OilPressure;
String bsonW="\xFF\xFF\xFF\xFF";



void setup (void) {
	
	uint8_t buffer[SIZE_OF_BSON];
	BSONPP bson(buffer, sizeof(buffer));
	// Setup serial port
	//8 bit, Odd parity and 1 bit for stop
	Serial.begin(115200);
	
	mySerial.begin(115200);
	//Serial begin on hardware TX and RX for an arduino nano
	if (!CAN.begin(1000E3)) {
		Serial.println("Starting CAN failed!");
		while (1);
	}
	while (1){
		_millis=millis();
		int32_t rpm; 

		
		int parsedPacketSize = CAN.parsePacket();
		//TODO check if this is the same as packetSize
		(void) parsedPacketSize;
		int _id= CAN.packetId(); 

		int packetSize = CAN.packetDlc();
		char msg[8];
		if (_id != -1) {
				size_t i;
				for (i = 0; i < (size_t)packetSize; i++){
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
		
		
		
		if(_millis-_millis_target>=period){
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

	

		
