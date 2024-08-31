#include <Arduino.h>
#include <ESP32_CAN.h>
#include <BSONPP.h>
#include "Can-Header-Map/CAN_datadb.h"
/*#include <WiFi.h>
#include <WiFiAP.h>
#include <WebServer.h>
#include <ESP2SOTA.h>*/


#define BSON_RPM "rpm"
#define BSON_VEHICLESPEED "vel"
#define BSON_MOTORTEMPERATURE "eng_t"
#define BSON_LV_BATTERYVOLTAGE "bat_v"
#define SIZE_OF_BSON 400
#define BSON_MENU "menu"
#define BSON_IGNITION "ignition"


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

  #define BSON_MENU "menu"
  #define BSON_SOC "soc"
  #define BSON_LV_SOC "lv_soc"
  #define BSON_BATTERYTEMPERATURE "bat_t"
  #define BSON_INVERTERTEMPERATURE "inv_t"
  #define BSON_POWER "pow"
  #define BSON_LAPCOUNT "lap_c"
  #define BSON_LAPTIME "lap_t"
  #define BSON_INVERTERVoltage "inv_v"
  #define BSON_HV_BATTERYVOLTAGE "hv_bat"
  #define BSON_MAX_CELL_TEMP "max_cell_temp"
  #define BSON_PAGE "page"
  #define BSON_POWER_LIMIT "power_available"
  #undef SIZE_OF_BSON


  #define BSON_VCU_OK "vcu_ok"
  #define BSON_TCU_OK "tcu_ok"
  #define BSON_DATALOGGER_OK "datalogger_ok"
  #define BSON_ACU_OK "acu_ok"
  #define BSON_DYNAMICS_R_OK "dynamics_r_ok"
  #define BSON_DYNAMICS_F_OK "dynamics_f_ok"
  #define BSON_BRAKE_LIGHT_OK "brake_light_ok"
  #define BSON_ALC_OK "alc_ok"
  #define BSON_INVERTER_OK "inverter_ok"
  #define BSON_MAX_CELL_TEMP "max_cell_temp"
  #define BSON_MIN_CELL_TEMP "min_cell_temp"
  #define BSON_MIN_VOLTAGE "min_cell_v"
  #define BSON_MAX_VOLTAGE "max_cell_v"
  #define BSON_POP_UP_COUNT "pop_up_count"
  #define BSON_POP_UP_HISTORY "pop_up_hist"
  #define BSON_POP_UP_ENABLE "pop_up_en"
/*
	**********************************
	*******	Valores Calibração *******
	**********************************
*/
#define BSON_ASK_CALIBRATION_VALUES "calib"
#define BSON_SUSPENSAO_TRASEIRA_L "suspencao_t_l"
#define BSON_SUSPENSAO_TRASEIRA_R "suspencao_t_r"
#define BSON_SUSPENSAO_DIANTEIRA_L "suspencao_d_l"
#define BSON_SUSPENSAO_DIANTEIRA_R "suspencao_d_r"


 
 // #define SIZE_OF_BSON 128
 #define SIZE_OF_BSON 1000

  
#endif 


#define ADC_LEVELS 4095
#define ROTARY_1 13
#define ROTARY_2 14

#define L_BTN_1 23
#define L_BTN_2 33
#define R_BTN_1 4
#define R_BTN_2 18


#define LED1 32
#define LED2 25
#define LED3 26


int vcu_ok = 0,tcu_ok = 0,datalogger_ok = 0,acu_ok = 0,dynamics_r_ok = 0,dynamics_f_ok = 0,brake_light_ok = 0,alc_ok = 0;



typedef union  {
  int32_t encodedValue;
  float decodedValue;
}EncodingUnion;


uint32_t _millis =0; 
uint32_t _millis_target=0;
uint32_t period=25,modules_period = 1000;
char *bsonW="\xFF\xFF\xFF\xFF";


unsigned long timeout = 0;
unsigned long modules_timeout = 0;
int menu = 0;

void loop()
{
	
}

void IRAM_ATTR L_BTN1_ISR();
void IRAM_ATTR L_BTN2_ISR();
void IRAM_ATTR R_BTN1_ISR();
void IRAM_ATTR R_BTN2_ISR();

int ask_software_c_values = 0;
int btn_calibration_values = 0;
int pop_up_visible = 0;
int pop_up_counter = 0;
int pop_up_history = 0;



TWAI_Interface CAN1(1000, 21, 22); // argument 1 - BaudRate,  argument 2 - CAN_TX PIN,  argument 3 - CAN_RX PIN




void setup (void) {
	int power_level = 0;
	int test_count = 0;
	pinMode(LED1,OUTPUT);
	pinMode(LED2,OUTPUT);
	pinMode(LED3,OUTPUT);
	pinMode(ROTARY_1,INPUT);
	pinMode(ROTARY_2,INPUT);
	pinMode(R_BTN_1,INPUT);
	pinMode(R_BTN_2,INPUT);
	pinMode(L_BTN_1,INPUT);
	pinMode(L_BTN_2,INPUT);
	uint16_t rpm=0;
	uint16_t motor_temperature=0;
	float inverter_temperature=0;
	float mean_battery_temperature=0;
	float inverter_voltage = 0;
	float lv_bat_v = 0;
	float hv_bat_v = 0;
	short int power = 0;
	float power_available = 0;
	float hv_soc = 0, lv_soc = 0;
	float suspension_b_l = 0;
	float suspension_b_r = 0;
	float suspension_f_l = 0;
	float suspension_f_r = 0;
	uint8_t power_limit = 0;
	uint8_t buffer[SIZE_OF_BSON];
	uint8_t buffer_calibration[128];
	uint8_t calib_size[4];

	BSONPP bson(buffer, sizeof(buffer));
	BSONPP bson_calib(buffer_calibration, sizeof(buffer_calibration));
	// Setup serial port
	//8 bit, Odd parity and 1 bit for stop


	Serial2.begin(115200);
	Serial.begin(115200);


	
	/***	-Atach interrupts to push buttons-	****/
	attachInterrupt(L_BTN_1,L_BTN1_ISR,FALLING);
	attachInterrupt(L_BTN_2,L_BTN2_ISR,FALLING);
	attachInterrupt(R_BTN_1,R_BTN1_ISR,FALLING);
	attachInterrupt(R_BTN_2,R_BTN2_ISR,FALLING);
	


	timeout = millis();
	int front_speed = 0,rear_speed = 0;
	while (1){
		
		
		uint8_t msg[8] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
		 int R1_menu = -1,R2_menu = -1;

		/***	-Read values for interface- 	****/

		float adc_value = analogRead(ROTARY_1);
		
		for(int i = 6, menu = 0;R1_menu == -1 || i == 0; i--){
			menu++;
			if(adc_value <= ADC_LEVELS/i){
				R1_menu = menu; 
			}
		}

		adc_value = analogRead(ROTARY_2);
		
		float volatage = adc_value*3.3/4095;
		//power_available = volatage*100/3.3;


		if(volatage <= 0.5){
			power_available = 17;
			power_level = 0;
		}
		else{
			if(volatage <= 0.63)
			{
				power_available = 35;
				power_level = 1;
			}
			else{
				if(volatage <= 0.8){
					power_available = 50;
					power_level = 2;
				}
				else{
					if(volatage <= 1.3){
						power_available = 65;
						power_level = 3;
					}
					else{
						if(volatage <= 2)
						{
							power_available = 85;
							power_level = 4;
						}
						else{
							power_available = 100;
							power_level = 5;
						}
					}
				}
			}
		}



		_millis=millis();
		int _id= CAN1.RXpacketBegin(); 
		//int _id= -1; 
		int  packetSize = CAN1.RXgetDLC();
		//int  packetSize = 0;
		
		if (_id != -1) {
				for(int i = 0; i<= packetSize; i++){
					msg[i] = CAN1.RXpacketRead(i);
				}
				
				digitalWrite(LED2,!digitalRead(LED2));
				
				if(msg==nullptr){
					//Serial.println("Error reading 
					continue;
				}
				switch (_id)
				{
				case CAN_VCU_ID_1:
					/* code */
					break;
				case CAN_VCU_ID_2:
					motor_temperature = MAP_DECODE_MOTOR_TEMPERATURE(msg);
					inverter_temperature = MAP_DECODE_INVERTER_TEMPERATURE(msg);
					hv_soc = MAP_DECODE_HV_SOC(msg);
					hv_bat_v = MAP_DECODE_HV_VOLTAGE(msg);
					Serial.println(hv_bat_v);
					break;
				case CAN_VCU_ID_3:
					vcu_ok = MAP_DECODE_VCU_STATE(msg);
					power_limit = MAP_DECODE_POWER_PLAN(msg);
					break;
				case CAN_VCU_ID_4:
					inverter_voltage = MAP_DECODE_INVERTER_VOLTAGE(msg);
					rpm = MAP_DECODE_RPM(msg);
					break;
				case CAN_VCU_ID_5:
				lv_soc = MAP_DECODE_LV_SOC(msg);
				break;
				case CAN_DYNAMICS_REAR_ID_1: //rear_speed
					rear_speed = MAP_DECODE_WHEEL_SPEED_RL(msg) + MAP_DECODE_WHEEL_SPEED_RR(msg);
					rear_speed = rear_speed/2;
					dynamics_r_ok = 1;
					break;
				case CAN_DYNAMICS_FRONT_ID_1: //rear_speed
					front_speed = MAP_DECODE_WHEEL_SPEED_FR(msg) + MAP_DECODE_WHEEL_SPEED_FL(msg);
					front_speed = front_speed/2;
					dynamics_f_ok = 1;
					break;
				break;
				case CAN_BRAKE_LIGHT:
					brake_light_ok = 1;
					break;
				default:
					continue;
				}
		}
		if(menu>=2)
		{
			menu = 0;
		}
		int speed = rear_speed + front_speed;
		speed = speed/2;
		inverter_voltage = 150;
		power = 100;
		R1_menu = 0;
		if(_millis-_millis_target>=period){
			bson.clear(); 
			bson.append(BSON_RPM, (int32_t)rpm);
			bson.append(BSON_LV_BATTERYVOLTAGE, (int32_t)lv_bat_v); //float 
			bson.append(BSON_MOTORTEMPERATURE, (int32_t) motor_temperature);
			
			bson.append(BSON_VEHICLESPEED,(int32_t)speed);
			bson.append(BSON_MENU,(int32_t)menu);
		#ifdef __LART_T14__
			bson.append(BSON_AFR, (int32_t)rpm);
			bson.append(BSON_GEARSHIFT, (int32_t) rpm);
			bson.append(BSON_DATALOGGERSTATUS, (int32_t) rpm);
			bson.append(BSON_TCSLIP, (int32_t) rpm);
			bson.append(BSON_TCLAUNCH, (int32_t) rpm);\x
			bson.append(BSON_OILTEMPERATURE,  (int32_t)rpm); //float
			bson.append(BSON_OILPRESSURE,  (int32_t)rpm); //float
		#endif
		#ifdef __LART_T24__
			bson.append(BSON_INVERTERTEMPERATURE, (int32_t)inverter_temperature);
			bson.append(BSON_POWER, (int16_t)power);//int16_t
			EncodingUnion packet;
			packet.decodedValue=hv_soc;
			bson.append(BSON_SOC, (int32_t)packet.encodedValue); 
			
			bson.append(BSON_INVERTERVoltage, (int32_t)inverter_voltage); 
			//
			bson.append(BSON_LAPCOUNT , (int32_t)rpm);//int16_t
			bson.append(BSON_LAPTIME,(int32_t)_millis);
			
			//EncodingUnion packet;
			packet.decodedValue=hv_bat_v;
			bson.append(BSON_HV_BATTERYVOLTAGE, (int32_t)packet.encodedValue); //float 
			bson.append(BSON_POWER, (int16_t)power); //float
			bson.append(BSON_LAPCOUNT, (int32_t)R1_menu);
			bson.append(BSON_MOTORTEMPERATURE,(int32_t)motor_temperature);
			//packet.decodedValue = power_available;
			bson.append(BSON_POWER_LIMIT,(int)power_limit);
			bson.append(BSON_VCU_OK,(int32_t)vcu_ok);
			bson.append(BSON_TCU_OK,(int32_t)tcu_ok);
			bson.append(BSON_DATALOGGER_OK,(int32_t)datalogger_ok);
			bson.append(BSON_ACU_OK,(int32_t)acu_ok);
			bson.append(BSON_DYNAMICS_R_OK,(int32_t)dynamics_r_ok);
			bson.append(BSON_DYNAMICS_F_OK,(int32_t)dynamics_f_ok);
			bson.append(BSON_BRAKE_LIGHT_OK,(int32_t)brake_light_ok);
			bson.append(BSON_ALC_OK,(int32_t)alc_ok);
			#endif
					

			
			
			
			_millis_target=_millis;
			
			digitalWrite(LED1,!digitalRead(LED1));
		}
		if(millis() >= modules_timeout+modules_period){
		vcu_ok = 0;
		tcu_ok = 0;
		datalogger_ok = 0;
		acu_ok = 0;
		dynamics_r_ok = 0;
		dynamics_f_ok = 0;
		brake_light_ok = 0;
		alc_ok = 0;
		CAN1.TXpacketBegin(0x254,0);
    	CAN1.TXpacketLoad(power_level);
		if(btn_calibration_values > 0){
		CAN1.TXpacketLoad(1);
		btn_calibration_values = 0;
		}
	else{
		CAN1.TXpacketLoad(0);
	}
    	CAN1.TXpackettransmit();
		modules_timeout = millis();
		digitalWrite(LED3,!digitalRead(LED3));
	}


	if(ask_software_c_values){
		bson.append(BSON_ASK_CALIBRATION_VALUES,true);
	}
	else{
		bson.append(BSON_ASK_CALIBRATION_VALUES,false);
	}

	if(Serial2.available() > 0){
		bson_calib.clear(); 
		int32_t size_of_string = 0;
		int8_t value1;
		Serial2.read(buffer_calibration,4);
		if(memcmp(buffer_calibration,"\xFF\xFF\xFF\xFF",4) == 0){
			Serial2.read(buffer_calibration,sizeof(byte)*8);
			for(int i = 0; i<4;i++){
				calib_size[i] = buffer_calibration[i]; 
				size_of_string <<= 8;  // Shift left by 8 bits
				size_of_string |= buffer_calibration[i];  
			}
			EncodingUnion packet;
			Serial2.read(buffer_calibration,size_of_string);
			if(bson_calib.get(BSON_SUSPENSAO_DIANTEIRA_L, (int32_t *) &(packet.encodedValue)) < 0){
				suspension_f_l = packet.decodedValue;
			}
			else{
				Serial.println("Failed to get Front Left Suspencion");
			}
			if(bson_calib.get(BSON_SUSPENSAO_DIANTEIRA_R, (int32_t *) &(packet.encodedValue)) < 0){
				suspension_f_r = packet.decodedValue;
			}
			else{
				Serial.println("Failed to get Front Right Suspencion");
			}
			if(bson_calib.get(BSON_SUSPENSAO_TRASEIRA_L, (int32_t *) &(packet.encodedValue)) < 0){
				suspension_b_l = packet.decodedValue;
			}
			else{
				Serial.println("Failed to get Back Left Suspencion");
			}
			if(bson_calib.get(BSON_SUSPENSAO_TRASEIRA_R, (int32_t *) &(packet.encodedValue)) < 0){
				suspension_b_r = packet.decodedValue;
			}
			else{
				Serial.println("Failed to get Back Right Suspencion");
			}
			if(bson_calib.get(BSON_POP_UP_COUNT,(int32_t* ) &pop_up_counter)>= 0){
				Serial.println("Failed to get POP UP count");
			}
			else{
				pop_up_history = pop_up_counter;
			}
		float suspension_b_l = 0;
		float suspension_b_r = 0;
		float suspension_f_l = 0;
		float suspension_f_r = 0;
		if (pop_up_visible != 0){
			bson.append(BSON_POP_UP_HISTORY,(int32_t) pop_up_history);
			bson.append(BSON_POP_UP_ENABLE,(int32_t) pop_up_visible);
		}
	}


}

	
	Serial2.write(bsonW);
	Serial2.write(bson.getBuffer(), bson.getSize());
	
	}
}

	



void IRAM_ATTR L_BTN1_ISR(){
	pop_up_visible++;
}

void IRAM_ATTR L_BTN2_ISR(){
	menu++;
}

void IRAM_ATTR R_BTN1_ISR(){
	ask_software_c_values++;
}

void IRAM_ATTR R_BTN2_ISR(){
	pop_up_history--;
	if(pop_up_history < 0){
		pop_up_history = 0;
	}
}

