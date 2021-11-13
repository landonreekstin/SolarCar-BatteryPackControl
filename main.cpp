// Author: Landon Reekstin
// Contributors: Delwys Glokpor, Brian Kamusinga

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <canAddresses.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // can1 port


IntervalTimer timer;

// Pins
#define CHGP_STAT     2
#define CHGP          3
#define RDP_STAT      4
#define RDP           5
#define DSG_E_STATp   6
#define MC_DSG_E      7   // Discharge enable
#define DSG_E_STATm   8
#define CHG_E         9
#define MC_CHG_E      10  // Charge enable
#define PUMP_PWM      11
#define FAN0_PWM      12
#define D1_LED        15
#define D2_LED        16
#define D3_LED        17
#define CHG_SENSE     24
#define RDP_SENSE     25
#define POWER_SENSE   26
#define MC_MP_E       27
#define PUMP_PWR_STAT 28
#define MC_MP_E_STAT  29
#define PUMP_PWR      30
#define CHG_E_STATp   31
#define MP_E          32
#define FAN4TACH      33
#define FAN3TACH      34
#define FAN2TACH      35
#define FAN2_PWM      36
#define FAN1_PWM      37
#define FAN1TACH      38
#define PUMP_FAN_TACH 39
#define COOLANT_TEMP  40
#define PUMP_TACH     41


//CURRENT LIMITS
int CHARGECURRENT_LIMIT = 45 ;
int DISCHARGE_CURRENT_LIMIT = 65;
int CHARGE_TEMP_LIMIT = 45;
int DISCHARGE_TEMP_LIMIT = 60;


union fVals
{
  byte b[4];
  float fVal;
}  BusCurrent, busVolt, busCur, dcCommand, mcVolt, mcRPM, mc1RPM;

union inVals
{
  byte b[2];
  int inVal;
} BATDISCurrent, BATCHGCurrent, ChargerCurrent, ChargerVoltage;


// Global variables
// battery pack data 1
int packVolt, packCur, packAHr;

// battery pack data 2
byte relayStatus;
int dischargeRelay, chargeRelay, batSOC;

// battery pack data 3
int H_Volt, L_Volt, A_Volt, Delta_Volt;
#define CELL_MULTI_VOLT .0001

//battery pack data 4
int H_Temp, L_Temp, A_Temp;

//universal write variables
int uni_Vel, uni_Amp;

int CurrentBATDISCurrent = 0;
int CurrentBATCHGCurrent = 0;

int currentSelect = 0; // 0 for net | 1 for MC

// error code variables
int errorCodeStatus[45]; // 0 or 1 for t/f of error codes
int errorCodeMsg[45]; // 1-44 to write to display
// Corresponding description index with bms index
int errorValues[24] = {21, 15, 11, 21, 9, 16, 8, 4, 25, 26, 41, 27, 31, 23, 33, 35, 37, 39, 14, 13, 12, 1, 2, 3};


void sendframe() {

}

void ReadCanBus(const CAN_message_t &msg) {
  switch (msg.id) {
    case BMS_PACK_1:

      // convert byte[2] to short int
      packCur = ((int)msg.buf[0] << 8) | (int)msg.buf[1];
      // write pack voltage
      packVolt = ((unsigned int)msg.buf[2] << 8) | (unsigned int)msg.buf[3];
      Serial.print("Battery Pack Current:");
      Serial.println(packCur);
      Serial.print("Battery Pack Voltage");
      Serial.println(packVolt);
      

      //SOC as calculated by BMS
      if(batSOC!=msg.buf[4])
      {
      batSOC = msg.buf[4];
      }
      Serial.print("Battery Pack SOC:");
      Serial.println(String(batSOC*0.5) +"%");

      // Read charge/discharge enable
      //Relay Status
      relayStatus = msg.buf[6];

      dischargeRelay = bitRead(relayStatus, 0);
      if (dischargeRelay == 1) { 
        digitalWrite(MC_DSG_E, HIGH);
      }
      else { 
        digitalWrite(MC_DSG_E, LOW);
      }

      chargeRelay = bitRead(relayStatus, 1);
      if (chargeRelay == 1) { 
        digitalWrite(MC_CHG_E, HIGH);
      }
      else { 
        digitalWrite(MC_CHG_E, LOW);
      }
    break;

    case EMERGENCY:
      for (size_t i = 0; i < 7; i++)
      {
        if (msg.buf[i] == 0x32) {
          digitalWrite(MC_DSG_E, LOW);
          digitalWrite(D1_LED, LOW);
          digitalWrite(MC_CHG_E, LOW);
        }
      }
    break;
  }
}

void setup() {
  // Pin initialization
  // Inputs
  pinMode(CHGP_STAT, INPUT);
  pinMode(RDP_STAT, INPUT);
  pinMode(RDP, INPUT); 
  pinMode(CHG_E, INPUT);

  pinMode(DSG_E_STATp, INPUT);
  pinMode(DSG_E_STATm, INPUT);
  pinMode(CHG_SENSE, INPUT);
  pinMode(RDP_SENSE, INPUT);
  pinMode(POWER_SENSE, INPUT);
  pinMode(PUMP_PWR_STAT, INPUT);
  pinMode(MC_MP_E_STAT, INPUT);
  pinMode(CHG_E_STATp, INPUT);
  pinMode(MP_E, INPUT);
  pinMode(FAN4TACH, INPUT);
  pinMode(FAN3TACH, INPUT);
  pinMode(FAN2TACH, INPUT);
  pinMode(FAN1TACH, INPUT);
  pinMode(PUMP_FAN_TACH, INPUT);
  pinMode(COOLANT_TEMP, INPUT);
  pinMode(PUMP_TACH, INPUT);

  // Outputs
  pinMode(CHGP, OUTPUT);
  pinMode(RDP, OUTPUT);

  pinMode(MC_DSG_E, OUTPUT);
  digitalWrite(MC_DSG_E,LOW);
  pinMode(MC_CHG_E, OUTPUT);
  digitalWrite(MC_CHG_E,LOW);


  pinMode(PUMP_PWM, OUTPUT);
  pinMode(FAN0_PWM, OUTPUT);
  pinMode(MC_MP_E, OUTPUT);
  pinMode(PUMP_PWR, OUTPUT);
  pinMode(FAN2_PWM, OUTPUT);
  pinMode(FAN1_PWM, OUTPUT);

  pinMode(D1_LED, OUTPUT);
  pinMode(D2_LED, OUTPUT);
  pinMode(D3_LED, OUTPUT);

analogWriteFrequency(FAN0_PWM, 25000);
analogWriteFrequency(FAN1_PWM, 325000);


  analogWrite(FAN0_PWM, 255);
  analogWrite(FAN1_PWM, 255);
  analogWrite(FAN2_PWM, 255);


  can1.begin();
  can1.setBaudRate(500000); // 500kbps data rate
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(FIFO, ReadCanBus);
  can1.mailboxStatus();

  timer.begin(sendframe, 50000); // Send frame every 50ms--100ms

  delay(1000);
  digitalWrite(MC_DSG_E,HIGH);
  digitalWrite(D1_LED,HIGH);
  digitalWrite(MC_CHG_E,HIGH);

}

void loop() {
  can1.events();
}