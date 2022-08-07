#include <Arduino.h>
#include <ModbusMaster.h>                //need to include Modbus Master library
#include <HardwareSerial.h>
HardwareSerial Serial3(PC6); //FTDA Serial
HardwareSerial Serial1(PA10, PA9); // RS485 Serial
HardwareSerial Serial2(PA3, PA2); //Modem Serial
//HardwareSerial Serial2(PA3, PA2);
#define number 2 //Device Number
#define MAX485_Enable     PC7
// instantiate ModbusMaster object
ModbusMaster node[number];
word CRC16 (const byte *nData, word wLength);
uint8_t slave [] = {0x001, 0x002};
String name1[]={"PDB_On","Generator_ON"};
void server_connect();
void receivedata();
void modem_start(void);
void preTransmission()
{
  digitalWrite(MAX485_Enable, 1); //Transmit data
}
void postTransmission()
{
  digitalWrite(MAX485_Enable, 0); //Receive Data
}
void setup ()
{
  pinMode(MAX485_Enable, OUTPUT);
  Serial1.begin(9600, SERIAL_8N2);
  Serial2.begin(115200, SERIAL_8N2);
  Serial3.begin(115200, SERIAL_8N2);
  pinMode (PC11, OUTPUT);
  pinMode (PA1, OUTPUT);
  pinMode (PC0, INPUT);
  modem_start();
  for (int i = 0; i < number; i++)
  {
    node[i].begin(slave[i], Serial1);

    // Callbacks allow us to configure the RS485 transceiver correctly
    node[i].preTransmission(preTransmission);
    node[i].postTransmission(postTransmission);
  }
  server_connect();
}
bool state = true;

void loop ()
{
}

//FTDA Serial Print
String mdm_rpl()
{
  if (Serial2.available())
  {
    String reply = Serial2.readString();
    reply.trim();
    return reply;
  }
}
//Modem Start When Enable Pin PA1 High, PWRKEY PC11 High  few Second and then Low
void modem_start(void)
{
  digitalWrite(PA1, HIGH);
  delay(500);
  digitalWrite(PC11, HIGH);
  delay(500);
  digitalWrite(PC11, LOW);
  delay(15000);
}
void server_connect()
{ uint8_t result [256];
  uint8_t data [50];
  Serial2.println("AT\r\n");
  delay(10);
  //AT Commands for Status Control
  Serial2.println("AT+CSQ\r\n");   //Query signal quality
  delay(10);
  Serial2.println("AT+CPIN?\r\n");  //Enter PIN
  delay(10);
  // AT Commands for Network
  Serial2.println("AT+COPS?\r\n"); //Operator selection
  delay(10);
  //Serial3.println(mdm_rpl());
  Serial2.println("AT+CREG=1\r\n"); //Network registration, 1 -registered, home network
  delay(10);
  Serial2.println("AT+CREG?\r\n");
  delay(10);
  Serial2.println("AT+CGMI\r\n"); //Request manufacturer identification
  delay(10);
  Serial2.println("AT+CNMP=2\r\n"); //Preferred mode selection, 2 – Automatic
  delay(500);
  Serial3.println(mdm_rpl());

  while (1)
  {
    for (int i = 0; i < number; i++) {
      result[i] = node[i].readHoldingRegisters(0x0061, 50);
      if (result[i] == node[i].ku8MBSuccess)
      { int j = i + 1;
        String string ;
        string += "{";
        string += "\"Name";
        string +=String(j);
        string +="\":";
        string += 1;
        string += ",";
        string += "\"PhaseA_Voltage";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x00));
        string += ",";
        string += "\"PhaseB_Voltage";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x01));
        string += ",";
        string += "\"PhaseC_Voltage";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x02));
        string += ",";
        string += "\"PhaseA_Current";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x03));
        string += ",";
        string += "\"PhaseB_Current";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x04));
        string += ",";
        string += "\"PhaseC_Current";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x05));
        string += ",";
        string += "\"PhaseA_Power";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x06));
        string += ",";
        string += "\"PhaseB_Power";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x07));
        string += ",";
        string += "\"PhaseC_Power";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x08));
        string += ",";
        string += "\"Total_Power";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x09));
        string += ",";
        string += "\"PhaseA_RE";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0xA));
        string += ",";
        string += "\"PhaseB_RE";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0xB));
        string += ",";
        string += "\"PhaseC_RE";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0xC));
        string += ",";
        string += "\"Total_RE";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0xD));
        string += ",";
        string += "\"PhaseA_AP";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0xE));
        string += ",";
        string += "\"PhaseB_Ap";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0xF));
        string += ",";
        string += "\"PhaseC_AP";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x10));
        string += ",";
        string += "\"Total_AP";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x11));
        string += ",";
        string += "\"PhaseA_PF";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x12));
        string += ",";
        string += "\"PhaseB_PF";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x13));
        string += ",";
        string += "\"PhaseC_PF";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x14));
        string += ",";
        string += "\"Total_PF";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x15));
        string += ",";
        string += "\"Frequency";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x16));
        string += ",";
        string += "\"PhaseA_B";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x17));
        string += ",";
        string += "\"PhaseB_C";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x18));
        string += ",";
        string += "\"PhaseC_A";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x19));
        string += ",";
        string += "\"Forward_Active_MAX_Demand";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x1A));
        string += ",";
        string += "\"Time_Forward_Active_MAX_Demand_Min/Hour";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x1B));
        string += ",";
        string += "\"Time_Forward_Active_MAX_Demand_Month";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x1C));
        string += ",";
        string += "\"Reverse_Active_MAX_Demand";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x1D));
        string += ",";
        string += "\"Time_Reverse_Active_MAX_Demand_Min/Hour";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x1E));
        string += ",";
        string += "\"Time_Reverse_Active_MAX_Demand_Month";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x1F));
        string += ",";
        string += "\"Forward_Reactive_MAX_Demand";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x20));
        string += ",";
        string += "\"Time_Reactive_MAX_Demand_Min/Hour";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x21));
        string += ",";
        string += "\"Time_Reactive_MAX_Demand_Month";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x22));
        string += ",";
        string += "\"Reverse_Reactive_MAX_Demand";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x23));
        string += ",";
        string += "\"Time_Reverse_Reactive_MAX_Demand_Min/Hour";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x24));
        string += ",";
        string += "\"Time_Reverse_Reactive_MAX_Demand_Month";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x25));
        string += ",";
        string += "\"PhaseA_Forward_Active_Energy";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x26));
        string += ",";
        string += "\"PhaseB_Forward_Active_Energy";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x28));
        string += ",";
        string += "\"PhaseC_Forward_Active_Energy";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x28));
        string += ",";
        string += "\"PT";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x2A));
        string += ",";
        string += "\"CT";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x2B));
        string += ",";
        string += "\"Running_state";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x2E));
        string += ",";
        string += "\"Zero_Sequence_C";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x2F));
        string += ",";
        string += "\"Voltage_Imbalance";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x30));
        string += ",";
        string += "\"Current_Imbalance";
        string +=String(j);
        string +="\":";
        string += String(node[i].getResponseBuffer(0x31));
        string += "}";
        Serial3.println(string);
        int data_length = string.length() + 2;
        //Serial3.println(data_length);
        String payload ;
        payload += "AT+CMQTTPAYLOAD=0,";
        payload += String(data_length);
        payload += "\r\n";
        char* ptopic = "v1/devices/me/telemetry"; //Publish Topic

        Serial2.println("AT+CMQTTSTART\r\n"); //Start MQTT server
        delay(500);
        Serial2.println("AT+CMQTTACCQ=0,\"GSM01\"\r\n"); //Acquire a client
        delay(500);
        Serial2.println("AT+CMQTTCONNECT=0,\"tcp://thinkiot.com.bd\",20,1,\"GSM011\",\"GSM01\"\r\n "); //Connect to the server
        delay(1000);
        Serial2.println("AT+CMQTTTOPIC=0,23\r\n"); //publish topic
        delay(500);
        Serial2.println(ptopic); //SET topic
        delay(500);
        Serial2.println(payload); //Set publishing message
        delay(500);
        Serial2.println(string); //Set publishing message
        delay(500);
        Serial2.println("AT+CMQTTPUB=0,0,60\r\n"); //Send a PUBLISH message to server
        delay(1000);
        Serial2.println("AT+CMQTTDISC=0,120\r\n");
        delay(500);
        Serial2.println("AT+CMQTTREL=0\r\n");
        delay(500);
        Serial2.println ("AT+CMQTTSTOP\r\n");
        delay(500);
      }
    }
  }
}
