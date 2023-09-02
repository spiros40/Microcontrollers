    #include <avr/wdt.h> /* Header for watchdog timers in AVR */
    #include <Ethernet.h>
    #include <SPI.h>
    #include <EEPROM.h>

    // Enter a MAC address and IP address for your controller below.
    // The IP address will be dependent on your local network:
    byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
    //my ip
    IPAddress ip(192, 168, 1, 177);
    // Enter the IP address of the server you're connecting to:
    IPAddress server(192, 168, 1, 100);
    // Initialize the Ethernet client library
    // with the IP address and port of the server
    // that you want to connect to (port 23 is default for telnet;
    // if you're using Processing's ChatServer, use port 10002):
    IPAddress gateway(192, 168, 1, 1);
    EthernetClient client;
      
    /*system_status*/      
      #define systemStatusCell         0
      #define systemDisarmed           0
      #define systemArmed              1    
      #define systemStayArmed          2
      /*systemSubStatus*/
      #define systemSubStatusCell       1
      #define systemNormal              0
      #define systemInEntryDelay        1
      #define systemInAlarm             2
      /*system_running now*/
      #define systemRunningNowCell      2
      #define systemAlarmNormal         0  
      #define systemAlarmWait           1
      #define systemCloseAlarm          2     
      #define systemEntryWait           3      
      #define systeΟpenedZones          5
    
    /*zone attributes*/      
      #define ZoneAttribute            0
      #define zoneInactive             0
      #define zoneDelay                1
      #define zoneInternal             2
      #define zoneInstant              3  
      #define cellZoneSet              2
      #define zoneBypassed             0
      #define zoneUsed                 1 

      #define zoneOpened               0 
      #define zoneClosed               1
      
      #define zoneAttributeCell        1
      #define zoneBypassCell           2
      #define zoneStatusCell           3
      #define zonePinCell              4
      
      #define zone1Pin                    30
      #define zone2Pin                    31
      #define zone3Pin                    32
      #define zone4Pin                    33
      #define zone5Pin                    34
      #define zone6Pin                    35
      #define zone7Pin                    36
      #define zone8Pin                    37
      #define zone9Pin                    22
      #define zone10Pin                   23
      #define zone11Pin                   24
      #define zone12Pin                   25
      #define zone13Pin                   26
      #define zone14Pin                   27
      #define zone15Pin                   28
      #define zone16Pin                   29
      #define maxZones                    16  

      #define zone1                    0
      #define zone2                    1
      #define zone3                    2
      #define zone4                    3
      #define zone5                    4
      #define zone6                    5
      #define zone7                    6
      #define zone8                    7
      #define zone9                    8
      #define zone10                   9
      #define zone11                   10
      #define zone12                   11
      #define zone13                   12
      #define zone14                   13
      #define zone15                   14
      #define zone16                   15
      
      #define zonePinCell                3
      #define zoneNumberCell             4

      /*output attributes*/    
      #define outputAttributeCell           1        
      #define outputSirenPin                39
      #define outputSiren2Pin               40
      #define PGM1Pin                       41
      #define PGM2Pin                       42
      #define PGM3Pin                       43
      #define PGM4Pin                       44
      #define PGM5Pin                       45
      #define PGM6Pin                       46
      #define PGM7Pin                       47
      #define PGM8Pin                       48
      #define PGM9Pin                       49
       /*output names*/
      #define outputSiren                   0
      #define outputSiren2                  1
      #define PGM1                          2
      #define PGM2                          3
      #define PGM3                          4
      #define PGM4                          5
      #define PGM5                          6
      #define PGM6                          7
      #define PGM7                          8
      #define PGM8                          9
      #define PGM9                          10
      
      /*delay times*/
      #define sirenDelayTime          8000
      #define entryDelayTime          8000
      #define exitDelayTime           8000
      unsigned long alarmRunTime=0;
      unsigned long entryRunTime=0;
      
      unsigned long systemRunCommunications=0; 
      int runCommunications=0;
      #define runCommunicationsCycle 5000
     
      #define enable                   1
      #define disable                  0
      
         
      /*password*/
      byte passwordAddress[5]={'0','0','0','0'};
      byte password[5]={'0','0','0','0'};
      
      
   /**********************************************************************************/  
     /*systemStatus []={systemStatusCell->[systemDisarmed=0, systemArmed=1,systemStayArmed=2], 
     systemSubStatusCell->[in_normal=0,in_entry_delay=1,in_alarm=2], 
     SystemRunningNowCell->[systemAlarmNormal=0,systemAlarmWait=1,systemCloseAlarm=2,systemEntryWait=3,openedZones=4]};*/
    byte systemStatus []={2,0,0};
   
    /*PORTC + PORTA zone attributes
    zone= {zoneAttributeCell->[0=zoneInactive , 1=zoneDelay, 2=zoneInternal, 3=zoneInstant], 
    zoneBypassCell->[1=zoneBypassed , 2=zoneUsed ], 
    zoneStatusCell->[0=zoneOpen, 1=zoneClosed],
    [zonePinCell],[zoneNumberCell]}}*/    
    int zones [16][5]={
      {zoneDelay,zoneUsed,zoneClosed,zone1Pin,zone1},{zoneDelay,zoneUsed,zoneClosed,zone2Pin,zone2},
      {zoneInternal,zoneUsed,zoneClosed,zone3Pin,zone3},{zoneInternal,zoneUsed,zoneClosed,zone4Pin,zone4},
      {zoneInternal,zoneUsed,zoneClosed,zone5Pin,zone5},{zoneInstant,zoneUsed,zoneClosed,zone6Pin,zone6},
      {zoneInstant,zoneUsed,zoneClosed,zone7Pin,zone7},{zoneInstant,zoneUsed,zoneClosed,zone8Pin,zone8},
      {zoneInstant,zoneUsed,zoneClosed,zone9Pin,zone9},{zoneInstant,zoneUsed,zoneClosed,zone10Pin,zone10},
      {zoneInstant,zoneUsed,zoneClosed,zone11Pin,zone11},{zoneInstant,zoneUsed,zoneClosed,zone12Pin,zone12},
      {zoneInstant,zoneUsed,zoneClosed,zone13Pin,zone13},{zoneInstant,zoneUsed,zoneClosed,zone14Pin,zone14},
      {zoneInstant,zoneUsed,zoneClosed,zone15Pin,zone15},{zoneInstant,zoneUsed,zoneClosed,zone16Pin,zone16}};
   
     /*PORTG + PORTL output attributes
      output= cell_output_attributes->[connected pin number and name], 
      cell_output_current->[0=disable, 1=enable]*/
    int outputs [11][2]={
      {outputSirenPin,disable},{outputSiren2Pin,disable},
      {PGM1Pin,disable},{PGM2Pin,disable},{PGM3Pin,disable},{PGM4Pin,disable},
      {PGM5Pin,disable},{PGM6Pin,disable},{PGM7Pin,disable},{PGM8Pin,disable},{PGM9Pin,disable}};

    /**********************************************************************************/  
void setup(){    
  wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */ 
      //Port C initialize as input for zones        
     for (int pin = 30; pin <= 37; pin++) {
          pinMode(pin, INPUT);
      }
      delay(50);  
      //Port A initialize as input for zones
      for (int pin = 22; pin <= 29; pin++) {
        pinMode(pin, INPUT);
      }    
      delay(50);    
      //Port G initialize as input for helping functions
      for (int pin = 39; pin <= 41; pin++) {
        pinMode(pin, INPUT);
      } 
      delay(50);
      //Port L initialize as output
      for (int pin = 42; pin <= 49; pin++) {
        pinMode(pin, OUTPUT);
      } 
      delay(50);
      
      // initialize serial ports
      Serial.begin(9600);//communicates with pc (programming-configuration port)
      delay(50);
      
   // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH Shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESPsystemStatus8266 with Adafruit FeatherWing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit FeatherWing Ethernet
     
     // start the Ethernet connection:
     Ethernet.begin(mac, ip, gateway); 
     // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet hardware was not found.");
      delay(1); // do nothing, no point running without Ethernet hardware    
  }
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    delay(500);
  }

  // give the Ethernet shield a second to initialize:
  delay(1000);
  Serial.println("connecting...");

  // if you get a connection, report back via serial:
  if (client.connect(server, 2323)) {
    Serial.println("connected");
  } else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }    
  /* Enable the watchdog with a timeout of 2 seconds */
  wdt_enable(WDTO_2S);  
  }
/**********************************************************************************/  
//writes to EEPROM
  byte writeToEeprom(int addr, byte val){
    /*Write the value to the appropriate byte of the EEPROM.
    these values will remain there when the board is turned off.*/
    EEPROM.write(addr, val);
  }
//scan inputs and load the arrays
  void scanZones(){
     for(int i=0;i<=maxZones;i++){                            
       if(digitalRead(zones[i][zonePinCell])==HIGH){              
           zones[i][zoneStatusCell]=1;
          }else{              
            zones[i][zoneStatusCell]=0;
          }          
        delay(5);
    }}
//bypass unbypass a zone
  int zoneBypass(int zoneName, int BypassUnBypass){        
          zones[zoneName][zoneBypassCell]=BypassUnBypass;
          if(BypassUnBypass==enable){
              return 1;
            }else if(BypassUnBypass==disable){
              return 0; 
             }
             delay(5);}   
//set an output
  int setOutput(int outputName, int setUnset){        
          outputs[outputName][outputAttributeCell]=setUnset;
          if(setUnset==enable){
              return 1;
            }else if(setUnset==disable){
              return 0; 
             }
             delay(5);
        }  
//scan when is stayArmed
  void ScanStayArmed(){
         
          // for(int i=0;i<=7;i++){
          //   if(zones[i][cell_zone_current]==zone_opened && !zones[i][cell_zone_attribute ]==zone_bypassed && !zones[i][cell_zone_attribute ]==zone_inactive && zones[i][cell_zone_set]==zone_used){               
          //       if(zones[i][cell_zone_attribute]==zone_delay){
          //           system_status[cell_system_status2]=system_in_entry_delay;
          //           system_IN_ENTRY_DELAY();                  
          //         }else{
          //           system_status[cell_system_status2]=system_in_alarm;
          //           system_IN_ALARM();
          //           }
          //       }
          //   }
        }
//scan when is full Armed
  void scanFullArmed(){        
        for(int i=0;i<=maxZones;i++){
            if(zones[i][zoneStatusCell]==zoneOpened && !zones[i][zoneAttributeCell]==zoneInactive && zones[i][zoneBypassCell]==zoneUsed){               
                if(zones[i][zoneAttributeCell]==zoneDelay){
                    systemStatus[systemSubStatusCell]=systemInEntryDelay;
                    inEntryDelay();                  
                  }else{
                    systemStatus[systemSubStatusCell]=systemInAlarm;
                    inAlarm();
                  }
              }
          }               
        }
//system is in entry delay
  void inEntryDelay(){           
          if(systemStatus[systemRunningNowCell]==systemAlarmNormal){
              systemStatus[systemRunningNowCell]=systemEntryWait ;
              entryRunTime=0;
              entryRunTime=entryDelayTime + millis();               
          }else if(systemStatus[systemRunningNowCell ]==systemEntryWait && entryRunTime<=millis()){
              systemStatus[systemRunningNowCell]=systemAlarmNormal;
              entryRunTime=0;
              inAlarm();           
            }else if(systemStatus[systemStatusCell]==systemDisarmed){
                systemStatus[systemRunningNowCell]=systemAlarmNormal;
                systemDisarm();
                entryRunTime=0;
              }                
        }
//systemis in alarm 
  void inAlarm(){                 
          if(systemStatus[systemRunningNowCell]==systemAlarmNormal){
              systemStatus[systemRunningNowCell]=systemAlarmWait ;
              alarmRunTime=0;
              setOutput(outputSiren, enable);              
              alarmRunTime=sirenDelayTime + millis();              
          }else if(systemStatus[systemRunningNowCell]==systemAlarmWait && alarmRunTime<=millis()){
              systemStatus[systemRunningNowCell]=systemAlarmNormal;
              systemStatus[systemSubStatusCell]=systemNormal;
              alarmRunTime=0;
              setOutput(outputSiren, disable);
            }else if(systemStatus[systemStatusCell]==systemDisarmed){
                systemStatus[systemRunningNowCell]=systemAlarmNormal;
                systemStatus[systemSubStatusCell]=systemNormal;
                alarmRunTime=0;
                setOutput(outputSiren, disable);
              }                  
        }      

//disarms the system
  void systemDisarm(){
        systemStatus[systemStatusCell]=systemDisarmed;
        systemStatus[systemSubStatusCell]=systemNormal;
        systemStatus[systemRunningNowCell]=systemAlarmNormal;
        setOutput(outputSiren, disable);        
        //reactivates the zones
        for(int i=0;i<=maxZones;i++){
            if(zones[i][zonePinCell]==zoneBypassed){
                zones[i][zonePinCell]=zoneUsed;
              }
          }        
      }
//system arms in stay mode
  void armInStay(){      
      int countOpenZones=0;
        for(int i=0;i<=maxZones;i++){
            if((zones[i][zoneStatusCell]==zoneOpened) && !(zones[i][zoneAttributeCell]==zoneInactive) &&!zones[i][zoneBypassCell]==zoneBypassed){
                countOpenZones++;
              }
          }
       if(countOpenZones==0){
            systemStatus[systemStatusCell]=systemStayArmed;            
       }}
//arms system in full mode  
  void armInFull(){
        int countOpenZones=0;        
        for(int i=0;i<=maxZones;i++){          
            if((zones[i][zoneStatusCell]==zoneOpened) && !(zones[i][zoneAttributeCell]==zoneInactive) && (zones[i][zoneBypassCell]==zoneUsed)){                                      
                countOpenZones++;  
              }
          }          
        if(countOpenZones==0){
            systemStatus[systemStatusCell]=systemArmed;                  
            
         }
      }
//communicate with telnet server 
byte receiceDataFromServer(){
  // if there are incoming bytes available
  // from the server, read them and print them:
    if (client.available()) {
      byte receiveData = client.read();
      return receiveData;      
    }else{
      return disable;
    }
  }

byte sendDataToServer(byte sendData[]){
  // read them and send them out the socket if it's open:
    if (client.connected()) {
      //client.print(sendData);
      //client.write(val)
      //series of bytes
      //client.write(buf, len);
       client.write(sendData, sizeof(sendData));
    }
  }


  void ethernetCommunicationOriginal(){
    // if there are incoming bytes available
  // from the server, read them and print them:
  if (client.available()) {
    char receiveData = client.read();
    Serial.print(receiveData);
  }
  // as long as there are bytes in the serial queue,
  // read them and send them out the socket if it's open:
  while (Serial.available() > 0) {
    char sendData = Serial.read();
    if (client.connected()) {
      client.print(sendData);
    }
  }
  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
    // do nothing:
    while (true) {
      delay(1);
    }
  }}

/**********************************************************************************/  
void loop(){    
  byte receivedData; 
  wdt_reset();
  scanZones();
  //receivedData=receiceDataFromServer();
  //sendDataToServer(systemStatus);
//Serial.println(systemStatus[systemSubStatusCell]);
delay(1400);
  switch(systemStatus[systemSubStatusCell]){    
    case systemNormal:  
      switch(systemStatus[systemStatusCell]){
        case systemDisarmed:   
        Serial.println("systemDisarmed");                                         
        break;             
        
        case systemArmed:
          scanFullArmed(); 
          Serial.println("Armed");                        
        break;
              
        case systemStayArmed:
          ScanStayArmed();  
          Serial.println("stay");                            
        break;             
      }
      break;
           
      case systemInEntryDelay:
        inEntryDelay();
        Serial.println("delay"); 
      break;  
                    
      case systemInAlarm:          
        inAlarm();
        Serial.println("alarm"); 
      break; 
    }
                 

    }

