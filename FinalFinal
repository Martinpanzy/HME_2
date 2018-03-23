#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"


//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs

PwmOut L1L(L1Lpin);                      //only the low side of driver is controlled by PWM
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);



Thread  commOutT(osPriorityNormal,1024);                         //define two threads
Thread  ReadCom(osPriorityNormal,1024);   
Thread  motorCtrlT(osPriorityNormal,1024);   //Create a new thread，specify the priority of the thread and the maximum stack size.
Mutex   newKey_mutex;                         //mutex prevents simultaneous access of newKey by the command decoder and the bitcoin miner.

RawSerial pc(SERIAL_TX, SERIAL_RX);           //Initialise the serial port


typedef struct{ uint8_t code;
                uint32_t data; 
               } message_t ;                                //DEFINE a new data structure

Mail<message_t,16> outMessages;                               

void putMessage(uint8_t code, uint32_t data){                 //a separate function to add messages to the queue. write each single message to outMessages.
     message_t *pMessage = outMessages.alloc(); 
     pMessage->code = code;
     pMessage->data = data; 
     outMessages.put(pMessage);
}

void commOutFn() {                                         //The function of the thread function commOutFn() is to take messages from the queue and print them on the serial port. 
    while(1) {
                osEvent newEvent = outMessages.get();
                message_t *pMessage = (message_t*)newEvent.value.p; 
                pc.printf("\r");
                pc.printf("Message %d with data 0x%016x\n\r", (int8_t)pMessage->code,(int32_t)pMessage->data); 
                outMessages.free(pMessage);           
             }
}

Queue <void, 8> inCharQ;              //buffer for storing single Char when input command
int buf_index=-1;                                 //index start from -1 so the first index is 0

volatile uint64_t newKey;                         //which will be used to pass the key from the command decoder to the bitcoin miner.
volatile uint32_t newTorque;
volatile uint32_t newVelocity;
volatile uint32_t ctrlPosition;
int8_t orState = 0;                  //Rotor offset at motor state 0


void serialISR(){                                            //get command from pc, strore it to inCharQ
                uint8_t newChar = pc.getc();
                 pc.putc(newChar);               // show the command typed on the screen
                inCharQ.put((void*)newChar); 
                 buf_index=buf_index+1;           //for every input, the index +1 
}

char newCmd[34];            //define an array to store Char and make string to recover command

void motorCtrlTick(){ 
        motorCtrlT.signal_set(0x1); 
}

int32_t motorPosition; 
int32_t abs_ctrlPosition;
int32_t ctrlTorque_s;
int32_t ctrlTorque_r;


void motorCtrlFn(){
     Ticker motorCtrlTicker;
     motorCtrlTicker.attach_us(&motorCtrlTick,100000); 
     int count_iteration=0;
     
     static int32_t oldPosition;
     static int32_t olderror_r;
     int32_t velocity = 0;
     int32_t Torque_y = 0;
     //static int32_t velocity_Ctrl;
     
     
     while(1){
         motorCtrlT.signal_wait(0x1);
         int32_t newPosition=abs(motorPosition);
         velocity=(newPosition-oldPosition)*10;    //position/sec
         oldPosition=newPosition;
         
         count_iteration= count_iteration+1;
        
         if(count_iteration ==10){
             putMessage(15, velocity/6.0f);
             putMessage(20, abs(newPosition/6.0f));
             count_iteration = 0;
         }
         int32_t y_s = 25*(newVelocity*6.0f - abs(velocity));    //rec/sec * 6 = position/sec
         
         ctrlTorque_s = y_s;
         
         if((int32_t)ctrlPosition<0){
           abs_ctrlPosition = - ctrlPosition;   
         } else{
           abs_ctrlPosition = ctrlPosition;    
         }
         
         int32_t error_r = abs_ctrlPosition*6.0f - newPosition;
         int32_t y_r = (int)( 25.0f * error_r + 245.0f * (error_r - olderror_r));
         olderror_r = error_r;
         
         ctrlTorque_r = y_r;
         //newTorque = pTorque;
         
         if(error_r>0){
             ctrlTorque_s = ctrlTorque_s;
         }
         else if(error_r==0){
             ctrlTorque_s = 0;
              }
         else{
             ctrlTorque_s = -ctrlTorque_s;
         } 
         
         //v<0,y_r<0, y_s>0    => y_r<0 =>
         if(velocity<0){
             if(ctrlTorque_s>ctrlTorque_r){
               Torque_y = ctrlTorque_s;
             }
             else{
               Torque_y = ctrlTorque_r; 
             }
         }
         
         //v>=0, y_r>0, y_s=0+-n   => y_s => max speed
         else{
            if(ctrlTorque_s<ctrlTorque_r){
              Torque_y = ctrlTorque_s;   
            } 
            else{
              Torque_y = ctrlTorque_r;
            }
         }
         

        
        //////change rotation direction according to +/- of R_Cmd
        if((int32_t)ctrlPosition<0){
            if (Torque_y < 0){
            newTorque = -Torque_y;
            lead = 2;
            } else {
            newTorque = Torque_y;
            lead = -2;
            }
        }
         
        else{
            if (Torque_y < 0){
            newTorque = -Torque_y;
            lead = -2;
            } else {
            newTorque = Torque_y;
            lead = 2;
            }
             
        }
         
    }  
     
}



void decodeFn(){
            
            buf_index=-1;
            pc.attach(&serialISR);                               
             while(1) {
                         osEvent newEvent = inCharQ.get();       
                         uint8_t newChar = (uint8_t)newEvent.value.p;        
                         
                        if(buf_index<34){                           
                              newCmd[buf_index]= newChar;             
                                    if(newChar=='\r'){                   
                                          newCmd[buf_index]='\0';
                                          printf("\n\r");
                                          printf("newCmd_array_is_%s\n\r",newCmd );     
                                          printf("\n\r");
                                          buf_index=-1;                     
                                               
                                               
                                          if (newCmd[0] == 'K'){                    
                                              newKey_mutex.lock();
                                              sscanf(newCmd, "K%X",&newKey);   
                                               newKey_mutex.unlock();
                                             
                                              printf("The_Value_of_newkey_0x%016x\n\r", newKey);
                                          }
                                         
                                         
                                          if (newCmd[0] == 'T'){                    
                                             sscanf(newCmd, "T%X", &newTorque);   
                                             printf("The_Value_of_newTorque_0x%016x\n\r", newTorque);

                                          }        
                                          
                                          if (newCmd[0] == 'V'){                    
                                             sscanf(newCmd, "V%X", &newVelocity);  
                                             printf("The_Value_of_newVelocity_0x%016x\n\r", newVelocity);

                                          }              
                                          if (newCmd[0] == 'R'){                   
                                             sscanf(newCmd, "R%X", &ctrlPosition);   
                                             printf("The_Value_of_newPosition_0x%016x\n\r", ctrlPosition);
                                             ctrlPosition = ctrlPosition+motorPosition/6; 

                                          }                                             
                                                                              
                                    }
                        }
                        
                        else{                                                         
                               printf("\n\r");
                               printf("the buffer to make a string is overflow");
                               printf("\n\r");
                               buf_index=-1; 
                        }                         
            }
}



//Set a given drive state
void motorOut(int8_t driveState,uint32_t Torque){                                  
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];          
    
    //putMessage(55,Torque);
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);                      
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(Torque);                 
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(Torque);                 
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(Torque);                 
    if (driveOut & 0x20) L3H = 0;
}
    
    //Convert photointerrupter inputs to a rotor state
    inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 1000);
    
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

int8_t intState = 0;


void motorISR() {                        //interrput for motor,
    static int8_t oldRotorState;
    int8_t rotorState = readRotorState(); 
    motorOut((rotorState-orState+lead+6)%6, newTorque);            //+6 to make sure the remainder is positive
    if (rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++; 
    else motorPosition += (rotorState - oldRotorState); 
    oldRotorState = rotorState;
}


Ticker OneSecondStuff;
int count=0;
void OneSecondFunction() 
{

    putMessage(1,count );
    count=0;
    
}

    
void mining(){
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);           
    uint64_t* nonce = (uint64_t*)((int)sequence + 56); 
    uint8_t hash[32];
    
    uint8_t *ptoseq;
    ptoseq=sequence;
    uint8_t *ptohash;
    ptohash=hash;
    
    for((*nonce)=0;(*nonce)<0xFFFFFFFFFFFFFFFF; (*nonce)++){
         newKey_mutex.lock();         //Make changes to the bitcoin miner to use the new key. Before every hash attempt, copy newKey into key.
         * key = newKey;
         newKey_mutex.unlock();
        
        SHA256::computeHash(ptohash, ptoseq, uint32_t(64));
        count=count+1;
        if(hash[0]==0x00 && hash[1]==0x00){
            printf("hash: ");
           for(int i = 0; i < 32; ++i){
               printf("%02x", hash[i]);
            }  
           
           printf("\n\r");
           putMessage(3,*nonce);
           putMessage(2,*key);
        }  
    }
}

    
//Main
int main() {

    pc.printf("Hello\n\r");
    
    
    L1L.period_us(2000);          
    L2L.period_us(2000);     
    L3L.period_us(2000);      
   
    
    orState = motorHome();
    pc.printf("Origin: %d\n\r",orState);
    
    OneSecondStuff.attach( &OneSecondFunction, 1.0 ) ;        //not allowed to use printf in an interrupt when using RTOS
    I1.rise(&motorISR);
    I2.rise(&motorISR);
    I3.rise(&motorISR);
    I1.fall(&motorISR);
    I2.fall(&motorISR);
    I3.fall(&motorISR);
    
    commOutT.start(commOutFn);       // start three threads
    ReadCom.start(decodeFn);
    motorCtrlT.start(motorCtrlFn);   
   
   
   while(1) {   
          mining();
   }

}
