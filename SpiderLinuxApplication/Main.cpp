#include "terasic_os.h"
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "CSpider.h"
#include "CSpiderLeg.h"
#include "CMotor.h"
#include "BtSppCommand.h"
#include "mmap.h"
#include "QueueCommand.h"
#include "PIO_LED.h"
#include "PIO_BUTTON.h"
#include "ADC.h"
#include <bitset>
#include <time.h>
#include <stack>
using namespace std;

typedef enum{
	CMD_AT,
	CMD_FORDWARD,
	CMD_BACKWARD,
	CMD_TURN_RIHGT,
	CMD_TURN_LEFT,
	CMD_TURN_RIHGT_DGREE,
	CMD_TURN_LEFT_DGREE,
	CMD_STOP,
	CMD_SPPED,
	CMD_TILTL,
	CMD_TILTR,
	CMD_TILTF,
	CMD_TILTB,
	CMD_TILTN,
	CMD_Query_Version,
	CMD_JOYSTICK,
	CMD_ALL,
	CMD_IDLE,
}COMMAND_ID;

int invertCommand(int command);
void runCommand(int command);

static void *bluetooth_spp_thread(void *ptr)
{

	CBtSppCommand BtSppCommand;
	CQueueCommand *pQueueCommand;
	int Command, Param;
	pQueueCommand = (CQueueCommand *)ptr;
	printf("[BT]Start Service\r\n");
	BtSppCommand.RegisterService();
	while(1){
		printf("[BT]Listen...\r\n");
		BtSppCommand.RfcommOpen();
		printf("[BT]Connected...\r\n");
		while(1){
			Command = BtSppCommand.CommandPolling(&Param);
			if (Command != CMD_IDLE){
				// push command to command queue
				if (Command == CMD_STOP)
				   pQueueCommand->Clear();
				// push command to command queue 
				if (!pQueueCommand->IsFull()){
				   pQueueCommand->Push(Command, Param);
				    }
				/*if (!pQueueCommand->IsFull()){
					pQueueCommand->Push(Command, Param);
				}*/
			}
		}
		printf("[BT]Disconneected...\r\n");
		BtSppCommand.RfcommClose();
	}

//	pthread_exit(0); /* exit */
	return 0;
}
CSpider Spider;
int Param;

int main(int argc, char *argv[]){

    
    CQueueCommand QueueCommand;
	stack<int> commandHistory;
	ADC adc;
	uint32_t sensorReading0 = 0;
	int Command;
    bool bSleep = false;
    CPIO_LED LED_PIO;
    CPIO_BUTTON BUTTON_PIO;
    pthread_t id0;
    int ret0;
    uint32_t LastActionTime;
    const uint32_t MaxIdleTime = 10*60*OS_TicksPerSecond(); // spider go to sleep when exceed this time
//    const uint32_t MaxIdleTime = 10*60*OS_TicksPerSecond(); // spider go to sleep when exceed this time

	printf("===== Spider Demo =====\r\n"); 
 

	printf("Spider Init & Standup\r\n");
	if (!Spider.Init()){
		printf("Spilder Init failed\r\n");
	}else{
		if (!Spider.Standup())
			printf("Spilder Standup failed\r\n");
	}
	Spider.SetSpeed(50);

	// 
	printf("Create Bluetooth Thread\r\n");
	ret0=pthread_create(&id0,NULL,bluetooth_spp_thread, (void *)&QueueCommand);
	if(ret0!=0){
		printf("Creat pthread-0 error!\n");
		//exit(1);	
	}	
	printf("Listen Command...\r\n");
	  LED_PIO.SetLED(0x7f);
	  LastActionTime = OS_GetTickCount();
		while(1)
		{	
			//sleep mode detect.
			if(!bSleep && ((OS_GetTickCount()-LastActionTime) > MaxIdleTime))
			{
				bSleep = true;
				LastActionTime = OS_GetTickCount();
				Spider.Sleep();
				LED_PIO.SetLED(0x1);
			}
			
			if(BUTTON_PIO.GetBUTTON()==0x2)
			{
				if (bSleep){
					bSleep = false;
					Spider.WakeUp();
					LED_PIO.SetLED(0x7f);
				}	
				while(!commandHistory.empty()){
					runCommand(commandHistory.top());
					commandHistory.pop();
					sleep(1);
					//wait to make sure its in position
				}
				printf("Folding\r\n");
				Spider.Fold();
				LastActionTime = OS_GetTickCount();
			}
			else if (BUTTON_PIO.GetBUTTON()==0x1)
			{
				if (bSleep){
					bSleep = false;
					Spider.WakeUp();
					LED_PIO.SetLED(0x7f);
				}	
				Spider.DEMO_Rollover();
				LastActionTime = OS_GetTickCount();
			}
			
			if(!QueueCommand.IsEmpty() && QueueCommand.Pop(&Command, &Param) ){
					 if (bSleep){
							bSleep = false;
						Spider.WakeUp();
						LED_PIO.SetLED(0x7f);
					  }  

					switch(Command){
						case CMD_FORDWARD:
							printf("CMD_FORDWARD\n");
							sensorReading0 = adc.GetChannel(0);
							printf("Ch0 Sensor Reading: %u\r\n", sensorReading0);
							
							if (sensorReading0 < 1000){
								Spider.MoveForward(1);
								commandHistory.push(invertCommand(Command));
							}
							else {
								Spider.Push_Position();
								Spider.Push_Over(3);
								while(sensorReading0 > 1000){
									Spider.Push_Over(1);
									sensorReading0 = adc.GetChannel(0);
									printf("Ch0 Sensor Reading: %u\r\n", sensorReading0);
								}
								
								Spider.Reset();
							}
							break;
						case CMD_BACKWARD:
							printf("CMD_BACKWARD\n");
							Spider.MoveBackward(1);
							commandHistory.push(invertCommand(Command));
							break;
						case CMD_TURN_RIHGT:
							printf("CMD_TURN_RIHGT\n");
							Spider.RotatelRight(1);
							commandHistory.push(invertCommand(Command));
							break;
						case CMD_TURN_LEFT:
							printf("CMD_TURN_LEFT\n");
							Spider.RotatelLeft(1);
							commandHistory.push(invertCommand(Command));
							break;
						case CMD_TILTL:
							printf("CMD_TILTL\n");
							Spider.TiltLeft();
							commandHistory.push(invertCommand(Command));
							break;
						case CMD_TILTR:
							printf("CMD_TILTR\n");
							Spider.TiltRight();
							commandHistory.push(invertCommand(Command));
							break;
						case CMD_TILTF:
							printf("CMD_TILTF\n");
							Spider.TiltForward();
							commandHistory.push(invertCommand(Command));
							break;
						case CMD_TILTB:
							printf("CMD_TILTB\n");
							Spider.TiltBackward();
							commandHistory.push(invertCommand(Command));
							break;
						case CMD_TILTN:
							printf("CMD_TILTN\n");
							Spider.TiltNone();
							break;
						case CMD_STOP:
							printf("CMD_STOP\n");
							Spider.Reset();
							break;
						case CMD_SPPED:
							printf("CMD_SPPED %d \n",Param);
							Spider.SetSpeed(Param);
							break;
						case CMD_TURN_LEFT_DGREE:
							printf("CMD_TURN_LEFT_DGREE %d \n",Param);
							break;
						case CMD_TURN_RIHGT_DGREE:
							printf("CMD_TURN_RIHGT_DGREE %d \n",Param);
							break;
						case CMD_Query_Version:
							printf("CMD_Query_Version\n");
							break;
						case CMD_JOYSTICK:
							printf("CMD_JOYSTICK (Param=%d)\n",Param);
							break;
						case CMD_ALL:
							printf("CMD_REVERSE\n");
							while(!commandHistory.empty()){
								runCommand(commandHistory.top());
								commandHistory.pop();
								sleep(1);
								//wait to make sure its in position
							}
							
							QueueCommand.Clear();
							break;
						default:printf("Nothing\n");
							break;
					} // switch
				LastActionTime = OS_GetTickCount();
				
			}

		}



	return 0;
}	

void runCommand(int command){
	switch(command){
		case CMD_FORDWARD:
			printf("CMD_FORDWARD\n");
			Spider.MoveForward(1);
			break;
		case CMD_BACKWARD:
			printf("CMD_BACKWARD\n");
			Spider.MoveBackward(1);
			break;
		case CMD_TURN_RIHGT:
			printf("CMD_TURN_RIHGT\n");
			Spider.RotatelRight(1);
			break;
		case CMD_TURN_LEFT:
			printf("CMD_TURN_LEFT\n");
			Spider.RotatelLeft(1);
			break;
		case CMD_TILTL:
			printf("CMD_TILTL\n");
			Spider.TiltLeft();
			break;
		case CMD_TILTR:
			printf("CMD_TILTR\n");
			Spider.TiltRight();
			break;
		case CMD_TILTF:
			printf("CMD_TILTF\n");
			Spider.TiltForward();
			break;
		case CMD_TILTB:
			printf("CMD_TILTB\n");
			Spider.TiltBackward();
			break;
		case CMD_TILTN:
			printf("CMD_TILTN\n");
			Spider.TiltNone();
			break;
		case CMD_STOP:
			printf("CMD_STOP\n");
			Spider.Reset();
			break;
		case CMD_SPPED:
			printf("CMD_SPPED %d \n",Param);
			Spider.SetSpeed(Param);
			break;
		case CMD_TURN_LEFT_DGREE:
			printf("CMD_TURN_LEFT_DGREE %d \n",Param);
			break;
		case CMD_TURN_RIHGT_DGREE:
			printf("CMD_TURN_RIHGT_DGREE %d \n",Param);
			break;
		default:
			break;
	}
}

int invertCommand(int command){
	switch(command){
		case CMD_FORDWARD:
			return CMD_BACKWARD;
			break;
		case CMD_BACKWARD:
			return CMD_FORDWARD;
			break;
		case CMD_TURN_RIHGT:
			return CMD_TURN_LEFT;
			break;
		case CMD_TURN_LEFT:
			return CMD_TURN_RIHGT;
			break;
		case CMD_TILTL:
			return CMD_TILTR;
			break;
		case CMD_TILTR:
			return CMD_TILTL;
			break;
		case CMD_TILTF:
		return CMD_TILTB;
			break;
		case CMD_TILTB:
			return CMD_TILTF;
			break;/*
		case CMD_STOP:
			return;
			break;
		case CMD_SPPED:
			return;
			break;
		case CMD_TURN_LEFT_DGREE:
			return;
			break;
		case CMD_TURN_RIHGT_DGREE:
			return;
			break;
		case CMD_Query_Version:
			return;
			break;
		case CMD_JOYSTICK:
			return;
			break;
		case CMD_ALL:
			return;
			break;*/
		default:
			return CMD_IDLE;
	}
}




