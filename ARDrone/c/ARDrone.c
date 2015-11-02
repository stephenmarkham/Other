/*
 * ARDrone.c
 *
 * Control ARDrone using c
 *
 * Stephen Markham 01/08/15
 * University of Otago
 *
 * to use create a new thread and run control method from within (has to keep sending commands)
 * then can use set values from main thread to control movements
 *
 * currently speeds have to be sent in as + or - 0.05,0.1,0.2 or 0.5
 *
 */

//Various Standard Includes
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

//For Socket Stuff
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//For boolean values
#include <stdbool.h>

//.h file
#include "ARDrone.h"


//Uncomment to print out raw commands being sent (Mainly debugging)
//#define printCommand

//Uncomment to print out the status updates
#define printUpdates

//Socket Variables
struct sockaddr_in myAddress;
int socketNum;
int IP_PORT = 5556; //default port number
const char * IP_ADDRESS;

//Control Values
double roll = 0;
double altitude = 0;
double pitch = 0;
double yaw = 0;

//UDP Packet Sequence
int count = 0;

//Are we performing these actions at the moment?
bool landing = false;
bool takingOff = false;
bool terminate = false;
bool inAir = false;

//Command Frequency in ms
int freq = 50; 	//Docs sugest 50 for ARDrone 1, 30 for ARDrone 2.

/*
 * Constructor with IP Address Specified
 *
 * Creates ar_drone and takes char * IP address as argument
 * @param char * IP (IP address of the ARDrone)
 *
 */
void ar_drone(char * ip)
{
	IP_ADDRESS = ip;
	#ifdef printUpdates
		printf("Setting Up Drone @ %s\n", ip);
	#endif
	setUpSocket();
	count = 0;
}

/*
 * sendCommand
 *
 * Sends a command to the ARDrone
 *
 * @param char * command to be sent
 */
void sendCommand(char *com)
{
	#ifdef printCommand
		printf("%s\n", com); //Print command if desired
	#endif

	if(sendto(socketNum, com, strlen(com), 0, (struct sockaddr *)&myAddress, sizeof(myAddress))!=strlen(com)){
      	perror("Failed to Send, Exiting...");
      	//Not the robust method, should probably try re-send and land
      	exit(EXIT_FAILURE);
  	}
}

/*
 * setValues
 *
 * Set the control values for the drone manually
 * @param double roll, double altitude, double pitch, double yaw
 */
void setValues(double r, double a, double p, double y)
{
	roll = r;
	altitude = a;
	pitch = p;
	yaw = y;
}


/*
 * A variety of control methods, each is a direction
 * and takes a speed (+ or - 0.05,0.1,0.2 or 0.5) as an argument
 *
 * @param double speed
 */
void forward(double speed)
{
	pitch = -speed;

	#ifdef printUpdates
 		printf("Moving forward\n");
 	#endif
}

void backward(double speed)
{
	pitch = speed;

	#ifdef printUpdates
 		printf("Moving backward\n");
 	#endif
}

void right(double speed)
{
	roll = speed;

	#ifdef printUpdates
 		printf("Moving right\n");
 	#endif
}

void left(double speed)
{
	roll = -speed;

	#ifdef printUpdates
 		printf("Moving left\n");
 	#endif
}

void rotateRight(double speed)
{
	yaw = speed;

	#ifdef printUpdates
 		printf("Rotating right\n");
 	#endif
}

void rotateLeft(double speed)
{
	yaw = -speed;

	#ifdef printUpdates
 		printf("Rotating left\n");
 	#endif
}

void up(double speed)
{
	altitude = speed;

	#ifdef printUpdates
 		printf("Increasing height\n");
 	#endif
}

void down(double speed)
{
	altitude = -speed;

	#ifdef printUpdates
 		printf("Decreasing height\n");
 	#endif
}

void hover()
{
	roll = 0;
	altitude = 0;
	pitch = 0;
	yaw = 0;

	#ifdef printUpdates
 		printf("Holding position\n");
 	#endif
}


/*
 * convertToInt
 *
 * Method to take in movement parameter and convert to 32 bit int
 *
 * Currently a look up table, need to write correct method
 *
 * @param double, number to be converted to int
 * @return integer value
 *
 */
int convertToInt(double f)
{
	if (f == 0.05) return (int)1028443341;
	else if (f == 0.1) return (int)1036831949;
	else if (f == 0.2) return (int)1045220557;
	else if (f == 0.5) return (int)1056964608;
	else if (f == -0.05) return (int)-1119040307;
	else if (f == -0.1) return (int)-1110651699;
	else if (f == -0.2) return (int)-1102263091;
	else if (f == -0.5) return (int)-1090519040;
	else return 0;
}

/*
 * setUpSocket
 *
 * sets up a socket for connection to ARDrone
 *
 */
void setUpSocket()
{
	memset(&myAddress, 0, sizeof(myAddress));
   	myAddress.sin_family=AF_INET;

	myAddress.sin_addr.s_addr=htonl(INADDR_ANY);

   	myAddress.sin_port=htons(IP_PORT);

   	if((socketNum=socket(AF_INET, SOCK_DGRAM, 0))<0) {
      perror("Socket Failed");
      exit(EXIT_FAILURE);
   	}

   	if(bind(socketNum,( struct sockaddr *) &myAddress, sizeof(myAddress))<0) {
      perror("Failed to Bind");
      exit(EXIT_FAILURE);
   	}

   	inet_pton(AF_INET,IP_ADDRESS,&myAddress.sin_addr.s_addr);

   	myAddress.sin_port=htons(IP_PORT);

   	//sleep ensures it is set up properly before commands get sent
   	sleep(1);
}

/*
 * prepareForTakeOff
 *
 * Prepares the ARDrone for takeoff
 * and sends the take off command
 *
 */
 void prepareForTakeOff()
{
	//Sequence Number
	count = 1;

 	#ifdef printUpdates
 		printf("Taking Off\n");
 	#endif

 	//Set Level
 	char s[15] = "AT*FTRIM=1,\r";
  	sendCommand(s);
  	count ++;

  	//Takeoff Command
  	char p[25] = "AT*REF=2,290718208\r";
	sendCommand(p);

	//Set bool values
	landing = false;
	takingOff = false;
	inAir = true;
}

/*
 * land
 *
 * Call method to land ARDrone
 *
 */
void land()
{
	landing = true;
}

/*
 * takeOff
 *
 * Call method to takeOff
 *
 */
void takeOff()
{
	takingOff = true;
}

/*
 * terminateThread
 *
 * Terminate the control Thread
 *
 */
void terminateThread()
{
	terminate = true;
}

/*
 * control
 *
 * control thread that runs constantly
 * sending controls every 50ms (required by the ARDrone)
 *
 */
void control()
{
	#ifdef printUpdates
		printf("Set Up Complete\nWaiting for takeoff command...\n");
	#endif

	while(1){

		//If terminating, wait till landing finished then exit
		if (terminate && !landing && !inAir){
			#ifdef printUpdates
				printf("Terminating Thread\n");
			#endif
			break;

		//If gonna terminate early, force land ARDrone, and terminate
		}else if(terminate && inAir && !landing){
			#ifdef printUpdates
				printf("ONLY TERMINATE AFTER LANDING!!!\n");
				printf("FORCING ARDRONE TO LAND\n");
			#else
				perror("Forced Landing after early termination call");
			#endif
			landing = true;
		}

		//If takeoff called, takeoff
		if (takingOff) prepareForTakeOff();

		//If the ARDrone is in the air (Has taken off and not landed)
		if (inAir){
			//Increment Sequence
			count = count + 1;

			char s[50] = "";//Command String

			//Convert control values to integer representations
			int xM = convertToInt(roll);
			int yM = convertToInt(altitude);
			int zM = convertToInt(pitch);
			int pM = convertToInt(yaw);

			//If Landing
			if (landing){
				//Create Landing Command
				char str1[50] = "AT*REF=";
				char str2[50] = ",290717696\r";
				sprintf(s, "%s%d%s", str1, count, str2);

				//Send Command and display message
				sendCommand(s);

				#ifdef printUpdates
					printf("Landing\n");
				#endif

				//Sleep ensures command is sent before thread terminates
				sleep(3);

				//Send Message
				#ifdef printUpdates
					if (!terminate) printf("Waiting for takeoff command...\n");
				#endif

				//Reset bool values
				landing = false;
				inAir = false;

			}else{
				//Start of command
				char str1[50] = "AT*PCMD=";

				//If all 0 (hovering)
				if (roll == 0 && pitch == 0 && altitude == 0 && yaw == 0){
					sprintf(s, "%s%d,0,%d,%d,%d,%d\r", str1, count, xM, zM, yM, pM);

				//Else build command using values
				}else{
					sprintf(s, "%s%d,1,%d,%d,%d,%d\r", str1, count, xM, zM, yM, pM);
				}
			}
			//Send command
			sendCommand(s);

			//Wait desired time before going again
			usleep(freq * 1000);
		}
	}
}
