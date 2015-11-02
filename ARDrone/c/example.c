/*
 * example.c
 * 
 * Example of how to use ARDrone.c to control ARDrone
 *
 * Stephen Markham 01/08/15 
 * University of Otago
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

//------ Required ----------
#include "ARDrone.h"       
#include <pthread.h> // may require compiling with -lpthread
//--------------------------


// ----- Required ---------------------------------
void *DroneControl()
{
   ar_drone("192.168.2.1");      //Sets up ARDrone at provided IP address (Default is 192.168.1.1)
   control();                    //Starts Control Thread
   return NULL;
}
// ------------------------------------------------


int main ()
{
   // ----- Required Code -------------------------
   //Create new control thread
   pthread_t ControlThread;
   if(pthread_create(&ControlThread, NULL, DroneControl, NULL)) {
      fprintf(stderr, "Error creating thread\n");
      return 1;
   }
   // ---------------------------------------------


   // The Following is an example of a basic flight
   // Currently speeds have to be +/- 0.05,0.1,0.2 or 0.5
   
   takeOff();     //take off
   sleep(5);      //Allows time for takeoff and settle before commands come in

   rotateRight(0.5); //Rotate Right at 50% speed
   sleep(2);         //For 2 Seconds

   hover();         //Hover in place
   sleep(3);        //For 3 Seconds

   land();          //Land

   // End Basic Flight Plan


   // ---- Recommended Code ------------------------

   terminateThread();   //Terminate Control Thread (should be called ONLY
                        //after landing for the last time)

   // Stops Program terminating early by waiting for control thread 
   // to terminate properly.
   if(pthread_join(ControlThread, NULL)) {
      return 1;
   }
   //---------------------------------------------

   return 0;
}
