/*
 * ar_drone.h
 *
 *
 * Stephen Markham 01/08/15 
 * University of Otago
 */

#ifndef ARDrone_H
#define ARDrone_H

        //Set up ARDrone with IP address (Call in control thread)
        void ar_drone(char * ip );

        //main control thread (Call after ad_drone in control thread)
        void control();


        //Various Controls for the ARDrone. Called from wherever after control
        //thread is running
        //currently speeds have to be sent in as + or - 0.05,0.1,0.2 or 0.5
        void takeOff();
        
        void land();
        
        void hover();

        void forward(double speed);

        void backward(double speed);

        void right(double speed);

        void left(double speed);

        void up(double speed);

        void down(double speed);

        void rotateRight(double speed);

        void rotateLeft(double speed);


        //Absolute control of all axis (roll, altitude, pitch, yaw) 
        //ADVANCED USE ONLY, please read main source code first to understand
        //How each value affects the ARDrone via the above methods
        void setValues(double r, double a, double p, double y);

        //Last function call (ONLY AFTER LANDING)
        void terminateThread();

        //Convert double to required int values (SHOULD NEVER NEED TO CALL)
        int convertToInt(double f);

        //sets up socket (SHOULD NEVER NEED TO CALL)
        void setUpSocket();

        //Prepares ARDrone for takeoff (SHOULD NEVER NEED TO CALL)
        void prepareForTakeOff();

#endif // ar_drone_H
