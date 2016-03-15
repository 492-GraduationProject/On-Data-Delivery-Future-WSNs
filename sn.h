/*
 * sn.h
 *
 *  Created on: Dec 14, 2015
 *      Author: mehmet
 */



#include<omnetpp.h>
#include <globals.h>
#include <Packet_m.h>
#include <csimulation.h>
#include <cmodule.h>
#include <ctopology.h>


#ifndef SN_H_
#define SN_H_

#define batteryCapacity 31104
#define drainReception 0.0000312
#define drainTransmitting 0.0000538


enum sensor_Type { temp=1, pres=2, co=3, hum=4, co2=5};
enum Receive_Transmit {Receive = 0, Transmit = 1 , Sense = 2} ; // for energy consumption


class SensorNode:public cSimpleModule
{

private:
    int snX;
    int snY;
    double totPower;


public:
        int getSn_XPos() {return snX;}
        int getSn_YPos() {return snY;}

        void builtIn();
protected:
        int rng,cache_size;
        int width,height;
        virtual void initialize();
        virtual void handleMessage(cMessage *msg);
        virtual Packet* newMessage ( int sensor , int size, int val );
        int findDistance(int,int,int,int);
        void calculateEnergy(int , int);

};




#endif /* SN_H_ */
