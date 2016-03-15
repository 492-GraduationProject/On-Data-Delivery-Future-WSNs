/*
 * sn.cc
 *
 *  Created on: Dec 14, 2015
 *      Author: mehmet
 */



#include <omnetpp.h>
#include <cnedvalue.h>
#include <time.h>
#include <stdlib.h>
#include <distrib.h>
#include <envirext.h>

#include "sn.h"
#include "lcn.h"
#include <Packet_m.h>

Define_Module(SensorNode);
void SensorNode::initialize(){


totPower = batteryCapacity;

}

void SensorNode:: handleMessage(cMessage *msg){


    int size;
    int power;
    int val;
    static int deger[5] = { 0,0,0,0,0};
    Packet * packet = check_and_cast <Packet *>(msg) ;
    switch (packet->getSensor())
    {
    case temp:
        size = getParentModule()->par("temp_dataSize");
        power =getParentModule()->par("bat_Drain_Temp");
        val = intuniform(0,100);
        deger[0]++;
        break;
    case pres :
        size = getParentModule()->par("pres_dataSize");
        power =getParentModule()->par("bat_Drain_Pressure");
        val =intuniform(0,100);
        deger[1]++;
        break;


    case hum:
        size = getParentModule()->par("hum_dataSize");
        power =getParentModule()->par("bat_Drain_Humudity");
        val = intuniform(0,100);
        deger[2]++;
        break;


    case co:
        size =getParentModule()->par("CO_dataSize");
        power =getParentModule()->par("bat_Drain_Temp");
        val = intuniform(0,100);
        deger[3]++;
        break;


    default:
        size = getParentModule()->par("CO2_dataSize");
        power =getParentModule()->par("bat_Drain_Co2");
        val = intuniform(0,100);
        deger[4]++;
        break;
    }
    Packet *newPacket = newMessage(packet->getSensor(),size,val) ;
   // ev<<"Value is "<<newPacket->getPayload()<<endl;
    //ev<<"Temperature count is"<<deger[0]<<endl;
    //ev<<"Pressure count is"<<deger[1]<<endl;
    //ev<<"Humidity count is"<<deger[2]<<endl;
    //ev<<"C0 count is"<<deger[3]<<endl;
    //ev<<"C02 count is"<<deger[4]<<endl;

    calculateEnergy (Receive,0) ;
    calculateEnergy (Sense,power );
    calculateEnergy (Transmit,size) ;


   send (newPacket ,"snIO$o",0) ;  // Sends a message through the gate given with its pointer.
   bubble("Data is send to the LCN.");


   delete ( packet ) ;


}
Packet*SensorNode::newMessage ( int sensor , int size,int val ){


    Packet * packet = new Packet () ;
    packet->setType (DATA);
    packet->setSensor(sensor) ;
    packet->setCoords(0,500);
    packet->setCoords(1,500);
    packet->setPayload(val) ;
    packet->setSize(9+size) ;
    return packet ;

}

int SensorNode::findDistance(int snX,int snY, int lcnX,int lcnY)
{
    int distance;
    for(int i = 0; i < 36; i++)
         {

             distance = sqrt(pow( (lcnX - snX) , 2) + pow((lcnY - snY), 2));
         }
    return distance;
}

void SensorNode::calculateEnergy ( int type , int s ) {
    ev<<"totPower Before"<<totPower<<endl;
    if ( type == Receive)
        {
        totPower -= drainReception* 6 ;
        //ev<<"totPower REceive"<<totPower<<endl;
        }
    else if (type == Transmit){
        totPower -= drainTransmitting*(9+s) ;
        //ev<<"totPower Transmit"<<totPower<<endl;
    }
    else{
        totPower -= s;
       // ev<<"totPower sense"<<totPower<<endl;
        }

}
