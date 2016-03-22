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
#include <algorithm>
#include "sn.h"
#include "lcn.h"
#include <Packet_m.h>
#include <vector>
static int data_cache_value_sn;
Define_Module(SensorNode);

std:: vector <int> sense_type_sn;
std:: vector <int> sense_value_sn;
std:: vector <int> sn_target_coord;
int kordinat;

void SensorNode::initialize(){

 data_cache_value_sn = getParentModule()->par("snCacheSize");
totPower = batteryCapacity;

}



void SensorNode:: handleMessage(cMessage *msg){


    int size;
    int power;
    int val;
    static int deger[5] = { 0,0,0,0,0};
    Packet * packet = check_and_cast <Packet *>(msg) ;
    int a,b;
    a = packet -> getCoords(1) / 200 *6;
    b = packet-> getCoords(0) / 200;
    kordinat = a+b;
    sn_target_coord.push_back(kordinat);

    switch (packet->getSensor())
    {

    case temp:
        size = getParentModule()->par("temp_dataSize");
        power =getParentModule()->par("bat_Drain_Temp");
        val = intuniform(0,50);
        deger[0]++;

        sn_caching(1,false, val);
        ev<< sense_type_sn[0] << "     " << sense_value_sn[0] << "      "<<data_cache_value_sn << endl;
        break;

    case pres :
        size = getParentModule()->par("pres_dataSize");
        power =getParentModule()->par("bat_Drain_Pressure");
        val =intuniform(0,100);
        deger[1]++;
        sn_caching(2,false, val);
        ev<< sense_type_sn[0] << "     " << sense_value_sn[0] << "      "<<data_cache_value_sn << endl;
        break;


    case hum:
        size = getParentModule()->par("hum_dataSize");
        power =getParentModule()->par("bat_Drain_Humudity");
        val = intuniform(0,100);
        deger[2]++;

        sn_caching(4,false, val);
        ev<< sense_type_sn[0] << "     " << sense_value_sn[0] << "      "<<data_cache_value_sn << endl;
        break;

    case co:
        size =getParentModule()->par("CO_dataSize");
        power =getParentModule()->par("bat_Drain_Temp");
        val = intuniform(0,100);
        deger[3]++;

        sn_caching(3, false, val);
        ev<< sense_type_sn[0] << "     " << sense_value_sn[0] << "      "<<data_cache_value_sn << endl;

        break;


    default:
        size = getParentModule()->par("CO2_dataSize");
        power =getParentModule()->par("bat_Drain_Co2");
        val = intuniform(0,100);
        deger[4]++;

        sn_caching(5,false, val);
        ev<< sense_type_sn[0] << "     " << sense_value_sn[0] << "      "<<data_cache_value_sn << endl;

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
   // std::vector<Packet*>temp;
   // temp.push_back(newMessage(packet->getSensor(),size,val));
    //ev<<"index in sensor"<<tempSN<<endl;
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
void SensorNode::sn_caching(int type, bool sit, int val){

    while (sit==false){

                if(data_cache_value_sn>=9){
                    sense_type_sn.push_back(type);
                    sense_value_sn.push_back(val);
                    data_cache_value_sn = data_cache_value_sn -9;
                    sit = true;

                }

                else {

                       bool break_cond=false;

                       for(int count=1; count<=sense_value_sn.size(); count++ ){

                           if(sense_type_sn[count]==1){
                               sense_type_sn.erase(sense_type_sn.begin()+count-1);
                               sense_value_sn.erase(sense_value_sn.begin()+count-1);
                               data_cache_value_sn = data_cache_value_sn +9;
                               break_cond=true;
                               //ev<<"1 " << endl;
                           }
                           break;
                       }

                       if (break_cond==true) break;

                       for(int count=1; count<=sense_value_sn.size(); count++){
                           if(sense_type_sn[count]==2){
                           sense_type_sn.erase(sense_type_sn.begin()+count-1);
                           sense_value_sn.erase(sense_value_sn.begin()+count-1);
                           data_cache_value_sn = data_cache_value_sn +9;
                           break_cond=true;
                           //ev<<"2 " << endl;
                           }
                           break;
                       }

                       if (break_cond==true) break;

                       for(int count=1; count<=sense_value_sn.size(); count++ ){
                            if(sense_type_sn[count]==3){
                            sense_type_sn.erase(sense_type_sn.begin()+count-1);
                            sense_value_sn.erase(sense_value_sn.begin()+count-1);
                            data_cache_value_sn = data_cache_value_sn +9;
                            break_cond=true;
                            //ev<<"3 " << endl;
                            }
                            break;
                        }

                        if (break_cond==true) break;

                        for(int count=1; count<=sense_value_sn.size(); count++ ){
                            if(sense_type_sn[count]==4){
                            sense_type_sn.erase(sense_type_sn.begin()+count-1);
                            sense_value_sn.erase(sense_value_sn.begin()+count-1);
                            data_cache_value_sn = data_cache_value_sn +9;
                            break_cond=true;
                            //ev<<"4 " << endl;
                            }
                            break;
                        }

                        if (break_cond==true) break;

                        for(int count=1; count<=sense_value_sn.size(); count++ ){
                             if(sense_type_sn[count]==5){
                             sense_type_sn.erase(sense_type_sn.begin()+count-1);
                             sense_value_sn.erase(sense_value_sn.begin()+count-1);
                             data_cache_value_sn = data_cache_value_sn +9;
                             //ev<<"5 " << endl;
                             }
                             break;
                        }
                }
            }

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
