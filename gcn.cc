/*
 * gcn.cc
 *
 *  Created on: Dec 14, 2015
 *      Author: mehmet
 */




#include <cdisplaystring.h>
#include <cobjectfactory.h>
#include <cownedobject.h>
#include <cpar.h>
#include <cregistrationlist.h>
#include <csimplemodule.h>
#include <cwatch.h>
#include <distrib.h>
#include <gcn.h>
#include <regmacros.h>
#include <simutil.h>
#include <stdlib.h>
#include <lcn.h>
#include <fstream>
#include <limits>
static int delay=1;

typedef std::numeric_limits< double > dbl;
Define_Module(GlobalNode);
void GlobalNode::initialize(){

    numberofsent = 0;
    numberofreceived = 0;
    int lcn_x,lcn_y;
    lcn_x  = (getParentModule()->par("width"));
    posX   = lcn_x / 2;
    lcn_x  = lcn_x / 5;
    lcn_y  = (getParentModule()->par("height"));
    posY   = lcn_y/ 2;
    lcn_y = lcn_y / 5;


    WATCH(numberofreceived);
    WATCH(numberofsent);
    for(int i = 0;i<36;i++ )
    {
        LCN[i][0] = (int) (i%6) * lcn_x;
        LCN[i][1] = ((int) i/6) * lcn_y;
    }

    getDisplayString().setTagArg("p", 0, posX);
    getDisplayString().setTagArg("p", 1, posY);


    Packet * paket = createMessage();
    scheduleAt(0.0,paket);

}

void GlobalNode:: handleMessage(cMessage *msg)
{
    static int count=1;

    int routingTable[36]={0,0,1,2,3,3,0,0,1,2,3,3,4,4,5,6,7,7,8,8,9,10,11,11,12,12,13,14,15,15,12,12,13,14,15,15};
    int index_1;
    Packet * paket = check_and_cast<Packet *>(msg);
    std::ofstream newFile1("received.txt", std::ios_base::app);
    newFile1.precision(dbl::max_digits10);
    if(paket-> getType() != REQ)
    {

        numberofreceived++;
        bubble("Data is arrived to the GCN.");
        delete(paket);

           newFile1<<numberofreceived<<endl;
    }
    else
    {
        index_1 = ((paket -> getCoords(1) / 200) *6 + paket-> getCoords(0) / 200);

        send(paket,"gcn_out",routingTable[index_1]);

        Packet * paket = createMessage();

        std::ofstream newFile2("sent.txt", std::ios_base::app);
        newFile2.precision(dbl::max_digits10);
        numberofsent++;


        newFile2<<numberofsent<<endl;


        scheduleAt(delay*300.0,paket);
        count++;
        delay++;
        return;
    }


}


Packet *GlobalNode :: createMessage()
{

    int request_type;
    int assign_Lcn;
    int nearest_lcn;
    int index;
    int size = gateSize("gcn_out");
    Packet *paket = new Packet();//create message

    request_type = intuniform(1,5); // sense type
    paket->setType(REQ);// REQ or DATA
    paket->setSensor(request_type);

    assign_Lcn =intuniform(0,35); //which LCN
    paket->setTempIndex(assign_Lcn);


    paket->setCoords(0,LCN[assign_Lcn][0]);
    paket->setCoords(1,LCN[assign_Lcn][1]);
    paket->setPayload(0);
    paket->setSize(0);




   // ev<<"SENT"<<numberofsent;
    return paket;


}



