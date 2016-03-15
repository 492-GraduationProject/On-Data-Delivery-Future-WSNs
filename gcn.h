/*
 * gcn.h
 *
 *  Created on: Dec 14, 2015
 *      Author: mehmet
 */

#ifndef GCN_H_
#define GCN_H_


#include<omnetpp.h>
#include <globals.h>
#include <Packet_m.h>
#include <csimulation.h>
#include <cmodule.h>
#include <ctopology.h>

#define CACHE_SIZE 8*1024*1024*1024*10
#define DELAY 300


class GlobalNode : public cSimpleModule
{
    private :

    int posX;
    int posY;
    int range;
    int LCN[36][2];// two dimensional array (number of lcn and their x and y coord)
    int numberofsent;
    int numberofreceived;
    int portTable[36];

    protected :

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    int controlDistance(int);
    virtual Packet* createMessage();
};



#endif /* GCN_H_ */
