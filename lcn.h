/*
 * lcn.h
 *
 *  Created on: Dec 14, 2015
 *      Author: mehmet
 */

#ifndef LCN_H_
#define LCN_H_


#include <omnetpp.h>
#include <cmessage.h>
#include <ccomponent.h>
#include <Packet_m.h>
#define RIGHT 0
#define LEFT 1
#define UP 2
#define DOWN 3
#define UP_RIGHT 4
#define UP_LEFT 5
#define DOWN_RIGHT 6
#define DOWN_LEFT 7

#define hopBits 10.0
#define M -1
#define LCN_CACHE 2*1024*1024*1024


enum RxTx{Rec=0,Trans=1};
enum direction{right=0, left=1, up=2, down=3, up_right=4, up_left =5, down_right=6, down_left=7};
//static int tempSN;

class LCN : public cSimpleModule {

private:
    int hopcount=0;
    int numberofsent = 0;
    int numberofreceived = 0;
    int lcnXcoord ;
    int lcnYcoord ;
int flag_req;
bool situation;
    long lcnBtrySt;
    double Rx;
    double Tx;
    cLongHistogram hopCountStats;
    cOutVector hopCountVector;

protected:

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    void forwardMessage (Packet *pckt, int port);
    void sensor_analyze(Packet *pckt);
    double calculateEnergy(int RxTx,double *energy);
    double findDistance(int coordinates[][2],int i,int j);
    void findNeigh(int index,int coordinates[][2]);
    virtual void finish();
    double dist(int coordinates[][2],int i){


        return sqrt(pow((500-coordinates[i][0]),2)+ pow((500-coordinates[i][1]),2));
    }



    void sendPacket(int target,Packet *pckt);
    int find(Packet *pckt,int neigh[][36],int target,int coordinates[][2],double *energy,double *tempEnergy);
    void disconnectPath();

    double checkEnergy(int neigh[][36],double neighEnergy[][36],int target,double *tempEnergy);

    int selectMax(int neigh[][36],double neighEnergy[][36],int target,double *tempEnergy,double *energy,int coordinates[][2]);

};




#endif /* LCN_H_ */
