/*
 * lcn.cc
 *
 *  Created on: Dec 14, 2015
 *      Author: emre
 */



#include <cmodule.h>
#include <cobjectfactory.h>
#include <cownedobject.h>
#include <cregistrationlist.h>
#include <csimplemodule.h>
#include <csimulation.h>
#include <lcn.h>
#include <regmacros.h>
#include <simutil.h>
#include <envirext.h>
#include <tgmath.h>
#include <limits.h>
#include <fstream>
#include <limits>
#include <simtime.h>
#include <time.h>
#include <stdlib.h>
#include <cgate.h>
#include <math.h>
#include <vector>

typedef std::numeric_limits< double > dbl;
#define Rx 0.312
#define Tx 0.538

static int hop = 0;
static int snNum = 0;
static int gcnNum = 0;
static double tempEnergy[36];
static double energy[36];
static int coordinates[36][2];
static int neigh[36][36];
static int flagLCN[36];
static int  data_cache_value;
std::vector <int>lcn_req_dest;
std::vector <int>lcn_req_type;

std::vector <Packet*>temp;



Define_Module(LCN);

void LCN::initialize() {
    //if(getIndex()==14 || getIndex()==15 || getIndex()==20 || getIndex()==21)
      //  energy[getIndex()]=15000;
       // else
            energy[getIndex()]=getParentModule()->par("lcnBat_Full").doubleValue();
            tempEnergy[getIndex()]=100000;//getParentModule()->par("lcnBat_Full").doubleValue()/2;
            coordinates[getIndex()][0]=(getIndex() % 6)*200;
            coordinates[getIndex()][1]=(getIndex() / 6)*200;

         data_cache_value = getParentModule()->par("lcnCache");

        if(getIndex()==35)
        {
            int i;
            for(i=0;i<36;i++)
                findNeigh(i, coordinates);
            for(int i=0;i<36;i++)
               flagLCN[i]=0;

        }

}



void LCN::handleMessage(cMessage *msg) {
    Packet *pckt = check_and_cast<Packet *>(msg);

    int type=getParentModule()->par("type");
    int fwdIndex = getIndex();
    //tempSN=getIndex();
    int targetX = (pckt->getCoords(0));
    int targetY = (pckt->getCoords(1));
    lcnXcoord = (fwdIndex % 6)*200;
    lcnYcoord = (fwdIndex / 6)*200;

    int target=getIndex();
    int connection_base = gateSize("lcnGCN_out");
    calculateEnergy(Rec,energy);
    calculateEnergy(1,energy);
    hopCountVector.record(hopcount);
    hopCountStats.collect(hopcount);
    if (energy[getIndex()] <= 0.0 && (type==0 || type==1))
       {

           bubble("Node battery depleted");
           endSimulation();

       }


   // ev<<endl<<clock()<<endl;


   // if(energy[getIndex()]<=0.0 && getIndex()!=14 && getIndex()!=15 && getIndex()!=20 && getIndex()!=21)
     //   endSimulation();

    if( energy[14] <= 0.0 && energy[15] <= 0.0 && energy[20] <= 0.0 && energy[21] <= 0.0)
            {
                if(energy[14] <= 0.0)
                    {

                        bubble("Node 14 battery depleted");
                    }

                if(energy[15] <= 0.0)
                    {

                        bubble("Node 15 battery depleted");
                    }

                if(energy[20] <= 0.0)
                    {

                        bubble("Node 20 battery depleted");
                    }

                if(energy[21] <= 0.0)
                    {

                        bubble("Node 21 battery depleted");
                    }

                endSimulation();
            }


   //NNA############################NNA##################################NNA#######################################################
    if(type == 0)
    {
        //NNA starts
    switch (pckt->getType())
    {
    case REQ:

             if( lcnXcoord == targetX)
             {
                 if (lcnYcoord < targetY)  fwdIndex = 3; //up
                 else if (lcnYcoord> targetY) fwdIndex=0;//7-1

             }

             else if(lcnXcoord < targetX)
             {

                if (lcnYcoord < targetY && pckt->getTempIndex() !=35 )
                     fwdIndex = up_left;//up
                else if (lcnYcoord > targetY && pckt->getTempIndex() == 5)
                    fwdIndex = 0; //5
                else if ((lcnYcoord == targetY) && (pckt->getTempIndex() == 11 || pckt->getTempIndex() == 17 || pckt->getTempIndex() == 23 || pckt->getTempIndex() == 29 || pckt->getTempIndex() == 35) )
                    fwdIndex = 2; //11
                else if (lcnYcoord < targetY && pckt->getTempIndex() == 35)
                   fwdIndex = 3; //35
                else fwdIndex = 1;//left


             }
             else {

                 if (lcnYcoord < targetY)
                    fwdIndex = 3;//
                 else if (lcnYcoord > targetY)
                    fwdIndex = 0;//7-0
                 else if (lcnYcoord == targetY && pckt->getTempIndex() == 0)
                    fwdIndex = 0; //0
                 else if (lcnYcoord == targetY && pckt->getTempIndex() == 30)
                    fwdIndex = 1 ; //0
                 else if (lcnYcoord == targetY && pckt->getTempIndex() == 6 || pckt->getTempIndex() == 12 || pckt->getTempIndex() == 18 || pckt->getTempIndex() == 24 )
                    fwdIndex = 1; //6




           }

            if(getIndex() != pckt->getTempIndex())
            {
            send(pckt, "lcnIO$o",fwdIndex);
            }

            if(getIndex() == pckt->getTempIndex() && targetX ==  (pckt->getCoords(0)) && targetY ==  (pckt->getCoords(1)))
                {
                hop=0;
                gcnNum=0;

                flag_req = -1;
                situation=false;
                sensor_analyze(pckt);
                bubble("Data is sent to the LCN.");
                if(calculateEnergy(Trans,energy)<=0)endSimulation();
                }

             break;



    default:
            ev<<"Data "<<endl;
            if(connection_base>0)
            {

                hop++;
                gcnNum++;
                send(pckt,"lcnGCN_out",0);

                if(calculateEnergy(Trans,energy)<=0)endSimulation();
                bubble("Data is routed to the GCN.");
                if(snNum==gcnNum)ev<<"Hop Count"<<hop/snNum<<endl;
                break;
            }

            else
            {

                if(fwdIndex == 7)
                {
                    fwdIndex = 2;////////////
                }
                else if(fwdIndex == 6)
                {
                    fwdIndex = 1;//6 den 7 e
                }
                else if(fwdIndex == 0)
                {
                    fwdIndex = 0;///

                }
                else if (fwdIndex == 10)
                {
                    fwdIndex =1;//sorun

                }
                else if(fwdIndex == 5)
                {
                    fwdIndex = 1;//5 den 10 a
                }
                else if(fwdIndex == 11)
                {
                    fwdIndex = 1;//11 den 10 a
                }
                else if (fwdIndex == 8 || fwdIndex == 9)
                {
                    fwdIndex = down;
                }
                else if(fwdIndex == 1)
                {
                    fwdIndex = 2;//1 den 7 e
                }
                else if(fwdIndex == 2)
                {
                    fwdIndex = 2;//2 den 8 e
                }
                else if(fwdIndex == 4)
                {
                    fwdIndex = 2;//4 den 10 e
                }
                else if(fwdIndex == 3)
                {
                    fwdIndex = 2;//3 den 9 a
                }
                else if (fwdIndex == 26 || fwdIndex == 27)
                {
                    fwdIndex = right;
                }
                else if (fwdIndex == 13|| fwdIndex == 19)
                {
                    fwdIndex = up;
                }
                else if(fwdIndex == 18)
                {
                    fwdIndex = 1;//18 den 13 e
                }
                else if(fwdIndex == 12)
                {
                   fwdIndex = 1;//12 den 13 e
                }
                else if (fwdIndex == 16  || fwdIndex == 22)
                {
                    fwdIndex = left;
                }
                else if(fwdIndex == 17)
                {
                    fwdIndex = 1;//17 den 16 a
                }
                else if(fwdIndex == 23)
                {
                    fwdIndex = 1;//23 den 22 e
                }
                else if  (fwdIndex == 25)
                {
                     fwdIndex = 2;
                }
                else if (fwdIndex == 24)
                {
                    fwdIndex = 1;//24'te 1 right oluyor.
                }
                else if (fwdIndex == 30)
                {
                    fwdIndex = 2;//30'dan 25 e
                }
                else if (fwdIndex == 31)
                {
                    fwdIndex = 0;//31 den 25
                }
                else if (fwdIndex == 32)
                {
                    fwdIndex = 0;// 32den 26 ya
                }
                else if(fwdIndex == 33)
                {
                    fwdIndex = 0;//33 den 27 ye
                }
                else if(fwdIndex == 34)
                {
                    fwdIndex = 0;//34 den 28 e
                }
                else if(fwdIndex == 35)
                {
                    fwdIndex = 0;//35 den 28 e
                }
                else if(fwdIndex == 29)
                {
                    fwdIndex = 1;//29 den 28 e
                }
                else if (fwdIndex == 28)
                {
                     fwdIndex = 1;
                }


            }
            hop++;
            send(pckt,"lcnIO$o",fwdIndex);
            hopcount++;
            if(calculateEnergy(Trans,energy)<=0)endSimulation();
            bubble("Data is sent to the LCN.");
            break;
    }
    //NNA----ENDSSSSSSNNA############################NNA##################################NNA#######################################################
    }







    //SPA-----SPA---------SPA------------SPA----------------SPA--------------------SPA----------------SPA
    else if(type ==1 )
    {


        switch (pckt->getType())
          {
          case REQ:

                   if( lcnXcoord == targetX)
                   {
                       if (lcnYcoord < targetY)  fwdIndex = down; // 25 ten 31 için
                       else if (lcnYcoord> targetY) fwdIndex=right;

                   }

                   else if(lcnXcoord < targetX)
                   {

                      if (lcnYcoord < targetY)
                           fwdIndex = up_left;
                      else if (lcnYcoord > targetY)
                           fwdIndex = 8;//10 dan 5 e için
                      else fwdIndex = up;


                   }
                   else {

                       if (lcnYcoord < targetY)
                          fwdIndex = 9;//25 ten 30 için
                       else if (lcnYcoord > targetY)
                          fwdIndex = 4;//7 den 0 için
                       else fwdIndex = left; //25 ten 24 için



                 }

                  if(getIndex() != pckt->getTempIndex())
                  {
                  send(pckt, "lcnIO$o",fwdIndex);
                  }
                  if(getIndex() == pckt->getTempIndex() && targetX ==  (pckt->getCoords(0)) && targetY ==  (pckt->getCoords(1)))
                      {
                      hop=0;
                      gcnNum=0;
                      flag_req = -1;
                      situation=false;
                      sensor_analyze(pckt);
                      bubble("Data is sent to the LCN.");
                      if(calculateEnergy(Trans,energy)<=0)endSimulation();
                      }

                   break;



          default:
                  ev<<"Data "<<endl;
                  if(connection_base>0)
                  {

                      hop++;
                      gcnNum++;
                      send(pckt,"lcnGCN_out",0);

                      if(calculateEnergy(Trans,energy)<=0)endSimulation();
                      bubble("Data is routed to the GCN.");
                      if(snNum==gcnNum)ev<<"Hop Count"<<hop/snNum<<endl;
                      break;
                  }

                  else
                  {

                      if(fwdIndex == 7)
                      {
                          fwdIndex = up_left;
                      }
                      else if(fwdIndex == 6)
                      {
                          fwdIndex = 1;//6 den 7 e
                      }
                      else if(fwdIndex == 0)
                      {
                          fwdIndex = 2;//0 den 7 e
                      }
                      else if (fwdIndex == 10)
                      {
                          fwdIndex =4;//sorun
                      }
                      else if(fwdIndex == 5)
                      {
                          fwdIndex = 2;//5 den 10 a
                      }
                      else if(fwdIndex == 11)
                      {
                          fwdIndex = 1;//11 den 10 a
                      }
                      else if (fwdIndex == 8 || fwdIndex == 9)
                      {
                          fwdIndex = down;
                      }
                      else if(fwdIndex == 1)
                      {
                          fwdIndex = 2;//1 den 7 e
                      }
                      else if(fwdIndex == 2)
                      {
                          fwdIndex = 2;//2 den 8 e
                      }
                      else if(fwdIndex == 4)
                      {
                          fwdIndex = 2;//4 den 10 e
                      }
                      else if(fwdIndex == 3)
                      {
                          fwdIndex = 2;//3 den 9 a
                      }
                      else if (fwdIndex == 26 || fwdIndex == 27)
                      {
                          fwdIndex = right;
                      }
                      else if (fwdIndex == 13|| fwdIndex == 19)
                      {
                          fwdIndex = up;
                      }
                      else if(fwdIndex == 18)
                      {
                          fwdIndex = 1;//18 den 13 e
                      }
                      else if(fwdIndex == 12)
                      {
                         fwdIndex = 1;//12 den 13 e
                      }
                      else if (fwdIndex == 16  || fwdIndex == 22)
                      {
                          fwdIndex = left;
                      }
                      else if(fwdIndex == 17)
                      {
                          fwdIndex = 1;//17 den 16 a
                      }
                      else if(fwdIndex == 23)
                      {
                          fwdIndex = 1;//23 den 22 e
                      }
                      else if  (fwdIndex == 25)
                      {
                           fwdIndex = down_left;
                      }
                      else if (fwdIndex == 24)
                      {
                          fwdIndex = 1;//24'te 1 right oluyor.
                      }
                      else if (fwdIndex == 30)
                      {
                          fwdIndex = 2;//30'dan 25 e
                      }
                      else if (fwdIndex == 31)
                      {
                          fwdIndex = 0;//31 den 25
                      }
                      else if (fwdIndex == 32)
                      {
                          fwdIndex = 0;// 32den 26 ya
                      }
                      else if(fwdIndex == 33)
                      {
                          fwdIndex = 0;//33 den 27 ye
                      }
                      else if(fwdIndex == 34)
                      {
                          fwdIndex = 0;//34 den 28 e
                      }
                      else if(fwdIndex == 35)
                      {
                          fwdIndex = 2;//35 den 28 e
                      }
                      else if(fwdIndex == 29)
                      {
                          fwdIndex = 1;//29 den 28 e
                      }
                      else if (fwdIndex == 28)
                      {
                           fwdIndex = 6;
                      }


                  }
                  hop++;
                  send(pckt,"lcnIO$o",fwdIndex);
                  hopcount++;
                  if(calculateEnergy(Trans,energy)<=0)endSimulation();
                  bubble("Data is sent to the LCN.");
                  break;
          }
        // ENS SPA END SPA ENDS SPA


    }






    else{
        //Enhanced algorithm starts
        numberofreceived++;
        switch(pckt->getType())

        {

        case REQ:
            if( lcnXcoord == targetX)
                     {
                         if (lcnYcoord < targetY)  fwdIndex = down; // 25 ten 31 için
                         else if (lcnYcoord> targetY) fwdIndex=right;

                     }

                     else if(lcnXcoord < targetX)
                     {

                        if (lcnYcoord < targetY)
                             fwdIndex = up_left;
                        else if (lcnYcoord > targetY)
                             fwdIndex = 8;//10 dan 5 e için
                        else fwdIndex = up;


                     }
                     else {

                         if (lcnYcoord < targetY)
                            fwdIndex = 9;//25 ten 30 için
                         else if (lcnYcoord > targetY)
                            fwdIndex = 4;//7 den 0 için
                         else fwdIndex = left; //25 ten 24 için



                   }

                    if(getIndex() != pckt->getTempIndex())
                    {
                    send(pckt, "lcnIO$o",fwdIndex);
                    }
                    if(getIndex() == pckt->getTempIndex() && targetX ==  (pckt->getCoords(0)) && targetY ==  (pckt->getCoords(1)))
                        {
                        hop=0;
                        gcnNum=0;
                        flag_req = -1;
                        situation=false;
                        sensor_analyze(pckt);
                        bubble("Data is sent to the LCN.");
                        calculateEnergy(1,energy);//endSimulation();
                        }

                     break;

        default:
                        ev<<"Data "<<endl;
                        numberofsent++;
                             if(connection_base>0)
                             {

                                 hop++;
                                 gcnNum++;
                                 send(pckt,"lcnGCN_out",0);

                                 calculateEnergy(1,energy);
                                 bubble("Data is routed to the GCN.");
                                 if(snNum==gcnNum)ev<<"Hop Count"<<hop/snNum<<endl;
                                 break;
                             }
                             else {


                                  target=find(pckt,neigh,target,coordinates,energy,tempEnergy);
                                 ev<<"target "<<target<<endl;
                                 sendPacket(target,pckt);
                               }

                             hop++;

                             calculateEnergy(1,energy);
                             bubble("Data is sent to the LCN.");
                           //  if(target ==14 || target ==15 || target ==20 ||target ==21)flag=0;

            break;


        }

        //Enhanced algorithm ends
    }
}

void LCN::findNeigh(int index, int coordinates[][2]){

    if(index==0 || index==6 ||index==12||index==18 ||index==24)

            {
                       for(int j=0;j<36;j++){
                            if(j==index)
                               neigh[index][j] =M;
                            else if(j-index==1 || j-index==6 || j-index==7)
                            {

                               neigh[index][j] = findDistance(coordinates,index,j),neigh[j][index]=neigh[index][j];
                               //ev<<"distance = "<< neigh[index][j]<<endl;
                            }
                            else if(neigh[index][j]=='\0'){
                                neigh[index][j] = M,neigh[j][index]=M;
                                //ev<<"distance = "<< neigh[index][j]<<endl;
                            }
                        }
            }
    else if (index==1 || index==2 ||index==3||index==4 ||index==7 || index==8 || index==9 ||index==10||index==13 ||index==14 || index==15 || index==16 ||index==19||index==20 ||index==21 || index==22 || index==25 ||index==26||index==27 ||index==28){

                            for(int j=0;j<36;j++){
                                    if(j==index)
                                        neigh[index][j] =M;
                                    else if(j-index==1 || j-index==5 || j-index==6 || j-index==7 )
                                    {

                                       neigh[index][j] = findDistance(coordinates,index,j),neigh[j][index]=neigh[index][j];
                                       //ev<<"distance = "<< neigh[index][j]<<endl;
                                    }
                                    else if(neigh[index][j]=='\0'){
                                        neigh[index][j] = M,neigh[j][index]=M;
                                        //ev<<"distance = "<< neigh[index][j]<<endl;
                                    }
                                }
    }
    else if(index==5 || index==11 ||index==17||index==23 ||index==29)
    {
        for(int j=0;j<36;j++){
                                            if(j==index)
                                                neigh[index][j] =M;
                                            else if(j-index==6 ||j-index==5)
                                            {

                                               neigh[index][j] = findDistance(coordinates,index,j),neigh[j][index]=neigh[index][j];
                                              // ev<<"distance = "<< neigh[index][j]<<endl;
                                            }
                                            else if(neigh[index][j]=='\0'){
                                                neigh[index][j] = M,neigh[j][index]=M;
                                                //ev<<"distance = "<< neigh[index][j]<<endl;
                                            }
                                        }

    }
    else if(index==30 || index==31 ||index==32||index==33||index==34 ||index==35){


                for(int j=0;j<36;j++){
                                            if(j==index)
                                               neigh[index][j] =M;
                                            else if(j-index==1 || j-index==6)
                                            {

                                               neigh[index][j] = findDistance(coordinates,index,j),neigh[j][index]=neigh[index][j];
                                              // ev<<"distance = "<< neigh[index][j]<<endl;
                                            }
                                            else if(neigh[index][j]=='\0'){
                                                neigh[index][j] = M,neigh[j][index]=M;
                                               // ev<<"distance = "<< neigh[index][j]<<endl;
                                            }
                                        }

    }
}

double LCN::findDistance(int coordinates[][2],int i,int j){
    double result;

    if(j-i==7 || j-i==5)

       result= sqrt(pow((coordinates[j][0]-coordinates[i][0]),2)+ pow((coordinates[j][1]-coordinates[i][1]),2));
    else if(j-i==1)
       result=(coordinates[j][0]-coordinates[i][0]);
    else if(j-i==6)
       result=(coordinates[j][1]-coordinates[i][1]);


    //ev<<"result "<<result<<endl;

    return result;
}




int LCN::find(Packet *pckt,int neigh[][36],int target,int coordinates[][2],double *energy, double *tempEnergy){

        int index;double min=10000;
        double x=0;
        double neighEnergy[36][36],y;

                 if(target<=17)
                 {
                     int flag=0;
                       for(int i=0;i<=23;i++)
                       {
                         if( neigh[target][i] !=M )
                           {
                              neighEnergy[target][i]=energy[i];

                                 x=dist(coordinates,i);
                                 if(x<=min)
                                 {
                                    min=x;
                                    y=checkEnergy(neigh,neighEnergy,target,tempEnergy);

                                    ev<<"ASIL TARGET"<<target<<endl;
                                    ev<<"TEMP ENERGY "<<y<<endl;
                                    ev<<"ENERGRY of index" <<i <<"is "<<" "<<energy[i]<<endl;

                                    if( energy[i] >=y && i > target)
                                    {
                                        flag=1;
                                        index=i;
                                        ev<<"TEMP ENERGY "<<y<<endl;
                                        ev<<"ENERGRY of index" <<i <<"is "<<" "<<energy[i]<<endl;
                                    }
                                    else if(flag==0)
                                        index=selectMax(neigh,neighEnergy,target,tempEnergy,energy,coordinates);
                                    else
                                        index=selectMax(neigh,neighEnergy,target,tempEnergy,energy,coordinates);
                                 }

                            }

                        }
                  }

                  else{

                       if(target >17 &&target < 35)
                       {
                           int flag=0;


                                                  for(int i=0;i<=36;i++)
                                                  {
                                                    if( neigh[target][i] !=M )
                                                      {
                                                        neighEnergy[target][i]=energy[i];

                                                            x=dist(coordinates,i);
                                                            if(x<=min)
                                                            {
                                                               min=x;
                                                               y=checkEnergy(neigh,neighEnergy,target,tempEnergy);

                                                               ev<<"ASIL TARGET"<<target<<endl;
                                                               ev<<"TEMP ENERGY "<<y<<endl;
                                                               ev<<"ENERGRY of index" <<i <<"is "<<" "<<energy[i]<<endl;

                                                               if( energy[i] >=y && i > target)
                                                               {
                                                                   flag=1;
                                                                   index=i;
                                                                   ev<<"TEMP ENERGY "<<y<<endl;
                                                                   ev<<"ENERGRY of index" <<i <<"is "<<" "<<energy[i]<<endl;
                                                               }
                                                               else if(flag==0)
                                                                   index=selectMax(neigh,neighEnergy,target,tempEnergy,energy,coordinates);
                                                               else
                                                                  index=selectMax(neigh,neighEnergy,target,tempEnergy,energy,coordinates);
                                                            }

                                                       }

                                                   }
                         }
                         else{

                             int flag=0;

                             ev<<"elsete"<<endl;
                                                    for(int i=0;i<target;i++)
                                                    {
                                                      if( neigh[target][i] !=M )
                                                        {
                                                          neighEnergy[target][i]=energy[i];

                                                              x=dist(coordinates,i);
                                                              if(x<=min)
                                                              {
                                                                 min=x;
                                                                 y=checkEnergy(neigh,neighEnergy,target,tempEnergy);

                                                                 ev<<"ASIL TARGET"<<target<<endl;
                                                                 ev<<"TEMP ENERGY "<<y<<endl;
                                                                 ev<<"ENERGRY of index" <<i <<"is "<<" "<<energy[i]<<endl;

                                                                 if( energy[i] >=y && i > target)
                                                                 {
                                                                     flag=1;
                                                                     index=i;
                                                                     ev<<"TEMP ENERGY "<<y<<endl;
                                                                     ev<<"ENERGRY of index" <<i <<"is "<<" "<<energy[i]<<endl;
                                                                 }
                                                                 else if(flag==0)
                                                                     index=selectMax(neigh,neighEnergy,target,tempEnergy,energy,coordinates);
                                                                 else
                                                                     index=selectMax(neigh,neighEnergy,target,tempEnergy,energy,coordinates);

                                                              }

                                                         }

                                                    }
                         }
                      }
      return index;
    }
double LCN::checkEnergy(int neigh[][36],double neighEnergy[][36],int target,double *tempEnergy){
         int size=0,j=0;
         for (int i=0;i<36;i++)
         {
             if(neigh[target][i]!=M)    // komsu sayisini bulmak icin
                 size++;
         }

         int index[size]; //komsu indexleri tutmak icin
         int flag[size];

         for (int i=0;i<size;i++)
             flag[i]=0;

         for (int i=0;i<36&&j<size;i++) // komsu indexleri almak icin
                {
                    if(neigh[target][i]!=M)
                       index[j++]=i;
                }

         for (int i=0;i<size;i++) //energyler tempEnergyden dusukse update etmek icin
                         {
                             if(neighEnergy[target][index[i]]<=tempEnergy[target])
                                      flag[i]=1;
                         }



         int check=0;
         for (int i=0;i<size;i++)
             check+=flag[i];

         for (int i=0;i<size;i++)
                      flag[i]=0;

         if(check==size)

            tempEnergy[target]=tempEnergy[target]/2.0;
         else
             tempEnergy[target];

         return tempEnergy[target];
     }


int LCN::selectMax(int neigh[][36],double neighEnergy[][36],int target,double *tempEnergy,double *energy,int coordinates[][2])
    {
        int size=0,j=0;double min=10000,x=0,z=0;


            for (int i=0;i<36;i++)
                          {
                            if(neigh[target][i]!=M)    // komsu sayisini bulmak icin
                            size++;
                          }



        int index[size]; //komsu indexleri tutmak icin

        if(target<35)
        {
            for (int i=0;i<36&&j<size;i++) // komsu indexleri almak icin
                                      {
                                          if(neigh[target][i]!=M)
                                             index[j++]=i;
                                      }
        }


        if(target==35)
                {

            ev<<"35te"<<endl;
                    for (int i=0;i<target && j<size;i++) // komsu indexleri almak icin
                                                         {
                                                             if(neigh[target][i]!=M)
                                                                index[j++]=i;
                                                         }

                }
        int max=-1000,result;



        for (int i=0;i<size;i++) //energyler tempEnergyden dusukse update etmek icin
            {

            if(target==1)
                                     {
                                        if(neighEnergy[target][index[i]]>=max&& target-index[i]!=1 )
                                             {
                                            max=neighEnergy[target][index[i]];
                                             result =index[i];
                                          }
                                     }
            if(target==2 ||target==3)
                         {
                            if(neighEnergy[target][index[i]]>=max&& target-index[i]!=-1 && target-index[i]!=1)
                                 {
                                max=neighEnergy[target][index[i]];
                                 result =index[i];
                              }
                         }




            else if(target==4  )
                {
                   if(neighEnergy[target][index[i]]>=max&& target-index[i]!=-1 && target-index[i]!=1 && target-index[i]!=-7 )
                        {
                       max=neighEnergy[target][index[i]];
                        result =index[i];
                     }
                }




            else if(target==0||target==5 )
                            {


                               if(neighEnergy[target][index[i]]>=max )
                                    {
                                       ev<<"burdaim"<<endl;

                                        max=neighEnergy[target][index[i]];

                                        result =index[i];
                                    }
                            }

            else if(target==35 )
                                        {


                                           if(neighEnergy[target][index[i]]>=max && index[i]!=35)
                                                {
                                                 x=dist(coordinates,index[i]);
                                                  if (z<=min)
                                                   {
                                                    min=z;

                                                    max=neighEnergy[target][index[i]];

                                                    result =index[i];
                                                   }
                                                }
                                        }
            else if(target==30)
            {


                                           if(neighEnergy[target][index[i]]>=tempEnergy[index[i]] )
                                                {
                                                   x=dist(coordinates,index[i]);
                                                   if (x<=min)
                                                   {
                                                       min=x;
                                                        max=neighEnergy[target][index[i]];

                                                        result =index[i];
                                                   }



                                                }
                                    }


            else if(target==6)
                                     {
                                        if(neighEnergy[target][index[i]]>=max && target-index[i]!=6 && target-index[i]!=5)
                                             {
                                            max=neighEnergy[target][index[i]];
                                             result =index[i];
                                          }
                                     }




            else if(target==7)
                                     {
                                        if(neighEnergy[target][index[i]]>=max && target-index[i]!=-5 && target-index[i]!=1 && target-index[i]!=7 && target-index[i]!=6 && target-index[i]!=5)
                                             {
                                            max=neighEnergy[target][index[i]];
                                             result =index[i];
                                          }

                                        if(neighEnergy[target][index[i]]>=max && target-index[i]!=-5 && target-index[i]!=1 && target-index[i]!=7 && target-index[i]!=6 && target-index[i]!=-6 && energy[14]<=0.0)
                                                                                    {
                                                                                   max=neighEnergy[target][index[i]];
                                                                                    result =index[i];
                                                                                 }
                                     }



            else if(target==8)
             {
              if(neighEnergy[target][index[i]]>=max && target-index[i]!=1 && target-index[i]!=-1 && target-index[i]!=7 && target-index[i]!=6 && target-index[i]!=5)
                                             {
                                                max=neighEnergy[target][index[i]];
                                             result =index[i];
                                          }
              if(neighEnergy[target][index[i]]>=max && target-index[i]!=1 && target-index[i]!=-1 && target-index[i]!=-7 && target-index[i]!=7 && energy[14]<=0.0)
                                                          {
                                                             max=neighEnergy[target][index[i]];
                                                          result =index[i];
                                                       }

             }

            else if(target==9 || target==10 )
                            {
                               if(neighEnergy[target][index[i]]>=max && target-index[i]!=-1  && target-index[i]!=1  && target-index[i]!=6 &&target-index[i]!=5 && target-index[i]!=-7&& target-index[i]!=7)
                                    {
                                   max=neighEnergy[target][index[i]];
                                    result =index[i];
                                 }

                               if(neighEnergy[target][index[i]]>=max &&target==9 && target-index[i]!=-1 && target-index[i]!=6 &&target-index[i]!=5 && target-index[i]!=-7&& target-index[i]!=7 && energy[14]<=0.0 && energy[15]<=0.0)
                               {
                                   max=neighEnergy[target][index[i]];
                                   result =index[i];
                               }

                               if(neighEnergy[target][index[i]]>=max &&target==10 && target-index[i]!=-1 && target-index[i]!=6 &&target-index[i]!=5 && target-index[i]!=-7&& target-index[i]!=7  && energy[15]<=0.0)
                                                              {
                                                                  max=neighEnergy[target][index[i]];
                                                                  result =index[i];
                                                              }
                            }

            else if(target==11 )
                   {
                      if(neighEnergy[target][index[i]]>=max && target-index[i]!=7 &&  target-index[i]!=6 && target-index[i]!=-6)
                           {
                               max=neighEnergy[target][index[i]];
                               result =index[i];
                            }
                   }


            else if(target==12 || target==18)
                   {
                      if(neighEnergy[target][index[i]]>=max && target-index[i]!=6 &&  target-index[i]!=-6)
                           {
                               max=neighEnergy[target][index[i]];
                               result =index[i];
                            }
                   }




            else if(target==13 )
                   {
                      if(neighEnergy[target][index[i]]>=max && target-index[i]!=7 &&  target-index[i]!=1 && target-index[i]!=-5 && target-index[i]!=5 && target-index[i]!=-6 && target-index[i]!=6)
                           {
                               max=neighEnergy[target][index[i]];
                               result =index[i];
                            }
                      if(neighEnergy[target][index[i]]>=max && target-index[i]!=7&& target-index[i]!=5 &&  target-index[i]!=1 && target-index[i]!=-5 && energy[14]<=0.0 && energy[20]<=0.0)
                                              {
                                                  max=neighEnergy[target][index[i]];
                                                  result =index[i];
                                               }

                   }




            else if(target==14 || target==15 || target==20 || target==21)
                                       {
                                          if(neighEnergy[target][index[i]]>=max)
                                               {
                                              max=neighEnergy[target][index[i]];
                                               result =index[i];
                                            }
                                       }

            else if(target==16 )
                           {
                              if(neighEnergy[target][index[i]]>=max && target-index[i]!=5 && target-index[i]!=-1 &&target-index[i]!=-7 && target-index[i]!=6 )
                                   {
                                  max=neighEnergy[target][index[i]];
                                   result =index[i];
                                }

                              if(neighEnergy[target][index[i]]>=max && target-index[i]!=5 && target-index[i]!=-1 &&target-index[i]!=-7 && energy[15]<=0.0 && energy[21]<=0.0 )
                                   {
                                  max=neighEnergy[target][index[i]];
                                   result =index[i];
                                }
                           }


            else if(target==17)
                           {
                              if(neighEnergy[target][index[i]]>=max && target-index[i]!=6)
                                   {
                                  max=neighEnergy[target][index[i]];
                                   result =index[i];
                                }
                           }



            else if(target==19 )
                           {
                              if(neighEnergy[target][index[i]]>=max && target-index[i]!=7 &&  target-index[i]!=1 && target-index[i]!=-5&& target-index[i]!=-6 && target-index[i]!=6 && target-index[i]!=-7)
                                   {
                                       max=neighEnergy[target][index[i]];
                                       result =index[i];
                                    }
                              if(neighEnergy[target][index[i]]>=max && target-index[i]!=7  && target-index[i]!=-7 &&  target-index[i]!=1 && target-index[i]!=-6 && target-index[i]!=-5  && energy[14]<=0.0 && energy[20]<=0.0)
                                                              {
                                                                  max=neighEnergy[target][index[i]];
                                                                  result =index[i];
                                                               }
                           }


            else if(target==22 )
                                      {
                                         if(neighEnergy[target][index[i]]>=max && target-index[i]!=5 && target-index[i]!=-1 &&target-index[i]!=-5 &&target-index[i]!=-7 &&target-index[i]!=-6 &&target-index[i]!=6)
                                              {
                                             max=neighEnergy[target][index[i]];
                                              result =index[i];
                                           }
                                         if(neighEnergy[target][index[i]]>=max && target-index[i]!=5 && target-index[i]!=-1 &&target-index[i]!=-7 && energy[15]<=0.0 && energy[21]<=0.0)
                                              {
                                             max=neighEnergy[target][index[i]];
                                              result =index[i];
                                           }



                                      }

            else if(target==24 )
                                      {
                                         if(neighEnergy[target][index[i]]>=max && target-index[i]!=6 &&  target-index[i]!=-6 && target-index[i]!=-7)
                                              {
                                                  max=neighEnergy[target][index[i]];
                                                  result =index[i];
                                               }
                                      }



            else if(target==25 )
                                      {
                                         if(neighEnergy[target][index[i]]>=max && target-index[i]!=7 &&  target-index[i]!=1 && target-index[i]!=-5 && target-index[i]!=-6 && target-index[i]!=-7)
                                              {
                                                  max=neighEnergy[target][index[i]];
                                                  result =index[i];
                                               }


                                      }



            else if(target==26 )
                      {
                       if(neighEnergy[target][index[i]]>=max && target-index[i]!=7 &&  target-index[i]!=-1 && target-index[i]!=-5 && target-index[i]!=-6 && target-index[i]!=-7 && target-index[i]!=1)
                         {
                          max=neighEnergy[target][index[i]];
                          result =index[i];
                         }

                       if(neighEnergy[target][index[i]]>=max && target-index[i]!=-5 && target-index[i]!=1 && target-index[i]!=-6 && target-index[i]!=-7 && target-index[i]!=1 && energy[20]<=0.0 && energy[21]<=0.0)
                         {
                          max=neighEnergy[target][index[i]];
                          result =index[i];
                         }
                      }



            else if(target==27 )
                           {
                              if(neighEnergy[target][index[i]]>=max && target-index[i]!=5 &&  target-index[i]!=-1 && target-index[i]!=-7 && target-index[i]!=-6 && target-index[i]!=-5)
                                   {
                                  max=neighEnergy[target][index[i]];
                                   result =index[i];
                                }

                              if(neighEnergy[target][index[i]]>=max &&  target-index[i]!=-1 && target-index[i]!=-7 && target-index[i]!=-6 && target-index[i]!=-5 && energy[20]<=0.0 && energy[21]<=0.0)
                                   {
                                  max=neighEnergy[target][index[i]];
                                   result =index[i];
                                }
                           }
            else if(target==28)
                           {
                              if(neighEnergy[target][index[i]]>=max && target-index[i]!=5 &&  target-index[i]!=-1 && target-index[i]!=-7 && target-index[i]!=-6 && target-index[i]!=-5)
                                   {
                                  max=neighEnergy[target][index[i]];
                                   result =index[i];
                                }
                           }


            else if(target==23 )
                                      {
                                         if(neighEnergy[target][index[i]]>=max && target-index[i]!=-6 && target-index[i]!=6 )
                                              {
                                             max=neighEnergy[target][index[i]];
                                              result =index[i];
                                           }

                                      }
            else if(target == 29)
                                            {
                                               if(neighEnergy[target][index[i]]>=max && target-index[i]!=-6 && target-index[i]!=-5 && target-index[i]!=6 )
                                                    {
                                                   max=neighEnergy[target][index[i]];
                                                    result =index[i];
                                                 }

                                            }




            else if(target==31 || target ==32)
                                                 {
                                                    if(neighEnergy[target][index[i]]>=max && target-index[i]!=-1 && target-index[i]!=1)
                                                         {
                                                        max=neighEnergy[target][index[i]];
                                                         result =index[i];
                                                      }
                                                 }



            else if(target==33 || target==34 )
                                      {
                                         if(neighEnergy[target][index[i]]>=max && target-index[i]!=-1 &&  target-index[i]!=1)
                                              {

                                              max=neighEnergy[target][index[i]];
                                              result =index[i];
                                              ev<<"REsult "<<result<<endl;
                                           }
                                      }
            }
        return result;
    }





void LCN::sendPacket(int target,Packet *pckt){
            hopcount++;
            const char * gateName = "lcnIO$o";
                               int destIndex = target; // index of destination lcn
                               cGate *destGate = NULL;
                               bool found = false;

                               int i = 0;
                               int gateSize = gate(gateName, 0)->size();

                               do {
                                   destGate = gate(gateName, i++);
                                   cGate *nextGate = destGate->getNextGate();
                                   if (nextGate && nextGate->getOwnerModule()->getIndex() == destIndex) {
                                       found = true;
                                       send(pckt, destGate);
                                   }
                               } while (!found && i < gateSize);
        }



void LCN::sensor_analyze(Packet *pckt) {

    int g_size = gateSize("lcnSN$o");
    //srand(12345);
    for(int i=0;i<10;i++){
        int random=intuniform(0,(g_size/6)-1);
       // temp.push_back(pckt->dup());

        send((Packet *) pckt->dup(), "lcnSN$o", random);
        if(flag_req<0){
            while(situation==false){
                if(data_cache_value>=6){
                    lcn_req_type.push_back(pckt->getSensor());
                    lcn_req_dest.push_back(getIndex());
                    data_cache_value = data_cache_value - 6;
                    flag_req = 1;
                    situation=true;
                    ev<<lcn_req_type[0]<< "    "<<lcn_req_dest[0]<<endl;
                }
                lcn_req_type.erase(lcn_req_type.begin());
                lcn_req_dest.erase(lcn_req_dest.begin());
                data_cache_value = data_cache_value + 6;
            }
      }
        //ev<<lcn_req_all[0][0]<<endl;

        calculateEnergy(1,energy);
    }
    snNum=g_size/6;

}



double LCN::calculateEnergy(int RxTx,double *energy){
    if(RxTx==0)
    {
        energy[getIndex()]=energy[getIndex()]-(Rx*hopBits);
       // ev<<"REceive "<<"Index is "<<getIndex()<<"  energy is "<<energy[getIndex()];
    }
    else
        {
        energy[getIndex()]=energy[getIndex()]-(Tx*hopBits);
       // ev<<"Transmit "<<"Index is "<<getIndex()<<"  energy is "<<energy[getIndex()];
        }


    std::ofstream newFile1("scorefile9_0.txt", std::ios_base::app);
    std::ofstream newFile2("scorefile9_1.txt", std::ios_base::app);
    std::ofstream newFile3("scorefile9_2.txt", std::ios_base::app);

    std::ofstream newFile4("scorefile20_0.txt", std::ios_base::app);
    std::ofstream newFile5("scorefile20_1.txt", std::ios_base::app);
    std::ofstream newFile6("scorefile20_2.txt", std::ios_base::app);


    newFile1.precision(dbl::max_digits10);
    newFile2.precision(dbl::max_digits10);
    newFile3.precision(dbl::max_digits10);
    newFile4.precision(dbl::max_digits10);
    newFile5.precision(dbl::max_digits10);
    newFile6.precision(dbl::max_digits10);

    if(getIndex() ==9 && newFile1.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile1<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 9 && newFile2.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile2<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 9 && newFile2.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile3<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==20 && newFile1.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile4<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==20 && newFile2.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile5<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==20 && newFile2.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile6<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    return energy[getIndex()];

}
void LCN::finish()
{
    // This function is called by OMNeT++ at the end of the simulation.
   // EV << "Sent:     " << numberofsent << endl;
    //EV << "Received: " << numberofreceived << endl;
    //EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
   // EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
    EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
  //  EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

   // recordScalar("#sent", numberofsent);
    //recordScalar("#received", numberofreceived);
    //std::vector<std::vector<int>>lcn_req_all={move(lcn_req_type),move(lcn_req_dest)};
    hopCountStats.recordAs("hop count");
}
