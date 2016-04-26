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


Define_Module(LCN);

void LCN::initialize() {
    
            energy[getIndex()]=getParentModule()->par("lcnBat_Full").doubleValue();
            tempEnergy[getIndex()]=100000;//getParentModule()->par("lcnBat_Full").doubleValue()/2;
            coordinates[getIndex()][0]=(getIndex() % 6)*200;
            coordinates[getIndex()][1]=(getIndex() / 6)*200;
           // ev<<"index "<<getIndex()<<"Coordinates   "<<coordinates[getIndex()][0]<<" "<< coordinates[getIndex()][1]<<endl;


         data_cache_value = getParentModule()->par("lcnCache");

        //int size_of_cahce_array= data_cache_value % 9;

        //int *data_cache = new int[data_cache_value]; // cache to bytes then divided by 9 (because of incoming data size is 9).
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
    if (energy[getIndex()] <= 0.0 && (type==0 || type==1 || type ==2))
       {

           bubble("Node battery depleted");
           endSimulation();

       }


  /*
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

*/
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


    std::ofstream newFile1("scorefile0_0.txt", std::ios_base::app);
    std::ofstream newFile2("scorefile0_1.txt", std::ios_base::app);
    std::ofstream newFile3("scorefile0_2.txt", std::ios_base::app);

    std::ofstream newFile4("scorefile1_0.txt", std::ios_base::app);
    std::ofstream newFile5("scorefile1_1.txt", std::ios_base::app);
    std::ofstream newFile6("scorefile1_2.txt", std::ios_base::app);

    std::ofstream newFile7("scorefile2_0.txt", std::ios_base::app);
    std::ofstream newFile8("scorefile2_1.txt", std::ios_base::app);
    std::ofstream newFile9("scorefile2_2.txt", std::ios_base::app);

    std::ofstream newFile10("scorefile3_0.txt", std::ios_base::app);
    std::ofstream newFile11("scorefile3_1.txt", std::ios_base::app);
    std::ofstream newFile12("scorefile3_2.txt", std::ios_base::app);

    std::ofstream newFile13("scorefile4_0.txt", std::ios_base::app);
    std::ofstream newFile14("scorefile4_1.txt", std::ios_base::app);
    std::ofstream newFile15("scorefile4_2.txt", std::ios_base::app);

    std::ofstream newFile16("scorefile5_0.txt", std::ios_base::app);
    std::ofstream newFile17("scorefile5_1.txt", std::ios_base::app);
    std::ofstream newFile18("scorefile5_2.txt", std::ios_base::app);

    std::ofstream newFile19("scorefile6_0.txt", std::ios_base::app);
    std::ofstream newFile20("scorefile6_1.txt", std::ios_base::app);
    std::ofstream newFile21("scorefile6_2.txt", std::ios_base::app);

    std::ofstream newFile22("scorefile7_0.txt", std::ios_base::app);
    std::ofstream newFile23("scorefile7_1.txt", std::ios_base::app);
    std::ofstream newFile24("scorefile7_2.txt", std::ios_base::app);

    std::ofstream newFile25("scorefile8_0.txt", std::ios_base::app);
    std::ofstream newFile26("scorefile8_1.txt", std::ios_base::app);
    std::ofstream newFile27("scorefile8_2.txt", std::ios_base::app);

    std::ofstream newFile28("scorefile9_0.txt", std::ios_base::app);
    std::ofstream newFile29("scorefile9_1.txt", std::ios_base::app);
    std::ofstream newFile30("scorefile9_2.txt", std::ios_base::app);

    std::ofstream newFile31("scorefile10_0.txt", std::ios_base::app);
    std::ofstream newFile32("scorefile10_1.txt", std::ios_base::app);
    std::ofstream newFile33("scorefile10_2.txt", std::ios_base::app);

    std::ofstream newFile34("scorefile11_0.txt", std::ios_base::app);
    std::ofstream newFile35("scorefile11_1.txt", std::ios_base::app);
    std::ofstream newFile36("scorefile11_2.txt", std::ios_base::app);

    std::ofstream newFile37("scorefile12_0.txt", std::ios_base::app);
    std::ofstream newFile38("scorefile12_1.txt", std::ios_base::app);
    std::ofstream newFile39("scorefile12_2.txt", std::ios_base::app);

    std::ofstream newFile40("scorefile13_0.txt", std::ios_base::app);
    std::ofstream newFile41("scorefile13_1.txt", std::ios_base::app);
    std::ofstream newFile42("scorefile13_2.txt", std::ios_base::app);

    std::ofstream newFile43("scorefile14_0.txt", std::ios_base::app);
    std::ofstream newFile44("scorefile14_1.txt", std::ios_base::app);
    std::ofstream newFile45("scorefile14_2.txt", std::ios_base::app);

    std::ofstream newFile46("scorefile15_0.txt", std::ios_base::app);
    std::ofstream newFile47("scorefile15_1.txt", std::ios_base::app);
    std::ofstream newFile48("scorefile15_2.txt", std::ios_base::app);

    std::ofstream newFile49("scorefile16_0.txt", std::ios_base::app);
    std::ofstream newFile50("scorefile16_1.txt", std::ios_base::app);
    std::ofstream newFile51("scorefile16_2.txt", std::ios_base::app);

    std::ofstream newFile52("scorefile17_0.txt", std::ios_base::app);
    std::ofstream newFile53("scorefile17_1.txt", std::ios_base::app);
    std::ofstream newFile54("scorefile17_2.txt", std::ios_base::app);

    std::ofstream newFile55("scorefile18_0.txt", std::ios_base::app);
    std::ofstream newFile56("scorefile18_1.txt", std::ios_base::app);
    std::ofstream newFile57("scorefile18_2.txt", std::ios_base::app);

    std::ofstream newFile58("scorefile19_0.txt", std::ios_base::app);
    std::ofstream newFile59("scorefile19_1.txt", std::ios_base::app);
    std::ofstream newFile60("scorefile19_2.txt", std::ios_base::app);

    std::ofstream newFile61("scorefile20_0.txt", std::ios_base::app);
    std::ofstream newFile62("scorefile20_1.txt", std::ios_base::app);
    std::ofstream newFile63("scorefile20_2.txt", std::ios_base::app);

    std::ofstream newFile64("scorefile21_0.txt", std::ios_base::app);
    std::ofstream newFile65("scorefile21_1.txt", std::ios_base::app);
    std::ofstream newFile66("scorefile21_2.txt", std::ios_base::app);

    std::ofstream newFile67("scorefile22_0.txt", std::ios_base::app);
    std::ofstream newFile68("scorefile22_1.txt", std::ios_base::app);
    std::ofstream newFile69("scorefile22_2.txt", std::ios_base::app);

    std::ofstream newFile70("scorefile23_0.txt", std::ios_base::app);
    std::ofstream newFile71("scorefile23_1.txt", std::ios_base::app);
    std::ofstream newFile72("scorefile23_2.txt", std::ios_base::app);

    std::ofstream newFile73("scorefile24_0.txt", std::ios_base::app);
    std::ofstream newFile74("scorefile24_1.txt", std::ios_base::app);
    std::ofstream newFile75("scorefile24_2.txt", std::ios_base::app);

    std::ofstream newFile76("scorefile25_0.txt", std::ios_base::app);
    std::ofstream newFile77("scorefile25_1.txt", std::ios_base::app);
    std::ofstream newFile78("scorefile25_2.txt", std::ios_base::app);

    std::ofstream newFile79("scorefile26_0.txt", std::ios_base::app);
    std::ofstream newFile80("scorefile26_1.txt", std::ios_base::app);
    std::ofstream newFile81("scorefile26_2.txt", std::ios_base::app);

    std::ofstream newFile82("scorefile27_0.txt", std::ios_base::app);
    std::ofstream newFile83("scorefile27_1.txt", std::ios_base::app);
    std::ofstream newFile84("scorefile27_2.txt", std::ios_base::app);

    std::ofstream newFile85("scorefile28_0.txt", std::ios_base::app);
    std::ofstream newFile86("scorefile28_1.txt", std::ios_base::app);
    std::ofstream newFile87("scorefile28_2.txt", std::ios_base::app);

    std::ofstream newFile88("scorefile29_0.txt", std::ios_base::app);
    std::ofstream newFile89("scorefile29_1.txt", std::ios_base::app);
    std::ofstream newFile90("scorefile29_2.txt", std::ios_base::app);

    std::ofstream newFile91("scorefile30_0.txt", std::ios_base::app);
    std::ofstream newFile92("scorefile30_1.txt", std::ios_base::app);
    std::ofstream newFile93("scorefile30_2.txt", std::ios_base::app);

    std::ofstream newFile94("scorefile31_0.txt", std::ios_base::app);
    std::ofstream newFile95("scorefile31_1.txt", std::ios_base::app);
    std::ofstream newFile96("scorefile31_2.txt", std::ios_base::app);

    std::ofstream newFile97("scorefile32_0.txt", std::ios_base::app);
    std::ofstream newFile98("scorefile32_1.txt", std::ios_base::app);
    std::ofstream newFile99("scorefile32_2.txt", std::ios_base::app);

    std::ofstream newFile100("scorefile33_0.txt", std::ios_base::app);
    std::ofstream newFile101("scorefile33_1.txt", std::ios_base::app);
    std::ofstream newFile102("scorefile33_2.txt", std::ios_base::app);

    std::ofstream newFile103("scorefile34_0.txt", std::ios_base::app);
    std::ofstream newFile104("scorefile34_1.txt", std::ios_base::app);
    std::ofstream newFile105("scorefile34_2.txt", std::ios_base::app);

    std::ofstream newFile106("scorefile35_0.txt", std::ios_base::app);
    std::ofstream newFile107("scorefile35_1.txt", std::ios_base::app);
    std::ofstream newFile108("scorefile35_2.txt", std::ios_base::app);


    newFile1.precision(dbl::max_digits10);
    newFile2.precision(dbl::max_digits10);
    newFile3.precision(dbl::max_digits10);

    newFile4.precision(dbl::max_digits10);
    newFile5.precision(dbl::max_digits10);
    newFile6.precision(dbl::max_digits10);

    newFile7.precision(dbl::max_digits10);
    newFile8.precision(dbl::max_digits10);
    newFile9.precision(dbl::max_digits10);

    newFile10.precision(dbl::max_digits10);
    newFile11.precision(dbl::max_digits10);
    newFile12.precision(dbl::max_digits10);

    newFile13.precision(dbl::max_digits10);
    newFile14.precision(dbl::max_digits10);
    newFile15.precision(dbl::max_digits10);

    newFile16.precision(dbl::max_digits10);
    newFile17.precision(dbl::max_digits10);
    newFile18.precision(dbl::max_digits10);

    newFile19.precision(dbl::max_digits10);
    newFile20.precision(dbl::max_digits10);
    newFile21.precision(dbl::max_digits10);

    newFile22.precision(dbl::max_digits10);
    newFile23.precision(dbl::max_digits10);
    newFile24.precision(dbl::max_digits10);

    newFile25.precision(dbl::max_digits10);
    newFile26.precision(dbl::max_digits10);
    newFile27.precision(dbl::max_digits10);

    newFile28.precision(dbl::max_digits10);
    newFile29.precision(dbl::max_digits10);
    newFile30.precision(dbl::max_digits10);

    newFile31.precision(dbl::max_digits10);
    newFile32.precision(dbl::max_digits10);
    newFile33.precision(dbl::max_digits10);

    newFile34.precision(dbl::max_digits10);
    newFile35.precision(dbl::max_digits10);
    newFile36.precision(dbl::max_digits10);

    newFile37.precision(dbl::max_digits10);
    newFile38.precision(dbl::max_digits10);
    newFile39.precision(dbl::max_digits10);

    newFile40.precision(dbl::max_digits10);
    newFile41.precision(dbl::max_digits10);
    newFile42.precision(dbl::max_digits10);

    newFile43.precision(dbl::max_digits10);
    newFile44.precision(dbl::max_digits10);
    newFile45.precision(dbl::max_digits10);

    newFile46.precision(dbl::max_digits10);
    newFile47.precision(dbl::max_digits10);
    newFile48.precision(dbl::max_digits10);

    newFile49.precision(dbl::max_digits10);
    newFile50.precision(dbl::max_digits10);
    newFile51.precision(dbl::max_digits10);

    newFile52.precision(dbl::max_digits10);
    newFile53.precision(dbl::max_digits10);
    newFile54.precision(dbl::max_digits10);

    newFile55.precision(dbl::max_digits10);
    newFile56.precision(dbl::max_digits10);
    newFile57.precision(dbl::max_digits10);

    newFile58.precision(dbl::max_digits10);
    newFile59.precision(dbl::max_digits10);
    newFile60.precision(dbl::max_digits10);

    newFile61.precision(dbl::max_digits10);
    newFile62.precision(dbl::max_digits10);
    newFile63.precision(dbl::max_digits10);

    newFile64.precision(dbl::max_digits10);
    newFile65.precision(dbl::max_digits10);
    newFile66.precision(dbl::max_digits10);

    newFile67.precision(dbl::max_digits10);
    newFile68.precision(dbl::max_digits10);
    newFile69.precision(dbl::max_digits10);

    newFile70.precision(dbl::max_digits10);
    newFile71.precision(dbl::max_digits10);
    newFile72.precision(dbl::max_digits10);

    newFile73.precision(dbl::max_digits10);
    newFile74.precision(dbl::max_digits10);
    newFile75.precision(dbl::max_digits10);

    newFile76.precision(dbl::max_digits10);
    newFile77.precision(dbl::max_digits10);
    newFile78.precision(dbl::max_digits10);

    newFile79.precision(dbl::max_digits10);
    newFile80.precision(dbl::max_digits10);
    newFile81.precision(dbl::max_digits10);

    newFile82.precision(dbl::max_digits10);
    newFile83.precision(dbl::max_digits10);
    newFile84.precision(dbl::max_digits10);

    newFile85.precision(dbl::max_digits10);
    newFile86.precision(dbl::max_digits10);
    newFile87.precision(dbl::max_digits10);

    newFile88.precision(dbl::max_digits10);
    newFile89.precision(dbl::max_digits10);
    newFile90.precision(dbl::max_digits10);

    newFile91.precision(dbl::max_digits10);
    newFile92.precision(dbl::max_digits10);
    newFile93.precision(dbl::max_digits10);

    newFile94.precision(dbl::max_digits10);
    newFile95.precision(dbl::max_digits10);
    newFile96.precision(dbl::max_digits10);

    newFile97.precision(dbl::max_digits10);
    newFile98.precision(dbl::max_digits10);
    newFile99.precision(dbl::max_digits10);

    newFile100.precision(dbl::max_digits10);
    newFile101.precision(dbl::max_digits10);
    newFile102.precision(dbl::max_digits10);

    newFile103.precision(dbl::max_digits10);
    newFile104.precision(dbl::max_digits10);
    newFile105.precision(dbl::max_digits10);

    newFile106.precision(dbl::max_digits10);
    newFile107.precision(dbl::max_digits10);
    newFile108.precision(dbl::max_digits10);

    newFile73.precision(dbl::max_digits10);
    newFile74.precision(dbl::max_digits10);
    newFile75.precision(dbl::max_digits10);



    if(getIndex() ==0 && newFile1.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile1<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 0 && newFile2.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile2<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 0 && newFile3.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile3<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;


    if(getIndex() ==1 && newFile4.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile4<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 1 && newFile5.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile5<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 1 && newFile6.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile6<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==2 && newFile7.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile7<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 2 && newFile8.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile8<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 2 && newFile9.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile9<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;


    if(getIndex() ==3 && newFile10.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile10<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 3 && newFile11.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile11<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 3 && newFile12.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile12<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;


    if(getIndex() == 4 && newFile13.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile13<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 4 && newFile14.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile14<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 4 && newFile15.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile15<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==5 && newFile16.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile16<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 5 && newFile17.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile17<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 5 && newFile18.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile18<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;


    if(getIndex() ==6 && newFile19.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile19<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 6 && newFile20.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile20<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 6 && newFile21.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile21<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==7 && newFile22.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile22<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 7 && newFile23.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile23<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 7  && newFile24.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile24<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==8 && newFile25.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile25<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 8 && newFile26.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile26<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 8 && newFile27.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile27<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;


    if(getIndex() ==9 && newFile28.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile28<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 9 && newFile29.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile29<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 9 && newFile30.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile30<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==10 && newFile31.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile31<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 10 && newFile32.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile32<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 10 && newFile33.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile33<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==11 && newFile34.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile34<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 11 && newFile35.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile35<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 11 && newFile36.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile36<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==12 && newFile37.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile37<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 12 && newFile38.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile38<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 12 && newFile39.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile39<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==13 && newFile40.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile40<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 13 && newFile41.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile41<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 13 && newFile42.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile42<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==14 && newFile43.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile43<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 14 && newFile44.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile44<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 14 && newFile45.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile45<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==15 && newFile46.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile46<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 15 && newFile47.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile47<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 15 && newFile48.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile48<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==16 && newFile49.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile49<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 16 && newFile50.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile50<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 16 && newFile51.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile51<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==17 && newFile52.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile52<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 17 && newFile53.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile53<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 17 && newFile54.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile54<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==18 && newFile55.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile55<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 18 && newFile56.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile56<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 18 && newFile57.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile57<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==19 && newFile58.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile58<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 19 && newFile59.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile59<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 19 && newFile60.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile60<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==20 && newFile61.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile61<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 20 && newFile62.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile62<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 20 && newFile63.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile63<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==21 && newFile64.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile64<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 21 && newFile65.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile65<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 21 && newFile66.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile66<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==22 && newFile67.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile67<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 22 && newFile68.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile68<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 22 && newFile69.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile69<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==23 && newFile70.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile70<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 23 && newFile71.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile71<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 23 && newFile72.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile72<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==24 && newFile73.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile73<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 24 && newFile74.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile74<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 24 && newFile75.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile75<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 25 && newFile76.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile76<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 25 && newFile77.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile77<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 25 && newFile78.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile78<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==26 && newFile79.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile79<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 26 && newFile80.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile80<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 26 && newFile81.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile81<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 27 && newFile82.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile82<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 27 && newFile83.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile83<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 27 && newFile84.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile84<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==28 && newFile85.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile85<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 28 && newFile86.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile86<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 28 && newFile87.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile87<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 29 && newFile88.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile88<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 29 && newFile89.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile89<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 29 && newFile90.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile90<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==30 && newFile91.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile91<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 30 && newFile92.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile92<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 30 && newFile93.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile93<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==31 && newFile94.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile94<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 31 && newFile95.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile95<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 31 && newFile96.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile96<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==32 && newFile97.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile97<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 32 && newFile98.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile98<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 32 && newFile99.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile99<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==33 && newFile100.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile100<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 33 && newFile101.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile101<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 33 && newFile102.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile102<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==34 && newFile103.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile103<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 34 && newFile104.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile104<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 34 && newFile105.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile105<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() ==35 && newFile106.is_open() && getParentModule()->par("type").doubleValue()==0)
           newFile106<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 35 && newFile107.is_open() && getParentModule()->par("type").doubleValue()==1)
            newFile107<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;

    if(getIndex() == 35 && newFile108.is_open() && getParentModule()->par("type").doubleValue()!=0 && getParentModule()->par("type").doubleValue()!=1)
            newFile108<<getIndex()<<"\t"<<simTime()<<"\t"<<energy[getIndex()]<<endl;



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
