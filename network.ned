//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package wsnfuture;

import ned.IUnidirectionalChannel;



network Network
{
    parameters:
        int numOfSensorNodes=250;
        int width =1000 ;
        int height =1000 ;
        int type;
      //  int distributionType;
        int x ;
        int y ;
        x = width/5;
        y = height/5;
        @display("bgb=$width,$height,white,white,00;bgs=2,m");
        int snCacheSize =31;
        int snRange=142;

        double Delay=1;
        double temp_dataSize=24;
        double pres_dataSize =24;
        double hum_dataSize =24;
        double CO_dataSize =24;
        double CO2_dataSize =24;

        double bat_Drain_Temp = 0.0000495;
        double bat_Drain_Humudity =0.0285;	
        double bat_Drain_Pressure =0.0007;
  		double bat_Drain_Co2 =22.5;
  	    double bat_Drain_Co =0.015;

        int lcnRange =300;
        int lcnCache =25;
        double lcnBat_Full=110160.0 ;
        double lcnBatTrasmitting =0.0000538;
        double lcnBatReceiving =0.0000312;
        int gcnRange =500;

    types:
        channel Channel extends ned.DatarateChannel
        {
            parameters:
                delay =0;
                ber =0;
                per=0;
                datarate = 23kbps;
        }

    submodules:


        sn[numOfSensorNodes]: SensorNode {
			
			x = intuniform(0,width);
			y = intuniform(0,height);
			 
            @display("i=block/star,pink;is=vs");
            //@display("p=,;r=100,,blue,2");
			
        }
        GCN: GlobalNode {
            @display("p=,;r=$gcnRange,,red,3");
            @display("i=device/antennatower,red;is=vl");
        }

        lcn[36]: LCN {

            @display("i=device/accesspoint,gold;p=0,0,m,6,$x,$y;r=300,,cyan,2;ls=,0;is=l");

        }

	connections:
	    //lcn to lcn
	for i=0..35 {
            lcn[i].lcnIO++ <--> Channel <--> lcn[i+1].lcnIO++ if (i!=5 && i!=11 && i!=17 && i!=23 && i!=29 && i!=35);
            lcn[i].lcnIO++ <--> Channel <--> lcn[i+6].lcnIO++ if (i!=30 && i!=31 && i!=32 && i!=33 && i!=34 && i!=35);
		
        }
        lcn[10].lcnIO++ <--> Channel <--> lcn[15].lcnIO++;
        for i=0..28 {
            lcn[i].lcnIO++ <--> Channel <--> lcn[i+7].lcnIO++ if (i!=5 && i!=11 && i!=17 && i!=23 && i!=29 && i!=35);

        }
        for i=1..29 {
            lcn[i].lcnIO++ <--> Channel <--> lcn[i+7].lcnIO++ if (i!=5 && i!=11 && i!=17 && i!=23 && i!=29 && i!=35);
            lcn[i].lcnIO++ <--> Channel <--> lcn[i+5].lcnIO++ if (i!=30 && i!=31 && i!=32 && i!=33 && i!=34 && i!=35);
        }
        
        //gcn and lcn connection
        for i=7..28{      
			GCN.gcn_out++ --> Channel --> lcn[i].lcnGCN_in++ if(i != 11 && i != 12 && i != 17 && i != 18 && i != 23 && i != 24 );
			lcn[i].lcnGCN_out++	--> Channel --> GCN.gcn_in++ if(i == 14 || i == 15 || i == 20 || i == 21);
        }
        
       // sn to lcn connection
       for j=0..35, for i=0..numOfSensorNodes-1{
             
            sn[i].snIO++ <--> Channel <--> lcn[0].lcnSN++ if( ( sn[i].x <= 100) && ( sn[i].y <= 100) &&j <=5);
            sn[i].snIO++ <--> Channel <--> lcn[1].lcnSN++ if( ( sn[i].x > 100 && sn[i].x <= 300) && ( sn[i].y <= 100)&& j>=6 && j<=11 );
            sn[i].snIO++ <--> Channel <--> lcn[2].lcnSN++ if( ( sn[i].x > 300 && sn[i].x <= 500) && ( sn[i].y <= 100)&& j>=12 && j<=17 );
            sn[i].snIO++ <--> Channel <--> lcn[3].lcnSN++ if( ( sn[i].x > 500 && sn[i].x <= 700) && ( sn[i].y <= 100)&& j>=18 && j<=23 );
            sn[i].snIO++ <--> Channel <--> lcn[4].lcnSN++ if( ( sn[i].x > 700 && sn[i].x <= 900) && ( sn[i].y <= 100)&& j>=24 && j<=29);
            sn[i].snIO++ <--> Channel <--> lcn[5].lcnSN++ if( ( sn[i].x > 900 && sn[i].x <= 1000) && ( sn[i].y <= 100)&& j>=30 && j<=35 );
            // 6. b�lge
            sn[i].snIO++ <--> Channel <--> lcn[30].lcnSN++ if( ( sn[i].x <= 100) && ( sn[i].y <= 1000 && (sn[i].y > 900) )&&j <=5);
            sn[i].snIO++ <--> Channel <--> lcn[31].lcnSN++ if( ( sn[i].x > 100 && sn[i].x <= 300) && ( sn[i].y <= 1000 && sn[i].y > 900 )&&j>=6 && j<=11 );
            sn[i].snIO++ <--> Channel <--> lcn[32].lcnSN++ if( ( sn[i].x > 300 && sn[i].x <= 500) && ( sn[i].y <= 1000 && sn[i].y > 900 )&& j>=12 && j<=17 );
            sn[i].snIO++ <--> Channel <--> lcn[33].lcnSN++ if( ( sn[i].x > 500 && sn[i].x <= 700) && ( sn[i].y <= 1000 && sn[i].y > 900 )&& j>=18 && j<=23 );
            sn[i].snIO++ <--> Channel <--> lcn[34].lcnSN++ if( ( sn[i].x > 700 && sn[i].x <= 900) && ( sn[i].y <= 1000 && sn[i].y > 900 )&& j>=24 && j<=29 );
            sn[i].snIO++ <--> Channel <--> lcn[35].lcnSN++ if( ( sn[i].x > 900 && sn[i].x <= 1000) && ( sn[i].y <= 1000 && sn[i].y > 900 )&& j>=30 && j<=35 );
            // 2. b�lge
            sn[i].snIO++ <--> Channel <--> lcn[6].lcnSN++ if( ( sn[i].x <= 100) && ( sn[i].y > 100 && (sn[i].y <= 300) )&&j <=5);
            sn[i].snIO++ <--> Channel <--> lcn[7].lcnSN++ if( ( sn[i].x > 100 && sn[i].x <= 300) && ( sn[i].y > 100 && (sn[i].y <= 300) )&&j>=6 && j<=11 );
            sn[i].snIO++ <--> Channel <--> lcn[8].lcnSN++ if( ( sn[i].x > 300 && sn[i].x <= 500) && ( sn[i].y > 100 && (sn[i].y <= 300) )&& j>=12 && j<=17 );
            sn[i].snIO++ <--> Channel <--> lcn[9].lcnSN++ if( ( sn[i].x > 500 && sn[i].x <= 700) && ( sn[i].y > 100 && (sn[i].y <= 300) )&& j>=18 && j<=23 );
            sn[i].snIO++ <--> Channel <--> lcn[10].lcnSN++ if( ( sn[i].x > 700 && sn[i].x <= 900) && ( sn[i].y > 100 && (sn[i].y <= 300) ) && j>=24 && j<=29);
            sn[i].snIO++ <--> Channel <--> lcn[11].lcnSN++ if( ( sn[i].x > 900 && sn[i].x <= 1000) && ( sn[i].y > 100 && (sn[i].y <= 300) )&& j>=30 && j<=35 );
            // 3. b�lge
            sn[i].snIO++ <--> Channel <--> lcn[12].lcnSN++ if( ( sn[i].x <= 100) && ( sn[i].y > 300 && (sn[i].y <= 500) )&&j <=5);
            sn[i].snIO++ <--> Channel <--> lcn[13].lcnSN++ if( ( sn[i].x > 100 && sn[i].x <= 300) && ( sn[i].y > 300 && (sn[i].y <= 500) )&&j>=6 && j<=11 );
            sn[i].snIO++ <--> Channel <--> lcn[14].lcnSN++ if( ( sn[i].x > 300 && sn[i].x <= 500) && ( sn[i].y > 300 && (sn[i].y <= 500) ) && j>=12 && j<=17);
            sn[i].snIO++ <--> Channel <--> lcn[15].lcnSN++ if( ( sn[i].x > 500 && sn[i].x <= 700) && ( sn[i].y > 300 && (sn[i].y <= 500) )&& j>=18 && j<=23 );
            sn[i].snIO++ <--> Channel <--> lcn[16].lcnSN++ if( ( sn[i].x > 700 && sn[i].x <= 900) && ( sn[i].y > 300 && (sn[i].y <= 500) )&& j>=24 && j<=29 );
            sn[i].snIO++ <--> Channel <--> lcn[17].lcnSN++ if( ( sn[i].x > 900 && sn[i].x <= 1000) && ( sn[i].y > 300 && (sn[i].y <= 500) )&& j>=30 && j<=35 );
            // 4. b�lge
            sn[i].snIO++ <--> Channel <--> lcn[18].lcnSN++ if( ( sn[i].x <= 100) && ( sn[i].y > 500 && (sn[i].y <= 700) )&&j <=5);
            sn[i].snIO++ <--> Channel <--> lcn[19].lcnSN++ if( ( sn[i].x > 100 && sn[i].x <= 300) && ( sn[i].y > 500 && (sn[i].y <= 700) )&&j>=6 && j<=11 );
            sn[i].snIO++ <--> Channel <--> lcn[20].lcnSN++ if( ( sn[i].x > 300 && sn[i].x <= 500) && ( sn[i].y > 500 && (sn[i].y <= 700) ) && j>=12 && j<=17);
            sn[i].snIO++ <--> Channel <--> lcn[21].lcnSN++ if( ( sn[i].x > 500 && sn[i].x <= 700) && ( sn[i].y > 500 && (sn[i].y <= 700) )&& j>=18 && j<=23 );
            sn[i].snIO++ <--> Channel <--> lcn[22].lcnSN++ if( ( sn[i].x > 700 && sn[i].x <= 900) && ( sn[i].y > 500 && (sn[i].y <= 700) )&& j>=24 && j<=29 );
            sn[i].snIO++ <--> Channel <--> lcn[23].lcnSN++ if( ( sn[i].x > 900 && sn[i].x <= 1000) && ( sn[i].y > 500 && (sn[i].y <= 700) )&& j>=30 && j<=35 );
            // 5. b�lge
            sn[i].snIO++ <--> Channel <--> lcn[24].lcnSN++ if( ( sn[i].x <= 100) && ( sn[i].y > 700 && (sn[i].y <= 900) )&&j <=5);
            sn[i].snIO++ <--> Channel <--> lcn[25].lcnSN++ if( ( sn[i].x > 100 && sn[i].x <= 300) && ( sn[i].y > 700 && (sn[i].y <= 900) ) &&j>=6 && j<=11);
            sn[i].snIO++ <--> Channel <--> lcn[26].lcnSN++ if( ( sn[i].x > 300 && sn[i].x <= 500) && ( sn[i].y > 700 && (sn[i].y <= 900) ) && j>=12 && j<=17);
            sn[i].snIO++ <--> Channel <--> lcn[27].lcnSN++ if( ( sn[i].x > 500 && sn[i].x <= 700) && ( sn[i].y > 700 && (sn[i].y <= 900) )&& j>=18 && j<=23 );
            sn[i].snIO++ <--> Channel <--> lcn[28].lcnSN++ if( ( sn[i].x > 700 && sn[i].x <= 900) && ( sn[i].y > 700 && (sn[i].y <= 900) ) && j>=24 && j<=29);
            sn[i].snIO++ <--> Channel <--> lcn[29].lcnSN++ if( ( sn[i].x > 900 && sn[i].x <= 1000) && ( sn[i].y > 700 && (sn[i].y <= 900) )&& j>=30 && j<=35 );
       	
        } 
}