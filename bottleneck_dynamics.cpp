#include<iostream> 
#include<vector> 
#include<map> 
#include<queue> 
#include<set> 
#include <fstream>
#include<algorithm>
#include <bits/stdc++.h>
/*
               This code is developed by Jinxiao Duan on the study
"Spatiotemporal dynamics of traffic bottlenecks: an early signal of heavy congestions"
 
*/


using namespace std;
typedef pair<double, int> pr;

namespace JT
{ 
    const int maxNode = 30050;
    const int buffSize = 1e6 + 10;
    const double eps = 1e-8;
    const double INF = 1e9;
    const double kap = 1e-4; // tolerance
    int date;
	 
    // define variables
    char buff[buffSize];
    int zoneNum, nodeNum, firstThruNode, linkNum, ITERS;
    vector<int> Gfrom[maxNode], Gto[maxNode];
    
    double dist[maxNode];
    int vis[maxNode], pre[maxNode];
    vector<int> gtvec;
	 
	struct link
    {
        int linkid, initNode, termNode;
        double freeFlowTime, cap;
        link(){}
        
        void read(ifstream & net)
        {
            net >> linkid>> initNode >> termNode>> freeFlowTime >> cap;
        }

    };   
    
    // read raods (links) information
    vector<link> links;
	map<int, int> id2from, id2to;
	int id;
    void ReadNet()
    {
        // read fileName_net.tntp
        cout<<"reading network data"<<endl;
        char filename[100];
        sprintf (filename,"sz_renet_0.tntp");

        ifstream net(filename);
        net >> buff >> buff >> buff >> zoneNum;
        net >> buff >> buff >> buff >> nodeNum;
        net >> buff >> buff >> buff >> firstThruNode;
        net >> buff >> buff >> buff >> linkNum;
        cout<<"linkNum"<<linkNum<<endl;
        net.getline(buff, buffSize, ';');
        for(int i = 1; i <= linkNum; i++)
        {
         	link cur;
            cur.read(net);
            links.push_back(cur);
            Gfrom[cur.initNode].push_back(links.size());
            Gto[cur.termNode].push_back(links.size());  
            net.getline(buff, buffSize, ';');      
        }
        net.close();
        
		//creat relationship between nodes and links
		cout<<"compute jam tree"<<endl;
	    int m, n; 
	    
	    ifstream gnet("gnet.tntp");   	    
	    for (int i = 0; i < linkNum; i++) 
	    { 
	        gnet>>id>>m>>n; //read links 
	        id2from[id]=m;  //get internode of link
	        id2to[id]=n;    //get termnode of link
	    }    
    }
    
    // read dataset of velocity
    vector<double> jamstatusOld[2000];
    vector<double> jamstatus[2000];
    vector<int> speedmiss[2000];
    void ReadSpeed(int day)
    {
    	cout<<"reading speed data"<<endl;

    	vector<double> reletivespeed[2000];
		for(int i=0;i<2000;i++) 
		{
			reletivespeed[i].clear();
			reletivespeed[i].resize(linkNum);  
		}     	
    	
    	//read relative speed smoothed in 5 mintue interval
		char filename[100];
		if (date<10)		sprintf (filename,  "relativevelocity_5min_2015100%d.tntp", date);   	
		if (date>=10)	sprintf (filename,  "relativevelocity_5min_201510%d.tntp", date);   	
        ifstream speedin(filename);
        for (int i=0;i<288;i++)
        {	
	    	for (int j=0; j<linkNum; j++)
	    	{
	    		speedin>>reletivespeed[i][j];
			}
			
		}
		
		//comupte jam status matrix
		cout<<"comupte jam status matrix"<<endl;
	    for(int i=0;i<2000;i++) jamstatusOld[i].clear();
	    for(int i=0;i<2000;i++) jamstatusOld[i].resize(linkNum);    		
		for(int i=0;i<2000;i++) jamstatus[i].clear();
	    for(int i=0;i<2000;i++) jamstatus[i].resize(linkNum);    
	    
		for (int j=0;j<linkNum;j++)		jamstatusOld[0][j]=0;
		for (int j=0;j<linkNum;j++)		jamstatus[0][j]=0;
		
		for (int i=1;i<=288;i++)
		{
			for (int j=0;j<linkNum;j++)
			{
				if (reletivespeed[i-1][j]<0.5)	 jamstatusOld[i][j]=1;//jam	
				if (reletivespeed[i-1][j]>=0.5)   jamstatusOld[i][j]=0;//no-jam				
			}
		}
		
		// read information of outlier of velocity
        for(int i=0;i<2000;i++) speedmiss[i].clear();
        for(int i=0;i<2000;i++) speedmiss[i].resize(linkNum);  
        
		char filename_miss[100];
		if (date<10)		sprintf (filename_miss,  "outlier_5min_2015100%d.tntp", date);   	
		if (date>=10)	sprintf (filename_miss,  "outlier_5min_201510%d.tntp", date);   	
        ifstream speedinmiss(filename_miss);
        for (int i=0;i<288;i++)
        {	
	    	for (int j=0; j<linkNum; j++)
	    	{
	    		speedinmiss>>speedmiss[i][j];
			}
			
		}		
		
	}
	
	//compute jam duration
	vector<int> jamdurationmatixOld; 
	void CreatOldJamDurationMatrix(int step)
	//step is the ranking number of minute in a day
	{
		//compute duration matrix 	 
		int jamduration; 
		double averageofstatus;
		jamdurationmatixOld.resize(linkNum);
        jamdurationmatixOld.clear();
        
		
		for (int j=0;j<linkNum;j++)//j is in the rank of linkNum
		{
			for (int duration=0;duration<=step;duration++)//duration for the jam 
			{
				averageofstatus=0;
				for (int i=(step-duration); i<=step; i++)//i is in the rank of 
				{
					averageofstatus +=jamstatusOld[i][j];
				}					
				averageofstatus=averageofstatus/(duration+1);
						
				if (averageofstatus<1) 
				{
					jamduration=duration;	
					break;	
				}
			}	
										
			jamdurationmatixOld[j]=jamduration;
		}		
		
	}
	
	

	void Updatejamstatus (int step)
	{
		
		// update jam status matrix using information of outlier matrix and jam duration matrix,  
		//For a road with an outlier of velocity record, we handle it as congested states if it connects its upstream and downstream roads that are congested in the predefined chronological order
		// jamdurationmatixOld updated to jamdurationmatix 
		int fixmissing=0;
		int maybefix=0; 
		int maybemissing=0; 
		int i=step;
		for (int j=0;j<linkNum;j++)
		{
			jamstatus[i][j]=jamstatusOld[i][j]; 
		}
	

		for (int j=0;j<linkNum;j++)
		{			
			if (speedmiss[i-1][j]==1)   
			{
				int linkIdMiss=j+1; 
				int fromNodeMiss=id2from[linkIdMiss];
				int toNodeMiss=id2to[linkIdMiss];
				
				maybemissing=maybemissing+1;
				int maybe=0;
				int fix=0;
				int lastcycle=0; 
				for  (int r=0; r<Gto[fromNodeMiss].size();r++)
				{
					int jamdurationComingDire=jamdurationmatixOld[Gto[fromNodeMiss][r]-1];
					
					for (int c=0; c<Gfrom[toNodeMiss].size();c++)
					{
						int jamdurationGoDire=jamdurationmatixOld[Gfrom[toNodeMiss][c]-1];
						
						if (jamdurationComingDire>=1 && jamdurationGoDire>=1)
						{
							if ( id2from[Gto[fromNodeMiss][r]]!=toNodeMiss && id2to[Gfrom[toNodeMiss][c]]!=fromNodeMiss)
							{

								maybe=1;
								if ( (jamdurationGoDire-jamdurationComingDire)>=0 && (jamdurationGoDire-jamdurationComingDire)<=4 )
								{
									jamstatus[i][j]=1;
									lastcycle=1;
									fix=1;					
								}									
							}
																			
						}	
						if (lastcycle==1) break;
					}	
					if (lastcycle==1) break;
				}
				fixmissing = fixmissing+fix;
				maybefix=maybefix+maybe;
			}
		}	
			
		cout<< "step= "	<<step<<":  maybemissing= "<< maybemissing<<endl;	
		cout<< "step= "	<<step<<":  maybefix= "<< maybefix<<endl;		
		cout<< "step= "	<<step<<":  fixmissing= "<< fixmissing<<endl;
	
		
	}


	//compute jam duration
	vector<int> jamdurationmatix; 
	void CreatJamDurationMatrix(int step)
	//step is the ranking number of time-interval in a day
	{
		//compute duration matrix 	 
		cout<<"compute jam duration"<<endl;
		int jamduration; 
		double averageofstatus;
		jamdurationmatix.resize(linkNum);
        jamdurationmatix.clear();
        
		
		for (int j=0;j<linkNum;j++)//j is in the rank of linkNum
		{
			for (int duration=0;duration<=step;duration++)//duration for the jam 
			{
				averageofstatus=0;
				for (int i=(step-duration); i<=step; i++)
				{
					averageofstatus +=jamstatus[i][j];
				}					
				averageofstatus=averageofstatus/(duration+1);
						
				if (averageofstatus<1) 
				{
					jamduration=duration;	
					break;	
				}
			}	
										
			jamdurationmatix[j]=jamduration;
		}			
	}

	using namespace std; 
	vector<int> vislink;  
	int maxig; 
	vector<int> order;
	
	bool is_element_in_vector(vector<int> v,int element)
	{
		vector<int>::iterator it;
		it=find(v.begin(),v.end(),element);
		if (it!=v.end())
		{
			return true;
		}
		else{
			return false;
		}
	} 

	int maxjamduration;
	int sumjamduration;	
	int trunkid; 
	vector<int> orderlink;
	//
	int FindJamTreeByBFS(int top,int step,int delta) 
	{ 	
		//linkfrom2to and linkto2from are respectively the set of links goging out and coming back
		//top is a begining  to serch jam tree cluester
		//step is the time window to find trees
		//delta is the threshold to assocaite the congestion with a bottleneck
	    queue<int> qe;  //saves potential links in the tree to find orders
	    
	    map<int, int> vis;      	    
	    
	 	orderlink.clear();
	    qe.push(top);  //taking the beginning link in the queue
	    orderlink.push_back(top); 

	    int head; 
	    // 1) to decide whether upper stream enters the cluster
	    while (!qe.empty())  
	    { 
	        head = qe.front();  //the first link 
	        qe.pop();  
	        int headfromnode= id2from[head];  // initial node of head link 
	        for (int i = 0; i < Gto[headfromnode].size(); i++)     // linkset that will go to headfromnode
	        { 	
				int previouslink=Gto[headfromnode][i];        	
	            if ( id2from[previouslink]!= id2to[head]  && is_element_in_vector(orderlink,previouslink)==0 && vis.find(previouslink) == vis.end() ) 
	            {			
					if(jamdurationmatix[previouslink-1]!=0)// if previouslink is congested 
					{					    
		            	if ((jamdurationmatix[head-1]-jamdurationmatix[previouslink-1])>=0 && (jamdurationmatix[head-1]-jamdurationmatix[previouslink-1]) <= delta ) // in a jam tree					
						{ 
			                qe.push(previouslink);  	                    
			                orderlink.push_back(previouslink); 
			                vis[previouslink]=1;
			            } 
			    	}
	        	}
	        }
		}

		//Compute the max duration of traffic dynamics
	    maxjamduration=jamdurationmatix[orderlink[0]-1];
	    trunkid=orderlink[0]-1;
	    sumjamduration=0;
	    
	    for(int i=0;i<orderlink.size();i++)
	    {
	    	if(jamdurationmatix[orderlink[i]-1]>maxjamduration) 
			{
				maxjamduration=jamdurationmatix[orderlink[i]-1]; 
				trunkid=orderlink[i]-1;
			}
	    	sumjamduration=sumjamduration+jamdurationmatix[orderlink[i]-1];
		}
	        				
	    set<int> glinkset;
		glinkset.clear(); 
	    for(int i=0; i<orderlink.size();i++) 		glinkset.insert(orderlink[i]);
	    vislink.insert(vislink.end(),glinkset.begin(),glinkset.end());
	    return glinkset.size(); 
	} 
	
	
	
	void AllJamTree(int step,int delta) 
	{ 
		int top, i = 0; 
	   	
	   	// find all the trunks
	   	vector<int> trunkidset,nextlinkstateset; 
	   	trunkidset.clear();
	   	
	   	
		for(int i=1; i<=linkNum;i++)
		{
			
			if (jamdurationmatix[i-1]>=1)		
			{
				
				int trunkidtemp=i;
				int jamdurationfortemp=jamdurationmatix[trunkidtemp-1];	
		        int trunkidtemptonode= id2to[trunkidtemp]; 
		        
		        int sumstates=0;  
		        nextlinkstateset.clear();
		        nextlinkstateset.resize(Gfrom[trunkidtemptonode].size());
		        for (int i = 0; i < Gfrom[trunkidtemptonode].size(); i++)  
		        {
		        	int nextlink=Gfrom[trunkidtemptonode][i];   
		        	int nextduration=jamdurationmatix[nextlink-1];
					nextlinkstateset[i]=0; 	
					
		        	if ( (id2from[trunkidtemp]!= id2to[nextlink]) && (nextduration>=jamdurationfortemp && (nextduration-jamdurationfortemp)<=delta) )
		        	{
		        		nextlinkstateset[i]=1; 
		        	}
					sumstates=sumstates+nextlinkstateset[i];				
				}
 	            
				
				if (sumstates==0)
 	            {
 	            	trunkidset.push_back(trunkidtemp); 
				}
        	}
		}
	   	
	   	
	   	// find all the jam trees one by one, for every trunk
	    order.clear();
	    vislink.clear();
	    
		char filename[100];
		if(date>=10)	sprintf (filename,"generated data/treesize_step=%d_201510%d.csv",step,date);
		if(date<10)		sprintf (filename,"generated data/treesize_step=%d_2015100%d.csv",step,date);
        ofstream trees(filename);
        trees<<"treeid"<<','<< "treelinkid"<<',' <<"jamduration"<<','<<"treesize"<<','<<"maxduration"<<','<<"sumduration"<< ',' << "trunkid" <<"\n";
        
        int treeid=0;	 
		int treelinkid;
		   
	    for(int i=1; i<=linkNum;i++)
	    {
	    	top=i;
	    	if (is_element_in_vector(trunkidset,top)==1) 
			{
				int treesizetemp=FindJamTreeByBFS(top,step,delta);
				treeid=treeid+1;
				for(int j=0; j<orderlink.size();j++)
				{
					treelinkid=orderlink[j]-1; 
					trees<<treeid <<','<< treelinkid <<',' << jamdurationmatix[orderlink[j]-1] <<','<<treesizetemp<<','<< maxjamduration <<','<< sumjamduration <<','<< top-1 <<"\n";					
				}
				order.push_back(treesizetemp); 
			} 
		}

	    maxig=order[0];
	    for(int i=0;i<order.size();i++)
	        if(order[i]>maxig) maxig=order[i];   
		cout<< endl; 
	    cout << "max size of jam trees:" <<  maxig <<endl; 
	    
		trees.close();
	}	
	
}

using namespace JT;

int main()
{
	//Read Network topology
	ReadNet();
	int timewindow=5;
	for (int j=1;j<31; j++)
	{
		date=j;
		cout<<"----date: "<<date<<endl;
		//read velocity data
		ReadSpeed(date);
		int step;
		int delta=2; // control this parameter to give the predefined time of jam assocaition, note that theta refferred in the paper is timewindow*delta
		for(int k=0;k<288;k++) 
	    {
	    	step=1+k;		
			cout<<"---max value of step should be"<< 1440/timewindow <<endl;
			//calcaluate jam states and continous durations up to now
			CreatOldJamDurationMatrix(step);
			Updatejamstatus(step);
			CreatJamDurationMatrix(step);
			//calculate bottleneck dynamics		
			AllJamTree(step,delta);
		}			
	}

	return 0;
}
	
	
	
	
	
	
    
    
    
    
