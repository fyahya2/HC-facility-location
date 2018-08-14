#include<iostream>
#include<vector>
#include<queue>
#include<map>
#include<set>
#include<algorithm>
#include<ctime>
#include<cstdlib>
using namespace std;

#define PL(x) cout<< #x << " = " << x <<endl;
#define P(x) cout<< #x << " = " << x <<"  ";
#define FOR(i,n) for(int i=0;i<n;++i)
#define CLR(x) memset( x , 0 ,sizeof(x) )
#define EL cout<<endl;

struct bitVector{
	unsigned long long bitset[5];
	void convert(vector<int> in, int num){
		CLR(bitset);
		for(int i=0; i<num ;++i)
			bitset[ i/64 ] += ((in[i]==1)<<(i%64));
	}
};

//************************************************************************************************
//************************************* INITIALIZATION *******************************************
//************************************************************************************************

const int MAX_NUMBER_OF_FACILITIES = 1010;
const int MAX_NUMBER_OF_ACTIVE_FACILITIES = 310;
const int MAX_NUMBER_OF_COSTUMER = 310;
int ROOT = 0;
const int INF = (1<<17);
const double EPS = 0.00000000001;
int  LOG=0;
int HUB_NUMBER = 10;

/**TODO**/
int SEMI_DYNAMIC_ITERATION = 3000;
int NUMBER_OF_ITERATIONS = 	min(100000,HUB_NUMBER*SEMI_DYNAMIC_ITERATION); //modify form below .. here has no impact 
int NUMBER_OF_DYNAMIC_ITERATION = 400;

const double PitchR = 0.01;
const double HSCR = 0.99;
const int HMSize = 45;


//************************************************************************************************
//************************************* INPUT ****************************************************
//************************************************************************************************

//respectively number of //all nodes //those which we can open //number of customers 
int f2FN,activeFacilityN,costumerN; 
//Facility to Facility  contains all Facility and estinery nodes
double f2F[MAX_NUMBER_OF_FACILITIES][MAX_NUMBER_OF_FACILITIES]; 
//Facility to Coustomer contains only the Facility that can be opened
double f2C[MAX_NUMBER_OF_ACTIVE_FACILITIES][MAX_NUMBER_OF_COSTUMER]; 
//cost of opening each facility contains only the facilities which can be opened 
double costOfOpeningEachFacility[MAX_NUMBER_OF_ACTIVE_FACILITIES]; 

//TESTED//
void ReadingINPUT(){
	//READING MATRIX FORM 
	cin >> f2FN >>activeFacilityN>>costumerN;

	//read an array of size activeFacility number the cost of opening each Facility 
	for(int f=0;f<activeFacilityN;++f)
		scanf("%lf",&costOfOpeningEachFacility[f] );

	//read the matrix of AllFacility*AllFacility [[active]+[eshtinery]][[active]+[eshtinery]]
	for(int ff=0;ff<f2FN;++ff){
		for(int ft=0;ft<f2FN;++ft){
			scanf("%lf",&f2F[ff][ft]);
			if(f2F[ff][ft]>INF)
				f2F[ff][ft]=INF;
		}
	}
	//read a Matrix of allFacility To all Costumers [activefacility][Costumer]
	for(int ff=0;ff<activeFacilityN;++ff)
		for(int cc=0;cc<costumerN;++cc)
			scanf("%lf",&f2C[ff][cc]);
}

void print(vector<int> &v ,int value=-1){
	cout<<"< ";
	FOR(i,v.size())
		if(value<0 ){
			if(i!=0) cout<<" ("<<f2F[ v[i-1] ][ v[i] ]<<") ";
			cout<< v[i]<< ",";
		}
		else if(v[i]==value)
			cout<< i<< ",";
	cout<<">"<<endl;
}

//************************************************************************************************
//*************************************INITIAL SHORTEST PATH**************************************
//************************************************************************************************

//Finding initial Shortest Paths ... form all active nodes to all nodes (active ones are needed in algorithm) ... 
// single source shortest path with h steps bwtween node u to v SSSP[h][u][v]
double SSSP[12][MAX_NUMBER_OF_ACTIVE_FACILITIES][MAX_NUMBER_OF_FACILITIES];
int SSSPparent[12][MAX_NUMBER_OF_ACTIVE_FACILITIES][MAX_NUMBER_OF_FACILITIES];

//accumulated SSSP ... with max hop of H... 
double minPathCost[12][MAX_NUMBER_OF_ACTIVE_FACILITIES][MAX_NUMBER_OF_FACILITIES];
int minPathCostHubNumber[12][MAX_NUMBER_OF_ACTIVE_FACILITIES][MAX_NUMBER_OF_FACILITIES];

void modifiedBellmanFord(){
	//with exactly h hub number not <= 
	for(int h=0;h<=HUB_NUMBER;++h){
		for(int u=0 ; u< activeFacilityN ; ++u){
			for(int v=0 ; v<f2FN ; ++v){
				SSSP[h][u][v]=INF;
				SSSPparent[h][u][v]=-1;
				minPathCost[h][u][v]=INF;
				minPathCostHubNumber[h][u][v]=INF;
			}
		}
	}
	
	for(int u=0 ; u<activeFacilityN ; ++u){
		SSSP[0][u][u]=0;
		SSSPparent[0][u][u]=u;
	}
	//for hop ==1 
	for(int u=0;u<activeFacilityN;++u){
		for(int v=0;v<f2FN;++v){
			SSSP[1][u][v] = f2F[u][v];
			SSSPparent[1][u][v]= u;
		}
	}

	for(int h=1;h<=HUB_NUMBER;++h){
		for(int i=0;i<activeFacilityN;++i){

			//Tamame yal ha <TO OPTIMIZE>
			for(int u=0;u<f2FN;++u){
				for(int v=0;v<f2FN;++v){
					if( SSSP[h-1][i][u] + f2F[u][v] < SSSP[h][i][v]-EPS ){
						if( h==1 ){
							cout<<"Error #72729"<<endl;
							PL(SSSP[h][i][v]);
							PL(SSSP[h-1][i][u] +f2F[u][v]);
						}
						
						SSSP[h][i][v] = SSSP[h-1][i][u] +f2F[u][v];
						SSSPparent[h][i][v]=u;
					}
				}
			}

		}
	}

	//accumulative ...
	for(int u=0;u<activeFacilityN;++u){
		for(int v=0;v<f2FN;++v){
			minPathCost[0][u][v] =SSSP[0][u][v];
			minPathCostHubNumber[0][u][v] = 0;
		}
	}

	for(int h=1;h<=HUB_NUMBER;++h){
		for(int u=0;u<activeFacilityN;++u){
			for(int v=0;v<f2FN;++v){
				minPathCost[h][u][v]=minPathCost[h-1][u][v];
				minPathCostHubNumber[h][u][v] = minPathCostHubNumber[h-1][u][v];
				if(SSSP[h][u][v] < minPathCost[h][u][v]-EPS ){
					minPathCost[h][u][v] = SSSP[h][u][v];
					minPathCostHubNumber[h][u][v]=h;
				}
			}
		}
	}

	return ;
}

vector<int> getPath(int U,int V,int hop=-1 ){ //exact hop
	if(V==U) { cout<<"ERROR #98745630"<<endl; }
	int tik=0;
	if(hop==-1)
		hop=minPathCostHubNumber[HUB_NUMBER][U][V],tik+=1;
	if(hop>HUB_NUMBER) cout<<"ERROR #485926"<<endl;;
	if(hop==0) cout<<"ERROR #985478"<<endl;;
	if(SSSP[hop][U][V]>= INF ) {
		P(U);
		P(V);
		P(hop);
		P(tik);
		cout<<"ERROR #4111926"<<endl;;
	}
	vector<int> result;
	int vlog=V;
	for (int h=hop;h>0;--h){
		result.push_back(V);
		V=SSSPparent[h][U][V];
	}
	if(V!=U){
		cout<<"Error # 66777"<<endl;
		P(U);
		P(V);
		P(vlog);
		P(hop);
		P(tik);
		EL;
	}
	result.push_back(V);
	if(result.size()<2) cout<<"Error #85462"<<endl;
	if(result.size()!=hop+1) cout<<"Error #121212"<<endl;
	reverse(result.begin(),result.end());
	return result;
}

//************************************************************************************************
//************************************* UTILITY **************************************************
//************************************************************************************************
void printGraph(){
	for(int ff=0;ff<f2FN;++ff){
		cout<<ff << " : ";
		for(int ft=0;ft<f2FN;++ft){
			if(f2F[ff][ft]<INF && minPathCost[HUB_NUMBER][ROOT][ft] < INF)
				cout<<ft<<"-"<<f2F[ff][ft]<<"  ";
		}
		cout<<endl;
	}

}
//get a Random Between [0-1]
double randf(){
	int r=rand()%10000;
	return (double)r/10000;
}
// Insures That the solotion is reachable ... 
void refine(vector<int> &a){
	//first is always open
	a[ROOT]=1;
	//if we cant reach a facility we cant open it 
	for(int i=1;i<activeFacilityN;++i)
		if( minPathCost[ HUB_NUMBER ][ROOT][i] >=INF)
			a[i]=0;
}

//************************************************************************************************
//*************************************COST FUNCTION *********************************************
//************************************************************************************************

//dont need to reset
int markFacilitiesWhichConnectToCustomer[MAX_NUMBER_OF_FACILITIES];
int markerForFacilitiesWhichConnectToCustomer=1;

double addcostOfOppeningFacilitiesAndCustomer( vector<int> &sample ){

	double cost=0;
	markerForFacilitiesWhichConnectToCustomer++;
	int NumberofOpenFacilites=0; 

	//adding the cost to costumers;
	for(int c=0;c<costumerN;++c){ //for each costumer
		double mini=f2C[ROOT][c];
		int ind=ROOT;
		for(int f=1;f<activeFacilityN;++f){ //find the facility which is open and the connection cost is minimum 
			if(sample[f]==1 && f2C[f][c] < mini){
				mini=f2C[f][c];
				ind=f;
			}
		}
		markFacilitiesWhichConnectToCustomer[ind] = markerForFacilitiesWhichConnectToCustomer;
		cost+=mini;
	}
	
	//REFINeING... removing facilities that no cusotomer is connected to //root is always open
	for(int f=1;f<activeFacilityN;++f){
		if(sample[f]==1 && markFacilitiesWhichConnectToCustomer[f]!=markerForFacilitiesWhichConnectToCustomer){
			sample[f]=0;
		}
	}
	//adding the cost of opening each facility 
	for(int f=0;f<activeFacilityN;++f)
		if(sample[f] )
			cost += costOfOpeningEachFacility[f];
	return cost;
}

int inTreeTT[MAX_NUMBER_OF_FACILITIES];
int markerForinTreeTT=1;

int inTreeTP[MAX_NUMBER_OF_FACILITIES];
int markerForinTreeTP=1;

int Ui[MAX_NUMBER_OF_FACILITIES];
double minDistanceInTT[MAX_NUMBER_OF_FACILITIES];
int parentInTT[MAX_NUMBER_OF_FACILITIES];
int hopToparentTT[MAX_NUMBER_OF_FACILITIES];
int EdgeList[MAX_NUMBER_OF_FACILITIES];

int edgeMark[MAX_NUMBER_OF_FACILITIES][MAX_NUMBER_OF_FACILITIES];
int markerE=1;

bool doRelaxTT(int U, int V){
	/*if(U==163 && V==1005){
		cout<<"DO RELAX?????: " <<U << "~>" << V << "{" << minPathCost[U][V] << "}";
			P(Ui[U]);
			P( minPathCostHubNumber[U][V] );
			P(Ui[V]);
			P( minDistanceInTT[V]);
			print(getPath(U,V));
			EL;
			

	}*/
	int dHop=HUB_NUMBER - Ui[U];
	if(dHop<0 || dHop>HUB_NUMBER ){
		cout<<"ERROR#7755 {"<<endl;
		
		cout<<"}ERROR#7755 "<<endl;
		return INF;
	}
	if( minDistanceInTT[V]==minPathCost[dHop][V][U] && Ui[U]+minPathCostHubNumber[dHop][V][U]<Ui[V] ){
			/*
		cout<<"DO RELAX: " <<U << "~>" << V << "{" << minPathCost[dHop][V][U] << "}";
		P(Ui[U]);
		P( minPathCostHubNumber[dHop][V][U] );
		P(Ui[V]);
		EL;
		//*/
		minDistanceInTT[V]=minPathCost[dHop][V][U];
		Ui[V]=Ui[U]+minPathCostHubNumber[dHop][V][U];
		parentInTT[V]=U;

		return  true;
	}else if( minPathCost[dHop][V][U] < minDistanceInTT[V] ){
/*		cout<<"DO RELAX: " <<U << "~>" << V << "{" << minPathCost[dHop][V][U] << "}";
		P(Ui[U]);
		P( minPathCostHubNumber[dHop][V][U] );
		P(Ui[V]);
		EL;
		//*/
		minDistanceInTT[V]=minPathCost[dHop][V][U];
		Ui[V]=Ui[U]+minPathCostHubNumber[dHop][V][U];
		parentInTT[V]=U;
		return true;
	}
	return false;
}

double costFunction( vector<int> &sample){

	refine( sample );
	//NOTE ... ALL FACILITIES IN SAMPLE CAN BE CONNECTED TO ROOT ... !!! THERE IS NO ILIGAL AND UNREACHABLE FACILITY 
	/*
	cout<< "cost function for :";
	print(sample,1);
	//*/
	double cost=0;
	cost = addcostOfOppeningFacilitiesAndCustomer(sample);
	//*********************************************************
	//***************** STINER TREE MAKING (updating ui) ******
	//*********************************************************

	markerForinTreeTT+=2;
	FOR(i,MAX_NUMBER_OF_FACILITIES)
		minDistanceInTT[i]=Ui[i]=INF;

	//adding root to Tree and relaxing newNodes ...
	inTreeTT[ROOT]=markerForinTreeTT;
	Ui[ROOT]=0;
	minDistanceInTT[ROOT]=0;
	parentInTT[ROOT]=0;

	FOR(vv,activeFacilityN){
		if(sample[vv]==true && inTreeTT[vv]!=markerForinTreeTT ){
			if( doRelaxTT(ROOT,vv)==false )
				cout<<"ERROR #23456"<<endl;
		}
	}
	//**now doing algorithm like MST Prim ... for all nodes and adding them to Tree ... 
	memset(EdgeList,-1,sizeof(EdgeList));
	while(1){
		/*
		printf("-----------------------------------------------------------------\n");
		FOR(vv,activeFacilityN){
			if(sample[vv]==true && inTreeTT[vv]!=markerForinTreeTT ){
				cout<<"["<<vv <<","<<minDistanceInTT[vv]<<"-"<< Ui[vv]<<"] ";
			}
		}
		EL;
		//*/

		//find min to add to Tree ...
		int miniVertexV=-1;
		double miniCost=INF;
		int miniHub=INF;

		FOR(vv,activeFacilityN){
			if(sample[vv]==true && inTreeTT[vv]!=markerForinTreeTT ){
				if(minDistanceInTT[vv]<miniCost){
					miniCost=minDistanceInTT[vv];
					miniHub=Ui[vv];
					miniVertexV=vv;
				}else if(minDistanceInTT[vv] == miniCost && Ui[vv]<miniHub){
					miniCost=minDistanceInTT[vv];
					miniHub=Ui[vv];
					miniVertexV=vv;
				}
			}
		}
		if(miniVertexV==-1){
			///TEST*********************
			FOR(i,activeFacilityN)
				if(sample[i]==true && inTreeTT[i]!=markerForinTreeTT )
					cout<<"ERROR # 45378393"<<endl;
			///************************
			break; //end of relaxing all THE nodes and finding Ui s ... 
		}

		int miniVertexU = parentInTT[miniVertexV];
		inTreeTT[miniVertexV]=markerForinTreeTT;
		/*
		PL(miniVertexU);
		PL(miniVertexV);
		//*/
		//SS
		if(minPathCostHubNumber[ HUB_NUMBER-Ui[miniVertexU] ][miniVertexV][miniVertexU] !=Ui[miniVertexV]-Ui[miniVertexU]){
			cout<<"ERROR # 474848 {"<<endl;
			PL(minPathCostHubNumber[ HUB_NUMBER-Ui[miniVertexU] ][miniVertexV][miniVertexU] );
			PL(Ui[miniVertexU] );
			PL(Ui[miniVertexV] );
			cout<<"} ERROR # 474848"<<endl;
		}
		vector<int> path = getPath(miniVertexV,miniVertexU,Ui[miniVertexV]-Ui[miniVertexU]);
		reverse(path.begin(),path.end() );
		/*print(path);
		//*/
		vector<int> intermediateNodes;
		
		for(int i=path.size()-2 ; i>=0 ; --i ){
			int u=path[i];
			int v=path[i+1];
			if(Ui[v]< Ui[miniVertexU]+i+1 ){
				cout<<"ERROR #8383"<<endl;
				break;
			}
			inTreeTT[v] = markerForinTreeTT;
			//if( v<activeFacilityN && sample[v]==true)
				intermediateNodes.push_back(v);
			//}
			Ui[v]=min(Ui[miniVertexU]+i+1,Ui[v]);
			EdgeList[v]=u;
		}
		/*
		P("intermediateNodes : ");
		print(intermediateNodes);
		//*/
		/// relaxing new nodes ... 
		for(int i=0;i<intermediateNodes.size();++i){
			int u=intermediateNodes[i];
			//relax u to all ...
			FOR(vv,activeFacilityN){
				if(sample[vv]==true && inTreeTT[vv]!=markerForinTreeTT ){
					doRelaxTT(u,vv);
				}
			}
		}

	//end of setting Uis loop;
	}

	markerE++;
	for(int v=1;v<activeFacilityN;++v){
		if(sample[v]==false)continue;
		int vv=v;
		int uu=EdgeList[vv];
		while( 1 ){
			if(edgeMark[uu][vv]!=markerE && edgeMark[vv][uu]!=markerE){
				cost+=f2F[vv][uu];
				edgeMark[uu][vv]=markerE;
				edgeMark[vv][uu]=markerE;
			}
			if(uu<activeFacilityN && sample[uu] )break;
			vv=uu;
			uu=EdgeList[vv];
		}
	}

	return cost;
	//*********************************************************
	//***************** making actual Tree and seting the costs ******
	//*********************************************************

	markerForinTreeTP++;
	markerE++;
	inTreeTP[ROOT]=markerForinTreeTP;
	for(int h=HUB_NUMBER ; h>0 ; --h){
		for( int v=1 ; v<activeFacilityN ; ++v){
			if(sample[v]==true && inTreeTP[v]!=markerForinTreeTP && Ui[v]==h ){
				/*cout<<"starated adding v"<<v;
				PL(Ui[v]);
				//*/
				double miniDistance=INF;
				int miniDistanceVertex=-1;
				for(int u=0;u<activeFacilityN;++u){
					//TODO: can change to make all hops 
					if(sample[u]==true && inTreeTP[u]==markerForinTreeTP && Ui[v]-Ui[u]>0 ){
						if( Ui[v]-Ui[u] <=0 ) cout<<"ERROR# 124"<<endl;

						if( minPathCost[ Ui[v]-Ui[u] ][u][v]<miniDistance ){
							miniDistance = minPathCost[ Ui[v]-Ui[u] ][u][v];
							miniDistanceVertex=u;
						}
					}
				}
				if(miniDistanceVertex==-1){
					P(v);
					P(minPathCostHubNumber[Ui[v]-Ui[miniDistanceVertex]][0][v]);
					PL(Ui[v]);
				} 
				vector<int> path = getPath( miniDistanceVertex,v,minPathCostHubNumber[Ui[v]-Ui[miniDistanceVertex]][miniDistanceVertex][v] );
				/*
				P( v );
				P( miniDistanceVertex );
				P( minPathCostHubNumber[Ui[v]-Ui[miniDistanceVertex]][miniDistanceVertex][v]  );
				P( Ui[v] );
				P( Ui[miniDistanceVertex] );
				print(path);
				//*/
				for(int i=path.size()-2;i>=0;--i){
					int uu=path[i];
					int vv=path[i+1];
					if(inTreeTP[vv]==markerForinTreeTP ) break;
					inTreeTP[vv]=markerForinTreeTP;
					if(edgeMark[uu][vv]!=markerE && edgeMark[vv][uu]!=markerE){
						cost+=f2F[vv][uu];
						edgeMark[uu][vv]=markerE;
						edgeMark[vv][uu]=markerE;
					}

				}
			}
		}
	}
	PL(cost);
	return cost;

}


//************************************************************************************************
//*************************************MAIN ALGORITHM ********************************************
//************************************************************************************************

pair< double,vector<int> > harmonyMemory[HMSize];

priority_queue< pair<double,int> >PQMAX; //cost index;
priority_queue< pair<double,int> > PQMIN; //cost index;
map<int, int > mapCost2Index;

bool isUNIQUE(int cost){
	if( mapCost2Index.count(cost) >0)
		return false;
	else return true;
}
int dynamicIterationCounter =0;
vector<int> HS(){

	//cout<<"Algorithm Started :"<<endl;
	dynamicIterationCounter =0;
	vector<int> result;
	double bestCost = INF ;
	int bestIteration = -1;

	//INITIALIZE HARMONEY MEMORY .....
	int ic=0;
	for(int i=0;i<HMSize;++i){
		ic++;
		harmonyMemory[i].second =vector<int>(); //new harmoney
		for(int var =0 ;var<activeFacilityN ; ++var)
			harmonyMemory[i].second.push_back( (int)(randf()+0.5) );//make random 0/1 
		refine( harmonyMemory[i].second );
		double cost = costFunction( harmonyMemory[i].second );
		if(cost<bestCost){
			bestCost=cost;
			bestIteration=0;
		}
		//add to harmony memory 
		harmonyMemory[i].first=cost;
		//used to make time searching for minimum faster
		if( isUNIQUE(cost) || HUB_NUMBER<=5  || ic>HMSize*2 ){
			PQMAX.push( make_pair(cost,i) );
			mapCost2Index[cost]=i;
		}
		else i--;
	}
	
	//PL("HARMONEY MEMORY INITIALIZIATION DONE ");
	//Algorithm Iterations 
	int it=0; 
	for( it=0;it<NUMBER_OF_ITERATIONS ;++it){
		
		if( dynamicIterationCounter >= NUMBER_OF_DYNAMIC_ITERATION )
			break;
		
		//cout<<"number"<<it<<endl;
		vector<int>	temp;
			double rnd = randf();
		for(int var =0 ;var<activeFacilityN ; ++var){
			if(rnd>HSCR ){ //chose Random
				temp.push_back( (int)(randf()+0.5) );//make random 0/1 
			}else { //chose from the others 
				//can chose someThing from The best !! 
				temp.push_back( harmonyMemory[rand()%HMSize].second[var] );
				if(rnd<PitchR){ //pitch currnet one 
					if(var!=temp.size()-1)cout<<"ERROR2"<<endl;
					temp[var] = 1-temp[var]; //change 0-1
				}
			}
		}
	 	refine(temp);
		double cost = costFunction(temp);
		
		if(cost<bestCost){
			bestCost=cost;
			bestIteration=it;
		}
		//double cost1 = costFunction1(temp);
		//if(cost!=cost1)
		///	cout<<cost<<" "<<cost1<<endl;
		pair<double,int> t= PQMAX.top();
		/*PL(cost);
		cost = costFunction(temp);
		PL("cost2:");PL(cost);
		PL(t.first);
		PL(t.second);*/
		if(t.first>cost && isUNIQUE(cost) ){ //if the new one is something better ... replace the worst in hormony memory with this new one ... 
			harmonyMemory[t.second].first=cost;
			harmonyMemory[t.second].second=temp;
			PQMAX.pop();
			PQMAX.push(make_pair(cost,t.second) );
			mapCost2Index[cost]=t.second;
			dynamicIterationCounter=0;
			//PL(costFunction(harmonyMemory[HUB_NUMBER][t.second].second));
		}else {
			dynamicIterationCounter++;
		}
	}

	cout<<"best Cost at Iteration: "<<bestIteration<<endl;
	cout<<"best Cost: "<<bestCost<<endl;
	cout<<"Number of Iteration: "<<it<<endl;
	//get the best value ... 
	while(PQMAX.size()>1){
		PQMAX.pop();
	}
	cout<<PQMAX.top().first<<endl;
	cout<<costFunction(harmonyMemory[PQMAX.top().second].second)<<endl;
	return harmonyMemory[PQMAX.top().second].second;
}

//************************************************************************************************
//************************************* MAIN *****************************************************
//************************************************************************************************
void resetAllData(){
	CLR(f2F);
	CLR(f2C);
	CLR(costOfOpeningEachFacility);

	CLR(SSSP);
	CLR(SSSPparent);
	CLR(minPathCost);
	CLR(minPathCostHubNumber);
	markerForFacilitiesWhichConnectToCustomer=1;

	markerForinTreeTT = 1;
	markerForinTreeTP = 1;
	markerE=1;
	PQMAX = priority_queue< pair<double,int> >();
	PQMIN = priority_queue< pair<double,int> >();
	mapCost2Index = map<int, int >();
	//*/
}

vector<int> HMTemp[501];
double testCost[501];
double maximumdifff=0;
void testOfCostFunctionReliablityInHighHopS(int hs ){
	resetAllData();
	ReadingINPUT();
	modifiedBellmanFord();
	if(1)printGraph();
	
	//*************************Test of verifying the cost function ... 
	if(hs==0){
		for(int count=0;count<501;++count){
			HMTemp[count] =vector<int>(); //new harmoney
			for(int var =0 ;var<activeFacilityN ; ++var)
				HMTemp[count].push_back( (int)(randf()+0.5) );//make random 0/1 
			refine( HMTemp[count] );
			testCost[count] = costFunction(HMTemp[count]);
			//cout<< count << "-" <<testCost[count]<<"  ";
		}
		//EL;
	}else{
		for(int count=0;count<501;++count){
			double cost = costFunction(HMTemp[count]);
			if(cost>testCost[count]){
				cout<<"MAXIMUM: "<< (maximumdifff = max(maximumdifff,cost-testCost[count]) )<<endl;
				cout << "Error "<<cost <<" > "<<testCost[count]<<endl;
				for(int i=0;i<activeFacilityN;++i)
					cout<<" "<<HMTemp[count][i];
			}
			testCost[count]=cost;
			///cout<< count << "-" <<testCost[count]<<"  ";
		}
		//EL;
		for(int count=0;count<501;++count){
			HMTemp[count] =vector<int>(); //new harmoney
			for(int var =0 ;var<activeFacilityN ; ++var)
				HMTemp[count].push_back( (int)(randf()+0.5) );//make random 0/1 
			refine( HMTemp[count] );
			testCost[count] = costFunction(HMTemp[count]);
			//cout<< count << "-" <<testCost[count]<<"  ";
		}	
		EL;
	}
	
}

void RUN(){
	resetAllData();
	ReadingINPUT();
	modifiedBellmanFord();
	
	int start,end;
	start =clock();
	vector<int> result = HS();
	end =clock();
	printf ("Time %.2lf seconds\n",((float)end-(float)start)/CLOCKS_PER_SEC );
	
	cout<<"OPENED FACILITES : " ;
	for(int i=0;i<result.size();++i){
		if(result[i]){
			cout<<i<<" ";
		}
	}	
	cout<<endl;
	cout<<"THE TOTAL COST IS "<< (1*costFunction(result) ) <<endl;
}

char dataSetSize[4][3]={"05","10","15","20"};
int hubslist[4]={3,5,7,10};
//COST FUNCTION TEST MAIN
int main(){
	srand(time(NULL));	
	freopen("harmoney CFversion1 1C200-300  .txt","w",stdout);

	for(int dataSetNumber='1'; dataSetNumber<='1';dataSetNumber++){
		for(int seri='c';seri<='c';++seri){
			for(int dss=0 ;dss<4;++dss){
				for(int size='2';size<='3';++size){
					for(int hs=0;hs<4;++hs){
						for(int runs=0;runs<4;++runs){
							char fileName[128]="1c20 300.txt";
							int rnd = rand();
							srand(rnd);
							fileName[0]=dataSetNumber;
							fileName[1]=seri;
							fileName[2]=dataSetSize[dss][0];
							fileName[3]=dataSetSize[dss][1];
							fileName[4]=' ';
							fileName[5]=size;
							freopen(fileName,"r",stdin);
							HUB_NUMBER = hubslist[hs];
							NUMBER_OF_ITERATIONS = 	min(100000,HUB_NUMBER*SEMI_DYNAMIC_ITERATION);

							cout<<"---------------------------"<< fileName <<" seed: "<<rnd <<"------------------------------"<<endl;
							cout<<">>>>>>>>>> HUB_NUMBER: "<< HUB_NUMBER <<" Run: "<<runs <<" <<<<<<<<<<<<<<"<<endl;
							RUN();
						}
					}	
				}
			}
		}
	}
	return 0;
}
/*
int main2(){

	srand(time(NULL));	
	for( NUMBER_OF_DYNAMIC_ITERATION=250 ; NUMBER_OF_DYNAMIC_ITERATION<=250 ; NUMBER_OF_DYNAMIC_ITERATION+=100)
	for( PitchR=0.01;PitchR<=0.01;PitchR+=0.03 ){
		HSCR=1.0-PitchR;
		char sssss[128];
		for( HMSize=45;HMSize<=45; HMSize+=25 ){	
			string ss="cost test after voss1 (3000, ";
			sprintf(sssss,"%d",NUMBER_OF_DYNAMIC_ITERATION);
			ss+= sssss;
			ss+= " - ";
			sprintf(sssss,"%1.2lf",PitchR);
				ss+= sssss;
				ss+= " - ";
				sprintf(sssss,"%d",HMSize);
				ss+= sssss;
				ss+= " ].txt";
		freopen(ss.c_str(),"w",stdout);
	//====================================================================
	for(int dataSetNumber='1'; dataSetNumber<='2';dataSetNumber++){
		for(int seri='c';seri<='d';++seri){
			for(int dss=0 ;dss<4;++dss){
				for(int size='2';size<='3';++size){
					for(int hs=0;hs<4;++hs){
						for(int runs=1;runs<2;++runs){
							srand(rand());	
							char fileName[128]="1c20 300.txt";
							fileName[0]=dataSetNumber;
							fileName[1]=seri;
							fileName[2]=dataSetSize[dss][0];
							fileName[3]=dataSetSize[dss][1];
							fileName[4]=' ';
							fileName[5]=size;
							freopen(fileName,"r",stdin);
							HUB_NUMBER = hubslist[hs];
							NUMBER_OF_ITERATIONS = 	min(100000,HUB_NUMBER*SEMI_DYNAMIC_ITERATION);
							cout<<"---------------------------"<< fileName <<"------------------------------"<<endl;
							cout<<">>>>>>>>>> HUB_NUMBER: "<< HUB_NUMBER <<" Run: "<<runs <<" <<<<<<<<<<<<<<"<<endl;
							RUN(hs);

						}
					}	
				}
			}
		}
	}
	//====================================================================
		}
	}
	return 0;
}
*/
