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
