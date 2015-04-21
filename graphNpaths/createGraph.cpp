#include"createGraph.h"
#include"error.h"
#include<sstream>

// k-nearest neighors
//#define KNN 250
#define KNN 500
#define FIXED_RADIUS_KNN
/*
 * Consumes the output of a simulator
 *
 * Xi - Xi' pairs for every mode
 * mode adjacency graph/matrix
 * normType and maxNorm
 *
 * Produces a graph suitable for shortest path search
 * 
 * cost Matrix
 */


std::string float2str (float num)
{
  std::ostringstream buff;
  buff<<num;
  return buff.str();
}

// computes only L2 norm now
double computeNorm(const modeInfo &mi,const double *xi, const double *xf)
{
  index_t i;
  double norm = 0;
  //double normWeight[] = {1.0, 1., 1., 1.};
  for(i=0;i<mi.numStateVars;i++)
  {
    //norm += pow((xi[i]-xf[i])/normWeight[i],2);
    norm += pow((xi[i]-xf[i]),2);
  }
  //norm = sqrt(norm);  
//put penalty on total time ~ path length
//  norm += 2*xf[1];
  return norm;
}

void buildGraphUsingANN(modeInfo &mi, const double threshNorm, sparseMat &graph)
{
#ifdef FIXED_RADIUS_KNN
    cout<<"using FIXED RADIUS for graph building...\n";
#else
    cout<<"using FIXED NUMBER of Nearest Neighbors for graph building....\n";
#endif
  //ANN uses squared distances
  double sqrdThreshNorm = pow(threshNorm,2);

  string modeFrom,modeTo;
  vector<string> vec_adjModes;
  vector<string>::iterator modeList_it, adjModeList_it;  
  map_modeToMatLength::iterator matLengths_it;
  index_t i,j,rowIdxFrom,rowIdxTo,status=0;
  double *p_matFromXf,*p_matToXi;
  progBar graphPB;
  graphPB.disp();
  for(modeList_it=mi.vec_modeList.begin();modeList_it!=mi.vec_modeList.end();modeList_it++,status++)
  {
    graphPB.update((float)status/mi.vec_modeList.size());
    modeFrom = *modeList_it;
    p_matFromXf = mi.map_trajXf[modeFrom];
    vec_adjModes = mi.map_adjModes[modeFrom];

    //printf("\nnum: %d\n",mi.map_matLengths[modeFrom]);
    if(mi.map_matLengths[modeFrom] == 0 )
    {
      continue;
    }

    i=0,j=0;
    int nPts;
    double eps=0.0;
    ANNpointArray	dataPts;
    //  ANNpoint queryPt;
    ANNidxArray nnIdx;
    ANNdistArray dists;
    ANNkd_tree* kdTree;
    nPts = 0;
    //  map_modeToMatLength map_cumMatLen;
    //vector<index_t> cumMatLen;
    map <string,vector<index_t> > map_modeRange;
    // compute the sum of points in the adjacent modes  
    for(adjModeList_it=vec_adjModes.begin();adjModeList_it!=vec_adjModes.end();adjModeList_it++)
    {
      modeTo = *adjModeList_it;
      //cumMatLen.push_back(nPts);
      map_modeRange[modeTo].push_back(nPts);
      nPts += mi.map_matLengths[modeTo];
      map_modeRange[modeTo].push_back(nPts);
    }
    if(nPts == 0 )
    {
      continue;
    }
    //cumMatLen.push_back(INDEX_MAX);
    //  queryPt = annAllocPt(mi.numStateVars);
    dataPts = annAllocPts(nPts, mi.numStateVars);
    delete [] dataPts[0];
    nnIdx = new ANNidx[KNN];
    dists = new ANNdist[KNN];
    //  cout<<"copying ...\n";
    j = 0;
    for(adjModeList_it=vec_adjModes.begin();adjModeList_it!=vec_adjModes.end();adjModeList_it++)
    {      
      modeTo = *adjModeList_it;
      p_matToXi = mi.map_trajXi[modeTo];
      for(i=0;i<mi.map_matLengths[modeTo];i++)
      {
        dataPts[i+j] = p_matToXi;
        p_matToXi += mi.numStateVars;
      }
      j += i;
    }
    //  cout<<"creating tree ...\n";
    kdTree = new ANNkd_tree(dataPts,nPts,mi.numStateVars);
    //  cout<<"creating graph, querying points ...\n";
    for(i=0,rowIdxFrom=0;i<mi.map_matLengths[modeFrom]*mi.numStateVars;i+=mi.numStateVars,rowIdxFrom++)
    {      
      //queryPt = &p_matFromXf[i];
#ifdef FIXED_RADIUS_KNN
      index_t frKnn = kdTree->annkFRSearch(&p_matFromXf[i],sqrdThreshNorm,KNN,nnIdx,dists,eps);
#else      
      kdTree->annkSearch(&p_matFromXf[i],KNN,nnIdx,dists,eps);
      index_t frKnn = KNN;
#endif
      // if there are more neighbors than KNN, restrict the below loop till KNN
      if(frKnn > KNN)
        frKnn = KNN;

      for(unsigned int k=0;k<frKnn;k++)
      {
        //modeTo = "";
        //cout<<"========================"<<endl;
        //cout<<"nnIdx: "<<nnIdx[k]<<endl;
        for(map <string,vector<index_t> >::iterator modeRange_it = map_modeRange.begin();
           modeRange_it != map_modeRange.end();
           modeRange_it++)
        {
          index_t begIdx = modeRange_it->second[0];
          index_t numIdx = modeRange_it->second[1];
          //cout<<"modeTO :"<<modeTo<<" "<<begIdx<<" "<<numIdx<<endl;
          if( (index_t)nnIdx[k] >= begIdx
              && (index_t)nnIdx[k] < numIdx )
          {
            rowIdxTo = nnIdx[k] - begIdx;
            modeTo = modeRange_it->first;
            break;
          }

        }
        //if(modeTo=="")
          //ERROR("unkown data point!!");
        //cout << rowIdxFrom+mi.map_cumMatLengths[modeFrom] << " " << rowIdxTo+mi.map_cumMatLengths[modeTo] << " " << dists[k] << endl;
        graph.set(rowIdxFrom+mi.map_cumMatLengths[modeFrom],
            rowIdxTo+mi.map_cumMatLengths[modeTo],
            sqrt(dists[k]));
      }
    }
    delete [] nnIdx;
    delete [] dists;
    delete kdTree;
    //annDeallocPts(dataPts);
    delete [] dataPts;
  }
  graphPB.close();
  cout<<endl;
  annClose();
  return;
}


void buildGraph(modeInfo &mi, const double threshNorm, sparseMat_OLD &graph)
{
  string modeFrom,modeTo;
  vector<string> vec_adjModes;
  vector<string>::iterator modeList_it, adjModeList_it;  
  map_modeToMatLength::iterator matLengths_it;
  index_t i,j,rowIdxFrom,rowIdxTo;
  double *p_matFromXf,*p_matToXi;
  double norm;

// progress bar
  index_t k,currBarLen=0;
  index_t barLen = 100;
  cout<<"[";
  for(k=0;k<barLen;k++)
    cout<<" ";
  cout<<"]"<<flush;
  for(k=0;k<barLen+1;k++)
    cout<<"\b";
  k=0;
////////////////

//    cout<<(double)(100*k/mi.numModes)<<"%\n";
  for(modeList_it=mi.vec_modeList.begin();modeList_it!=mi.vec_modeList.end();modeList_it++)
  {
    k++;
    if((int)((k*100)/mi.vec_modeList.size() > currBarLen))
      for(index_t l=0;l<(k*100)/mi.vec_modeList.size()-currBarLen;l++)
        cout<<"^"<<flush;
    currBarLen = (k*100)/mi.vec_modeList.size();

    modeFrom = *modeList_it;
    p_matFromXf = mi.map_trajXf[modeFrom];
    vec_adjModes = mi.map_adjModes[modeFrom];
//    cout<<"\n"<<modeFrom<<":";
    //loop for its every adjacent mode
    for(adjModeList_it=vec_adjModes.begin();adjModeList_it!=vec_adjModes.end();adjModeList_it++)
    {
      modeTo = *adjModeList_it;
//      cout<<" "<<modeTo;
      p_matToXi = mi.map_trajXi[modeTo];
//      cout<<modeFrom<<"-->"<<modeTo<<endl;          
      for(i=0,rowIdxFrom=0;i<mi.map_matLengths[modeFrom]*mi.numStateVars;i+=mi.numStateVars,rowIdxFrom++)
      {
        for(j=0,rowIdxTo=0;j<mi.map_matLengths[modeTo]*mi.numStateVars;j+=mi.numStateVars,rowIdxTo++)
        {
          norm = computeNorm(mi,&p_matFromXf[i],&p_matToXi[j]);
          if(norm<=threshNorm)
          {
          // for bball dijkstra crashes if this is not there..wonder why?
          //  if(norm == 0)
          //    norm = 0.00001;
            // direction of a vectpr in the graph: row --> col
            WRITE_SPARSE_MAT(graph,
              rowIdxFrom+mi.map_cumMatLengths[modeFrom],
              rowIdxTo+mi.map_cumMatLengths[modeTo],
              norm);
          }
        }
      }
    }
  }
  cout<<endl;
  return;
}

void addSuperTerminalNodes(modeInfo &mi, sparseMat &graph)
{
  index_t superSinkID = graph.getTotR() - 1;
  index_t superSourceID = superSinkID - 1;
  // add super source node
  for(index_t i=0;i<mi.vec_xi.size();i++)
  {
    //WRITE_SPARSE_MAT(graph,superSourceID,mi.vec_xi[i],nominalWt);
    graph.set(superSourceID,mi.vec_xi[i],SMALL_WT);
  }
  // add super sink node
  for(index_t i=0;i<mi.vec_xf.size();i++)
  {
    //WRITE_SPARSE_MAT(graph,mi.vec_xf[i],superSinkID,SMALL_WT);
    graph.set(mi.vec_xf[i],superSinkID,SMALL_WT);
  }
}

int main(int argc, char *argv[])
{
  
  string sysName;// = "./heat01-";//"./heat01-";
  //cout<<"Name of the system: ";
  //cin>>sysName;
  if(argc < 2)
  {
    printf("syntax: %s <info file name>\n", argv[0]);
    exit(0);
  }
  else
    sysName = (string)argv[1];
  //sysName = sysName+"-";
  //cout<<"check name: "<<sysName<<endl;

  index_t i;
  double threshNorm;
  double threshDist;
  boxThresh bt_threshPathLength;
  fstream ofpG,ofpD,ofpP,ofpFD,ofpFP,ifpG;
  bool readFromSavedGraph = false;

  modeInfo mi;

  cout<<"reading file..\n";
  getInputFromFile(mi,sysName);
  cout<<"done.\n";
  cout<<"compensating for MATLAB's indexing...\n";
  //compensate for Matlab's !@#!@# indexing
  for(i=0;i<mi.vec_xi.size();i++)
    mi.vec_xi[i]--;
  for(i=0;i<mi.vec_xf.size();i++)  
    mi.vec_xf[i]--;
  cout<<"done.\n";

//  readConfig(threshNorm,threshDist,readFromSavedGraph);

  //overwrite threshNorm
  if(argc == 3)
  {
    cout<<"commandline threshNorm value gets priority over config value!"<<endl;
    threshNorm = ::atof(argv[2]);
  }

  cout<<"================== config ================\n";
  cout.precision(10);
  cout<<"threshNorm = "<<threshNorm<<endl;
//  cout<<"threshDist = "<<threshDist<<endl;
  cout<<"==================        ================\n";

  //getFp((char*)"distMat.dat",fstream::out|fstream::binary,ofpD);
  //getFp((char*)"pathMat.dat",fstream::out|fstream::binary,ofpP);

  if(readFromSavedGraph==true)
  {
    // do nothing
  }
  else
  {
//    string graphFileName = "./"+sysName+float2str(threshNorm)+".graph";
    string graphFileName = "./"+sysName+".graph";
    getFp(graphFileName.c_str(),fstream::out|fstream::binary,ofpG);
    // total num of rows/cols = total num of pi + 2 super term nodes
    sparseMat graph(mi.totalTrajCount+2,mi.totalTrajCount+2,&ofpG);
    cout<<"building graph...\n";
    //cout<<"writing graph to file..\n";
    //ofpG<<"length: "<<len<<endl;
    buildGraphUsingANN(mi,threshNorm,graph);
    addSuperTerminalNodes(mi,graph);
    graph.write();
    //delete graph;
    //cout<<"length: "<<graph.vec_colIdx.size()<<endl;
    //writeGraph(graph,ofpG);

/*    
    sparseMat graph(mi.totalTrajCount+2,mi.totalTrajCount+2);
    cout<<"building graph...\n";    
    buildGraphUsingANN(mi,threshNorm,graph);
    addSuperTerminalNodes(mi,graph);
    cout<<"writing graph to file..\n";
    graph.write(&ofpG);
*/
  }
/*
  cout<<"running dijkstra...\n";
//  cout<<mi.vec_xi.size()<<endl;
//  cout<<mi.vec_xf.size()<<endl;
  pathMat_t pathMatrix (mi.vec_xi.size(),vector<vector<index_t> >(mi.vec_xf.size()));
  distMat_t distMatrix (mi.vec_xi.size(),vector<double>(mi.vec_xf.size()));
  if(readFromSavedGraph==true)
  {
    getFp((char*)"./graph.dat",fstream::in|fstream::binary,ifpG);
    djk_igraph(graph,mi.vec_xi,mi.vec_xf,pathMatrix,distMatrix,&ifpG);
    ifpG.close();
  }
  else
  {
    djk_igraph(graph,mi.vec_xi,mi.vec_xf,pathMatrix,distMatrix,NULL);
  }
  cout<<"writing dijkstra results...\n";
  writeDijkstraResults(ofpD,ofpP,pathMatrix,distMatrix,mi);

  cout<<"filtering and writing traj\n";
  vector<index_t> vec_rowIdx, vec_colIdx;

  // interactive filtering loop
  int retry = 1;
  do
  {
    vec_rowIdx.clear();
    vec_colIdx.clear();    
    filterPaths(threshDist, bt_threshPathLength, mi, distMatrix, pathMatrix, vec_rowIdx, vec_colIdx);

    //write the end result
    string filePath = sysName+"filtDistMat.dat";
    getFp((const char*)(filePath.c_str()),fstream::out|fstream::binary,ofpFD);
    filePath = sysName+"filtPathMat.dat";
    getFp((const char*)(filePath.c_str()),fstream::out|fstream::binary,ofpFP);
    writeFiltered(vec_rowIdx,vec_colIdx,ofpFD, ofpFP,pathMatrix,distMatrix);
    ofpFD.close();
    ofpFP.close();

    cout<<"total number of paths = "<<vec_colIdx.size()<<endl;
    cout<<"Retry again with a new total cost threshold?(0/1): ";
    cin>>retry;
    if(retry==1)
    {
      cout<<"Enter total cost threshold: ";
      cin>>threshDist;
    }
  }while(retry == 1);
*/
  // cleanup
  ofpG.close();
  ofpD.close();
  ofpP.close();
  destroyMi(mi);
}
