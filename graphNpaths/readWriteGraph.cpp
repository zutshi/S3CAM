#include "common.h"
#include "readWriteGraph.h"
void chkNread(ifstream &fp, string s_tag, void* x)
{
  string s_tmp;
  fp>>s_tmp;
  if(s_tmp!=s_tag)
  {
    FATAL("expected <" + s_tag + ">, but found <" + s_tmp + ">\n")
  }
  else
  {
    if(s_tag == NUMMODES||s_tag == NUMSTATEVARS)
      fp>>*(index_t*)x;
    // current implementation treats size the same as length by ignoring the columns of the matrix
    else if (s_tag == SIZE||s_tag == LENGTH)
      fp>>*(index_t*)x;
    else if (s_tag == NAME)
      fp>>*(string*)x;
    else if (s_tag == TrajXi||s_tag == TrajXf||s_tag == Time
              ||s_tag == Xi||s_tag == Xf||s_tag == MODELIST||s_tag == AdjGraph)
    {
      //do nothing
    }
    else
      FATAL("chkNread, undefined TAG: "+s_tag)
    }
  return;
}

void readIdxArr(ifstream &fp, index_t* p_mat,index_t len)
{
  index_t i;
  for(i=0;i<len;i++)
  {
    fp>>p_mat[i];
   // cout<<p_mat[i]<<endl;
  }
  return;
}
void readDoubleArr(ifstream &fp, double* p_mat,index_t len)
{
  index_t i;
  for(i=0;i<len;i++)
  {
    fp>>p_mat[i];
   // cout<<p_mat[i]<<endl;
  }
  return;
}

void readMatHdr(ifstream &fp, string &s_tmp, index_t &matRows)
{
  string s_dummy;
    chkNread(fp,NAME,&s_tmp);
    //put this in the map
    chkNread(fp,SIZE,&matRows);
    //ignore column infor for now
    fp>>s_dummy;
}

void getInputFromFile(modeInfo &mi, string sysName)
{
  string s_mat,s_tmp, s_modeName, s_dummy,s_adjModeName;
  vector <string> vec_adjModeList;
  index_t i,j, matRows, arrLength,len;
  double *p_mat;
  ifstream fp;
//  char fileName[] = "heat01-info.dat";//"./heat01-info.dat";
  string fileName = sysName + "-info.dat";

  fp.exceptions (ifstream::failbit);
  try
    {fp.open(fileName.c_str(),ios::binary);}
  catch (ifstream::failure e)
    {FATAL("cannot open input file")}
  cout<<"opening file: "<<fileName<<endl;  
  /*
  numModes: 144
  numStateVars: 4
  trajsCellMatXi
  name: mode0
  size: 200 4
  */
  chkNread(fp,NUMMODES,&mi.numModes);
  // allocate memory for the matrices
  //mi->p_matLengths = new index_t[mi->numModes];
//  mi->pp_mat_trajXi = new double* [mi->numModes];
//  mi->pp_mat_trajXf = new double* [mi->numModes];
//  mi->pp_arr_time = new double* [mi->numModes];

  chkNread(fp,NUMSTATEVARS,&mi.numStateVars);

  chkNread(fp,MODELIST,NULL);
  for(i=0;i<mi.numModes;i++)
  {
    fp>>s_modeName;
    mi.vec_modeList.push_back(s_modeName);
  }
  chkNread(fp,AdjGraph,NULL);
  for(i=0;i<mi.numModes;i++)
  {
    fp>>s_modeName;
    s_modeName.erase(s_modeName.size()-1);
    fp>>len;
    vec_adjModeList.resize(0);
    vec_adjModeList.resize(len);
    for(j=0;j<len;j++)
    {
      fp>>s_adjModeName;
      vec_adjModeList[j] = s_adjModeName;
    }
    mi.map_adjModes[s_modeName] = vec_adjModeList;
  }
  chkNread(fp,TrajXi,NULL);
  for(i=0;i<mi.numModes;i++)
  {
    readMatHdr(fp,s_modeName,matRows);
    mi.map_matLengths[s_modeName] = matRows;
    arrLength = matRows*mi.numStateVars;
    p_mat = new double [arrLength];
    readDoubleArr(fp,p_mat,arrLength);
    mi.map_trajXi[s_modeName] = p_mat;
  }

  chkNread(fp,TrajXf,NULL);
  for(i=0;i<mi.numModes;i++)
  {
    readMatHdr(fp,s_modeName,matRows);
    arrLength = matRows*mi.numStateVars;
    p_mat = new double [arrLength];
    readDoubleArr(fp,p_mat,arrLength);
    mi.map_trajXf[s_modeName] = p_mat;
//    mi->pp_mat_trajXf[i] = p_mat;
  }

  chkNread(fp,Time,NULL);
  for(i=0;i<mi.numModes;i++)
  {
    readMatHdr(fp,s_modeName,matRows);
    arrLength = matRows;
    p_mat = new double [arrLength];
    readDoubleArr(fp,p_mat,arrLength);
    mi.map_time[s_modeName] = p_mat;
//    mi->pp_arr_time[i] = p_mat;
  }

  chkNread(fp,Xi,NULL);
  chkNread(fp,SIZE,&arrLength);
  fp>>s_dummy;
  mi.vec_xi.resize(arrLength);
  for(i=0;i<arrLength;i++)
    fp>>mi.vec_xi[i];
//  readIdxArr(fp,mi.p_arr_xi,arrLength);

  chkNread(fp,Xf,NULL);
  chkNread(fp,SIZE,&arrLength);
  fp>>s_dummy;
  mi.vec_xf.resize(arrLength);
//  readIdxArr(fp,mi.p_arr_xf,arrLength);
  for(i=0;i<arrLength;i++)
    fp>>mi.vec_xf[i];
  
  fp.close();

  index_t cumLength = 0;
  for(vector<string>::iterator it = mi.vec_modeList.begin();it!=mi.vec_modeList.end();it++)
  {
    s_modeName = *it;
    mi.map_cumMatLengths[s_modeName] = cumLength;
    cumLength += mi.map_matLengths[s_modeName];
  }
  mi.totalTrajCount = cumLength;
}

void destroyMi(modeInfo &mi)
{
  map_modeToMat::iterator modeToMat_it;
//  delete [] mi.p_arr_xi;
//  delete [] mi.p_arr_xf;
  for(modeToMat_it=mi.map_trajXi.begin();modeToMat_it!=mi.map_trajXi.end();modeToMat_it++)
  {
    delete [] (*modeToMat_it).second;
  }
  for(modeToMat_it=mi.map_trajXf.begin();modeToMat_it!=mi.map_trajXf.end();modeToMat_it++)
  {
    delete [] (*modeToMat_it).second;
  }
  for(modeToMat_it=mi.map_time.begin();modeToMat_it!=mi.map_time.end();modeToMat_it++)
  {
    delete [] (*modeToMat_it).second;
  }
}

void writeDijkstraResults(fstream &ofpD, fstream &ofpP, pathMat_t pathMatrix,distMat_t distMatrix,modeInfo &mi)
{
  index_t i,j,k;
  for(i=0;i<distMatrix.size();i++)
  {
    for(j=0;j<distMatrix[i].size();j++)
    {
      ofpD<<distMatrix[i][j]<<",";
    }
    //rease the alst comma
    ofpD.seekp(-1,ios::cur);
    ofpD<<"\n";
  }
/*

  for(i=0;i<pathMatrix.size();i++)
  {
    for(j=0;j<pathMatrix[i].size();j++)
    {
      for(k=0;k<pathMatrix[i][j].size();k++)
      {
        ofpP<<pathMatrix[i][j][k]<<",";
      }
      //erase the last comma
      ofpP.seekp(-1,ios::cur);
      ofpP<<"\n";
    }
  }

*/

/*
  //calculate max path length
  mi.longestPath = 0;
  for(i=0;i<pathMatrix.size();i++)
    for(j=0;j<pathMatrix[i].size();j++)
      for(k=0;k<pathMatrix[i][j].size();k++)
        if(mi.longestPath < pathMatrix[i][j].size())
          mi.longestPath = pathMatrix[i][j].size();
*/
  //compensate for MATLAB's @$@#$ columnwise layout and write the path matrix in columnwise form.
  // mi.vec_xi/f.size() should be in sync with pathMatrix.size and pathMatrix[i].size
/*
  for(j=0;j<mi.vec_xf.size();j++)
  {
    for(i=0;i<mi.vec_xi.size();i++)
    {
      if(distMatrix[i][j] == INFINITY)
      {
        for(k=0;k<mi.longestPath;k++)
        {
          ofpP<<0<<",";
        }
      }
      else
      {
        for(k=0;k<pathMatrix[i][j].size();k++)
        {
            ofpP<<pathMatrix[i][j][k]<<",";
        }
        //pad with 0s
        for(;k<mi.longestPath;k++)
        {
            ofpP<<0<<",";
        }
      }
      //erase the last comma
      ofpP.seekp(-1,ios::cur);
      ofpP<<"\n";
    }
  }
*/

  for(j=0;j<mi.vec_xf.size();j++)
  {
    for(i=0;i<mi.vec_xi.size();i++)
    {
      //write the size as the first element
      ofpP<<pathMatrix[i][j].size()<<",";
      for(k=0;k<pathMatrix[i][j].size();k++)
      {
          ofpP<<pathMatrix[i][j][k]<<",";
      }      
      //erase the last comma
      ofpP.seekp(-1,ios::cur);
      ofpP<<"\n";
    }
  }
}

void filterPaths(const double &threshDist, boxThresh &bt_threshPathLength, const modeInfo &mi, const distMat_t distMat, pathMat_t pathMat,
  vector<index_t> &vec_rowIdx, vector<index_t> &vec_colIdx)
{
  index_t i,j;
  bt_threshPathLength.lb = 0;
  bt_threshPathLength.ub = 10;

  for(i=0;i<mi.vec_xi.size();i++)
    for(j=0;j<mi.vec_xf.size();j++)
    {
      if(distMat[i][j]<=threshDist 
      && pathMat[i][j].size()<=bt_threshPathLength.ub && pathMat[i][j].size()>=bt_threshPathLength.lb)
      {
        vec_rowIdx.push_back(i);
        vec_colIdx.push_back(j);
      }
    }
}
void writeFiltered(const vector<index_t> vec_rowIdx, const vector<index_t> vec_colIdx, 
  fstream &ofpFD, fstream &ofpFP,
  const pathMat_t pathMat, const distMat_t distMat)
{
  index_t i,k;
  //write
  for(i=0;i<vec_rowIdx.size();i++)
    ofpFD<<vec_rowIdx[i]<<",";
  ofpFD.seekp(-1,ios::cur);
  ofpFD<<"\n";
  
  for(i=0;i<vec_colIdx.size();i++)
    ofpFD<<vec_colIdx[i]<<",";
  ofpFD.seekp(-1,ios::cur);
  ofpFD<<"\n";
      
  for(i=0;i<vec_rowIdx.size();i++)
    ofpFD<<distMat[vec_rowIdx[i]][vec_colIdx[i]]<<",";
  ofpFD.seekp(-1,ios::cur);
  ofpFD<<"\n";

  //write Path matrix
  for(i=0;i<vec_rowIdx.size();i++)
  {
    for(k=0;k<pathMat[vec_rowIdx[i]][vec_colIdx[i]].size();k++)
    {
      ofpFP<<pathMat[vec_rowIdx[i]][vec_colIdx[i]][k]<<",";
    }
    ofpFP.seekp(-1,ios::cur);
    ofpFP<<"\n";
  }
}

void getFp(const char fileName[], const ios_base::openmode mode, fstream &fp)
{
  try
  {
    fp.open(fileName,mode);
  }
  catch (fstream::failure e)
  {
    FATAL("cannot open file")
  }
}

void readConfig(double &threshNorm,double &threshDist,bool &readFromSavedGraph)
{
  fstream fp;
  string str_tmp;
  int tmp;
  getFp((char*)"./createGraph.config",fstream::in|fstream::binary,fp);

  fp>>str_tmp;
  if(str_tmp!="IndividualGapThreshold")
    FATAL("IndividualGapThreshold not found")
  else
    fp>>threshNorm;

  fp>>str_tmp;
  if(str_tmp!="TotalDistanceThreshold")
    FATAL("TotalDistanceThreshold not found")
  else
    fp>>threshDist;

  fp>>str_tmp;
  if(str_tmp!="ReadFromSavedGraph")
    FATAL("ReadFromSavedGraph not found")
  else
  {
    fp>>tmp;
    if(tmp==0)
      readFromSavedGraph = false;
    else
      readFromSavedGraph = true;
  }
  return;
}

