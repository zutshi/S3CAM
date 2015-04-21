
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>
#include <map>
#include <stdlib.h>
#include <math.h>
#include "myTypes.h"
using namespace std;
typedef unsigned int mode_t;

#define FATAL(_msg_) {cout<<"fatal error: "<<(_msg_)<<endl;exit(-1);}

#define NUMMODES "numModes:"
#define NUMSTATEVARS "numStateVars:"
#define MODELIST "modeList:"
#define AdjGraph "adjGraph:"
#define LENGTH "length:"
#define SIZE "size:"
#define NAME "name:"
#define TrajXi "trajsCellMatXi"
#define TrajXf "trajsCellMatXf"
#define Time "timeCellMat"
#define Xi "Xi"
#define Xf "Xf"

#define SMALL_WT (0.000001)

#define WRITE_SPARSE_MAT(_m_, _r_, _c_, _dat_)  \
{                                               \
  _m_.vec_rowIdx.push_back(_r_);                \
  _m_.vec_colIdx.push_back(_c_);                \
  _m_.vec_data.push_back(_dat_);                \
};

struct sparseMat_OLD
{  
  index_t r;
  index_t c;
  vector <index_t> vec_rowIdx;
  vector <index_t> vec_colIdx;
  vector <double> vec_data;
};



typedef vector<vector<vector<index_t> > > pathMat_t;
typedef vector<vector<double> > distMat_t;

void djk_igraph(const sparseMat_OLD &inputGraph, 
  const vector<index_t> vec_src, 
  const vector<index_t> vec_dst,
  vector<vector<vector<index_t> > > &pathMatrix,
  vector<vector<double> > &distMatrix, fstream *fp);

struct boxThresh
{
  unsigned int ub;
  unsigned int lb;
};


//typedef map <mode_t,double*> map_modeToArr;
typedef map<string,vector <string> > map_modeToVec_modes;
typedef map<string, double*> map_modeToMat;
typedef map<string, index_t> map_modeToMatLength;
//typedef map<string, modeInfo> map_modeStrTomodeObj;

struct modeInfo
{
  index_t numModes;
  index_t numStateVars;
  index_t totalTrajCount;
  vector<string> vec_modeList;
  map_modeToMatLength map_matLengths;
  map_modeToMatLength map_cumMatLengths;
  map_modeToVec_modes map_adjModes;
//  index_t *p_matLengths;
//  double **pp_mat_trajXi;
//  double **pp_mat_trajXf;
//  double **pp_arr_time;
  //kepp them separate to avaoid memory fragmentation
  map_modeToMat map_trajXi;
  map_modeToMat map_trajXf;
  map_modeToMat map_time;
  vector<index_t> vec_xi;
  vector<index_t> vec_xf;
  index_t longestPath;
};


struct systemInfo
{
  index_t numModes;
  index_t numStateVars;
  index_t totalTrajCount;
  vector<string> vec_modeList;

  //map_modeStrTomodeObj map_modeObjs;

  map_modeToMatLength map_matLengths;
  map_modeToMatLength map_cumMatLengths;
  map_modeToVec_modes map_adjModes;

  //keep them separate to avaoid memory fragmentation
  map_modeToMat map_trajXi;
  map_modeToMat map_trajXf;
  map_modeToMat map_time;
  vector<index_t> vec_xi;
  vector<index_t> vec_xf;
  index_t longestPath;
};
