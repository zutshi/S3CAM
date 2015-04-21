
void chkNread(ifstream &fp, string s_tag, void* x);
void readIdxArr(ifstream &fp, index_t* p_mat,index_t len);
void readDoubleArr(ifstream &fp, double* p_mat,index_t len);
void readMatHdr(ifstream &fp, string &s_tmp, index_t &matRows);
void getInputFromFile(modeInfo &mi, string sysName);
//void writeGraph(sparseMat &graph, fstream &fp);
void destroyMi(modeInfo &mi);
void writeDijkstraResults(fstream &ofpD, fstream &ofpP, pathMat_t pathMatrix,distMat_t distMatrix,modeInfo &mi);
void filterPaths(const double &threshDist, boxThresh &bt_threshPathLength, const modeInfo &mi, const distMat_t distMat, pathMat_t pathMat,
  vector<index_t> &vec_rowIdx, vector<index_t> &vec_colIdx);
void writeFiltered(const vector<index_t> vec_rowIdx, const vector<index_t> vec_colIdx, 
  fstream &ofpFD, fstream &ofpFP,
  const pathMat_t pathMat, const distMat_t distMat);
void getFp(const char fileName[], const ios_base::openmode mode, fstream &fp);
void readConfig(double &threshNorm,double &threshDist,bool &readFromSavedGraph);
//void readSavedGraph(sparseMat &graph, fstream &fp);


