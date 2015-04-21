#include <fstream>
#include <vector>
#include "myTypes.h"
using namespace std;

class sparseMat
{
  index_t totR;
  index_t totC;
  index_t len;
  fstream *fp;
  vector <index_t> vec_rowIdx;
  vector <index_t> vec_colIdx;
  vector <double> vec_data;
  bool conserveMem;

  public:
  sparseMat(index_t totR_, index_t totC_, fstream *fp);
  sparseMat(index_t totR_, index_t totC_);
  ~sparseMat();
  index_t getTotR();
  index_t getTotC();
  index_t getTotLen();
  void set(index_t r, index_t c, double data);
  void write(fstream *fp_=NULL);
  void read_withLen(fstream *fp_);
  void read(fstream *fp_);

};
