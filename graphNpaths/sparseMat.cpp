#include <iostream>
#include "error.h"
#include "sparseMat.h"

sparseMat::sparseMat(index_t totR_, index_t totC_, fstream *fp_)
{
  totR = totR_;
  totC = totC_;
  len = 0;
  conserveMem = true;
  fp = fp_;
  //write header
  //(*fp)<<"size: "<<totR<<" "<<totC<<endl;

  //temporary fix for k_paths
  (*fp)<<totR<<endl;
}
sparseMat::sparseMat(index_t totR_, index_t totC_)
{
  fp = NULL;
  totR = totR_;
  totC = totC_;
  len = 0;
  conserveMem = false;
}

void sparseMat::set(index_t r, index_t c, double data)
{
  if(conserveMem==true)
  {
      (*fp)<< r << " " << c << " " << fixed << data << endl;
  }
  else
  {
    vec_rowIdx.push_back(r);
    vec_colIdx.push_back(c);
    vec_data.push_back(data);
  }
  len++;
}
// file pointer optional
void sparseMat::write(fstream *fp_)
{
  if(conserveMem==true)
  {
    //do nothing
    cout<<"sparseMat written\n";
  }
  else
  {
    fp = fp_;
    if(fp == NULL)
      ERROR("fp = NULL");
    //(*fp)<<"size: "<<totR<<" "<<totC<<endl;
    //(*fp)<<"length: "<<len<<endl;
    // for k_paths..temporary fix
    (*fp)<<totR<<endl;
    for(index_t i=0;i<len;i++)
    {
      (*fp)<<vec_rowIdx[i]<<" "<<vec_colIdx[i]<<" "<<fixed<<vec_data[i]<<endl;
    }
  }
  return;
}

void sparseMat::read_withLen(fstream *fp_)
{
  fp = fp_;
  index_t len;
  string str_tmp;  

  (*fp)>>str_tmp;
  if(str_tmp!="size:")
    ERROR("sparse mat header: <size> not found");

  (*fp)>>totR;
  (*fp)>>totC;
  cout<<"graph size: "<<totR<<endl;

  (*fp)>>str_tmp;
  if(str_tmp!="length:")
    ERROR("sparse mat header: <length> not found");
  
  (*fp)>>len;
  cout<<"graph length: "<<len<<endl;
  vec_rowIdx.resize(len);
  vec_colIdx.resize(len);
  vec_data.resize(len);
  for(index_t i=0;i<len;i++)
  {
    (*fp)>>vec_rowIdx[i];
    (*fp)>>vec_colIdx[i];
    (*fp)>>vec_data[i];
  }
  return;
}

void sparseMat::read(fstream *fp_)
{
  fp = fp_;
  index_t len = 0;
  string str_tmp;  

  (*fp)>>str_tmp;
  if(str_tmp!="size:")
    ERROR("sparse mat header: <size> not found");

  (*fp)>>totR;
  (*fp)>>totC;
  cout<<"graph size: "<<totR<<endl;

  //TODO:untested!
  while(fp->good())
  {
    index_t idx;
    double data;
    (*fp)>>idx;
    vec_rowIdx.push_back(idx);
    (*fp)>>idx;
    vec_colIdx.push_back(idx);
    (*fp)>>data;
    vec_data.push_back(data);
    len++;
  }
  return;
}
index_t sparseMat::getTotR()
{
  return totR;
}
index_t sparseMat::getTotC()
{
  return totC;
}
index_t sparseMat::getTotLen()
{
  return len;
}
sparseMat::~sparseMat()
{
  /*
  if(fp != NULL)
    fp->close();
 */
}
