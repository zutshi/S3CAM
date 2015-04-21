#include <iostream>
#include <string>
#include <math.h>
#include "progBar.h"

#define CYCLIST1 "    ---------- __o\n"
#define CYCLIST2 "   --------  _ \\<,_\n"
#define CYCLIST3 " -------    (*)/ (*)\n"

#define MAX_BAR_LEN 100

  //takes a default param in h file
  progBar::progBar(string name_,int typ_)
  {
    //initialize the progBar object
    name = name_;
    typ = typ_;
    currBarLen = 0;
  }

  void progBar::disp()
  {
    if (typ == 0)
    {
      cout<<name<<endl;
      cout<<"[";
      for(unsigned int k=0;k<MAX_BAR_LEN;k++)
        cout<<" ";
      cout<<"]"<<flush;
      for(unsigned int k=0;k<MAX_BAR_LEN+1;k++)
        cout<<"\b";
    }
    else
    {
      //do nothing
    }
  }
  void progBar::update(float newBarLen)
  {
    newBarLen = round(newBarLen*100.0);
    int diff = newBarLen - currBarLen;
    if(diff)
    {
      for(unsigned int k=0;k<diff;k++)
      {
        if(typ == 0)
        {
            cout<<"="<<flush;
        }
        else
        {
          //do nothing
        }
      }
      currBarLen = newBarLen;
    }
    else
    {
      //why are you making the bar shorter!
    }
  }
  void progBar::close()
  {
    if(typ==0)
      //nothing to do
      cout<<endl;
  }

/*
    if((int)((bar_k*100)/mi.vec_modeList.size() > currBarLen))
      for(index_t l=0;l<(bar_k*100)/mi.vec_modeList.size()-currBarLen;l++)
        cout<<"^"<<flush;
    currBarLen = (bar_k*100)/mi.vec_modeList.size();
*/
