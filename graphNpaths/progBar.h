#include <string>
#include <ncurses.h>
using namespace std;
class progBar
{
  int typ;
  int currBarLen;
  string name;
  public:
  progBar(string name_="", int typ_=0);
  void disp();
  void update(float newBarLen);
  void close();
};
