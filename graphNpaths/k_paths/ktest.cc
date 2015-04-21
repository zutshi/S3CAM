// Copyright 2014 Jonathan Graehl - http://graehl.org/
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/* k Shortest Paths in O(E*log V + L*k*log k) time (L is path length)
   Implemented by Jonathan Graehl (jonathan@graehl.org)
   Following David Eppstein's "Finding the k Shortest Paths" March 31, 1997 draft
   (http://www.ics.uci.edu/~eppstein/
   http://www.ics.uci.edu/~eppstein/pubs/p-kpath.ps)
*/


/* k Shortest Paths in O(E*log V + L*k*log k) time (L is path length)
   Implemented by Jon Graehl (jongraehl@earthling.net)
   Following David Eppstein's "Finding the k Shortest Paths" March 31, 1997 draft
   (http://www.ics.uci.edu/~eppstein/
    http://www.ics.uci.edu/~eppstein/pubs/p-kpath.ps)
   */


#include "graph.h"
//#include <strstream>
# include <sstream>
#include <string>

int main(int argc, char *argv[])
{
  if ( argc != 2 ) {
    cout << "Test of k best paths algorithm.  Supply three integer arguments (source state number, destination state number, number of paths) and a graph to stdin.  See readme.txt, sample.graph, or ktest.cc for the graph format.\n";
    return 0;
  }
  int k;
  int source, dest;

  const int maxPathLength = 10;

//  istrstream sstr(argv[1]);
//  istrstream dstr(argv[2]);
//  istrstream kstr(argv[3]);
  //istrstream kstr(argv[1]);
  stringstream kstr(argv[1]);
 /* 
  if ( !((sstr >> source) && (dstr >> dest) && (kstr >> k)) ) {
    cerr << "Bad argument (should be integer) - aborting.\n";    
    return -1;
*/    
  if ( !(kstr >> k) ) {
    cerr << "Bad argument (should be integer) - aborting.\n";    
    return -1;
  }  
  Graph graph;
  cin >> graph;
  source = graph.nStates - 2;
  dest = graph.nStates - 1;
  cerr<<"Enumerating "<<k<<" shortest paths from "<<source<<" to "<<dest<<endl;
  List<List<GraphArc *> > *paths = bestPaths(graph, source, dest, k);

  
  for ( ListIter<List<GraphArc *> > pathIter(*paths) ; pathIter ; ++pathIter ) {
    float pathWeight = 0;
    GraphArc *a;
    ListIter<GraphArc *> arcIter(pathIter.data());//adi: declare outside loop 
    cout<<(*(arcIter.data())).source<<',';//adi: print the super source
    for (  ; arcIter ; ++arcIter ) {//adi: print only dest nodes of the arcs; modified arc <<
      a = arcIter.data();
      pathWeight += a->weight;
      cout << *a << ',';
    }
    cout << pathWeight << '\n';
  }


// same as original code (above), but does not print paths longer than
// maxPathLength
/*
  stringstream s;
  int pl = 0;
  for ( ListIter<List<GraphArc *> > pathIter(*paths) ; pathIter ; ++pathIter ) {
    float pathWeight = 0;
    GraphArc *a;
    ListIter<GraphArc *> arcIter(pathIter.data());//adi: declare outside loop 
    s << (*(arcIter.data())).source;
    s << ',';//adi: print the super source
    for (  ; arcIter ; ++arcIter ) {//adi: print only dest nodes of the arcs; modified arc <<      
      if (pl > 10)
      {
        break;
      }
      pl ++;
      a = arcIter.data();
      pathWeight += a->weight;
      s << *a;
      s << ',';
    }
    if (pl <= 10)
    {
      cout << s.str();
      cout << pathWeight << '\n';
    }
    pl = 0;
    s.str("");
  }
*/



//diagnostic print
/*
  for ( ListIter<List<GraphArc *> > pathIter(*paths) ; pathIter ; ++pathIter ) {
    float pathWeight = 0;
    GraphArc *a;
    for (ListIter<GraphArc *> arcIter(pathIter.data())  ; arcIter ; ++arcIter ) {//adi: print only dest nodes of the arcs; modified arc <<
      a = arcIter.data();
      pathWeight += a->weight;
      cout << a->weight << " ";
      cout << *a << ' ';
    }
    cout << pathWeight << '\n';
  }
*/

  //  cout << graph;
  return 0;
}
