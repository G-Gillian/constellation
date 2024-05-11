//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef RHPP_H_
#define RHPP_H_
#include "Dijkstra.h"
#include <omnetpp.h>
#include <cstdlib> // 包含rand()和srand()
#include <ctime>   // 包含time()函数

struct Loc{
    int orbit_num;
    int sat_num;
};

class RHPP {
public:
    RHPP();
    virtual ~RHPP();
    Path findPath(const Graph& graph, int start, int des);
    void renew_data(int this_index, int satellites_per_plane, int& this_orbit, int& this_sat, int& above, int& below, int& left, int& right);
};
#endif /* RHPP_H_ */
