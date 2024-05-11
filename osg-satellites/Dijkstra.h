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

#ifndef DIJKSTRA_H_
#define DIJKSTRA_H_

#include "OsgEarthScene.h"

#include <vector>
#include <queue>
#include <limits>
#include <iostream>
#include <typeinfo>
#include <string>
#include <cmath>

struct Edge {
    int to;
    double weight;
};
struct Path {
    std::vector<int> nodes;
    double distance;
};
struct CartesianCoord {
    double x, y, z;
};
using Graph = std::vector<std::vector<Edge>>;

using namespace omnetpp;

class Dijkstra {
    public:
        Dijkstra();
        virtual ~Dijkstra();
        virtual std::vector<Path> dijkstra(const Graph& graph, int source);
        virtual Path dijkstra(const Graph& graph, int source, int destination);
        double deg2rad(double degrees);
        void convertGeodeticToECEF(double latitude, double longitude, double altitude, double &x, double &y, double &z);
        bool lineOfSightBlocked(const CartesianCoord& s, const CartesianCoord& g, double earthRadius = 6371000);
};

#endif /* DIJKSTRA_H_ */
