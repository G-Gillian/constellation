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

#include "Dijkstra.h"

#include <sstream>
#include <iomanip>

#include "omnetpp/osgutil.h"

using namespace omnetpp;

Dijkstra::Dijkstra() {
    // TODO Auto-generated constructor stub

}

Dijkstra::~Dijkstra() {
    // TODO Auto-generated destructor stub
}

std::vector<Path> Dijkstra::dijkstra(const Graph& graph, int source) {
    int n = graph.size();
    std::vector<std::vector<int>> paths(n);
    std::vector<double> minDistance(n, std::numeric_limits<double>::infinity());
    std::vector<int> previous(n, -1);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    minDistance[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        int u = pq.top().second;
        double dist = pq.top().first;
        pq.pop();

        if (dist > minDistance[u])
            continue;

        for (const Edge& edge : graph[u]) {
            int v = edge.to;
            double weight = edge.weight;
            double distanceThroughU = dist + weight;
            if (distanceThroughU < minDistance[v]) {
                minDistance[v] = distanceThroughU;
                previous[v] = u;
                pq.push({minDistance[v], v});
            }
        }
    }

    // 构造最短路径
    std::vector<Path> shortestPaths;
    for (int i = 0; i < n; ++i) {
        if (i == source) // 跳过源节点
            continue;

        Path shortestPath;
        shortestPath.distance = minDistance[i]; // 终点的最短路径距离
        int current = i; // 终点
        std::stack<int> reversePath; // 用栈存储路径，以便反向获取路径
        while (current != -1) {
            reversePath.push(current);
            current = previous[current];
        }
        while (!reversePath.empty()) {
            shortestPath.nodes.push_back(reversePath.top());
            reversePath.pop();
        }
        shortestPaths.push_back(shortestPath);
    }

    return shortestPaths;
}

Path Dijkstra::dijkstra(const Graph& graph, int source, int destination) {
    int n = graph.size();

    std::vector<double> minDistance(n, std::numeric_limits<double>::infinity());
    std::vector<int> previous(n, -1);

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;

    minDistance[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        int u = pq.top().second;
        double dist = pq.top().first;
        pq.pop();

        if (dist > minDistance[u])
            continue;

        for (const Edge& edge : graph[u]) {
            int v = edge.to;
            double weight = edge.weight;

            if (weight == -1) // Skip if the edge is blocked
                continue;

            double distanceThroughU = dist + weight;

            if (distanceThroughU < minDistance[v]) {
                minDistance[v] = distanceThroughU;
                previous[v] = u;
                pq.push({distanceThroughU, v});
            }
        }
    }

    // Construct the shortest path
    Path shortestPath;
    if (minDistance[destination] == std::numeric_limits<double>::infinity()) {
        // If no path found, return an empty path
        shortestPath.distance = -1;
    } else {
        shortestPath.distance = minDistance[destination];
        std::stack<int> reversePath;
        int current = destination;
        while (current != -1) {
            reversePath.push(current);
            current = previous[current];
        }
        while (!reversePath.empty()) {
            shortestPath.nodes.push_back(reversePath.top());
            reversePath.pop();
        }
    }

    return shortestPath;
}

const double a = 6378137.0;      // WGS-84 地球赤道半径 (meters)
const double b = 6356752.3142;   // WGS-84 地球极半径 (meters)
const double pi = 3.141592653589793;

double Dijkstra::deg2rad(double degrees) {
    return degrees * pi / 180.0;
}

void Dijkstra::convertGeodeticToECEF(double latitude, double longitude, double altitude, double &x, double &y, double &z) {
    double lat_rad = deg2rad(latitude);
    double lon_rad = deg2rad(longitude);

    double N = a * a / sqrt(a * a * cos(lat_rad) * cos(lat_rad) + b * b * sin(lat_rad) * sin(lat_rad));

    x = ((N + altitude) * cos(lat_rad) * cos(lon_rad)) / 100000;
    y = ((N + altitude) * cos(lat_rad) * sin(lon_rad)) / 100000;
    z = ((b * b / (a * a) * N + altitude) * sin(lat_rad)) / 100000;
}

bool Dijkstra::lineOfSightBlocked(const CartesianCoord& pos1, const CartesianCoord& pos2, double earthRadius) {
    CartesianCoord G = pos2;
    CartesianCoord D = {
        pos2.x - pos1.x,
        pos2.y - pos1.y,
        pos2.z - pos1.z
    };

    double a = D.x * D.x + D.y * D.y + D.z * D.z;
    double b = 2 * (D.x * G.x + D.y * G.y + D.z * G.z);
    double c = G.x * G.x + G.y * G.y + G.z * G.z - earthRadius * earthRadius;

    double delta = b * b - 4 * a * c;
    return delta >= 0; // 如果判别式大于或等于0，则直线与地球相交，视线被阻挡
}
