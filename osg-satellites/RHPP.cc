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

#include "RHPP.h"

RHPP::RHPP() {
    // TODO Auto-generated constructor stub

}

RHPP::~RHPP() {
    // TODO Auto-generated destructor stub
}


Path RHPP::findPath(const Graph& graph, int start, int des) {
    int total_satellites = 66; // 总卫星数
    int satellites_per_plane = 11; // 每个轨道面的卫星数
    int planes = total_satellites / satellites_per_plane; // 轨道面数

    Path RHPPPath;
    RHPPPath.nodes.push_back(66);
    RHPPPath.distance = 0;
    int this_index = start;
    int this_orbit = start / satellites_per_plane;
    int this_sat = start % satellites_per_plane;

    int des_index = des;
    int des_orbit = des / satellites_per_plane;
    int des_sat = des % satellites_per_plane;

    int satellite_index = this_index; // 给定的卫星编号

    int plane = satellite_index / satellites_per_plane;
    int position = satellite_index % satellites_per_plane;

    int above = (position == 0) ? satellite_index + 10 : satellite_index - 1;
    int below = (position == 10) ? satellite_index - 10 : satellite_index + 1;
    int left = (plane == 0) ? satellite_index + 11 * (planes - 1) : satellite_index - 11;
    int right = (plane == planes - 1) ? satellite_index - 11 * (planes - 1) : satellite_index + 11;

    std::vector<Loc> indexes;
    indexes.push_back({left/satellites_per_plane, left%satellites_per_plane});
    indexes.push_back({right/satellites_per_plane, right%satellites_per_plane});
    indexes.push_back({above/satellites_per_plane, above%satellites_per_plane});
    indexes.push_back({below/satellites_per_plane, below%satellites_per_plane});

    // 找到下一跳

    while(1){
        if (this_index == des_index){
            RHPPPath.nodes.push_back(67);
            return RHPPPath;
        } else {
            bool right_step = (this_orbit < des_orbit) ? true : false;
            bool left_step = (this_orbit > des_orbit) ? true : false;
            bool above_step = (this_sat > des_sat) ? true : false;
            bool below_step = (this_sat < des_sat) ? true : false;
            int flag = 0;
            while(flag == 0){
                std::srand(static_cast<unsigned int>(std::time(nullptr)));

                // 生成一个0到3之间的随机整数（rand()生成的随机数是从0开始的）
                int randomIndex = std::rand() % 4;

                // 将随机索引转换为1到4之间的整数
                int next_hop = randomIndex + 1;

                switch (next_hop){
                case 1:
                    if (left_step){
                        RHPPPath.nodes.push_back(left);
                        this_index = left;
                        renew_data(this_index, satellites_per_plane, this_orbit, this_sat, above, below, left, right);
                        flag = 1;
                    }
                    break;
                case 2:
                    if (right_step){
                        RHPPPath.nodes.push_back(right);
                        this_index = right;
                        renew_data(this_index, satellites_per_plane, this_orbit, this_sat, above, below, left, right);
                        flag = 1;
                    }
                    break;
                case 3:
                    if (above_step){
                        RHPPPath.nodes.push_back(above);
                        this_index = above;
                        renew_data(this_index, satellites_per_plane, this_orbit, this_sat, above, below, left, right);
                        flag = 1;
                    }
                    break;
                case 4:
                    if (below_step){
                        RHPPPath.nodes.push_back(below);
                        this_index = below;
                        renew_data(this_index, satellites_per_plane, this_orbit, this_sat, above, below, left, right);
                        flag = 1;
                    }
                    break;
                }
            }
        }
    }
}

void RHPP::renew_data(int this_index, int satellites_per_plane, int& this_orbit, int& this_sat, int& above, int& below, int& left, int& right){
    int planes = 66 / satellites_per_plane; // 轨道面数
    int plane = this_index / satellites_per_plane;
    int position = this_index % satellites_per_plane;
    this_orbit = this_index / satellites_per_plane;
    this_sat = this_index % satellites_per_plane;
    above = (position == 0) ? this_index + 10 : this_index - 1;
    below = (position == 10) ? this_index - 10 : this_index + 1;
    left = (plane == 0) ? this_index + 11 * (planes - 1) : this_index - 11;
    right = (plane == planes - 1) ? this_index - 11 * (planes - 1) : this_index + 11;
}
