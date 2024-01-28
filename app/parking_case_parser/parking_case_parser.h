/**
 * @Author: Xia Yunkai
 * @Date:   2024-01-27 23:29:04
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-01-28 21:44:24
 */
#ifndef __PARKING_CASE_PARSER_H__
#define __PARKING_CASE_PARSER_H__
#include "common/data_types.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
namespace parking_case_parser {

class ParkingCase {

public:
  bool Parse(const std::string &path) {

    std::vector<float> mapData;
    if (!ReadCsv(path, mapData)) {
      return false;
    }
    m_startPose[0] = mapData[0];
    m_startPose[1] = mapData[1];
    m_startPose[2] = mapData[2];
    m_tarPose[0] = mapData[3];
    m_tarPose[1] = mapData[4];
    m_tarPose[2] = mapData[5];

    int obs_num = static_cast<int>(mapData[6]);
    int offset = 6 + obs_num;
    m_obs.clear();
    for (int i = 1; i <= obs_num; i++) {
      common::Polygon2f ob;

      int point_num = mapData[6 + i];
      for (int j = 1; j <= point_num; ++j) {
        common::Vec2f point;

        point[0] = mapData[offset + 2 * j - 1];
        point[1] = mapData[offset + 2 * j];
        ob.emplace_back(point);
      }
      m_obs.emplace_back(ob);

      offset += 2 * point_num;
    }

    return true;
  }

  bool ReadCsv(const std::string &file_name, std::vector<float> &data) {
    data.clear();
    std::ifstream in_file(file_name, std::ios::in);
    if (!in_file.is_open()) {
      std::cout << "can not open fine" << std::endl;
      return false;
    }
    std::string line;
    std::string word;
    std::stringstream ss;
    getline(in_file, line);
    ss << line;
    while (getline(ss, word, ',')) {
      double temp;
      data.push_back(stof(word));
    }
    in_file.close();
    return true;
  }

public:
  common::Vec3f m_startPose;
  common::Vec3f m_tarPose;
  std::vector<common::Polygon2f> m_obs;
};

} // namespace parking_case_parser

#endif /* __PARKING_CASE_PARSER_H__ */
