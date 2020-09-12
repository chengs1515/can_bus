#pragma once

#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace can_bus{

class Interpolation2d{

 public:
    typedef std::vector<std::tuple<double, double, double>> DataType;
    typedef std::pair<double, double> KeyType;

    Interpolation2d() = default;

    /**
     * @brief initialize Interpolation2D internal table
     * @param xyz passing interpolation initialization table data
     * @return true if init is ok.
     */
    bool Init(const DataType &xyz);

    /**
     * @brief linear interpolate from 2D key (double, double) to one double value.
     * @param xyz passing interpolation initialization table data
     * @return true if init is ok.
     */
    double Interpolate(const KeyType &xy) const;

    private:
    double InterpolateYz(const std::map<double, double> &yz_table,
                        double y) const;

    double InterpolateValue(const double value_before, const double dist_before,
                            const double value_after,
                            const double dist_after) const;

    std::map<double, std::map<double, double>> xyz_;

};
}