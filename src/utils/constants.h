#pragma once

#include <Eigen/Dense>


namespace robot::constants {

    template<typename T> 
    using Vector3 = Eigen::Vector<T, 3>;
    template<typename T>
    using Vector4 = Eigen::Vector<T, 4>;

} // namespace robot::constants