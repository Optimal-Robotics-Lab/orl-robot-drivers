#pragma once

#include "src/utils/enum_converter_def.h"

#include "src/h1_2/lib/utils/constants.h"

namespace robot::utilities {

    template<>
    struct enum_definitions_trait<robot::h1_2::constants::Joint> {
        static constexpr auto& definitions = robot::h1_2::constants::joint_definitions;
    };

} // namespace robot::utilities
