#pragma once

#include <optional>
#include <string_view>
#include <ranges>
#include <type_traits>  //std::is_scoped_enum_v
#include <utility>  // std::pair


namespace robot::utilities {

    template <typename EnumType>
    struct enum_definitions_trait;

    template <typename EnumType>
        requires std::is_scoped_enum_v<EnumType>
    constexpr std::optional<EnumType> to_joint_enum(std::string_view name) {
        const auto& definitions = enum_definitions_trait<EnumType>::definitions;

        // Remove Method:
        // using ContainerType = std::remove_cvref_t<decltype(definitions)>;
        // auto it = std::ranges::find(definitions, name, &ContainerType::value_type::second);

        // Lambda Method:
        auto it = std::ranges::find_if(definitions, [&](const auto& def) {
            return def.second == name;
        });

        if (it != std::ranges::end(definitions)) {
            return it->first;
        }

        return std::nullopt;
    }

} // namespace robot::utilities