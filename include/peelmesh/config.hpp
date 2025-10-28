#ifndef PEELMESH_CONFIG_H
#define PEELMESH_CONFIG_H

#include <string>
#include <iostream>

#if defined(USE_STD_FORMAT)
#include <format>
namespace pm_format
{
    using std::format;
}
#elif defined(USE_FMT_FORMAT)
#include <fmt/format.h>
namespace pm_format
{
    template <typename... Args>
    std::string format(const std::string &format_str, Args &&...args)
    {
        return fmt::format(format_str, std::forward<Args>(args)...);
    }
}
#else
#error "Format library not defined by CMake!"
#endif

#endif /* PEELMESH_CONFIG_H */
