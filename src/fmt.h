// printf into a std::string of any length (a state with a dozen ragdolls passes the end of any fixed buffer).
#pragma once
#include <cstdarg>
#include <cstdio>
#include <string>

inline std::string Fmt(const char *fmt, ...) {
    va_list ap, again;
    va_start(ap, fmt);
    va_copy(again, ap);
    int n = vsnprintf(nullptr, 0, fmt, ap);
    va_end(ap);
    std::string s(n > 0 ? n : 0, '\0');
    if (n > 0) vsnprintf(s.data(), s.size() + 1, fmt, again);
    va_end(again);
    return s;
}
