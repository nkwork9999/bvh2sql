#pragma once

#if defined(_MSC_VER)
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

// DuckDB v1.5.3 vendors fmt 6.1.2, which still selects the old
// stdext::checked_array_iterator path when _SECURE_SCL is defined. VS 2026
// no longer provides that type. Preload the STL headers that define the
// compatibility macro, then undefine it before fmt/format.h is parsed.
#undef _SECURE_SCL
#endif
