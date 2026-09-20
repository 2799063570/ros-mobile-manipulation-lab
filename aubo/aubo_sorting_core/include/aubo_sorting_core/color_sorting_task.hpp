#ifndef AUBO_SORTING_CORE_COLOR_SORTING_TASK_COMPAT_HPP
#define AUBO_SORTING_CORE_COLOR_SORTING_TASK_COMPAT_HPP

#include <aubo_sorting_core/sorting_task.hpp>

namespace aubo_sorting_core
{
// Backward-compatible C++ API. New code should include sorting_task.hpp and use SortingTask.
using ColorSortingTask = SortingTask;
}

#endif
