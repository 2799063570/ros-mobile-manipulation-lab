#pragma once

#include <cstddef>
#include <cstdint>

namespace aubo_ros_control
{
// Only SDK-accepted batches contribute to the stationary FIFO suffix.
class MotionHoldFence
{
public:
    static constexpr int max_mac_scalars = 400;

    void acceptedBatch(std::uint64_t fence, std::size_t hold_suffix, std::size_t batch_size)
    {
        if (fence != fence_ || hold_suffix != batch_size)
            accepted_holds_ = 0;
        fence_ = fence;
        accepted_holds_ += hold_suffix;
    }

    std::uint64_t confirmedAfterDiagnosis(int mac_scalars, std::size_t axes) const
    {
        return mac_scalars > 0 && mac_scalars <= max_mac_scalars && axes &&
               accepted_holds_ > static_cast<std::size_t>(max_mac_scalars) / axes ? fence_ : 0;
    }

private:
    std::uint64_t fence_{0};
    std::size_t accepted_holds_{0};
};
} // namespace aubo_ros_control
