#pragma once

#include "occ_errors.hpp"

namespace open_power
{
namespace occ
{

/** @class Presence
 *  @brief Monitors the number of OCCs present
 */
class Presence : public Error
{
  public:
    Presence() = delete;
    Presence(const Presence&) = delete;
    Presence& operator=(const Presence&) = delete;
    Presence(Presence&&) = default;
    Presence& operator=(Presence&&) = delete;

    /** @brief Constructs the Presence object
     *
     *  @param[in] event    - Reference to sd_event unique_ptr
     *  @param[in] file     - File used by driver to communicate errors
     *  @param[in] mgr      - OCC manager instance
     *  @param[in] callBack - Optional function callback on error condition
     */
    Presence(EventPtr& event, const fs::path& file, size_t numOccs,
             std::function<void(int)> callBack = nullptr) :
        Error(event, file, callBack), occCount(numOccs)
    {
        // Nothing to do here.
    }

  private:
    /** Number of OCCs on this chassis, for presence mismatch detection */
    const size_t occCount;

    /** @brief When the error event is received, analyzes it
     *         and makes a callback to error handler if the
     *         content denotes an error condition
     */
    void analyzeEvent() override;
};

} // namespace occ
} // namespace open_power
