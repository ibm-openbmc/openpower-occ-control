#pragma once

#include "occ_errors.hpp"
#include "occ_object.hpp"
#include "occ_pass_through.hpp"
#include "utils.hpp"

#include <nlohmann/json.hpp>

#include <functional>
#include <memory>
#include <set>
#include <vector>

namespace open_power
{
namespace occ
{

/** @class ChassisObject
 *  @brief Owns all per-chassis state and the collection of OCC objects
 *         for one chassis (1-based chassisID).
 *
 *  Initially one OCC per chassis is supported (occObjects[0] is always the
 *  master). The data structures are designed to accommodate multiple OCCs per
 *  chassis in a future sub-task.
 */
class ChassisObject
{
  public:
    ChassisObject() = delete;
    ChassisObject(const ChassisObject&) = delete;
    ChassisObject& operator=(const ChassisObject&) = delete;
    ChassisObject(ChassisObject&&) = delete;
    ChassisObject& operator=(ChassisObject&&) = delete;
    ~ChassisObject() = default;

    /** @brief Construct a ChassisObject for the given 1-based chassis number.
     *
     *  @param[in] chassisNum   - 1-based chassis identifier (1–MAX_CHASSIS)
     *  @param[in] mgr_callback - callable(chassisID, bool) invoked when the
     *                            chassis-level active state changes; wired to
     *                            Manager::chassisStatusCallback by the Manager.
     */
    explicit ChassisObject(
        chassisID chassisNum,
        std::function<void(chassisID, bool)> mgr_callback = nullptr) :
        chassis(chassisNum), managerCallback(std::move(mgr_callback))
    {}

    // -----------------------------------------------------------------------
    // OCC object management
    // -----------------------------------------------------------------------

    /** @brief Add an OCC object to this chassis.  Takes ownership. */
    void addOcc(std::unique_ptr<OccObject> occ)
    {
        occObjects.emplace_back(std::move(occ));
    }

    /** @brief Add a PassThrough object to this chassis.  Takes ownership. */
    void addPassThrough(std::unique_ptr<PassThrough> pt)
    {
        passThroughObjects.emplace_back(std::move(pt));
    }

    /** @brief Return a reference to the master (first/only) OCC.
     *  The caller must ensure occObjects is not empty. */
    OccObject& getMasterOcc()
    {
        return *occObjects.front();
    }

    /** @brief Return the number of OCC objects on this chassis. */
    size_t occCount() const
    {
        return occObjects.size();
    }

    // -----------------------------------------------------------------------
    // Per-chassis state
    // -----------------------------------------------------------------------

    /** @brief 1-based chassis identifier */
    chassisID chassis;

    /** @brief True when all OCCs on this chassis are active.
     *  Set/cleared by onOccActiveChanged(); read by Manager for system-wide
     *  "all chassis active" checks. */
    bool active = false;

    /** @brief True when this chassis is in OCC safe mode */
    bool safeMode = false;

    // -----------------------------------------------------------------------
    // OCC active-state notification (called by OccObject)
    // -----------------------------------------------------------------------

    /** @brief Called by an OccObject when its occActive property changes.
     *
     *  Owns all chassis-local consequences of the state change:
     *   - updating the chassis-level @a active flag
     *   - clearing sensors on the deactivating OCC
     *  Then forwards the chassis-level active/inactive event up to the Manager
     *  via @a managerCallback.
     *
     *  @param[in] status - new occActive state of the reporting OCC
     */
    void onOccActiveChanged(bool status);

    // -----------------------------------------------------------------------
    // Active sensor resolution (called by Manager::checkAllActiveSensors)
    // -----------------------------------------------------------------------

    /** @brief Check whether this chassis's master OCC has its PLDM active
     *         sensor resolved, processing any queued active state and
     *         optionally invoking a PLDM sensor check.
     *
     *  Processes queued active state first.  If still unresolved and
     *  @a checkSensor is provided, calls it with the instance ID so Manager
     *  can invoke pldmHandle->checkActiveSensor() without needing to know
     *  the instance itself.
     *
     *  @param[in,out] queuedActiveState - instances queued before objects
     * existed
     *  @param[in]     checkSensor       - optional callable(instanceID) invoked
     *                                    when the sensor is still unresolved
     *  @return true if sensor is resolved, false if still waiting
     */
    bool isSensorResolved(
        std::set<uint8_t>& queuedActiveState,
        std::function<void(instanceID)> checkSensor = nullptr);

    // -----------------------------------------------------------------------
    // Named operations delegated from Manager
    // -----------------------------------------------------------------------

    /** @brief Call occActive(false) on every OCC that is currently active.
     *  Used before initiating a PM Complex reset. */
    void deactivateAll();

    /** @brief Set sensor values to NaN on every OCC that is not active. */
    void setSensorsToNaN();

    /** @brief Set sensor values to NaN for the OCC with the given instance. */
    void setSensorToNaN(instanceID instance);

    /** @brief Set the occActive state for the OCC with the given instance. */
    void setOccActive(instanceID instance, bool occActive);

    /** @brief Set the PLDM sensor received flag for the OCC with the given
     *         instance. */
    void setPldmSensorReceived(instanceID instance, bool received);

    /** @brief Return the PLDM sensor received flag for the OCC with the given
     *         instance. */
    bool getPldmSensorReceived(instanceID instance) const;

    /** @brief Poll all OCCs: NaN inactive ones, call PollHandler on active. */
    void poll();

    /** @brief Update the throttle state on all OCCs for this chassis. */
    void updateThrottle(bool isThrottled, uint8_t reason);

    /** @brief Send ambient temperature and altitude to all active OCCs. */
    void sendAmbient(uint8_t ambient, uint16_t altitude);

    /** @brief Update processor D-Bus association for any OCC whose path is
     *         not yet resolved.
     */
    void updateProcAssociations();

    // -----------------------------------------------------------------------
    // Dump data collection
    // -----------------------------------------------------------------------

    /** @brief Collect dump data for every OCC on this chassis.
     *
     *  Sends POLL and WOF commands via each OCC's sendCmd() and populates
     *  @a data with per-OCC JSON sections keyed by "occ<N>".
     *
     *  @param[in,out] data - top-level JSON object to populate
     */
    void collectDumpData(nlohmann::json& data);

    // -----------------------------------------------------------------------
    // Per-chassis master validation
    // -----------------------------------------------------------------------

    /** @brief Verify that exactly one OCC master exists on this chassis,
     *         start presence monitoring, and update safe-mode state.
     *
     *  Active-sensor checks (pldmHandle->checkActiveSensor) are intentionally
     *  kept in Manager — ChassisObject has no reason to know about PLDM.
     *  Manager resolves queued/sensor state first, then calls this method.
     *
     *  @param[in] pmode - PowerMode object; called to clear safe-mode once
     *                     the master is confirmed
     */
    void validateOccMaster(
        std::unique_ptr<open_power::occ::powermode::PowerMode>& pmode);

  private:
    /** @brief Callback into Manager when chassis-level active state changes. */
    std::function<void(chassisID, bool)> managerCallback;

    /** @brief OCC objects on this chassis (index 0 = master today) */
    std::vector<std::unique_ptr<OccObject>> occObjects;

    /** @brief PassThrough objects on this chassis (one per OCC) */
    std::vector<std::unique_ptr<PassThrough>> passThroughObjects;

    /** @brief Returns true when every OCC on this chassis is active. */
    bool allOccsActive() const;
};

} // namespace occ
} // namespace open_power
