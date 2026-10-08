#include "config.h"

#include "occ_chassis.hpp"

#include "utils.hpp"

#include <phosphor-logging/lg2.hpp>

namespace open_power
{
namespace occ
{

bool ChassisObject::allOccsActive() const
{
    return !occObjects.empty() &&
           std::all_of(occObjects.begin(), occObjects.end(),
                       [](const auto& o) { return o->occActive(); });
}

void ChassisObject::onOccActiveChanged(bool status)
{
    if (status)
    {
        // Chassis is up only when every OCC on it is active.
        active = allOccsActive();
        if (active && managerCallback)
        {
            managerCallback(chassis, true);
        }
    }
    else
    {
        if (active)
        {
            // If one of the OCC is no longer active then the
            // chassis can't be considered to be fully active
            active = false;
            if (managerCallback)
            {
                managerCallback(chassis, false);
            }
        }
    }
}

bool ChassisObject::isSensorResolved(
    std::set<uint8_t>& queuedActiveState,
    std::function<void(instanceID)> checkSensor)
{
    OccObject& occ = *occObjects.front();
    if (occ.occActive() || occ.getPldmSensorReceived())
        return true;

    auto instance = occ.getOccInstanceID();
    auto match = queuedActiveState.find(instance);
    if (match != queuedActiveState.end())
    {
        queuedActiveState.erase(match);
        lg2::info(
            "ChassisObject::isSensorResolved: OCC{INST} is ACTIVE (queued)",
            "INST", instance);
        occ.occActive(true);
        return true;
    }

    if (checkSensor)
    {
        checkSensor(instance);
    }

    return false;
}

void ChassisObject::deactivateAll()
{
    for (auto& obj : occObjects)
    {
        if (obj->occActive())
            obj->occActive(false);
    }
    active = false;
}

void ChassisObject::setSensorsToNaN()
{
    for (auto& obj : occObjects)
    {
        if (!obj->occActive())
            obj->setSensorValueToNaN();
    }
}

void ChassisObject::setSensorToNaN(instanceID instance)
{
    for (auto& obj : occObjects)
    {
        if (obj->getOccInstanceID() == instance)
        {
            obj->setSensorValueToNaN();
            break;
        }
    }
}

void ChassisObject::setOccActive(instanceID instance, bool occActive)
{
    for (auto& obj : occObjects)
    {
        if (obj->getOccInstanceID() == instance)
        {
            obj->occActive(occActive);
            break;
        }
    }
}

void ChassisObject::setPldmSensorReceived(instanceID instance, bool received)
{
    for (auto& obj : occObjects)
    {
        if (obj->getOccInstanceID() == instance)
        {
            obj->setPldmSensorReceived(received);
            break;
        }
    }
}

bool ChassisObject::getPldmSensorReceived(instanceID instance) const
{
    for (const auto& obj : occObjects)
    {
        if (obj->getOccInstanceID() == instance)
            return obj->getPldmSensorReceived();
    }
    return false;
}

void ChassisObject::poll()
{
    for (auto& obj : occObjects)
    {
        if (!obj->occActive())
        {
            obj->setSensorValueToNaN();
            continue;
        }
        obj->PollHandler();
    }
}

void ChassisObject::updateThrottle(bool isThrottled, uint8_t reason)
{
    for (auto& obj : occObjects)
        obj->updateThrottle(isThrottled, reason);
}

void ChassisObject::sendAmbient(uint8_t ambient, uint16_t altitude)
{
    for (auto& obj : occObjects)
    {
        if (obj->occActive())
            obj->sendAmbient(ambient, altitude);
    }
}

void ChassisObject::updateProcAssociations()
{
    for (auto& obj : occObjects)
    {
        if (obj->getProcPath().empty())
            obj->updateProcAssociation();
    }
}

void ChassisObject::collectDumpData(nlohmann::json& data)
{
    for (auto& obj : occObjects)
    {
        nlohmann::json occData;
        auto instance = obj->getOccInstanceID();
        std::string occName = "occ" + std::to_string(instance);

        if (obj->occActive())
        {
            occData["occState"] = "ACTIVE";
            occData["occRole"] = obj->isMasterOcc() ? "MASTER" : "SECONDARY";
            occData["occHwmonPath"] = obj->getHwmonPath().generic_string();

            // POLL command
            std::vector<uint8_t> cmd = {0x00, 0x00, 0x01, 0x20};
            std::vector<uint8_t> rsp;
            obj->sendCmd(cmd, rsp);
            if (rsp.size() > 5)
            {
                rsp.erase(rsp.begin(), rsp.begin() + 5);
                occData["pollResponse"] = utils::hex_dump(rsp);
            }

            // WOF dynamic data
            cmd = {0x40, 0x00, 0x01, 0x01};
            rsp.clear();
            obj->sendCmd(cmd, rsp);
            if (rsp.size() > 5)
            {
                rsp.erase(rsp.begin(), rsp.begin() + 5);
                occData["wofDataDynamic"] = utils::hex_dump(rsp);
            }

            // WOF static data
            cmd = {0x40, 0x00, 0x01, 0x0A};
            rsp.clear();
            obj->sendCmd(cmd, rsp);
            if (rsp.size() > 5)
            {
                rsp.erase(rsp.begin(), rsp.begin() + 5);
                occData["wofDataStatic"] = utils::hex_dump(rsp);
            }
        }
        else
        {
            occData["occState"] = "NOT ACTIVE";
        }

        data[occName] = occData;
    }
}

void ChassisObject::validateOccMaster(
    std::unique_ptr<open_power::occ::powermode::PowerMode>& pmode)
{
    int masterInstance = -1;

    for (auto& obj : occObjects)
    {
        auto instance = obj->getOccInstanceID();

        if (!obj->occActive() && !utils::isHostRunning())
        {
            lg2::warning(
                "ChassisObject::validateOccMaster: HOST is not running (OCC{INST})",
                "INST", instance);
            return;
        }

        if (obj->isMasterOcc())
        {
#ifndef ENABLE_APP_POLL_SUPPORT
            obj->addPresenceWatchMaster();
#endif
            if (masterInstance == -1)
            {
                masterInstance = instance;
            }
            else
            {
                lg2::error(
                    "ChassisObject::validateOccMaster: Multiple OCC masters on "
                    "chassis{CH}! ({MAST1} and {MAST2})",
                    "CH", chassis, "MAST1", masterInstance, "MAST2", instance);
                obj->deviceError(Error::Descriptor(PRESENCE_ERROR_PATH));
            }
        }
    }

    if (masterInstance < 0)
    {
        lg2::error("ChassisObject::validateOccMaster: Master OCC not found on "
                   "chassis{CH}! (of {NUM} OCCs)",
                   "CH", chassis, "NUM", occObjects.size());
        occObjects.front()->deviceError(Error::Descriptor(PRESENCE_ERROR_PATH));
    }
    else
    {
        lg2::info(
            "ChassisObject::validateOccMaster: OCC{INST} is master on chassis{CH}",
            "INST", masterInstance, "CH", chassis);
        pmode->updateDbusSafeMode(false);
    }
}

} // namespace occ
} // namespace open_power
