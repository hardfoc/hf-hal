/**
 * @file BldcCanOpenController.cpp
 * @brief Implementation of BldcCanOpenController.
 */

#include "BldcCanOpenController.h"
#include <UTILITIES/canopen/CanOpenUtils.h>

BldcCanOpenController::BldcCanOpenController(FlexCan& busArg, uint8_t nodeId) noexcept
    : bus(busArg), node(nodeId), initialized(false) {}

bool BldcCanOpenController::Initialize() noexcept {
    if (initialized) return true;
    if (!bus.Open()) return false;
    initialized = SendNmt(static_cast<uint8_t>(CanOpen::NmtCommand::StartNode));
    return initialized;
}

bool BldcCanOpenController::StartMotor() noexcept {
    if (!initialized) return false;
    return SendNmt(static_cast<uint8_t>(CanOpen::NmtCommand::StartNode));
}

bool BldcCanOpenController::StopMotor() noexcept {
    if (!initialized) return false;
    return SendNmt(static_cast<uint8_t>(CanOpen::NmtCommand::StopNode));
}

bool BldcCanOpenController::SetTargetVelocity(int32_t velocity) noexcept {
    if (!initialized) return false;
    return SendSdo(0x60FF, 0x00, static_cast<uint32_t>(velocity), 4);
}

bool BldcCanOpenController::Process() noexcept {
    if (!initialized) return false;
    FlexCan::Frame frame;
    if (!bus.Read(frame, 0)) {
        return true; // nothing to process
    }
    // TODO: Handle responses and status updates
    (void)frame;
    return true;
}

bool BldcCanOpenController::SendNmt(uint8_t command) noexcept {
    FlexCan::Frame frame = CanOpen::BuildNmt(node, static_cast<CanOpen::NmtCommand>(command));
    return bus.Write(frame);
}

bool BldcCanOpenController::SendSdo(uint16_t index, uint8_t subIndex, uint32_t data, uint8_t size) noexcept {
    FlexCan::Frame frame = CanOpen::BuildSdoDownload(node, index, subIndex, data, size);
    return bus.Write(frame);
}
