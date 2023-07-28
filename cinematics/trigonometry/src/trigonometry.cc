#include <chrono>
#include <thread>

#include <trigonometry/trigonometry.h>

namespace smov {

void TrigonometryState::on_start() {}

void TrigonometryState::on_loop() {} 

void TrigonometryState::on_quit() {}

void TrigonometryState::set_legs_distance_to(float distance) {}

}

DECLARE_STATE_NODE_CLASS_INCLUDE_PARAMS("smov_trigonometry_state", smov::TrigonometryState, 100ms)