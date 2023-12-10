#include "cm/cm.h"

namespace cm {
SignalHandle* SignalHandle::pSignal = nullptr;
std::once_flag SignalHandle::init_flag;

/* Singleton instance  */
SignalHandle* SignalHandle::instance() {
  std::call_once(init_flag, [] { pSignal = new SignalHandle; });

  return pSignal;
}

SignalHandle::SignalHandle() { m_state = true; }

SignalHandle::~SignalHandle() {}

void SignalHandle::Destroy() {
  if (pSignal) {
    delete pSignal;  // PRQA S 5217 # SingleInstance mode, only once
    pSignal = nullptr;
  }
}

void SignalHandle::SetState(bool state) {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_state = state;
}

bool SignalHandle::GetState() { return m_state; }

}  // namespace cm
