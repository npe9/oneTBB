/*
    Lithe co-scheduling for the Thread Composability Manager (lithe-docker, 2026-10-24).

    TCM is a library, not a runtime with threads of its own: its entry points run on the
    callers' threads -- under Lithe, on contexts of oneTBB, an OpenMP runtime, or anyone
    else that asks for a permit. Two things in it assumed kernel threads:
      - its waits (the topology spin lock's std::this_thread::yield, the data and
        reference-count std::mutexes) parked or rescheduled the caller's hart in the
        kernel; under Lithe a contended wait yields the calling context instead;
      - the concurrency it hands out permits for was the process affinity mask, while
        under Lithe the CPUs a process gets are its harts (VCORE_LIMIT): permits are now
        capped at parlib's hart limit, so TCM and the Lithe scheduler agree.
    Every Lithe symbol is a weak reference: without liblithe this is stock TCM.
    TCM_LITHE=0 also forces stock behaviour.
*/
#ifndef __TCM_LITHE_COOP_H
#define __TCM_LITHE_COOP_H

#include <cstdint>
#include <cstdlib>
#include <unordered_map>
#include <mutex>
#include <thread>

extern "C" {
__attribute__((weak)) int lithe_can_block(void);
__attribute__((weak)) void lithe_context_yield(void);
__attribute__((weak)) int parlib_vcore_limit(void);
__attribute__((weak)) int lithe_sched_trace_enabled(void);
__attribute__((weak)) void lithe_sched_trace_emit(uint8_t, void*, uint32_t, uint8_t, int32_t);
}

namespace tcm {
namespace internal {

inline bool lithe_on() {
  static const int on = [] {
    const char* e = std::getenv("TCM_LITHE");
    return !(e && e[0] == '0') && lithe_context_yield != nullptr;
  }();
  return on;
}

//! The caller is a Lithe context that may yield (not a constructor, destructor or
//! foreign thread: CLAUDE.md rule 6).
inline bool lithe_can_yield() {
  return lithe_on() && lithe_can_block && lithe_can_block();
}

//! A spin-wait step: yield the Lithe context, else the OS thread.
inline void coop_yield() {
  if (lithe_can_yield())
    lithe_context_yield();
  else
    std::this_thread::yield();
}

//! Lithe's hart budget for this process, 0 when Lithe is absent or not started.
inline int lithe_hart_limit() {
  return lithe_on() && parlib_vcore_limit ? parlib_vcore_limit() : 0;
}

//! std::mutex whose contended lock yields a Lithe context instead of sleeping its hart
//! in the kernel. Critical sections in TCM are short bookkeeping; callbacks run unlocked.
class coop_mutex {
  std::mutex m;
public:
  void lock() {
    if (!lithe_can_yield()) { m.lock(); return; }
    while (!m.try_lock())
      lithe_context_yield();
  }
  bool try_lock() { return m.try_lock(); }
  void unlock() { m.unlock(); }
};

//! Lithe Perfetto trace (LITHE_SCHED_TRACE=1): a permit's granted concurrency becomes a
//! counter track "tcm permit N (granted)" (LITHE_ST_TCM_PERMIT = 38; 0 = released/idle).
//! Callers hold TCM's data mutex, which guards the key table. Emitted only from a Lithe
//! context, since the trace ring is per hart.
inline void trace_permit(const void* ph, uint32_t granted, bool erase = false) {
  if (!lithe_sched_trace_emit || !lithe_sched_trace_enabled || !lithe_sched_trace_enabled()
      || !lithe_can_yield())
    return;
  static std::unordered_map<const void*, uint32_t> keys;
  static uint32_t next = 1;
  auto it = keys.find(ph);
  uint32_t key = it != keys.end() ? it->second : (keys[ph] = next++);
  lithe_sched_trace_emit(38, nullptr, key, 0, int32_t(granted));
  if (erase)
    keys.erase(ph);
}

} // namespace internal
} // namespace tcm

#endif /* __TCM_LITHE_COOP_H */
