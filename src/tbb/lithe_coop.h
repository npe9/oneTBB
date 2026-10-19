/*
    Lithe co-scheduling for oneTBB (lithe-docker, 2026-10-19).

    TBB's worker threads become Lithe fork-join contexts in the root scheduler
    (rml_thread_monitor.h), and its futex waits park the calling context instead of
    its hart (semaphore.h), so TBB shares harts cooperatively with whatever else
    runs under Lithe in the process -- OpenMP teams, MPI rank contexts, GPU runtime
    progress contexts -- as in the original Lithe paper's TBB + OpenMP composition.

    Every Lithe symbol is a weak reference: a process without liblithe gets stock
    TBB. TBB_LITHE=0 also forces stock behaviour. The choice is made once, at the
    first worker launch; from then on futex wakes reach both parked contexts and
    kernel-futex waiters.
*/

#ifndef __TBB_lithe_coop_H
#define __TBB_lithe_coop_H

#include <atomic>
#include <cstddef>
#include <cstdlib>

extern "C" {
typedef struct lithe_sched lithe_sched_t;
typedef struct lithe_fork_join_sched lithe_fork_join_sched_t;
typedef struct lithe_fork_join_context lithe_fork_join_context_t;
__attribute__((weak)) lithe_sched_t* lithe_sched_current(void);
__attribute__((weak)) int lithe_can_block(void);
__attribute__((weak)) lithe_fork_join_sched_t* lithe_ensure_root_fork_join_sched(void);
__attribute__((weak)) lithe_fork_join_context_t* lithe_fork_join_context_create(
    lithe_fork_join_sched_t*, std::size_t, void (*)(void*), void*);
/* liblithe's futex emulation (src/lithe/src/futex.c); its names collide with
   TBB's own static futex helpers, hence the asm labels. */
__attribute__((weak)) void lithe_fork_join_context_set_daemon(lithe_fork_join_context_t*);
/* only its address: a fork-join scheduler's first member (funcs) points at it */
__attribute__((weak)) extern const char lithe_fork_join_sched_funcs[];
__attribute__((weak)) int tbb_lithe_futex_wait_(int*, int) __asm__("futex_wait");
__attribute__((weak)) int tbb_lithe_futex_wake_some_(int*, int) __asm__("futex_wake_some");
}

namespace tbb {
namespace detail {
namespace r1 {

//! 1 once TBB has started using Lithe in this process.
extern std::atomic<int> g_lithe_active;
//! Threads (not Lithe contexts) currently in a kernel futex wait.
extern std::atomic<int> g_lithe_kwaiters;

//! Decide once whether to use Lithe; bootstraps the root scheduler if needed.
int lithe_init();

//! Fork-join scheduler for new workers: the caller's when it is one, else the root.
inline lithe_fork_join_sched_t* lithe_worker_sched() {
    lithe_sched_t* cur = lithe_sched_current();
    if (cur && lithe_fork_join_sched_funcs &&
        *reinterpret_cast<const void* const*>(cur) == static_cast<const void*>(lithe_fork_join_sched_funcs))
        return reinterpret_cast<lithe_fork_join_sched_t*>(cur);
    return lithe_ensure_root_fork_join_sched();
}

//! The caller is a Lithe context that may park.
inline bool lithe_can_park() {
    return g_lithe_active.load(std::memory_order_relaxed) && lithe_can_block && lithe_can_block();
}

} // namespace r1
} // namespace detail
} // namespace tbb

#endif /* __TBB_lithe_coop_H */
