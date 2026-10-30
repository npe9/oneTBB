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
__attribute__((weak)) long lithe_fork_join_current_hartless_runnable(void);
/* Lithe's Perfetto trace (sched_trace.h; no-ops unless LITHE_SCHED_TRACE=1) */
__attribute__((weak)) int lithe_sched_trace_enabled(void);
__attribute__((weak)) void lithe_sched_trace_label_self(const char*);
__attribute__((weak)) int lithe_sched_trace_intern(const char*);
__attribute__((weak)) void lithe_sched_trace_phase_begin(int);
__attribute__((weak)) void lithe_sched_trace_phase_end(void);
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

//! Stop spinning in the steal loop: another context of this scheduler is runnable but
//! has no hart, so every further pause/yield of this idle worker delays it (libgomp's
//! gomp_lithe_should_stop_spinning). The waiter then goes straight to out_of_work and
//! sleeps, parking the context and freeing its hart. TBB_LITHE_STOP_SPIN=1; default off.
//! NEGATIVE RESULT (2026-10-24, .build_logs/alt/stopspin, n=8 on 4 harts): on the TBB+OpenMP
//! mix (1000 rounds) makespan 0.147 -> 0.153 s, TBB tenant 0.135 -> 0.148 s. A worker that
//! sleeps must be re-woken by the next round's spawn, which costs more than the yields it
//! skips: d0::yield is already a Lithe context yield that hands the hart to the runnable
//! peer. The mix is within 6 % of ideal (alone-run sum 0.139 s) without it.
inline bool lithe_should_stop_spinning() {
    static const bool on = [] { const char* e = std::getenv("TBB_LITHE_STOP_SPIN"); return e && e[0] == '1'; }();
    return on && lithe_fork_join_current_hartless_runnable && lithe_can_park() &&
           lithe_fork_join_current_hartless_runnable() > 0;
}

//! De-spin gates (lithe-docker .build_logs/spin_census/DESPIN_RULES.md, 2026-10-24). Each is
//! read once, default off, and applies only to a caller that may park (a Lithe context);
//! anything else keeps the stock spin. Results: .build_logs/despin/tasks/tbb.
inline bool lithe_env_on(const char* name) {
    const char* e = std::getenv(name);
    return e && e[0] == '1';
}
//! TBB_LITHE_NOSPIN_IDLE=1: the steal loop's stealing_loop_backoff (2*(slots+1) prolonged
//! pauses then 100 yields -- 1000 for an external waiter -- before out_of_work) gets budget 0:
//! the first empty steal pass goes to out_of_work, then the worker leaves for RML's sleep
//! (futex park, woken by advertise_new_work -> wake_or_launch) and an external waiter sleeps on
//! the arena monitor (woken by the wait_context release).
//! DEFAULT OFF -- the spin budget buys TBB-alone throughput (A/B 2026-10-24, cores 8-11, 4 harts,
//! n=10-20 interleaved, .build_logs/despin/tasks/tbb/{ab,focus,r3}.csv; medians, with LEAVE on):
//! co-tenant gains -- TBB+OpenMP mix makespan 0.149 -> 0.143 s, phased mix (TBB master serial
//! gap) OpenMP tenant 0.156 -> 0.133 -- but TBB alone loses: 1000x70 us rounds 0.0727 -> 0.0738,
//! 10000x8 us rounds 0.0833 -> 0.0943 (+13 %): every round now re-wakes the workers through RML's
//! futex and Lithe's unblock. TBB_LITHE_IDLE_YIELD_US=20 (yield, no PAUSE, for 20 us) is parity
//! on the 70 us rounds (0.0726) with the same co-tenant gain, but still +9 % on 8 us rounds.
inline bool lithe_nospin_idle() {
    static const bool on = lithe_env_on("TBB_LITHE_NOSPIN_IDLE");
    return on && lithe_can_park();
}
//! TBB_LITHE_IDLE_YIELD_US=N: with NOSPIN_IDLE, yield (no PAUSE) for up to N us before out_of_work.
inline long lithe_idle_yield_ns() {
    static const long ns = [] { const char* e = std::getenv("TBB_LITHE_IDLE_YIELD_US"); return e ? std::atol(e) * 1000L : 0L; }();
    return ns;
}
//! TBB_LITHE_NOSPIN_LEAVE (DEFAULT ON, =0 restores the stock 1 ms yield loop): a recalled worker
//! leaves at once instead of yielding for up to 1 ms (outermost_worker_waiter's delayed leave,
//! the default leave policy on non-hybrid CPUs), then sleeps in RML (futex park). A/B 2026-10-24
//! (cores 8-11, 4 harts, interleaved, n=20 over two batches, 0 failures; medians): phased
//! TBB+OpenMP mix, OpenMP tenant 0.156 -> 0.138 s; 10000 x 8 us rounds with OpenMP, makespan
//! 0.177 -> 0.166; 1000 x 70 us rounds with OpenMP 0.149 -> 0.151 (noise); TBB alone 0.0727 ->
//! 0.0720, 0.0833 -> 0.0827, phased 0.3338 -> 0.3324 (parity).
inline bool lithe_nospin_leave() {
    static const bool on = [] { const char* e = std::getenv("TBB_LITHE_NOSPIN_LEAVE"); return !(e && e[0] == '0'); }();
    return on && lithe_can_park();
}
//! TBB_LITHE_NOSPIN_MONMTX=1: concurrent_monitor_mutex::lock skips timed_spin_wait_until
//! (5 pause rounds + 32 yields) and parks on its futex at once. Default off: a lock around a short
//! critical section whose holder runs (DESPIN rule 4); A/B neutral (with LEAVE: mix 0.1507 = 0.1507,
//! phased OpenMP 0.138 vs 0.137, 8 us rounds with OpenMP 0.178 vs 0.166, n=10).
inline bool lithe_nospin_monmtx() {
    static const bool on = lithe_env_on("TBB_LITHE_NOSPIN_MONMTX");
    return on && lithe_can_park();
}

//! Trace phase id for a worker's idle stealing loop (spin, yields, until a task or sleep);
//! 0 when tracing is off.
inline int lithe_trace_steal_id() {
    static int id = (lithe_sched_trace_enabled && lithe_sched_trace_enabled() && lithe_sched_trace_intern)
                    ? lithe_sched_trace_intern("tbb: idle steal loop") : 0;
    return id;
}

} // namespace r1
} // namespace detail
} // namespace tbb

#endif /* __TBB_lithe_coop_H */
