#ifndef _TRACE_RT_BE_MUTEX_H
#define _TRACE_RT_BE_MUTEX_H

#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM rt_be_mutex

TRACE_EVENT(rt_be_mutex_acquire,
	TP_PROTO(pid_t pid, pid_t owner, int is_rt, bool account_overrun),
	TP_ARGS(pid, owner, is_rt, account_overrun),
	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, owner)
		__field(int, is_rt)
		__field(bool, account_overrun)
	),
	TP_fast_assign(
		__entry->pid = pid;
		__entry->owner = owner;
		__entry->is_rt = is_rt;
		__entry->account_overrun = account_overrun;
	),
	TP_printk("pid=%d owner=%d is_rt=%d account_overrun=%d",
		  __entry->pid, __entry->owner, __entry->is_rt, __entry->account_overrun)
);

TRACE_EVENT(rt_be_mutex_release,
	TP_PROTO(pid_t pid, pid_t next_owner),
	TP_ARGS(pid, next_owner),
	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(pid_t, next_owner)
	),
	TP_fast_assign(
		__entry->pid = pid;
		__entry->next_owner = next_owner;
	),
	TP_printk("pid=%d next_owner=%d",
		  __entry->pid, __entry->next_owner)
);

TRACE_EVENT(rt_be_mutex_enter_deferred,
	TP_PROTO(pid_t pid, u64 fire_time_ns),
	TP_ARGS(pid, fire_time_ns),
	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(u64, fire_time_ns)
	),
	TP_fast_assign(
		__entry->pid = pid;
		__entry->fire_time_ns = fire_time_ns;
	),
	TP_printk("pid=%d fire_time_ns=%llu",
		  __entry->pid, __entry->fire_time_ns)
);

TRACE_EVENT(rt_be_mutex_deferred_fire,
	TP_PROTO(pid_t pid),
	TP_ARGS(pid),
	TP_STRUCT__entry(
		__field(pid_t, pid)
	),
	TP_fast_assign(
		__entry->pid = pid;
	),
	TP_printk("pid=%d",
		  __entry->pid)
);

TRACE_EVENT(rt_be_mutex_boost,
	TP_PROTO(pid_t pid, int non_preemptible, int account_overrun),
	TP_ARGS(pid, non_preemptible, account_overrun),
	TP_STRUCT__entry(
		__field(pid_t, pid)
		__field(int, non_preemptible)
		__field(int, account_overrun)
	),
	TP_fast_assign(
		__entry->pid = pid;
		__entry->non_preemptible = non_preemptible;
		__entry->account_overrun = account_overrun;
	),
	TP_printk("pid=%d non_preemptible=%d account_overrun=%d",
		  __entry->pid, __entry->non_preemptible, __entry->account_overrun)
);

TRACE_EVENT(rt_be_mutex_unboost,
	TP_PROTO(pid_t pid),
	TP_ARGS(pid),
	TP_STRUCT__entry(
		__field(pid_t, pid)
	),
	TP_fast_assign(
		__entry->pid = pid;
	),
	TP_printk("pid=%d",
		  __entry->pid)
);

#endif /* _TRACE_RT_BE_MUTEX_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace_rt_be_mutex
#include <trace/define_trace.h>