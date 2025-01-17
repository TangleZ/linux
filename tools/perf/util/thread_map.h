/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PERF_THREAD_MAP_H
#define __PERF_THREAD_MAP_H

#include <sys/types.h>
#include <stdio.h>
#include <linux/refcount.h>
#include <internal/threadmap.h>
#include <perf/threadmap.h>

struct thread_map_event;

struct perf_thread_map *thread_map__new_dummy(void);
struct perf_thread_map *thread_map__new_by_pid(pid_t pid);
struct perf_thread_map *thread_map__new_by_tid(pid_t tid);
struct perf_thread_map *thread_map__new_by_uid(uid_t uid);
struct perf_thread_map *thread_map__new_all_cpus(void);
struct perf_thread_map *thread_map__new(pid_t pid, pid_t tid, uid_t uid);
struct perf_thread_map *thread_map__new_event(struct thread_map_event *event);

struct perf_thread_map *thread_map__new_str(const char *pid,
		const char *tid, uid_t uid, bool all_threads);

struct perf_thread_map *thread_map__new_by_tid_str(const char *tid_str);

size_t thread_map__fprintf(struct perf_thread_map *threads, FILE *fp);

static inline int thread_map__nr(struct perf_thread_map *threads)
{
	return threads ? threads->nr : 1;
}

static inline pid_t thread_map__pid(struct perf_thread_map *map, int thread)
{
	return map->map[thread].pid;
}

void thread_map__read_comms(struct perf_thread_map *threads);
bool thread_map__has(struct perf_thread_map *threads, pid_t pid);
int thread_map__remove(struct perf_thread_map *threads, int idx);
#endif	/* __PERF_THREAD_MAP_H */
