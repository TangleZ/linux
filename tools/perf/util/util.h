/* SPDX-License-Identifier: GPL-2.0 */
#ifndef GIT_COMPAT_UTIL_H
#define GIT_COMPAT_UTIL_H

#define _BSD_SOURCE 1
/* glibc 2.20 deprecates _BSD_SOURCE in favour of _DEFAULT_SOURCE */
#define _DEFAULT_SOURCE 1

#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <linux/compiler.h>
#include <sys/types.h>
#include <internal/lib.h>

/* General helper functions */
void usage(const char *err) __noreturn;
void die(const char *err, ...) __noreturn __printf(1, 2);

struct dirent;
struct nsinfo;
struct strlist;

int mkdir_p(char *path, mode_t mode);
int rm_rf(const char *path);
int rm_rf_perf_data(const char *path);
struct strlist *lsdir(const char *name, bool (*filter)(const char *, struct dirent *));
bool lsdir_no_dot_filter(const char *name, struct dirent *d);
int copyfile(const char *from, const char *to);
int copyfile_mode(const char *from, const char *to, mode_t mode);
int copyfile_ns(const char *from, const char *to, struct nsinfo *nsi);
int copyfile_offset(int ifd, loff_t off_in, int ofd, loff_t off_out, u64 size);

size_t hex_width(u64 v);

extern unsigned int page_size;
int __pure cacheline_size(void);

int sysctl__max_stack(void);

int fetch_kernel_version(unsigned int *puint,
			 char *str, size_t str_sz);
#define KVER_VERSION(x)		(((x) >> 16) & 0xff)
#define KVER_PATCHLEVEL(x)	(((x) >> 8) & 0xff)
#define KVER_SUBLEVEL(x)	((x) & 0xff)
#define KVER_FMT	"%d.%d.%d"
#define KVER_PARAM(x)	KVER_VERSION(x), KVER_PATCHLEVEL(x), KVER_SUBLEVEL(x)

const char *perf_tip(const char *dirpath);

#ifndef HAVE_SCHED_GETCPU_SUPPORT
int sched_getcpu(void);
#endif

extern bool perf_singlethreaded;

void perf_set_singlethreaded(void);
void perf_set_multithreaded(void);

char *perf_exe(char *buf, int len);

#ifndef O_CLOEXEC
#ifdef __sparc__
#define O_CLOEXEC      0x400000
#elif defined(__alpha__) || defined(__hppa__)
#define O_CLOEXEC      010000000
#else
#define O_CLOEXEC      02000000
#endif
#endif

#endif /* GIT_COMPAT_UTIL_H */
