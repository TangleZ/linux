// SPDX-License-Identifier: GPL-2.0

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/prctl.h>
#include <sys/utsname.h>

#define SHIFT_TAG(tag)		((uint64_t)(tag) << 56)
#define SET_TAG(ptr, tag)	(((uint64_t)(ptr) & ~SHIFT_TAG(0xff)) | \
					SHIFT_TAG(tag))

int main(void)
{
	static int tbi_enabled = 0;
	struct utsname *ptr, *tagged_ptr;
	int err;

	if (prctl(PR_SET_TAGGED_ADDR_CTRL, PR_TAGGED_ADDR_ENABLE, 0, 0, 0) == 0)
		tbi_enabled = 1;
	ptr = (struct utsname *)malloc(sizeof(*ptr));
	if (tbi_enabled)
		tagged_ptr = (struct utsname *)SET_TAG(ptr, 0x42);
	err = uname(tagged_ptr);
	free(ptr);

	return err;
}
