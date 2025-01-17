// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/* Copyright (C) 2019 Facebook */

#include <errno.h>
#include <fcntl.h>
#include <linux/err.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <bpf.h>
#include <libbpf.h>
#include <linux/btf.h>

#include "btf.h"
#include "json_writer.h"
#include "main.h"

static const char * const btf_kind_str[NR_BTF_KINDS] = {
	[BTF_KIND_UNKN]		= "UNKNOWN",
	[BTF_KIND_INT]		= "INT",
	[BTF_KIND_PTR]		= "PTR",
	[BTF_KIND_ARRAY]	= "ARRAY",
	[BTF_KIND_STRUCT]	= "STRUCT",
	[BTF_KIND_UNION]	= "UNION",
	[BTF_KIND_ENUM]		= "ENUM",
	[BTF_KIND_FWD]		= "FWD",
	[BTF_KIND_TYPEDEF]	= "TYPEDEF",
	[BTF_KIND_VOLATILE]	= "VOLATILE",
	[BTF_KIND_CONST]	= "CONST",
	[BTF_KIND_RESTRICT]	= "RESTRICT",
	[BTF_KIND_FUNC]		= "FUNC",
	[BTF_KIND_FUNC_PROTO]	= "FUNC_PROTO",
	[BTF_KIND_VAR]		= "VAR",
	[BTF_KIND_DATASEC]	= "DATASEC",
};

static const char *btf_int_enc_str(__u8 encoding)
{
	switch (encoding) {
	case 0:
		return "(none)";
	case BTF_INT_SIGNED:
		return "SIGNED";
	case BTF_INT_CHAR:
		return "CHAR";
	case BTF_INT_BOOL:
		return "BOOL";
	default:
		return "UNKN";
	}
}

static const char *btf_var_linkage_str(__u32 linkage)
{
	switch (linkage) {
	case BTF_VAR_STATIC:
		return "static";
	case BTF_VAR_GLOBAL_ALLOCATED:
		return "global-alloc";
	default:
		return "(unknown)";
	}
}

static const char *btf_str(const struct btf *btf, __u32 off)
{
	if (!off)
		return "(anon)";
	return btf__name_by_offset(btf, off) ? : "(invalid)";
}

static int dump_btf_type(const struct btf *btf, __u32 id,
			 const struct btf_type *t)
{
	json_writer_t *w = json_wtr;
	int kind, safe_kind;

	kind = BTF_INFO_KIND(t->info);
	safe_kind = kind <= BTF_KIND_MAX ? kind : BTF_KIND_UNKN;

	if (json_output) {
		jsonw_start_object(w);
		jsonw_uint_field(w, "id", id);
		jsonw_string_field(w, "kind", btf_kind_str[safe_kind]);
		jsonw_string_field(w, "name", btf_str(btf, t->name_off));
	} else {
		printf("[%u] %s '%s'", id, btf_kind_str[safe_kind],
		       btf_str(btf, t->name_off));
	}

	switch (BTF_INFO_KIND(t->info)) {
	case BTF_KIND_INT: {
		__u32 v = *(__u32 *)(t + 1);
		const char *enc;

		enc = btf_int_enc_str(BTF_INT_ENCODING(v));

		if (json_output) {
			jsonw_uint_field(w, "size", t->size);
			jsonw_uint_field(w, "bits_offset", BTF_INT_OFFSET(v));
			jsonw_uint_field(w, "nr_bits", BTF_INT_BITS(v));
			jsonw_string_field(w, "encoding", enc);
		} else {
			printf(" size=%u bits_offset=%u nr_bits=%u encoding=%s",
			       t->size, BTF_INT_OFFSET(v), BTF_INT_BITS(v),
			       enc);
		}
		break;
	}
	case BTF_KIND_PTR:
	case BTF_KIND_CONST:
	case BTF_KIND_VOLATILE:
	case BTF_KIND_RESTRICT:
	case BTF_KIND_TYPEDEF:
		if (json_output)
			jsonw_uint_field(w, "type_id", t->type);
		else
			printf(" type_id=%u", t->type);
		break;
	case BTF_KIND_ARRAY: {
		const struct btf_array *arr = (const void *)(t + 1);

		if (json_output) {
			jsonw_uint_field(w, "type_id", arr->type);
			jsonw_uint_field(w, "index_type_id", arr->index_type);
			jsonw_uint_field(w, "nr_elems", arr->nelems);
		} else {
			printf(" type_id=%u index_type_id=%u nr_elems=%u",
			       arr->type, arr->index_type, arr->nelems);
		}
		break;
	}
	case BTF_KIND_STRUCT:
	case BTF_KIND_UNION: {
		const struct btf_member *m = (const void *)(t + 1);
		__u16 vlen = BTF_INFO_VLEN(t->info);
		int i;

		if (json_output) {
			jsonw_uint_field(w, "size", t->size);
			jsonw_uint_field(w, "vlen", vlen);
			jsonw_name(w, "members");
			jsonw_start_array(w);
		} else {
			printf(" size=%u vlen=%u", t->size, vlen);
		}
		for (i = 0; i < vlen; i++, m++) {
			const char *name = btf_str(btf, m->name_off);
			__u32 bit_off, bit_sz;

			if (BTF_INFO_KFLAG(t->info)) {
				bit_off = BTF_MEMBER_BIT_OFFSET(m->offset);
				bit_sz = BTF_MEMBER_BITFIELD_SIZE(m->offset);
			} else {
				bit_off = m->offset;
				bit_sz = 0;
			}

			if (json_output) {
				jsonw_start_object(w);
				jsonw_string_field(w, "name", name);
				jsonw_uint_field(w, "type_id", m->type);
				jsonw_uint_field(w, "bits_offset", bit_off);
				if (bit_sz) {
					jsonw_uint_field(w, "bitfield_size",
							 bit_sz);
				}
				jsonw_end_object(w);
			} else {
				printf("\n\t'%s' type_id=%u bits_offset=%u",
				       name, m->type, bit_off);
				if (bit_sz)
					printf(" bitfield_size=%u", bit_sz);
			}
		}
		if (json_output)
			jsonw_end_array(w);
		break;
	}
	case BTF_KIND_ENUM: {
		const struct btf_enum *v = (const void *)(t + 1);
		__u16 vlen = BTF_INFO_VLEN(t->info);
		int i;

		if (json_output) {
			jsonw_uint_field(w, "size", t->size);
			jsonw_uint_field(w, "vlen", vlen);
			jsonw_name(w, "values");
			jsonw_start_array(w);
		} else {
			printf(" size=%u vlen=%u", t->size, vlen);
		}
		for (i = 0; i < vlen; i++, v++) {
			const char *name = btf_str(btf, v->name_off);

			if (json_output) {
				jsonw_start_object(w);
				jsonw_string_field(w, "name", name);
				jsonw_uint_field(w, "val", v->val);
				jsonw_end_object(w);
			} else {
				printf("\n\t'%s' val=%u", name, v->val);
			}
		}
		if (json_output)
			jsonw_end_array(w);
		break;
	}
	case BTF_KIND_FWD: {
		const char *fwd_kind = BTF_INFO_KFLAG(t->info) ? "union"
							       : "struct";

		if (json_output)
			jsonw_string_field(w, "fwd_kind", fwd_kind);
		else
			printf(" fwd_kind=%s", fwd_kind);
		break;
	}
	case BTF_KIND_FUNC:
		if (json_output)
			jsonw_uint_field(w, "type_id", t->type);
		else
			printf(" type_id=%u", t->type);
		break;
	case BTF_KIND_FUNC_PROTO: {
		const struct btf_param *p = (const void *)(t + 1);
		__u16 vlen = BTF_INFO_VLEN(t->info);
		int i;

		if (json_output) {
			jsonw_uint_field(w, "ret_type_id", t->type);
			jsonw_uint_field(w, "vlen", vlen);
			jsonw_name(w, "params");
			jsonw_start_array(w);
		} else {
			printf(" ret_type_id=%u vlen=%u", t->type, vlen);
		}
		for (i = 0; i < vlen; i++, p++) {
			const char *name = btf_str(btf, p->name_off);

			if (json_output) {
				jsonw_start_object(w);
				jsonw_string_field(w, "name", name);
				jsonw_uint_field(w, "type_id", p->type);
				jsonw_end_object(w);
			} else {
				printf("\n\t'%s' type_id=%u", name, p->type);
			}
		}
		if (json_output)
			jsonw_end_array(w);
		break;
	}
	case BTF_KIND_VAR: {
		const struct btf_var *v = (const void *)(t + 1);
		const char *linkage;

		linkage = btf_var_linkage_str(v->linkage);

		if (json_output) {
			jsonw_uint_field(w, "type_id", t->type);
			jsonw_string_field(w, "linkage", linkage);
		} else {
			printf(" type_id=%u, linkage=%s", t->type, linkage);
		}
		break;
	}
	case BTF_KIND_DATASEC: {
		const struct btf_var_secinfo *v = (const void *)(t+1);
		__u16 vlen = BTF_INFO_VLEN(t->info);
		int i;

		if (json_output) {
			jsonw_uint_field(w, "size", t->size);
			jsonw_uint_field(w, "vlen", vlen);
			jsonw_name(w, "vars");
			jsonw_start_array(w);
		} else {
			printf(" size=%u vlen=%u", t->size, vlen);
		}
		for (i = 0; i < vlen; i++, v++) {
			if (json_output) {
				jsonw_start_object(w);
				jsonw_uint_field(w, "type_id", v->type);
				jsonw_uint_field(w, "offset", v->offset);
				jsonw_uint_field(w, "size", v->size);
				jsonw_end_object(w);
			} else {
				printf("\n\ttype_id=%u offset=%u size=%u",
				       v->type, v->offset, v->size);
			}
		}
		if (json_output)
			jsonw_end_array(w);
		break;
	}
	default:
		break;
	}

	if (json_output)
		jsonw_end_object(json_wtr);
	else
		printf("\n");

	return 0;
}

static int dump_btf_raw(const struct btf *btf,
			__u32 *root_type_ids, int root_type_cnt)
{
	const struct btf_type *t;
	int i;

	if (json_output) {
		jsonw_start_object(json_wtr);
		jsonw_name(json_wtr, "types");
		jsonw_start_array(json_wtr);
	}

	if (root_type_cnt) {
		for (i = 0; i < root_type_cnt; i++) {
			t = btf__type_by_id(btf, root_type_ids[i]);
			dump_btf_type(btf, root_type_ids[i], t);
		}
	} else {
		int cnt = btf__get_nr_types(btf);

		for (i = 1; i <= cnt; i++) {
			t = btf__type_by_id(btf, i);
			dump_btf_type(btf, i, t);
		}
	}

	if (json_output) {
		jsonw_end_array(json_wtr);
		jsonw_end_object(json_wtr);
	}
	return 0;
}

static void __printf(2, 0) btf_dump_printf(void *ctx,
					   const char *fmt, va_list args)
{
	vfprintf(stdout, fmt, args);
}

static int dump_btf_c(const struct btf *btf,
		      __u32 *root_type_ids, int root_type_cnt)
{
	struct btf_dump *d;
	int err = 0, i;

	d = btf_dump__new(btf, NULL, NULL, btf_dump_printf);
	if (IS_ERR(d))
		return PTR_ERR(d);

	if (root_type_cnt) {
		for (i = 0; i < root_type_cnt; i++) {
			err = btf_dump__dump_type(d, root_type_ids[i]);
			if (err)
				goto done;
		}
	} else {
		int cnt = btf__get_nr_types(btf);

		for (i = 1; i <= cnt; i++) {
			err = btf_dump__dump_type(d, i);
			if (err)
				goto done;
		}
	}

done:
	btf_dump__free(d);
	return err;
}

static int do_dump(int argc, char **argv)
{
	struct btf *btf = NULL;
	__u32 root_type_ids[2];
	int root_type_cnt = 0;
	bool dump_c = false;
	__u32 btf_id = -1;
	const char *src;
	int fd = -1;
	int err;

	if (!REQ_ARGS(2)) {
		usage();
		return -1;
	}
	src = GET_ARG();

	if (is_prefix(src, "map")) {
		struct bpf_map_info info = {};
		__u32 len = sizeof(info);

		if (!REQ_ARGS(2)) {
			usage();
			return -1;
		}

		fd = map_parse_fd_and_info(&argc, &argv, &info, &len);
		if (fd < 0)
			return -1;

		btf_id = info.btf_id;
		if (argc && is_prefix(*argv, "key")) {
			root_type_ids[root_type_cnt++] = info.btf_key_type_id;
			NEXT_ARG();
		} else if (argc && is_prefix(*argv, "value")) {
			root_type_ids[root_type_cnt++] = info.btf_value_type_id;
			NEXT_ARG();
		} else if (argc && is_prefix(*argv, "all")) {
			NEXT_ARG();
		} else if (argc && is_prefix(*argv, "kv")) {
			root_type_ids[root_type_cnt++] = info.btf_key_type_id;
			root_type_ids[root_type_cnt++] = info.btf_value_type_id;
			NEXT_ARG();
		} else {
			root_type_ids[root_type_cnt++] = info.btf_key_type_id;
			root_type_ids[root_type_cnt++] = info.btf_value_type_id;
		}
	} else if (is_prefix(src, "prog")) {
		struct bpf_prog_info info = {};
		__u32 len = sizeof(info);

		if (!REQ_ARGS(2)) {
			usage();
			return -1;
		}

		fd = prog_parse_fd(&argc, &argv);
		if (fd < 0)
			return -1;

		err = bpf_obj_get_info_by_fd(fd, &info, &len);
		if (err) {
			p_err("can't get prog info: %s", strerror(errno));
			goto done;
		}

		btf_id = info.btf_id;
	} else if (is_prefix(src, "id")) {
		char *endptr;

		btf_id = strtoul(*argv, &endptr, 0);
		if (*endptr) {
			p_err("can't parse %s as ID", *argv);
			return -1;
		}
		NEXT_ARG();
	} else if (is_prefix(src, "file")) {
		btf = btf__parse_elf(*argv, NULL);
		if (IS_ERR(btf)) {
			err = PTR_ERR(btf);
			btf = NULL;
			p_err("failed to load BTF from %s: %s", 
			      *argv, strerror(err));
			goto done;
		}
		NEXT_ARG();
	} else {
		err = -1;
		p_err("unrecognized BTF source specifier: '%s'", src);
		goto done;
	}

	while (argc) {
		if (is_prefix(*argv, "format")) {
			NEXT_ARG();
			if (argc < 1) {
				p_err("expecting value for 'format' option\n");
				goto done;
			}
			if (strcmp(*argv, "c") == 0) {
				dump_c = true;
			} else if (strcmp(*argv, "raw") == 0) {
				dump_c = false;
			} else {
				p_err("unrecognized format specifier: '%s', possible values: raw, c",
				      *argv);
				goto done;
			}
			NEXT_ARG();
		} else {
			p_err("unrecognized option: '%s'", *argv);
			goto done;
		}
	}

	if (!btf) {
		err = btf__get_from_id(btf_id, &btf);
		if (err) {
			p_err("get btf by id (%u): %s", btf_id, strerror(err));
			goto done;
		}
		if (!btf) {
			err = ENOENT;
			p_err("can't find btf with ID (%u)", btf_id);
			goto done;
		}
	}

	if (dump_c) {
		if (json_output) {
			p_err("JSON output for C-syntax dump is not supported");
			err = -ENOTSUP;
			goto done;
		}
		err = dump_btf_c(btf, root_type_ids, root_type_cnt);
	} else {
		err = dump_btf_raw(btf, root_type_ids, root_type_cnt);
	}

done:
	close(fd);
	btf__free(btf);
	return err;
}

static int do_help(int argc, char **argv)
{
	if (json_output) {
		jsonw_null(json_wtr);
		return 0;
	}

	fprintf(stderr,
		"Usage: %s btf dump BTF_SRC [format FORMAT]\n"
		"       %s btf help\n"
		"\n"
		"       BTF_SRC := { id BTF_ID | prog PROG | map MAP [{key | value | kv | all}] | file FILE }\n"
		"       FORMAT  := { raw | c }\n"
		"       " HELP_SPEC_MAP "\n"
		"       " HELP_SPEC_PROGRAM "\n"
		"       " HELP_SPEC_OPTIONS "\n"
		"",
		bin_name, bin_name);

	return 0;
}

static const struct cmd cmds[] = {
	{ "help",	do_help },
	{ "dump",	do_dump },
	{ 0 }
};

int do_btf(int argc, char **argv)
{
	return cmd_select(cmds, argc, argv, do_help);
}
