/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */
/*
 * linux/drivers/staging/erofs/erofs_fs.h
 *
 * Copyright (C) 2017-2018 HUAWEI, Inc.
 *             http://www.huawei.com/
 * Created by Gao Xiang <gaoxiang25@huawei.com>
 */
#ifndef __EROFS_FS_H
#define __EROFS_FS_H

/* Enhanced(Extended) ROM File System */
#define EROFS_SUPER_MAGIC_V1    0xE0F5E1E2
#define EROFS_SUPER_OFFSET      1024

/*
 * Any bits that aren't in EROFS_ALL_REQUIREMENTS should be
 * incompatible with this kernel version.
 */
#define EROFS_REQUIREMENT_LZ4_0PADDING	0x00000001
#define EROFS_ALL_REQUIREMENTS		EROFS_REQUIREMENT_LZ4_0PADDING

struct erofs_super_block {
/*  0 */__le32 magic;           /* in the little endian */
/*  4 */__le32 checksum;        /* crc32c(super_block) */
/*  8 */__le32 features;        /* (aka. feature_compat) */
/* 12 */__u8 blkszbits;         /* support block_size == PAGE_SIZE only */
/* 13 */__u8 reserved;

/* 14 */__le16 root_nid;
/* 16 */__le64 inos;            /* total valid ino # (== f_files - f_favail) */

/* 24 */__le64 build_time;      /* inode v1 time derivation */
/* 32 */__le32 build_time_nsec;
/* 36 */__le32 blocks;          /* used for statfs */
/* 40 */__le32 meta_blkaddr;
/* 44 */__le32 xattr_blkaddr;
/* 48 */__u8 uuid[16];          /* 128-bit uuid for volume */
/* 64 */__u8 volume_name[16];   /* volume name */
/* 80 */__le32 requirements;    /* (aka. feature_incompat) */

/* 84 */__u8 reserved2[44];
} __packed;                     /* 128 bytes */

/*
 * erofs inode data mapping:
 * 0 - inode plain without inline data A:
 * inode, [xattrs], ... | ... | no-holed data
 * 1 - inode VLE compression B (legacy):
 * inode, [xattrs], extents ... | ...
 * 2 - inode plain with inline data C:
 * inode, [xattrs], last_inline_data, ... | ... | no-holed data
 * 3 - inode compression D:
 * inode, [xattrs], map_header, extents ... | ...
 * 4~7 - reserved
 */
enum {
	EROFS_INODE_FLAT_PLAIN,
	EROFS_INODE_FLAT_COMPRESSION_LEGACY,
	EROFS_INODE_FLAT_INLINE,
	EROFS_INODE_FLAT_COMPRESSION,
	EROFS_INODE_LAYOUT_MAX
};

static inline bool erofs_inode_is_data_compressed(unsigned int datamode)
{
	if (datamode == EROFS_INODE_FLAT_COMPRESSION)
		return true;
	return datamode == EROFS_INODE_FLAT_COMPRESSION_LEGACY;
}

/* bit definitions of inode i_advise */
#define EROFS_I_VERSION_BITS            1
#define EROFS_I_DATA_MAPPING_BITS       3

#define EROFS_I_VERSION_BIT             0
#define EROFS_I_DATA_MAPPING_BIT        1

struct erofs_inode_v1 {
/*  0 */__le16 i_advise;

/* 1 header + n-1 * 4 bytes inline xattr to keep continuity */
/*  2 */__le16 i_xattr_icount;
/*  4 */__le16 i_mode;
/*  6 */__le16 i_nlink;
/*  8 */__le32 i_size;
/* 12 */__le32 i_reserved;
/* 16 */union {
		/* file total compressed blocks for data mapping 1 */
		__le32 compressed_blocks;
		__le32 raw_blkaddr;

		/* for device files, used to indicate old/new device # */
		__le32 rdev;
	} i_u __packed;
/* 20 */__le32 i_ino;           /* only used for 32-bit stat compatibility */
/* 24 */__le16 i_uid;
/* 26 */__le16 i_gid;
/* 28 */__le32 i_reserved2;
} __packed;

/* 32 bytes on-disk inode */
#define EROFS_INODE_LAYOUT_V1   0
/* 64 bytes on-disk inode */
#define EROFS_INODE_LAYOUT_V2   1

struct erofs_inode_v2 {
/*  0 */__le16 i_advise;

/* 1 header + n-1 * 4 bytes inline xattr to keep continuity */
/*  2 */__le16 i_xattr_icount;
/*  4 */__le16 i_mode;
/*  6 */__le16 i_reserved;
/*  8 */__le64 i_size;
/* 16 */union {
		/* file total compressed blocks for data mapping 1 */
		__le32 compressed_blocks;
		__le32 raw_blkaddr;

		/* for device files, used to indicate old/new device # */
		__le32 rdev;
	} i_u __packed;

	/* only used for 32-bit stat compatibility */
/* 20 */__le32 i_ino;

/* 24 */__le32 i_uid;
/* 28 */__le32 i_gid;
/* 32 */__le64 i_ctime;
/* 40 */__le32 i_ctime_nsec;
/* 44 */__le32 i_nlink;
/* 48 */__u8   i_reserved2[16];
} __packed;                     /* 64 bytes */

#define EROFS_MAX_SHARED_XATTRS         (128)
/* h_shared_count between 129 ... 255 are special # */
#define EROFS_SHARED_XATTR_EXTENT       (255)

/*
 * inline xattrs (n == i_xattr_icount):
 * erofs_xattr_ibody_header(1) + (n - 1) * 4 bytes
 *          12 bytes           /                   \
 *                            /                     \
 *                           /-----------------------\
 *                           |  erofs_xattr_entries+ |
 *                           +-----------------------+
 * inline xattrs must starts in erofs_xattr_ibody_header,
 * for read-only fs, no need to introduce h_refcount
 */
struct erofs_xattr_ibody_header {
	__le32 h_reserved;
	__u8   h_shared_count;
	__u8   h_reserved2[7];
	__le32 h_shared_xattrs[0];      /* shared xattr id array */
} __packed;

/* Name indexes */
#define EROFS_XATTR_INDEX_USER              1
#define EROFS_XATTR_INDEX_POSIX_ACL_ACCESS  2
#define EROFS_XATTR_INDEX_POSIX_ACL_DEFAULT 3
#define EROFS_XATTR_INDEX_TRUSTED           4
#define EROFS_XATTR_INDEX_LUSTRE            5
#define EROFS_XATTR_INDEX_SECURITY          6

/* xattr entry (for both inline & shared xattrs) */
struct erofs_xattr_entry {
	__u8   e_name_len;      /* length of name */
	__u8   e_name_index;    /* attribute name index */
	__le16 e_value_size;    /* size of attribute value */
	/* followed by e_name and e_value */
	char   e_name[0];       /* attribute name */
} __packed;

#define ondisk_xattr_ibody_size(count)	({\
	u32 __count = le16_to_cpu(count); \
	((__count) == 0) ? 0 : \
	sizeof(struct erofs_xattr_ibody_header) + \
		sizeof(__u32) * ((__count) - 1); })

#define EROFS_XATTR_ALIGN(size) round_up(size, sizeof(struct erofs_xattr_entry))
#define EROFS_XATTR_ENTRY_SIZE(entry) EROFS_XATTR_ALIGN( \
	sizeof(struct erofs_xattr_entry) + \
	(entry)->e_name_len + le16_to_cpu((entry)->e_value_size))

/* available compression algorithm types */
enum {
	Z_EROFS_COMPRESSION_LZ4,
	Z_EROFS_COMPRESSION_MAX
};

/*
 * bit 0 : COMPACTED_2B indexes (0 - off; 1 - on)
 *  e.g. for 4k logical cluster size,      4B        if compacted 2B is off;
 *                                  (4B) + 2B + (4B) if compacted 2B is on.
 */
#define Z_EROFS_ADVISE_COMPACTED_2B_BIT         0

#define Z_EROFS_ADVISE_COMPACTED_2B     (1 << Z_EROFS_ADVISE_COMPACTED_2B_BIT)

struct z_erofs_map_header {
	__le32	h_reserved1;
	__le16	h_advise;
	/*
	 * bit 0-3 : algorithm type of head 1 (logical cluster type 01);
	 * bit 4-7 : algorithm type of head 2 (logical cluster type 11).
	 */
	__u8	h_algorithmtype;
	/*
	 * bit 0-2 : logical cluster bits - 12, e.g. 0 for 4096;
	 * bit 3-4 : (physical - logical) cluster bits of head 1:
	 *       For example, if logical clustersize = 4096, 1 for 8192.
	 * bit 5-7 : (physical - logical) cluster bits of head 2.
	 */
	__u8	h_clusterbits;
};

#define Z_EROFS_VLE_LEGACY_HEADER_PADDING       8

/*
 * Z_EROFS Variable-sized Logical Extent cluster type:
 *    0 - literal (uncompressed) cluster
 *    1 - compressed cluster (for the head logical cluster)
 *    2 - compressed cluster (for the other logical clusters)
 *
 * In detail,
 *    0 - literal (uncompressed) cluster,
 *        di_advise = 0
 *        di_clusterofs = the literal data offset of the cluster
 *        di_blkaddr = the blkaddr of the literal cluster
 *
 *    1 - compressed cluster (for the head logical cluster)
 *        di_advise = 1
 *        di_clusterofs = the decompressed data offset of the cluster
 *        di_blkaddr = the blkaddr of the compressed cluster
 *
 *    2 - compressed cluster (for the other logical clusters)
 *        di_advise = 2
 *        di_clusterofs =
 *           the decompressed data offset in its own head cluster
 *        di_u.delta[0] = distance to its corresponding head cluster
 *        di_u.delta[1] = distance to its corresponding tail cluster
 *                (di_advise could be 0, 1 or 2)
 */
enum {
	Z_EROFS_VLE_CLUSTER_TYPE_PLAIN,
	Z_EROFS_VLE_CLUSTER_TYPE_HEAD,
	Z_EROFS_VLE_CLUSTER_TYPE_NONHEAD,
	Z_EROFS_VLE_CLUSTER_TYPE_RESERVED,
	Z_EROFS_VLE_CLUSTER_TYPE_MAX
};

#define Z_EROFS_VLE_DI_CLUSTER_TYPE_BITS        2
#define Z_EROFS_VLE_DI_CLUSTER_TYPE_BIT         0

struct z_erofs_vle_decompressed_index {
	__le16 di_advise;
	/* where to decompress in the head cluster */
	__le16 di_clusterofs;

	union {
		/* for the head cluster */
		__le32 blkaddr;
		/*
		 * for the rest clusters
		 * eg. for 4k page-sized cluster, maximum 4K*64k = 256M)
		 * [0] - pointing to the head cluster
		 * [1] - pointing to the tail cluster
		 */
		__le16 delta[2];
	} di_u __packed;		/* 8 bytes */
} __packed;

#define Z_EROFS_VLE_LEGACY_INDEX_ALIGN(size) \
	(round_up(size, sizeof(struct z_erofs_vle_decompressed_index)) + \
	 sizeof(struct z_erofs_map_header) + Z_EROFS_VLE_LEGACY_HEADER_PADDING)

/* dirent sorts in alphabet order, thus we can do binary search */
struct erofs_dirent {
	__le64 nid;     /*  0, node number */
	__le16 nameoff; /*  8, start offset of file name */
	__u8 file_type; /* 10, file type */
	__u8 reserved;  /* 11, reserved */
} __packed;

/*
 * EROFS file types should match generic FT_* types and
 * it seems no need to add BUILD_BUG_ONs since potential
 * unmatchness will break other fses as well...
 */

#define EROFS_NAME_LEN      255

/* check the EROFS on-disk layout strictly at compile time */
static inline void erofs_check_ondisk_layout_definitions(void)
{
	BUILD_BUG_ON(sizeof(struct erofs_super_block) != 128);
	BUILD_BUG_ON(sizeof(struct erofs_inode_v1) != 32);
	BUILD_BUG_ON(sizeof(struct erofs_inode_v2) != 64);
	BUILD_BUG_ON(sizeof(struct erofs_xattr_ibody_header) != 12);
	BUILD_BUG_ON(sizeof(struct erofs_xattr_entry) != 4);
	BUILD_BUG_ON(sizeof(struct z_erofs_map_header) != 8);
	BUILD_BUG_ON(sizeof(struct z_erofs_vle_decompressed_index) != 8);
	BUILD_BUG_ON(sizeof(struct erofs_dirent) != 12);

	BUILD_BUG_ON(BIT(Z_EROFS_VLE_DI_CLUSTER_TYPE_BITS) <
		     Z_EROFS_VLE_CLUSTER_TYPE_MAX - 1);
}

#endif

