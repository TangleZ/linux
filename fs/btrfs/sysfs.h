/* SPDX-License-Identifier: GPL-2.0 */

#ifndef BTRFS_SYSFS_H
#define BTRFS_SYSFS_H

#include <linux/kobject.h>

enum btrfs_feature_set {
	FEAT_COMPAT,
	FEAT_COMPAT_RO,
	FEAT_INCOMPAT,
	FEAT_MAX
};

char *btrfs_printable_features(enum btrfs_feature_set set, u64 flags);
const char * const btrfs_feature_set_name(enum btrfs_feature_set set);
int btrfs_sysfs_add_device_link(struct btrfs_fs_devices *fs_devices,
		struct btrfs_device *one_device);
int btrfs_sysfs_rm_device_link(struct btrfs_fs_devices *fs_devices,
                struct btrfs_device *one_device);
int btrfs_sysfs_add_fsid(struct btrfs_fs_devices *fs_devs,
				struct kobject *parent);
int btrfs_sysfs_add_device(struct btrfs_fs_devices *fs_devs);
void btrfs_sysfs_remove_fsid(struct btrfs_fs_devices *fs_devs);
void btrfs_sysfs_update_sprout_fsid(struct btrfs_fs_devices *fs_devices,
				    const u8 *fsid);
void btrfs_sysfs_feature_update(struct btrfs_fs_info *fs_info,
		u64 bit, enum btrfs_feature_set set);
void btrfs_kobject_uevent(struct block_device *bdev, enum kobject_action action);

int __init btrfs_init_sysfs(void);
void __cold btrfs_exit_sysfs(void);
int btrfs_sysfs_add_mounted(struct btrfs_fs_info *fs_info);
void btrfs_sysfs_remove_mounted(struct btrfs_fs_info *fs_info);
void btrfs_add_raid_kobjects(struct btrfs_fs_info *fs_info);
void btrfs_sysfs_add_block_group_type(struct btrfs_block_group_cache *cache);
int btrfs_sysfs_add_space_info_type(struct btrfs_fs_info *fs_info,
				    struct btrfs_space_info *space_info);
void btrfs_sysfs_remove_space_info(struct btrfs_space_info *space_info);

#endif
