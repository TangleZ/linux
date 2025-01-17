/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  Copyright IBM Corp. 2019
 *  Author(s): Harald Freudenberger <freude@linux.ibm.com>
 *	       Ingo Franzki <ifranzki@linux.ibm.com>
 *
 *  Collection of CCA misc functions used by zcrypt and pkey
 */

#ifndef _ZCRYPT_CCAMISC_H_
#define _ZCRYPT_CCAMISC_H_

#include <asm/zcrypt.h>
#include <asm/pkey.h>

/* Key token types */
#define TOKTYPE_NON_CCA		0x00 /* Non-CCA key token */
#define TOKTYPE_CCA_INTERNAL	0x01 /* CCA internal key token */

/* For TOKTYPE_NON_CCA: */
#define TOKVER_PROTECTED_KEY	0x01 /* Protected key token */

/* For TOKTYPE_CCA_INTERNAL: */
#define TOKVER_CCA_AES		0x04 /* CCA AES key token */

/* header part of a CCA key token */
struct keytoken_header {
	u8  type;     /* one of the TOKTYPE values */
	u8  res0[3];
	u8  version;  /* one of the TOKVER values */
	u8  res1[3];
} __packed;

/* inside view of a CCA secure key token (only type 0x01 version 0x04) */
struct secaeskeytoken {
	u8  type;     /* 0x01 for internal key token */
	u8  res0[3];
	u8  version;  /* should be 0x04 */
	u8  res1[1];
	u8  flag;     /* key flags */
	u8  res2[1];
	u64 mkvp;     /* master key verification pattern */
	u8  key[32];  /* key value (encrypted) */
	u8  cv[8];    /* control vector */
	u16 bitsize;  /* key bit size */
	u16 keysize;  /* key byte size */
	u8  tvv[4];   /* token validation value */
} __packed;

/*
 * Simple check if the token is a valid CCA secure AES data key
 * token. If keybitsize is given, the bitsize of the key is
 * also checked. Returns 0 on success or errno value on failure.
 */
int cca_check_secaeskeytoken(debug_info_t *dbg, int dbflvl,
			     const u8 *token, int keybitsize);

/*
 * Generate (random) CCA AES DATA secure key.
 */
int cca_genseckey(u16 cardnr, u16 domain, u32 keytype, u8 *seckey);

/*
 * Generate CCA AES DATA secure key with given clear key value.
 */
int cca_clr2seckey(u16 cardnr, u16 domain, u32 keytype,
		   const u8 *clrkey, u8 *seckey);

/*
 * Derive proteced key from an CCA AES DATA secure key.
 */
int cca_sec2protkey(u16 cardnr, u16 domain,
		    const u8 seckey[SECKEYBLOBSIZE],
		    u8 *protkey, u32 *protkeylen,
		    u32 *protkeytype);

/*
 * Query cryptographic facility from CCA adapter
 */
int cca_query_crypto_facility(u16 cardnr, u16 domain,
			      const char *keyword,
			      u8 *rarray, size_t *rarraylen,
			      u8 *varray, size_t *varraylen);

/*
 * Search for a matching crypto card based on the Master Key
 * Verification Pattern provided inside a secure key.
 * Returns < 0 on failure, 0 if CURRENT MKVP matches and
 * 1 if OLD MKVP matches.
 */
int cca_findcard(const u8 *seckey, u16 *pcardnr, u16 *pdomain, int verify);

/* struct to hold info for each CCA queue */
struct cca_info {
	char new_mk_state;  /* '1' empty, '2' partially full, '3' full */
	char cur_mk_state;  /* '1' invalid, '2' valid */
	char old_mk_state;  /* '1' invalid, '2' valid */
	u64  new_mkvp;	    /* truncated sha256 hash of new master key */
	u64  cur_mkvp;	    /* truncated sha256 hash of current master key */
	u64  old_mkvp;	    /* truncated sha256 hash of old master key */
	char serial[9];     /* serial number string (8 ascii numbers + 0x00) */
};

/*
 * Fetch cca information about an CCA queue.
 */
int cca_get_info(u16 card, u16 dom, struct cca_info *ci, int verify);

void zcrypt_ccamisc_exit(void);

#endif /* _ZCRYPT_CCAMISC_H_ */
