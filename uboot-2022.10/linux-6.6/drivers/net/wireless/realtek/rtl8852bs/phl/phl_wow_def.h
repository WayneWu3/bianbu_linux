/******************************************************************************
 *
 * Copyright(c)2019 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#ifndef _PHL_WOW_DEF_H_
#define _PHL_WOW_DEF_H_

enum rtw_wow_op_mode {
	RTW_WOW_OP_NONE = 0,
	RTW_WOW_OP_PWR_DOWN,
	RTW_WOW_OP_DISCONNECT_STBY,
	RTW_WOW_OP_CONNECT_STBY,
	RTW_WOW_OP_MAX = 0xF
};

enum rtw_wow_cfg_step {
	/* suspend */
	WOW_INIT_PRECONFIG = BIT0,
	WOW_INIT = BIT1,
	WOW_FUNC_EN = BIT2,
	WOW_FUNC_START = BIT3,
	WOW_INIT_POSTCONFIG = BIT4,
	/* resume */
	WOW_PHL_START = BIT5,
	WOW_GET_PWR_STATE = BIT6,
	WOW_HANDLE_WAKE_RSN = BIT7,
	WOW_MODULE_START = BIT8,
	WOW_HANDLE_AOAC_RPT_PHASE0 = BIT9,
	WOW_HANDLE_AOAC_RPT_PHASE1 = BIT10,
	WOW_FUNC_DIS = BIT11,
	WOW_FUNC_STOP = BIT12,
	WOW_DEINIT = BIT13,
	/* both */
	WOW_PS_PWR_CFG = BIT14,
	WOW_SER_CTRL = BIT15,
	WOW_SET_WOWLAN = BIT16,
	WOW_CFG_WOW_SLEEP = BIT17,
	WOW_PS_PROTO_CFG = BIT18,
	WOW_PPDU_STS_CFG = BIT19,
	WOW_DBG_DUMP = BIT20
};

#define SET_WOW_INIT_ERR(_wow_info, _flags) \
	(_wow_info)->err.init |= (_flags)

#define SET_WOW_DEINIT_ERR(_wow_info, _flags) \
	(_wow_info)->err.deinit |= (_flags)

struct rtw_keep_alive_info {
	/* core */
	u8 keep_alive_en;
	u8 keep_alive_period;
	u8 keep_alive_pkt_type;
	/* phl/hal */
	u8 keep_alive_pkt_id;
};

struct rtw_disc_det_info {
	/* core */
	u8 disc_det_en;
	u8 disc_wake_en;
	u8 try_pkt_count;
	u8 check_period;
	u8 cnt_bcn_lost_en;
	u8 cnt_bcn_lost_limit;
};

#define MAX_NLO_NUM 10
#define MAX_SSID_LEN 32
#define MAX_NLO_CHANNEL 40

struct rtw_nlo_info {
	u8 nlo_en;
	u8 num_of_networks;
	u8 num_of_hidden_ap;
	u8 ssid[MAX_NLO_NUM][MAX_SSID_LEN];
	u8 ssidlen[MAX_NLO_NUM];
	u8 chipertype[MAX_NLO_NUM];
	u8 compare_cipher_type;
	u8 probe_req_id;
	struct scan_ofld_ch_info channel_list[MAX_NLO_CHANNEL];
	u8 channel_num;
	u32 delay; /* ms */
	u32 period; /* ms */
	u8 cycle;
	u32 slow_period; /* ms */
	void (* construct_pbreq)(void *priv, u8 *pkt_buf, u16 *len);
};

struct rtw_arp_ofld_content {
	u8 arp_en;
	u8 a3[MAC_ADDRESS_LENGTH];
	u8 remote_ipv4_addr[IPV4_ADDRESS_LENGTH];
	u8 host_ipv4_addr[IPV4_ADDRESS_LENGTH];
	u8 remote_mac_addr[MAC_ADDRESS_LENGTH];
};

struct rtw_arp_ofld_info {
	u8 arp_en;
	u8 arp_action; /* 0 = send arp response, 1 = wake up host */
	u8 arp_rsp_id;
	struct rtw_arp_ofld_content arp_ofld_content;
};

struct rtw_ndp_ofld_content {
	u8 ndp_en;
	u8 chk_remote_ip;
	u8 num_target_ip;
	u8 mac_addr[MAC_ADDRESS_LENGTH];
	u8 remote_ipv6_addr[IPV6_ADDRESS_LENGTH];
	u8 target_ipv6_addr[2][IPV6_ADDRESS_LENGTH];
};

struct rtw_ndp_ofld_info {
	u8 ndp_en;
	u8 ndp_id;
	struct rtw_ndp_ofld_content ndp_ofld_content[2];
};

struct rtw_gtk_ofld_content {
	u8 kck[32];
	u32 kck_len;
	u8 kek[32];
	u32 kek_len;
	u8 tk1[16];
	u8 txmickey[8];
	u8 rxmickey[8];
	u8 replay_cnt[8];

	u8 igtk_keyid[4];
	u8 ipn[8];
	u8 igtk[2][32];
	u8 igtk_len;
	u8 psk[32];
	u8 psk_len;
};

struct rtw_gtk_ofld_info {
	/* core */
	u8 gtk_en;
	u8 tkip_en;
	u8 ieee80211w_en;
	u8 pairwise_wakeup;
	u8 bip_sec_algo;
	u8 akmtype_byte3;
	struct rtw_gtk_ofld_content gtk_ofld_content;

	/* phl */
	u8 hw_11w_en; /* keep 1 for BIP-CMAC-128 so far */
	u8 gtk_rsp_id; /* eapol pkt id */
	u8 sa_query_id;
};

#define MAX_WOW_PATTERN_SIZE_BIT 128
#define MAX_WOW_PATTERN_SIZE_BYTE 16
#define MAX_WOW_PATTERN_SIZE_DWORD 4

struct rtw_wowcam_upd_info {
	u8 rw;
	u8 wow_cam_idx;
	u32 wake_mask[4];
	u16 match_crc;

	u8 is_negative_pattern_match;
	u8 skip_mac_hdr;
	u8 uc;
	u8 mc;
	u8 bc;

	u8 valid;
	u8 ptrn[MAX_WOW_PATTERN_SIZE_BIT];
	u32 ptrn_len;
	u8 mask[MAX_WOW_PATTERN_SIZE_BYTE];
};

#define MAX_WOW_CAM_NUM 18
struct rtw_pattern_match_info{
	struct rtw_wowcam_upd_info wowcam_info[MAX_WOW_CAM_NUM];
};

#define MAX_REALWOW_KCP_SIZE 124 /* (100 + 24) */
#define MAX_REALWOW_PAYLOAD 64

struct rtw_realwow_ofld_content {
	u16 interval; /* unit : 1 ms */
	u16 keep_alive_pkt_size;
	u16 ack_lost_limit;
	u16 ack_ptrn_size;
	u16 wakeup_ptrn_size;
	u16 keep_alive_pkt_ptrn[MAX_REALWOW_KCP_SIZE];
	u8 ack_ptrn[MAX_REALWOW_PAYLOAD];
	u8 wakeup_ptrn[MAX_REALWOW_PAYLOAD];
	u32 wakeup_sec_num;
};

struct rtw_realwow_info {
	u8 realwow_en;
	u8 auto_wakeup;
	u8 keepalive_id;
	u8 wakeup_pattern_id;
	u8 ack_pattern_id;
	struct rtw_realwow_ofld_content realwow_ofld_content;
};

struct rtw_wow_gpio_info {
	struct rtw_dev2hst_gpio_info d2h_gpio_info;
	enum rtw_gpio_mode dev2hst_gpio_mode;
	u8 dev2hst_gpio;
	u8 dev2hst_high;
};

struct rtw_periodic_wake_info {
	u8 periodic_wake_en;
	u32 wake_period;
	u32 wake_duration;
};

struct rtw_remote_wake_ctrl_info {
	/* core */
	u8 ptk_tx_iv[IV_LENGTH];
	u8 valid_check;
	u8 symbol_check_en;
	u8 gtk_key_idx;
	u8 ptk_rx_iv[IV_LENGTH];
	u8 gtk_rx_iv_idx0[IV_LENGTH];
	u8 gtk_rx_iv_idx1[IV_LENGTH];
	u8 gtk_rx_iv_idx2[IV_LENGTH];
	u8 gtk_rx_iv_idx3[IV_LENGTH];
};


#ifdef CONFIG_PHL_MDNS_OFFLOAD
#define MAX_MDNS_RESP_NUM 8
#define MAX_MDNS_RESP_LEN 512
#define MAX_MDNS_MATCH_CRITERIA_NUM 8
#define MAX_MDNS_PASSTHRU_NAME_NUM 8
#define MAX_MDNS_DOMAIN_NAME_LEN 255
#define PASSTHRU_FORWARD_ALL 0
#define PASSTHRU_DROP_ALL 1
#define PASSTHRU_LIST 2

struct rtw_mdns_ipv4_header {
	u8 src_ipv4_addr[IPV4_ADDRESS_LENGTH];
	u8 dst_ipv4_addr[IPV4_ADDRESS_LENGTH];
	u8 remote_mac_addr[MAC_ADDRESS_LENGTH];
	u8 protect_bit;
	u8 sec_hdr_len;
	u8 ipv4_pktid;
};

struct rtw_mdns_ipv6_header {
	u8 src_ipv6_addr[IPV6_ADDRESS_LENGTH];
	u8 dst_ipv6_addr[IPV6_ADDRESS_LENGTH];
	u8 remote_mac_addr[MAC_ADDRESS_LENGTH];
	u8 protect_bit;
	u8 sec_hdr_len;
	u8 ipv6_pktid;
};

struct rtw_mdns_match_criteria {
	u16 name_offset;
	u16 type;
	u8 name_len;
};

struct rtw_mdns_resp_entry {
	u16 content_len;
	u8 data_pktid;
	u8 match_ct_num;
	u8 content[MAX_MDNS_RESP_LEN];
	struct rtw_mdns_match_criteria match_ct[MAX_MDNS_MATCH_CRITERIA_NUM];
};

struct rtw_mdns_passthru_name {
	u8 name[MAX_MDNS_DOMAIN_NAME_LEN];
	u8 name_len;
	u8 pass_pktid;
};

struct rtw_mdns_passthru_list {
	u8 passthru_behavior;
	u8 passthru_name_num;
	struct rtw_mdns_passthru_name passthru_name[MAX_MDNS_PASSTHRU_NAME_NUM];
};

struct rtw_mdns_ofld_info {
	u8 mdns_en;
	u8 offload_state;
	struct rtw_mdns_resp_entry resp_entry[MAX_MDNS_RESP_NUM];
	struct rtw_mdns_passthru_list passthru_list;
	struct rtw_mdns_ipv4_header mdns_ipv4_header;
	struct rtw_mdns_ipv6_header mdns_ipv6_header;
};
#endif /* CONFIG_PHL_MDNS_OFFLOAD */

struct rtw_wow_wake_info {
	/* core */
	u8 wow_en;
	u8 drop_all_pkt;
	u8 rx_parse_after_wake;
	u8 pairwise_sec_algo;
	u8 group_sec_algo;
	u8 bip_sec_algo;
	u8 pattern_match_en;
	u8 magic_pkt_en;
	u8 hw_unicast_en;
	u8 fw_unicast_en;
	u8 deauth_wakeup;
	u8 rekey_wakeup;
	u8 eap_wakeup;
	u8 all_data_wakeup;
	struct rtw_remote_wake_ctrl_info remote_wake_ctrl_info;
};

struct rtw_aoac_report {
	/* status check */
	u8 rpt_fail;

	/* report from fw */
	u8 rpt_ver;
	u8 sec_type;
	u8 key_idx;
	u8 pattern_idx;
	u8 rekey_ok;
	u8 ptk_tx_iv[IV_LENGTH];
	u8 eapol_key_replay_count[8];
	u8 gtk[32];
	u8 ptk_rx_iv[IV_LENGTH];
	u8 gtk_rx_iv[4][IV_LENGTH];
	u8 igtk_key_id[8];
	u8 igtk_ipn[8];
	u8 igtk[32];
};

#ifdef CONFIG_WOWLAN

/* Exported APIs to core */
enum rtw_phl_status rtw_phl_cfg_keep_alive_info(void *phl, struct rtw_keep_alive_info *info);
enum rtw_phl_status rtw_phl_cfg_disc_det_info(void *phl, struct rtw_disc_det_info *info);
void rtw_phl_cfg_nlo_info(void *phl, struct rtw_nlo_info *info);
void rtw_phl_cfg_periodic_wake_info(void *phl, struct rtw_periodic_wake_info *info);
void rtw_phl_cfg_arp_ofld_info(void *phl, struct rtw_arp_ofld_info *info);
void rtw_phl_cfg_ndp_ofld_info(void *phl, struct rtw_ndp_ofld_info *info);
#ifdef CONFIG_PHL_MDNS_OFFLOAD
void rtw_phl_cfg_mdns_ofld_info(void *phl, struct rtw_mdns_ofld_info *info);
#endif
enum rtw_phl_status rtw_phl_remove_wow_ptrn_info(void *phl, u8 phl_ptrn_id);
enum rtw_phl_status rtw_phl_add_wow_ptrn_info(void *phl, struct rtw_wowcam_upd_info *info, u8 *phl_ptrn_id);
enum rtw_phl_status rtw_phl_cfg_gtk_ofld_info(void *phl, struct rtw_gtk_ofld_info *info);
enum rtw_phl_status rtw_phl_cfg_realwow_info(void *phl, struct rtw_realwow_info *info);
enum rtw_phl_status rtw_phl_cfg_wow_wake(void *phl, struct rtw_wow_wake_info *info);
enum rtw_phl_status rtw_phl_cfg_gpio_wake_pulse(void *phl, struct rtw_wow_gpio_info *info);
const char *rtw_phl_get_wow_rsn_str(void *phl, u8 wake_rsn);
enum rtw_phl_status rtw_phl_cfg_wow_set_sw_gpio_mode(void *phl, struct rtw_wow_gpio_info *info);
enum rtw_phl_status rtw_phl_cfg_wow_sw_gpio_ctrl(void *phl, struct rtw_wow_gpio_info *info);
void rtw_phl_wow_set_no_link_mode(void *phl, u8 no_link_mode);
u32 rtw_phl_get_wow_fail_stat(void *phl);
#endif /* CONFIG_WOWLAN */

#endif /* _PHL_WOW_DEF_H_ */

