#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util;
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFA

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update) {
  unsigned int i;
  for(i = 0; i < count; i++) {
    unsigned cmd;
    cmd = table[i].cmd;
    switch (cmd) {
      case REGFLAG_DELAY :
        MDELAY(table[i].count);
        break;
      case REGFLAG_END_OF_TABLE :
        break;
      default:
        dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
    }
  }
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{ 0xF1, 0x08, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    { 0xB1, 0x04, {0x22, 0x1F, 0x1F, 0x87}},
    { 0xB2, 0x01, {0x22}},
    { 0xB3, 0x08, {0x00, 0x00, 0x06, 0x06, 0x20, 0x20, 0x30, 0x30}},
    { 0xBA, 0x11, {0x31, 0x00, 0x44, 0x25, 0x91, 0x0A, 0x00, 0x00, 0xC1, 0x00, 0x00, 0x00, 0x0D, 0x02, 0x4F, 0xB9, 0xEE}},
    { 0xE3, 0x05, {0x04, 0x04, 0x01, 0x01, 0x00}},
    { 0xB4, 0x01, {0x00}},
    { 0xB5, 0x02, {0x0A, 0x0A}},
    { 0xB6, 0x02, {0x4E, 0x4E}},
    { 0xB8, 0x02, {0x64, 0x20}},
    { 0xCC, 0x01, {0x02}},
    { 0xBC, 0x01, {0x44}},
    { 0xE9, 0x33, {0x00, 0x00, 0x06, 0x00, 0x00, 0x81, 0x89, 0x12, 0x31, 0x23, 0x23, 0x08, 0x81, 0x80, 0x23, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x13, 0x18, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x89, 0x02, 0x08, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    { 0xEA, 0x16, {0x90, 0x00, 0x00, 0x00, 0x88, 0x02, 0x09, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x13, 0x19, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88}},
    { 0xE0, 0x22, {0x00, 0x16, 0x0B, 0x36, 0x3F, 0x3F, 0x28, 0x41, 0x08, 0x0D, 0x0E, 0x11, 0x13, 0x10, 0x11, 0x0E, 0x16, 0x00, 0x16, 0x0B, 0x36, 0x3F, 0x3F, 0x28, 0x41, 0x08, 0x0D, 0x0E, 0x11, 0x13, 0x10, 0x11, 0x0E, 0x16}},
    { 0x11, 0x01, {0x00}},
    { REGFLAG_DELAY, 120, {}},
    { 0x29, 0x01, {0x00}},
    { REGFLAG_DELAY, 50, {}},
    { REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util) {
  memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
};

static void lcm_get_params(LCM_PARAMS *params) {
  memset(params, 0, sizeof(LCM_PARAMS));
  params->dsi.mode = 1;
  params->dsi.packet_size = 256;
  params->dsi.vertical_sync_active = 4;
  params->dsi.vertical_backporch = 12;
  params->dsi.vertical_frontporch = 10;
  params->dsi.horizontal_sync_active = 8;
  params->type = 2;
  params->dsi.PLL_CLOCK = 203;
  params->dsi.LANE_NUM = 2;
  params->dsi.data_format.format = 2;
  params->dsi.intermediat_buffer_num = 2;
  params->dsi.PS = 2;
  params->width = 480;
  params->dsi.horizontal_active_pixel = 480;
  params->height = 854;
  params->dsi.vertical_active_line = 854;
  params->dbi.te_mode = 0;
  params->dbi.te_edge_polarity = 0;
  params->dsi.data_format.color_order = 0;
  params->dsi.data_format.trans_seq = 0;
  params->dsi.data_format.padding = 0;
  params->dsi.horizontal_backporch = 80;
  params->dsi.horizontal_frontporch = 80;
};

static void lcm_init(void) {
  SET_RESET_PIN(1);
  MDELAY(20);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
};

static void lcm_suspend(void) {
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);
};

static void lcm_resume(void) {
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
};

static unsigned int lcm_compare_id(void) {
  return 1;
/*
  unsigned char buffer[2];
  unsigned int data_array[8];

  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(10);
  SET_RESET_PIN(1);
  MDELAY(120);

  data_array[0] = 0x00043902;
  data_array[1] = 0x0108F1B9;
  dsi_set_cmdq(data_array, 2, 1);
  MDELAY(10);

  data_array[0] = 0x00123902;
  data_array[1] = 0x440031BA;
  data_array[2] = 0x000A9125;
  data_array[4] = 0x4F020D00;
  data_array[3] = 0x0000C100;
  data_array[5] = 0x0000EEB9;
  dsi_set_cmdq(data_array, 6, 1);

  data_array[0] = 0x00033700;
  dsi_set_cmdq(data_array, 1, 1);
  MDELAY(10);

  read_reg_v2(0xD0, buffer, 2);

  return buffer - 3 <= 0;
*/
};

static void lcm_setbacklight(unsigned int level) {
  unsigned int data_array[16];

  if ( level >= 255 )
    level = 255;

  data_array[0] = 0x00023902;
  data_array[1] = (level << 8) | 0x51;
  dsi_set_cmdq( 0x51 | ( level << 8 ) );
};

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER fl10802_fwvga_dsi_vdo_lcm_drv =
{
    .name           = "fl10802_fwvga_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .set_backlight  = lcm_setbacklight,
    .compare_id     = lcm_compare_id,
};
