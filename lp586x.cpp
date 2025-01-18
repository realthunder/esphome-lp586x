#include <cinttypes>
#include "lp586x.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <esp_attr.h>

/***************************************************************************//**
 *Register Address
 ******************************************************************************/
/* Device Configuration Register Address*/
#define Chip_Enable_Register               0x000
#define Device_Initial_Register            0x001
#define Device_Configuration_1_Register    0x002
#define Device_Configuration_2_Register    0x003
#define Device_Configuration_3_Register    0x004

#define LED_Group_Selection_Register_start 0x00C //Register address from 0x009 to 0x042
#define LED_Dot_Onoff_Register_start       0x043 //Register address from 0x043 to 0x063

/* Current Configuration Register Address */
#define Color_Group_R_Current_Register     0x009
#define Color_Group_G_Current_Register     0x00A
#define Color_Group_B_Current_Register     0x00B
#define LED_Dot_Current_Register_Start     0x100 //Register address from 0x100 to 0x1C5

/* Brightness Configuration Register Address */
#define Master_Brightness_Register         0x005
#define Group_1_Brightness_Register        0x006
#define Group_2_Brightness_Register        0x007
#define Group_3_Brightness_Register        0x008
#define LED_Dot_Brightness_Register_Start  0x200 //Register address from 0x200 to 0x2C5 (8-bit) Or to 0x38B (16-bit)

/* Fault Register Address*/
#define Fault_State_Register               0x064
#define LED_Dot_LOD_Register_start         0x065 //Register address from 0x065 to 0x085
#define LED_Dot_LSD_Register_start         0x086 //Register address from 0x086 to 0x0A6

/* Reset Register Address*/
#define LOD_Clear_Register                 0x0A7
#define LSD_Clear_Register                 0x0A8
#define Reset_Register                     0x0A9

/***************************************************************************//**
 *Register Value
 ******************************************************************************/
#define Chip_Disable                       0x00
#define Chip_Enable                        0x01

#define LOD_Clear_En                       0xff
#define LSD_Clear_En                       0xff
#define Reset_En                           0xff

/***************************************************************************//**
 *Register Dev_initial Value
 ******************************************************************************/
#define Max_Line_11                        0xB 
#define Max_Line_10                        0xA
#define Max_Line_9                         0x9
#define Max_Line_8                         0x8
#define Max_Line_7                         0x7
#define Max_Line_6                         0x6
#define Max_Line_5                         0x5
#define Max_Line_4                         0x4
#define Max_Line_3                         0x3
#define Max_Line_2                         0x2
#define Max_Line_1                         0x1	//default

#define Mode_3_1                           0x3 
#define Mode_3                             0x2
#define Mode_2                             0x1
#define Mode_1                             0x0	//default

#define PWM_Frequency_62_5k                0x1
#define PWM_Frequency_125k                 0x0  //default

/***************************************************************************//**
 *Register Dev_config1 Value
 ******************************************************************************/
#define SW_BLK_05us                        0x1
#define SW_BLK_1us                         0x0 //default

#define PWM_Scale_Exponential              0x1
#define PWM_Scale_Linear                   0x0 //default

#define PWM_PhaseShift_on                  0x1
#define PWM_PhaseShift_off                 0x0 //default

#define CS_Delay_on                        0x1
#define CS_Delay_off                       0x0 //default

/***************************************************************************//**
 *Register Dev_config2 Value
 ******************************************************************************/
#define Comp_Group3_3clock                 0x3
#define Comp_Group3_2clock                 0x2
#define Comp_Group3_1clock                 0x1
#define Comp_Group3_Off                    0x0 //default

#define Comp_Group2_3clock                 0x3
#define Comp_Group2_2clock                 0x2
#define Comp_Group2_1clock                 0x1
#define Comp_Group2_Off                    0x0 //default

#define Comp_Group1_3clock                 0x3
#define Comp_Group1_2clock                 0x2
#define Comp_Group1_1clock                 0x1
#define Comp_Group1_Off                    0x0 //default

#define LOD_Removal_En                     0x1
#define LOD_Removal_Off                    0x0 //default

#define LSD_Removal_En                     0x1
#define LSD_Removal_Off                    0x0 //default

/***************************************************************************//**
 *Register Dev_config3 Value
 ******************************************************************************/
#define Down_Deghost_Strong                0x3
#define Down_Deghost_Medium                0x2
#define Down_Deghost_Weak                  0x1 //default
#define Down_Deghost_off                   0x0

#define Up_Deghost_GND                     0x3
#define Up_Deghost_3                       0x2
#define Up_Deghost_2_5                     0x1
#define Up_Deghost_2                       0x0 //default

#define Maximum_Current_50                 0x7	//default
#define Maximum_Current_40                 0x6
#define Maximum_Current_30                 0x5
#define Maximum_Current_20                 0x4
#define Maximum_Current_15                 0x3 
#define Maximum_Current_10                 0x2
#define Maximum_Current_5                  0x1
#define Maximum_Current_3                  0x0

#define Up_Deghost_En                      0x1 
#define Up_Deghost_Off                     0x0	//default

/***************************************************************************//**
 *PWM Value
 ******************************************************************************/
#define Group3                             0x3
#define Group2                             0x2
#define Group1                             0x1
#define Group_None                         0x0 //default

/***************************************************************************//**
 *PWM Value
 ******************************************************************************/
#define PWM_Global_Max                     0xff
#define PWM_Group_Max                      0xff
#define PWM_Dot_Max                        0xff

/***************************************************************************//**
 *CURRENT Value
 ******************************************************************************/
#define ColorGroup_Current_Default         0x0A  //0x7F	//default

typedef enum{
	Check_LED_Idle			= 0,
	Turn_Off_All_LED		= 1,
	Check_LED_Open 			= 2,
	Turn_On_All_LED			= 3,
	Check_LED_Short 		= 4,
	Check_LED_Finish		= 5,
}_LP586x_Selftest_Step_INDEX;	

namespace esphome {
namespace lp586x {

static const char *const TAG = "lp586x";

void LP586XLightOutput::DC_Color(uint8_t Dot,uint8_t R_DC,uint8_t G_DC, uint8_t B_DC)
{
  uint8_t brg[] = {B_DC, R_DC, G_DC};
	this->write_data(LED_Dot_Current_Register_Start+3*Dot,brg,3);
}

void LP586XLightOutput::Dot_Brightness_8bit(uint8_t Dot, uint8_t R_PWM, uint8_t G_PWM, uint8_t B_PWM)
{
  uint8_t rgb[] = {R_PWM, G_PWM, B_PWM};
	this->write_data(LED_Dot_Brightness_Register_Start+3*Dot,rgb,3);
}

void LP586XLightOutput::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LP586X...");

  dump_config();

  if (this->sync_pin_) {
    this->sync_pin_->setup();
    this->sync_pin_->digital_write(false);
  }
  if (this->vio_en_pin_) {
    this->vio_en_pin_->setup();
    this->vio_en_pin_->digital_write(false);
	  delay(50);
    this->vio_en_pin_->digital_write(true);
	  delay(50);
  }
  
  size_t buffer_size = this->get_buffer_size_();

  RAMAllocator<uint8_t> allocator(0);
  this->buf_ = allocator.allocate(buffer_size);
  if (this->buf_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate LED buffer!");
    this->mark_failed();
    return;
  }

  this->effect_data_ = allocator.allocate(this->num_leds_);
  if (this->effect_data_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate effect data!");
    this->mark_failed();
    return;
  }

  this->write_data(Chip_Enable_Register,Chip_Enable);
  delay(100);

	this->write_data(Device_Initial_Register,
      (Max_Line_1<<3) + (Mode_1<<1) + PWM_Frequency_125k);

  this->write_data(Device_Configuration_1_Register,
      (SW_BLK_1us<<3) + (PWM_Scale_Linear<<2) + (PWM_PhaseShift_off<<1) + CS_Delay_off);

  this->write_data(Device_Configuration_2_Register,
      (Comp_Group3_Off<<6) + (Comp_Group2_Off<<4) + (Comp_Group1_Off<<2) + (LOD_Removal_Off<<1) + LSD_Removal_Off);

  this->write_data(Device_Configuration_3_Register,
      (Down_Deghost_Weak<<3) + (Up_Deghost_2<<4) + (Maximum_Current_50<<1) + Up_Deghost_Off);	

  this->write_data(Master_Brightness_Register, PWM_Global_Max);
  this->write_data(Group_1_Brightness_Register, PWM_Group_Max);
  this->write_data(Group_2_Brightness_Register, PWM_Group_Max);
  this->write_data(Group_3_Brightness_Register, PWM_Group_Max);

  current_limits_[0] = ColorGroup_Current_Default;
  current_limits_[1] = ColorGroup_Current_Default;
  current_limits_[2] = ColorGroup_Current_Default;
  this->write_data(Color_Group_R_Current_Register, current_limits_, 3);

  for (int i=0; i<this->num_leds_; ++i) {
    // set All LED brightness 0
    DC_Color(i, 0, 0, 0);
    // set All LED PWM 255
    Dot_Brightness_8bit(i,0xFF,0xFF,0xFF);
  }

  this->write_data(LOD_Clear_Register,0x0F);
  this->write_data(LSD_Clear_Register,0x0F);
}

void LP586XLightOutput::write_state(light::LightState *state) {
  // protect from refreshing too often
  uint32_t now = micros();
  if (*this->max_refresh_rate_ != 0 && (now - this->last_refresh_) < *this->max_refresh_rate_) {
    // try again next loop iteration, so that this change won't get lost
    ESP_LOGW(TAG, "skip write state");
    this->schedule_show();
    return;
  }

  // ESP_LOGI(TAG, "%lu write state", esp_log_timestamp());
  this->last_refresh_ = now;
  this->mark_shown_();

  ESP_LOGVV(TAG, "Writing RGB values...");

  if (current_limit_changed_) {
    current_limit_changed_ = false;
    this->write_data(Color_Group_R_Current_Register, current_limits_, 3);
  }

  if (current_changed_) {
    current_changed_ = false;
	  this->write_data(LED_Dot_Current_Register_Start,this->buf_,3*this->num_leds_);
  }

  this->status_clear_warning();
}

light::ESPColorView LP586XLightOutput::get_view_internal(int32_t index) const {
  int32_t r = 0, g = 0, b = 0;
  switch (this->rgb_order_) {
    case ORDER_RGB:
      r = 0;
      g = 1;
      b = 2;
      break;
    case ORDER_RBG:
      r = 0;
      g = 2;
      b = 1;
      break;
    case ORDER_GRB:
      r = 1;
      g = 0;
      b = 2;
      break;
    case ORDER_GBR:
      r = 2;
      g = 0;
      b = 1;
      break;
    case ORDER_BGR:
      r = 2;
      g = 1;
      b = 0;
      break;
    case ORDER_BRG:
      r = 1;
      g = 2;
      b = 0;
      break;
  }
  current_changed_ = true;
  return {this->buf_ + (index * 3) + r,
          this->buf_ + (index * 3) + g,
          this->buf_ + (index * 3) + b,
          nullptr,
          &this->effect_data_[index],
          &this->correction_};
}

void LP586XLightOutput::dump_config() {
  const char *rgb_order;
  switch (this->rgb_order_) {
    case ORDER_RGB:
      rgb_order = "RGB";
      break;
    case ORDER_RBG:
      rgb_order = "RBG";
      break;
    case ORDER_GRB:
      rgb_order = "GRB";
      break;
    case ORDER_GBR:
      rgb_order = "GBR";
      break;
    case ORDER_BGR:
      rgb_order = "BGR";
      break;
    case ORDER_BRG:
      rgb_order = "BRG";
      break;
    default:
      rgb_order = "UNKNOWN";
      break;
  }
  ESP_LOGCONFIG(TAG, "  Address: %02x", (unsigned)this->address_);
  LOG_PIN("  SYNC Pin: ", this->sync_pin_);
  LOG_PIN("  VIO_EN Pin: ", this->vio_en_pin_);
  ESP_LOGCONFIG(TAG, "  RGB Order: %s", rgb_order);
  ESP_LOGCONFIG(TAG, "  Max refresh rate: %" PRIu32, *this->max_refresh_rate_);
  ESP_LOGCONFIG(TAG, "  Number of LEDs: %u", this->num_leds_);
  ESP_LOGCONFIG(TAG, "  Current limits: R %u, G %u, B %u",
      (unsigned)this->current_limits_[0],
      (unsigned)this->current_limits_[1],
      (unsigned)this->current_limits_[2]);
}

float LP586XLightOutput::get_setup_priority() const {
  return setup_priority::HARDWARE;
}

}  // namespace lp586x
}  // namespace esphome

// vim: noai:ts=2:sw=2
