#ifndef __HW_CFG_H__
#define __HW_CFG_H__

//==============================================================================
// CPLD
//==============================================================================

#define CPLD_BASE               0xF0000000
#define CPLD_RMSIZE             0x08

#define BASE_VERSION            (CPLD_BASE+0x00)
#define OFFSET_HARD_VER         0x00
#define OFFSET_CPLD_VER         0x01
#define OFFSET_UBOOT_VER        0x02

#define BASE_STATUS             (CPLD_BASE+0x04)
#define OFFSET_STATUS           0x00
#define OFFSET_DETECT           0x01

#define BASE_RESET              (CPLD_BASE+0x08)
#define OFFSET_RESET_CTRL       0x00
#define OFFSET_RESET_ENABLE     0x01

#define BASE_RX_ADC             (CPLD_BASE+0x14)
#define OFFSET_RX_ADC_CTRL      0x00

#define BASE_TX_DAC             (CPLD_BASE+0x18)
#define OFFSET_TX_DAC_CTRL      0x00

#define BASE_SFP                (CPLD_BASE+0x1C)
#define OFFSET_SFP_STAT         0x00

#define BASE_CPLD_IIC           (CPLD_BASE+0x1C)
#define OFFSET_CPLD_IIC0_CTRL   0x01
#define OFFSET_CPLD_IIC1_CTRL   0x02

#define BASE_FPGA_CFG           (CPLD_BASE+0x20)
#define OFFSET_FPGA_CFG_DATA    0x00
#define OFFSET_FPGA_CFG_CTRL    0x01
#define OFFSET_FPGA_CFG_CLK     0x02

#define BASE_POWER              (CPLD_BASE+0x24)
#define OFFSET_POWER_INT        0x00
#define OFFSET_POWER_PEND       0x01
#define OFFSET_POWER_STAT       0x02

#define BASE_AD7814             (CPLD_BASE+0x28)
#define OFFSET_AD7814_CTRL      0x00

//==============================================================================
// FPGA
//==============================================================================
#define IF_FPGA_BASE            0xF0120000
#define IF_FPGA_RMSIZE          0x00020000

#define BASE_TLK3132            0xF0100000
#define OFFSET_TLK3132_CTRL     0xF
#define OFFSET_TLK3132_TDATA    0xD
#define OFFSET_TLK3132_RDATA    0xE

//==============================================================================
// interrupt list
//==============================================================================

#define MPC8313_IRQ0        48
#define MPC8313_IRQ1        17
#define MPC8313_IRQ2        18
#define MPC8313_IRQ3        19
#define MPC8313_IRQ4        20

#define POWER_IRQ           MPC8313_IRQ0
#define IR_5MS_IRQ          MPC8313_IRQ1
#define IR_ALARM_IRQ        MPC8313_IRQ2
#define IR_ETH_IRQ          MPC8313_IRQ3
#define IF_AGC_IRQ          MPC8313_IRQ4

//==============================================================================
// Device list
//==============================================================================
#define NAME_VERSION        "r108_version"
#define NAME_STATUS         "r108_status"
#define NAME_RESET          "r108_reset"
#define NAME_LMK04031       "r108_lmk04031"
#define NAME_LMX2531        "r108_lmx2531"
#define NAME_SFP            "r108_sfp"
#define NAME_AD7814         "r108_ad7814"
#define NAME_FPGA_CFG       "r108_fpga_cfg"
#define NAME_FB_ADC         "r108_fb_adc "
#define NAME_RX_ADC         "r108_rx_adc"
#define NAME_TX_DAC         "r108_tx_dac"
#define NAME_LED            "r108_led"
#define NAME_RTC            "r108_rtc"
#define NAME_POWER          "r108_power"
#define NAME_IR_FPGA        "r108_ir_fpga"
#define NAME_IF_FPGA        "r108_if_fpga"
#define NAME_TLK3132        "r108_tlk3132"
#define NAME_EEPROM         "r108_eeprom"
#define NAME_SPIROM         "r108_spirom"

#define MAJ_VERSION         220
#define MAJ_STATUS          221
#define MAJ_RESET           222
#define MAJ_LMK04031        223
#define MAJ_LMX2531         224
#define MAJ_SFP             225
#define MAJ_AD7814          226
#define MAJ_FPGA_CFG        227
#define MAJ_RX_ADC          229
#define MAJ_TX_DAC          230
#define MAJ_LED             231
#define MAJ_RTC             232
#define MAJ_POWER           233
#define MAJ_IR_FPGA         236
#define MAJ_IF_FPGA         237
#define MAJ_TLK3132         238
#define MAJ_EEPROM          239
#define MAJ_SPIROM          240

#define MIN_VERSION         0
#define MIN_STATUS          0
#define MIN_RESET           0
#define MIN_LMK04031        0
#define MIN_LMX2531         0
#define MIN_SFP             0
#define MIN_AD7814          0
#define MIN_FPGA_CFG        0
#define MIN_RX_ADC          0
#define MIN_TX_DAC          0
#define MIN_LED             0
#define MIN_RTC             0
#define MIN_POWER           0
#define MIN_IR_FPGA         0
#define MIN_IF_FPGA         0
#define MIN_TLK3132         0
#define MIN_EEPROM          0
#define MIN_SPIROM          0

#define MAGIC_VERSION       0xE0
#define MAGIC_STATUS        0xE1
#define MAGIC_RESET         0xE2
#define MAGIC_LMK04031      0xE3
#define MAGIC_LMX2531       0xE4
#define MAGIC_SFP           0xE5
#define MAGIC_AD7814        0xE6
#define MAGIC_FPGA_CFG      0xE7
#define MAGIC_RX_ADC        0xE9
#define MAGIC_TX_DAC        0xEA
#define MAGIC_LED           0xEB
#define MAGIC_RTC           0xEC
#define MAGIC_POWER         0xED
#define MAGIC_IR_FPGA       0xF0
#define MAGIC_IF_FPGA       0xF1
#define MAGIC_TLK3132       0xF2
#define MAGIC_EEPROM        0xF3
#define MAGIC_SPIROM        0xF4

#endif // __R804XY_SYS_H__
