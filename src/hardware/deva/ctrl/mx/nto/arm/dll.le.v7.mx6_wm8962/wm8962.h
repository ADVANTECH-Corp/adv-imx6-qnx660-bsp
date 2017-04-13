/*
 * $QNXLicenseC:
 * Copyright 2014 QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#ifndef __MIXER_H
#define __MIXER_H

/* Default clock values. */
#define MCLK_REF                16500000 /* This value assumes that mx6q_init_audmux_pins() in startup is configured for 16.5MHz */
#define FVCO_MAXVAL             100000000 /* Maximum value for Fvco: 100Mhz */
#define FVCO_MINVAL             90000000 /* Minimum value for Fvco: 90Mhz */
#define SYSCLK                  24576000 /* The value we want the PLL to produce for a given MCLK_REF. */

/* I2C 7-bit slave address */
#define WM8962_SLAVE_ADDR       (0x34 >> 1) /* Device ID from data sheet. */

/*
 * WM8962 CODEC Registers
 */
#define SOFTWARE_RESET          0x0F
#define SW_RESET_PLL            0x7F

#define LINPUT_VOL              0x00 // R0
#define RINPUT_VOL              0x01 // R1
#define HPOUTL_VOL              0x02 // R2
#define HPOUTR_VOL              0x03 // R3
#define ADCDACTL1               0x05 // R5
#define CLOCKING_1              0x04 // R4
#define AUDIO_INTERFACE_0       0x07 // R7
#define CLOCKING_2              0x08 // R8
#define LDAC_VOLUME             0x0A // R10
#define RDAC_VOLUME             0x0B // R11
#define AUDIO_INTERFACE_2       0x0E // R14
#define ADCL_VOL                0x15 // R21
#define ADCR_VOL                0x16 // R22
#define PWR_MGMT_1              0x19 // R25
#define PWR_MGMT_2              0x1A // R26
#define ADDITIONAL_CTRL_3       0x1B // R27
#define CLOCKING_3              0x1E // R30
#define INPUT_MIXER_CTRL_1      0x1F // R31
#define RIGHT_INPUT_MIX_VOL     0x21 // R33
#define INPUT_MIXER_CTRL_2      0x22 // R34
#define LINPUT_PGA_CONTROL      0x25 // R37
#define RINPUT_PGA_CONTROL      0x26 // R38
#define SPKOUTL_VOL             0x28 // R40
#define SPKOUTR_VOL             0x29 // R41
#define ADDITIONAL_CTL          0x30 // R48
#define CLASSDCTL_1             0x31 // R49
#define CLASSDCTL_2             0x33 // R51
#define SPAKER_MIXER_1          0x69 // R105
#define SPAKER_MIXER_2          0x6A // R105
#define SPAKER_MIXER_3          0x6B // R107
#define SPAKER_MIXER_4          0x6C // R108
#define MIXERENABLE             0x63 // R99
#define WRITE_SEQ_CONTROL_1     0x57 // R87
#define WRITE_SEQ_CONTROL_2     0x5a // R90
#define WRITE_SEQ_CONTROL_3     0x5d // R93
#define HEADPHONE_MIXER_1       0x64 // R100
#define HEADPHONE_MIXER_2       0x65 // R101
#define HEADPHONE_MIXER_3       0x66 // R102
#define HEADPHONE_MIXER_4       0x67 // R103
#define ANALOGUE_CLOCKING_1     0x7C // R124
#define ANALOGUE_CLOCKING_2     0x7D // R125
#define ANALOGUE_CLOCKING_3     0x7E // R126
#define PLL_2                   0x81 // R129
#define PLL_4                   0x83 // R131
#define PLL_13                  0x8C // R140
#define PLL_14                  0x8D // R141
#define PLL_15                  0x8E // R142
#define PLL_16                  0x8F // R142
#define PLL_DLL                 0x96 // R150
#define ANALOGUE_CLOCKING_4     0x98 // R152 (also called "ANALOGUE CONTROL 4" in datasheet.)
#define GPIO_2                  0x201 // R513
#define SOUNDSTAGE_ENABLE       0x4005 //R16389

/* Right Input Volume */
#define INPGAR_MUTE(x)          ((x) << 7)
#define IN_VU                   (0x1 << 8)
#define CHIP_REV_MASK           (0x7 << 9)
#define CUST_ID_MASK            (0xF << 11)

#define INPGAR_MUTE_MASK        (0x1 << 7)
#define IN_VU_MASK              (0x1 << 8)

/* DAC VOL */
#define DAC_MUTE(x)             ((x) << 3)
#define DACOUT_VU               (0x1 << 8) /* Update L & R simultaneously. */
#define DACOUT_VU_MASK          (0x1 << 8)

/* HPOUTL VOL */
#define HPOUT_VU                (0x1 << 8) /* Update L & R simultaneously. */
#define HPOUT_ZC                (0x1 << 7) /* Zero Cross Function. */
#define HPOUT_VOL_0DB_ATT       (0x79 << 0) /* 0dB volume */

#define HPOUT_VU_MASK           (0x1 << 8)
#define HPOUT_ZC_MASK           (0x1 << 7)
#define HPOUTL_VOL_MASK         (0x7F << 0)

/* SPEAKEROUTL VOL*/
#define SPKOUT_VU               (0x1 << 8) /* Update L & R simultaneously. */
#define SPKOUT_ZC               (0x1 << 7) /* Zero Cross Function. */
#define SPKOUT_VOL_0DB_ATT      (0x79 << 0) /* 0dB volume */

#define SPKOUT_VU_MASK          (0x1 << 8)
#define SPKOUT_ZC_MASK          (0x1 << 7)
#define SPKOUTL_VOL_MASK        (0x7F << 0)

/* HPOUTR VOL */
#define HPOUTR_VOL_MASK         (0x7F << 0)

/* Audio Interface 0 */
#define FMT_I2S                 (0x2 << 0)
#define MSTR                    (0x1 << 6)
#define SLAVE                   (0x0 << 6)
#define WL_16                   (0x00)

#define WL_MASK                 (0x3 << 2)
#define MSTR_MASK               (0x1 << 6)
#define FMT_MASK                (0x3 << 0)

/* ADC/DAC Control 1 */
#define DAC_MUTE_SET(x)         ((x) << 3)

/* Clocking 2 */
#define CLKREG_OVD              (0x1 << 11)
#define MCLK_SRC_MCLK           (0x0 << 9)
#define MCLK_SRC_FLL            (0x1 << 9)
#define MCLK_SRC_PLL            (0x2 << 9)
#define SYSCLK_ENA              (0x1 << 5)

#define SYSCLK_ENA_MASK         (0x1 << 5)
#define MCLK_SRC_MASK           (0x3 << 9)
#define CLKREG_OVD_MASK         (0x1 << 11)

/* Clocking 3 */
#define OPCLK_DIV_1             (0x00 << 10)
#define BCLK_DIV_8              (0x7 << 0)

#define OPCLK_DIV_MASK          (0x7 << 10)
#define BCLK_DIV_MASK           (0xF << 0)

/* Input Mixer Control 1 */
#define MIXINR_ENA              (0x1 << 0)
#define MIXINL_ENA              (0x1 << 1)
#define MIXINR_MUTE(x)          ((x) << 2)

#define MIXINR_ENA_MASK         (0x1 << 0)
#define MIXINL_ENA_MASK         (0x1 << 1)
#define MIXINR_MUTE_MASK        (0x1 << 2)
#define MIXINL_MUTE_MASK        (0x1 << 3)

/* Right Input Mixer Volume */
#define IN3R_MIXINR_VOL_0DB     (0x5 << 0)
#define INPGAR_MIXINR_VOL_0DB   (0x0 << 3)
#define INPGAR_MIXINR_VOL_29DB  (0x7 << 3)

#define INPGAR_MIXINR_VOL_MASK  (0x7 << 3)
#define IN3R_MIXINR_VOL_MASK    (0x7 << 0)

/* Input Mixer Control 2 */
#define IN3R_TO_MIXINR          (0x1 << 1)
#define INPGAR_TO_MIXINR        (0x1 << 0)

#define INPGAR_TO_MIXINR_MASK   (0x1 << 0)
#define IN3R_TO_MIXINR_MASK     (0x1 << 1)
#define IN2R_TO_MIXINR_MASK     (0x1 << 2)
#define INPGAL_TO_MIXINL_MASK   (0x1 << 3)
#define IN3L_TO_MIXINL_MASK     (0x1 << 4)
#define IN2L_TO_MIXINL_MASK     (0x1 << 5)

/* Input PGA Control */
#define INPGAL_ENA              (0x1 << 4)
#define INPGAR_ENA              (0x1 << 4)
#define IN3R_TO_INPGAR          (0x1 << 1)

#define INPGAR_ENA_MASK         (0x1 << 4)
#define IN1R_TO_INPGAR_MASK     (0x1 << 3)
#define IN2R_TO_INPGAR_MASK     (0x1 << 2)
#define IN3R_TO_INPGAR_MASK     (0x1 << 1)

/* DAC Volume */
#define DAC_VOLUME_0DB          (0xC0)
#define DAC_VU                  (0x1 << 8) /* Updates left and right simultaneously. */

#define DAC_VU_MASK             (0x1 << 8)
#define DAC_VOLUME_MASK         (0xFF << 0)

/* Audio Interface 2 */
#define AIF_RATE_32             (0x20)
#define AIF_RATE_MASK           (0x7FF << 0)

/* ADC Volume */
#define ADC_VOL_0DB             (0xC0) /* 0dB volume on left ADC. */
#define ADC_VOL_23DB            (0xff) /* 23.625dB volume on left ADC. */

#define ADC_VU                  (0x1 << 8)
#define ADC_VOL_MASK            (0xFF << 0)
#define ADC_VU_MASK             (0x1 << 8)

/* PWR_MGMT_1 */
#define MICBIAS_ENA             (0x1 << 1)
#define MICBIAS_SET(x)          ((x) << 1)
#define ADCR_ENA                (0x1 << 2)
#define ADCL_ENA                (0x1 << 3)
#define INR_ENA                 (0x1 << 4)
#define BIAS_ENA                (0x1 << 6)
#define VMID_SEL_50K_DIV        (0x1 << 7)
#define OPCLK_ENA               (0x1 << 9)
#define DMIC_ENA                (0x1 << 10)

#define MICBIAS_ENA_MASK        (0x1 << 1)
#define ADCR_ENA_MASK           (0x1 << 2)
#define ADCL_ENA_MASK           (0x1 << 3)
#define INR_ENA_MASK            (0x1 << 4)
#define BIAS_ENA_MASK           (0x1 << 6)
#define VMID_SEL_MASK           (0x3 << 7)
#define OPCLK_ENA_MASK          (0x1 << 9)
#define DMIC_ENA_MASK           (0x1 << 10)

/* PWR_MGMT_2 */
#define DACL_ENA                (0x1 << 8)
#define DACR_ENA                (0x1 << 7)
#define HPOUTL_PGA_ENA          (0x1 << 6)
#define HPOUTR_PGA_ENA          (0x1 << 5)
#define HPOUTL_MUTE(x)          ((x) << 1)
#define HPOUTR_MUTE(x)          ((x) << 0)

#define DACL_ENA_MASK           (0x1 << 8)
#define DACR_ENA_MASK           (0x1 << 7)
#define HPOUTL_PGA_ENA_MASK     (0x1 << 6)
#define HPOUTR_PGA_ENA_MASK     (0x1 << 5)
#define SPKOUTL_PGA_ENA_MASK    (0x1 << 4)
#define SPKOUTR_PGA_ENA_MASK    (0x1 << 3)
#define HPOUTL_PGA_MUTE_MASK    (0x1 << 1)
#define HPOUTR_PGA_MUTE_MASK    (0x1 << 0)

/* Additional Control 3 */
#define SAMPLE_INT_MODE         (0x1 << 4)
#define SAMPLE_FRAC_MODE        (0x0 << 4)
#define SAMPLE_RATE_32KHZ       (0x1) // Mode 1 (INT)
#define SAMPLE_RATE_44KHZ       (0x0) // Mode 0 (FRACTIONAL) (44.1Khz)
#define SAMPLE_RATE_48KHZ       (0x0) // Mode 1 (INT)
#define SAMPLE_RATE_96KHZ       (0x6) // Mode 1 (INT)

#define SAMPLE_MODE_MASK        (0x1 << 4)
#define SAMPLE_RATE_MASK        (0x7 << 0)

/* Write Sequencer Control */
#define WSEQ_ENA                (0x1 << 5)
#define WSEQ_ABORT              (0x1 << 8)
#define WSEQ_HP_POWER_UP        (0x80)
#define WSEQ_INPUT_POWER_UP     (0x92)
#define WSEQ_POWER_DOWN         (0x9B)
#define WSEQ_SPK_WAKE_UP        (0xE8)

#define WSEQ_BUSY_MASK          (0x1 << 0)
#define WSEQ_ENA_MASK           (0x1 << 5)

/* Headphone Mixer */
#define HPMIXL_TO_HPOUTL_PGA_MASK   (0x1 << 7)
#define HPMIXR_TO_HPOUTR_PGA_MASK   (0x1 << 7)

#define HPMIXL_MUTE_MASK        (0x1 << 8)
#define HPMIXR_MUTE_MASK        (0x1 << 8)

/* Analogue Clocking */
#define CLKOUT2_SEL_GPIO2       (0x1 << 5)
#define CLKOUT2_OE              (0x1 << 3)

#define CLKOUT2_SEL_MASK        (0x3 << 5)
#define CLKOUT2_OE_MASK         (0x1 << 3)

#define PLL_SYSCLK_DIV_1        (0x00)
#define PLL_SYSCLK_DIV_2        (0x1 << 3)
#define PLL_SYSCLK_DIV_4        (0x2 << 3)

#define PLL3_OUTDIV(x)          (((x) >> 2) << 6) /* x Should only ever be 2 or 4. */

#define PLL3_OUTDIV_MASK        (0x1 << 6)
#define PLL_SYSCLK_DIV_MASK     (0x3 << 3)

/* PLL Registers */
#define PLL3_ENA                (0x1 << 4)

#define PLL_CLK_SRC_MCLK        (0x1 << 1)
#define FLL_TO_PLL3_MCLK        (0x0 << 0)

#define PLL3_FRAC               (0x1 << 6)

#define OSC_MCLK_SRC_MCLK       (0x00)
#define OSC_MCLK_SRC_OSC        (0x01 << 4 )

#define SEQ_ENA_MASK            (0x1 << 1)

#define OSC_ENA_MASK            (0x1 << 7)
#define PLL2_ENA_MASK           (0x1 << 5)
#define PLL3_ENA_MASK           (0x1 << 4)

#define PLL_CLK_SRC_MASK        (0x1 << 1)
#define FLL_TO_PLL3_MASK        (0x1 << 0)

#define PLL3_FRAC_MASK          (0x1 << 6)
#define PLL3_N_MASK             (0x1F << 0)

#define PLL3_K_23_16_MASK       (0xFF << 0)
#define PLL3_K_15_8_MASK        (0xFF << 0)
#define PLL3_K_7_0_MASK         (0xFF << 0)

#define OSC_MCLK_SRC_MASK       (0x1 << 4)

/* GPIO */
#define GPIO2_FN_OPCLK          (0x12 << 0)

#define GPIO2_FN_MASK           (0x1F << 0)

/* Analogue Clocking */
#define CLKOUT5_OE_MASK         (0x1 << 0)

/* Input mux selection */
#define INPUT_MUX_MIC_IN        0

#define MAX_WAIT_TIME           500   /* Total sleep time is (WAIT_DELAY * MAX_WAIT_TIME) */
#define WAIT_DELAY              25       /* Time to wait in miliseconds. */

struct wm8962_context;

#define MIXER_CONTEXT_T struct wm8962_context

typedef struct wm8962_context
{
    ado_mixer_delement_t *mic;
    ado_mixer_delement_t *line;

    char                 i2c_num;   /* I2C bus number */
    int                  i2c_fd;    /* I2C device handle */
    int                  mclk;      /* external SYS_MCLK */
    int                  adr0cs;    /* wm8962 slave address select pin */
    int                  samplerate;
    int                  input_mux;
    uint16_t             dac_mute;
    uint16_t             adc_mute;
    uint16_t             hp_mute;
    uint16_t             spk_mute;
}
wm8962_context_t;

/*
  * R0 (0x00) - Left Input volume
*/
#define WM8962_IN_VU                            0x0100  /* IN_VU */
#define WM8962_IN_VU_MASK                       0x0100  /* IN_VU */
#define WM8962_IN_VU_SHIFT                           8  /* IN_VU */
#define WM8962_IN_VU_WIDTH                           1  /* IN_VU */
#define WM8962_INPGAL_MUTE                      0x0080  /* INPGAL_MUTE */
#define WM8962_INPGAL_MUTE_MASK                 0x0080  /* INPGAL_MUTE */
#define WM8962_INPGAL_MUTE_SHIFT                     7  /* INPGAL_MUTE */
#define WM8962_INPGAL_MUTE_WIDTH                     1  /* INPGAL_MUTE */
#define WM8962_INL_ZC                           0x0040  /* INL_ZC */
#define WM8962_INL_ZC_MASK                      0x0040  /* INL_ZC */
#define WM8962_INL_ZC_SHIFT                          6  /* INL_ZC */
#define WM8962_INL_ZC_WIDTH                          1  /* INL_ZC */
#define WM8962_INL_VOL_MASK                     0x003F  /* INL_VOL - [5:0] */
#define WM8962_INL_VOL_SHIFT                         0  /* INL_VOL - [5:0] */
#define WM8962_INL_VOL_WIDTH                         6  /* INL_VOL - [5:0] */

/*
  * R2 (0x02) - HPOUTL volume
*/
#define WM8962_HPOUT_VU                         0x0100  /* HPOUT_VU */
#define WM8962_HPOUT_VU_MASK                    0x0100  /* HPOUT_VU */
#define WM8962_HPOUT_VU_SHIFT                        8  /* HPOUT_VU */
#define WM8962_HPOUT_VU_WIDTH                        1  /* HPOUT_VU */
#define WM8962_HPOUTL_ZC                        0x0080  /* HPOUTL_ZC */
#define WM8962_HPOUTL_ZC_MASK                   0x0080  /* HPOUTL_ZC */
#define WM8962_HPOUTL_ZC_SHIFT                       7  /* HPOUTL_ZC */
#define WM8962_HPOUTL_ZC_WIDTH                       1  /* HPOUTL_ZC */
#define WM8962_HPOUTL_VOL_MASK                  0x007F  /* HPOUTL_VOL - [6:0] */
#define WM8962_HPOUTL_VOL_SHIFT                      0  /* HPOUTL_VOL - [6:0] */
#define WM8962_HPOUTL_VOL_WIDTH                      7  /* HPOUTL_VOL - [6:0] */

/*
 * R3 (0x03) - HPOUTR volume
*/
#define WM8962_HPOUT_VU                         0x0100  /* HPOUT_VU */
#define WM8962_HPOUT_VU_MASK                    0x0100  /* HPOUT_VU */
#define WM8962_HPOUT_VU_SHIFT                        8  /* HPOUT_VU */
#define WM8962_HPOUT_VU_WIDTH                        1  /* HPOUT_VU */
#define WM8962_HPOUTR_ZC                        0x0080  /* HPOUTR_ZC */
#define WM8962_HPOUTR_ZC_MASK                   0x0080  /* HPOUTR_ZC */
#define WM8962_HPOUTR_ZC_SHIFT                       7  /* HPOUTR_ZC */
#define WM8962_HPOUTR_ZC_WIDTH                       1  /* HPOUTR_ZC */
#define WM8962_HPOUTR_VOL_MASK                  0x007F  /* HPOUTR_VOL - [6:0] */
#define WM8962_HPOUTR_VOL_SHIFT                      0  /* HPOUTR_VOL - [6:0] */
#define WM8962_HPOUTR_VOL_WIDTH                      7  /* HPOUTR_VOL - [6:0] */

/*
 * R5 (0x05) - ADC & DAC Control 1
 */
#define WM8962_ADCR_DAT_INV                     0x0040  /* ADCR_DAT_INV */
#define WM8962_ADCR_DAT_INV_MASK                0x0040  /* ADCR_DAT_INV */
#define WM8962_ADCR_DAT_INV_SHIFT                    6  /* ADCR_DAT_INV */
#define WM8962_ADCR_DAT_INV_WIDTH                    1  /* ADCR_DAT_INV */
#define WM8962_ADCL_DAT_INV                     0x0020  /* ADCL_DAT_INV */
#define WM8962_ADCL_DAT_INV_MASK                0x0020  /* ADCL_DAT_INV */
#define WM8962_ADCL_DAT_INV_SHIFT                    5  /* ADCL_DAT_INV */
#define WM8962_ADCL_DAT_INV_WIDTH                    1  /* ADCL_DAT_INV */
#define WM8962_DAC_MUTE_RAMP                    0x0010  /* DAC_MUTE_RAMP */
#define WM8962_DAC_MUTE_RAMP_MASK               0x0010  /* DAC_MUTE_RAMP */
#define WM8962_DAC_MUTE_RAMP_SHIFT                   4  /* DAC_MUTE_RAMP */
#define WM8962_DAC_MUTE_RAMP_WIDTH                   1  /* DAC_MUTE_RAMP */
#define WM8962_DAC_MUTE                         0x0008  /* DAC_MUTE */
#define WM8962_DAC_MUTE_MASK                    0x0008  /* DAC_MUTE */
#define WM8962_DAC_MUTE_SHIFT                        3  /* DAC_MUTE */
#define WM8962_DAC_MUTE_WIDTH                        1  /* DAC_MUTE */
#define WM8962_DAC_DEEMP_MASK                   0x0006  /* DAC_DEEMP - [2:1] */
#define WM8962_DAC_DEEMP_SHIFT                       1  /* DAC_DEEMP - [2:1] */
#define WM8962_DAC_DEEMP_WIDTH                       2  /* DAC_DEEMP - [2:1] */
#define WM8962_ADC_HPF_DIS                      0x0001  /* ADC_HPF_DIS */
#define WM8962_ADC_HPF_DIS_MASK                 0x0001  /* ADC_HPF_DIS */
#define WM8962_ADC_HPF_DIS_SHIFT                     0  /* ADC_HPF_DIS */
#define WM8962_ADC_HPF_DIS_WIDTH                     1  /* ADC_HPF_DIS */

/*
* R10 (0x0A) - Left DAC volume
*/
#define WM8962_DAC_VU                           0x0100  /* DAC_VU */
#define WM8962_DAC_VU_MASK                      0x0100  /* DAC_VU */
#define WM8962_DAC_VU_SHIFT                          8  /* DAC_VU */
#define WM8962_DAC_VU_WIDTH                          1  /* DAC_VU */
#define WM8962_DACL_VOL_MASK                    0x00FF  /* DACL_VOL - [7:0] */
#define WM8962_DACL_VOL_SHIFT                        0  /* DACL_VOL - [7:0] */
#define WM8962_DACL_VOL_WIDTH                        8  /* DACL_VOL - [7:0] */

/*
  * R11 (0x0B) - Right DAC volume
  */
#define WM8962_DAC_VU                           0x0100  /* DAC_VU */
#define WM8962_DAC_VU_MASK                      0x0100  /* DAC_VU */
#define WM8962_DAC_VU_SHIFT                          8  /* DAC_VU */
#define WM8962_DAC_VU_WIDTH                          1  /* DAC_VU */
#define WM8962_DACR_VOL_MASK                    0x00FF  /* DACR_VOL - [7:0] */
#define WM8962_DACR_VOL_SHIFT                        0  /* DACR_VOL - [7:0] */
#define WM8962_DACR_VOL_WIDTH                        8  /* DACR_VOL - [7:0] */


/*
 * R21 (0x15) - Left ADC volume
*/
#define WM8962_ADC_VU                           0x0100  /* ADC_VU */
#define WM8962_ADC_VU_MASK                      0x0100  /* ADC_VU */
#define WM8962_ADC_VU_SHIFT                          8  /* ADC_VU */
#define WM8962_ADC_VU_WIDTH                          1  /* ADC_VU */
#define WM8962_ADCL_VOL_MASK                    0x00FF  /* ADCL_VOL - [7:0] */
#define WM8962_ADCL_VOL_SHIFT                        0  /* ADCL_VOL - [7:0] */
#define WM8962_ADCL_VOL_WIDTH                        8  /* ADCL_VOL - [7:0] */

/*
  * R22 (0x16) - Right ADC volume
  */
#define WM8962_ADC_VU                           0x0100  /* ADC_VU */
#define WM8962_ADC_VU_MASK                      0x0100  /* ADC_VU */
#define WM8962_ADC_VU_SHIFT                          8  /* ADC_VU */
#define WM8962_ADC_VU_WIDTH                          1  /* ADC_VU */
#define WM8962_ADCR_VOL_MASK                    0x00FF  /* ADCR_VOL - [7:0] */
#define WM8962_ADCR_VOL_SHIFT                        0  /* ADCR_VOL - [7:0] */
#define WM8962_ADCR_VOL_WIDTH                        8  /* ADCR_VOL - [7:0] */

/*
 * R26 (0x1A) - Pwr Mgmt (2)
*/
#define WM8962_DACL_ENA                         0x0100  /* DACL_ENA */
#define WM8962_DACL_ENA_MASK                    0x0100  /* DACL_ENA */
#define WM8962_DACL_ENA_SHIFT                        8  /* DACL_ENA */
#define WM8962_DACL_ENA_WIDTH                        1  /* DACL_ENA */
#define WM8962_DACR_ENA                         0x0080  /* DACR_ENA */
#define WM8962_DACR_ENA_MASK                    0x0080  /* DACR_ENA */
#define WM8962_DACR_ENA_SHIFT                        7  /* DACR_ENA */
#define WM8962_DACR_ENA_WIDTH                        1  /* DACR_ENA */
#define WM8962_HPOUTL_PGA_ENA                   0x0040  /* HPOUTL_PGA_ENA */
#define WM8962_HPOUTL_PGA_ENA_MASK              0x0040  /* HPOUTL_PGA_ENA */
#define WM8962_HPOUTL_PGA_ENA_SHIFT                  6  /* HPOUTL_PGA_ENA */
#define WM8962_HPOUTL_PGA_ENA_WIDTH                  1  /* HPOUTL_PGA_ENA */
#define WM8962_HPOUTR_PGA_ENA                   0x0020  /* HPOUTR_PGA_ENA */
#define WM8962_HPOUTR_PGA_ENA_MASK              0x0020  /* HPOUTR_PGA_ENA */
#define WM8962_HPOUTR_PGA_ENA_SHIFT                  5  /* HPOUTR_PGA_ENA */
#define WM8962_HPOUTR_PGA_ENA_WIDTH                  1  /* HPOUTR_PGA_ENA */
#define WM8962_SPKOUTL_PGA_ENA                  0x0010  /* SPKOUTL_PGA_ENA */
#define WM8962_SPKOUTL_PGA_ENA_MASK             0x0010  /* SPKOUTL_PGA_ENA */
#define WM8962_SPKOUTL_PGA_ENA_SHIFT                 4  /* SPKOUTL_PGA_ENA */
#define WM8962_SPKOUTL_PGA_ENA_WIDTH                 1  /* SPKOUTL_PGA_ENA */
#define WM8962_SPKOUTR_PGA_ENA                  0x0008  /* SPKOUTR_PGA_ENA */
#define WM8962_SPKOUTR_PGA_ENA_MASK             0x0008  /* SPKOUTR_PGA_ENA */
#define WM8962_SPKOUTR_PGA_ENA_SHIFT                 3  /* SPKOUTR_PGA_ENA */
#define WM8962_SPKOUTR_PGA_ENA_WIDTH                 1  /* SPKOUTR_PGA_ENA */
#define WM8962_HPOUTL_PGA_MUTE                  0x0002  /* HPOUTL_PGA_MUTE */
#define WM8962_HPOUTL_PGA_MUTE_MASK             0x0002  /* HPOUTL_PGA_MUTE */
#define WM8962_HPOUTL_PGA_MUTE_SHIFT                 1  /* HPOUTL_PGA_MUTE */
#define WM8962_HPOUTL_PGA_MUTE_WIDTH                 1  /* HPOUTL_PGA_MUTE */
#define WM8962_HPOUTR_PGA_MUTE                  0x0001  /* HPOUTR_PGA_MUTE */
#define WM8962_HPOUTR_PGA_MUTE_MASK             0x0001  /* HPOUTR_PGA_MUTE */
#define WM8962_HPOUTR_PGA_MUTE_SHIFT                 0  /* HPOUTR_PGA_MUTE */
#define WM8962_HPOUTR_PGA_MUTE_WIDTH                 1  /* HPOUTR_PGA_MUTE */

/*
 * R25 (0x19) - Pwr Mgmt (1)
 */
#define WM8962_DMIC_ENA                         0x0400  /* DMIC_ENA */
#define WM8962_DMIC_ENA_MASK                    0x0400  /* DMIC_ENA */
#define WM8962_DMIC_ENA_SHIFT                       10  /* DMIC_ENA */
#define WM8962_DMIC_ENA_WIDTH                        1  /* DMIC_ENA */
#define WM8962_OPCLK_ENA                        0x0200  /* OPCLK_ENA */
#define WM8962_OPCLK_ENA_MASK                   0x0200  /* OPCLK_ENA */
#define WM8962_OPCLK_ENA_SHIFT                       9  /* OPCLK_ENA */
#define WM8962_OPCLK_ENA_WIDTH                       1  /* OPCLK_ENA */
#define WM8962_VMID_SEL_MASK                    0x0180  /* VMID_SEL - [8:7] */
#define WM8962_VMID_SEL_SHIFT                        7  /* VMID_SEL - [8:7] */
#define WM8962_VMID_SEL_WIDTH                        2  /* VMID_SEL - [8:7] */
#define WM8962_BIAS_ENA                         0x0040  /* BIAS_ENA */
#define WM8962_BIAS_ENA_MASK                    0x0040  /* BIAS_ENA */
#define WM8962_BIAS_ENA_SHIFT                        6  /* BIAS_ENA */
#define WM8962_BIAS_ENA_WIDTH                        1  /* BIAS_ENA */
#define WM8962_INL_ENA                          0x0020  /* INL_ENA */
#define WM8962_INL_ENA_MASK                     0x0020  /* INL_ENA */
#define WM8962_INL_ENA_SHIFT                         5  /* INL_ENA */
#define WM8962_INL_ENA_WIDTH                         1  /* INL_ENA */
#define WM8962_INR_ENA                          0x0010  /* INR_ENA */
#define WM8962_INR_ENA_MASK                     0x0010  /* INR_ENA */
#define WM8962_INR_ENA_SHIFT                         4  /* INR_ENA */
#define WM8962_INR_ENA_WIDTH                         1  /* INR_ENA */
#define WM8962_ADCL_ENA                         0x0008  /* ADCL_ENA */
#define WM8962_ADCL_ENA_MASK                    0x0008  /* ADCL_ENA */
#define WM8962_ADCL_ENA_SHIFT                        3  /* ADCL_ENA */
#define WM8962_ADCL_ENA_WIDTH                        1  /* ADCL_ENA */
#define WM8962_ADCR_ENA                         0x0004  /* ADCR_ENA */
#define WM8962_ADCR_ENA_MASK                    0x0004  /* ADCR_ENA */
#define WM8962_ADCR_ENA_SHIFT                        2  /* ADCR_ENA */
#define WM8962_ADCR_ENA_WIDTH                        1  /* ADCR_ENA */
#define WM8962_MICBIAS_ENA                      0x0002  /* MICBIAS_ENA */
#define WM8962_MICBIAS_ENA_MASK                 0x0002  /* MICBIAS_ENA */
#define WM8962_MICBIAS_ENA_SHIFT                     1  /* MICBIAS_ENA */
#define WM8962_MICBIAS_ENA_WIDTH                     1  /* MICBIAS_ENA */

/*
  * R38 (0x26) - Right input PGA control
  */
#define WM8962_INPGAR_ENA                       0x0010  /* INPGAR_ENA */
#define WM8962_INPGAR_ENA_MASK                  0x0010  /* INPGAR_ENA */
#define WM8962_INPGAR_ENA_SHIFT                      4  /* INPGAR_ENA */
#define WM8962_INPGAR_ENA_WIDTH                      1  /* INPGAR_ENA */
#define WM8962_IN1R_TO_INPGAR                   0x0008  /* IN1R_TO_INPGAR */
#define WM8962_IN1R_TO_INPGAR_MASK              0x0008  /* IN1R_TO_INPGAR */
#define WM8962_IN1R_TO_INPGAR_SHIFT                  3  /* IN1R_TO_INPGAR */
#define WM8962_IN1R_TO_INPGAR_WIDTH                  1  /* IN1R_TO_INPGAR */
#define WM8962_IN2R_TO_INPGAR                   0x0004  /* IN2R_TO_INPGAR */
#define WM8962_IN2R_TO_INPGAR_MASK              0x0004  /* IN2R_TO_INPGAR */
#define WM8962_IN2R_TO_INPGAR_SHIFT                  2  /* IN2R_TO_INPGAR */
#define WM8962_IN2R_TO_INPGAR_WIDTH                  1  /* IN2R_TO_INPGAR */
#define WM8962_IN3R_TO_INPGAR                   0x0002  /* IN3R_TO_INPGAR */
#define WM8962_IN3R_TO_INPGAR_MASK              0x0002  /* IN3R_TO_INPGAR */
#define WM8962_IN3R_TO_INPGAR_SHIFT                  1  /* IN3R_TO_INPGAR */
#define WM8962_IN3R_TO_INPGAR_WIDTH                  1  /* IN3R_TO_INPGAR */
#define WM8962_IN4R_TO_INPGAR                   0x0001  /* IN4R_TO_INPGAR */
#define WM8962_IN4R_TO_INPGAR_MASK              0x0001  /* IN4R_TO_INPGAR */
#define WM8962_IN4R_TO_INPGAR_SHIFT                  0  /* IN4R_TO_INPGAR */
#define WM8962_IN4R_TO_INPGAR_WIDTH                  1  /* IN4R_TO_INPGAR */

/*
 * R40 (0x28) - SPKOUTL volume
*/
#define WM8962_SPKOUT_VU                        0x0100  /* SPKOUT_VU */
#define WM8962_SPKOUT_VU_MASK                   0x0100  /* SPKOUT_VU */
#define WM8962_SPKOUT_VU_SHIFT                       8  /* SPKOUT_VU */
#define WM8962_SPKOUT_VU_WIDTH                       1  /* SPKOUT_VU */
#define WM8962_SPKOUTL_ZC                       0x0080  /* SPKOUTL_ZC */
#define WM8962_SPKOUTL_ZC_MASK                  0x0080  /* SPKOUTL_ZC */
#define WM8962_SPKOUTL_ZC_SHIFT                      7  /* SPKOUTL_ZC */
#define WM8962_SPKOUTL_ZC_WIDTH                      1  /* SPKOUTL_ZC */
#define WM8962_SPKOUTL_VOL_MASK                 0x007F  /* SPKOUTL_VOL - [6:0] */
#define WM8962_SPKOUTL_VOL_SHIFT                     0  /* SPKOUTL_VOL - [6:0] */
#define WM8962_SPKOUTL_VOL_WIDTH                     7  /* SPKOUTL_VOL - [6:0] */

/*
 * R41 (0x29) - SPKOUTR volume
*/
#define WM8962_SPKOUTR_ZC                       0x0080  /* SPKOUTR_ZC */
#define WM8962_SPKOUTR_ZC_MASK                  0x0080  /* SPKOUTR_ZC */
#define WM8962_SPKOUTR_ZC_SHIFT                      7  /* SPKOUTR_ZC */
#define WM8962_SPKOUTR_ZC_WIDTH                      1  /* SPKOUTR_ZC */
#define WM8962_SPKOUTR_VOL_MASK                 0x007F  /* SPKOUTR_VOL - [6:0] */
#define WM8962_SPKOUTR_VOL_SHIFT                     0  /* SPKOUTR_VOL - [6:0] */
#define WM8962_SPKOUTR_VOL_WIDTH                     7  /* SPKOUTR_VOL - [6:0] */

/*
 * R49 (0x31) - Class D Control 1
  */
#define WM8962_SPKOUTR_ENA                      0x0080  /* SPKOUTR_ENA */
#define WM8962_SPKOUTR_ENA_MASK                 0x0080  /* SPKOUTR_ENA */
#define WM8962_SPKOUTR_ENA_SHIFT                     7  /* SPKOUTR_ENA */
#define WM8962_SPKOUTR_ENA_WIDTH                     1  /* SPKOUTR_ENA */
#define WM8962_SPKOUTL_ENA                      0x0040  /* SPKOUTL_ENA */
#define WM8962_SPKOUTL_ENA_MASK                 0x0040  /* SPKOUTL_ENA */
#define WM8962_SPKOUTL_ENA_SHIFT                     6  /* SPKOUTL_ENA */
#define WM8962_SPKOUTL_ENA_WIDTH                     1  /* SPKOUTL_ENA */
#define WM8962_SPKOUTL_PGA_MUTE                 0x0002  /* SPKOUTL_PGA_MUTE */
#define WM8962_SPKOUTL_PGA_MUTE_MASK            0x0002  /* SPKOUTL_PGA_MUTE */
#define WM8962_SPKOUTL_PGA_MUTE_SHIFT                1  /* SPKOUTL_PGA_MUTE */
#define WM8962_SPKOUTL_PGA_MUTE_WIDTH                1  /* SPKOUTL_PGA_MUTE */
#define WM8962_SPKOUTR_PGA_MUTE                 0x0001  /* SPKOUTR_PGA_MUTE */
#define WM8962_SPKOUTR_PGA_MUTE_MASK            0x0001  /* SPKOUTR_PGA_MUTE */
#define WM8962_SPKOUTR_PGA_MUTE_SHIFT                0  /* SPKOUTR_PGA_MUTE */
#define WM8962_SPKOUTR_PGA_MUTE_WIDTH                1  /* SPKOUTR_PGA_MUTE */

/*
 * R51 (0x33) - Class D Control 2
 */
#define WM8962_SPK_MONO                         0x0040  /* SPK_MONO */
#define WM8962_SPK_MONO_MASK                    0x0040  /* SPK_MONO */
#define WM8962_SPK_MONO_SHIFT                        6  /* SPK_MONO */
#define WM8962_SPK_MONO_WIDTH                        1  /* SPK_MONO */
#define WM8962_CLASSD_VOL_MASK                  0x0007  /* CLASSD_VOL - [2:0] */
#define WM8962_CLASSD_VOL_SHIFT                      0  /* CLASSD_VOL - [2:0] */
#define WM8962_CLASSD_VOL_WIDTH                      3  /* CLASSD_VOL - [2:0] */

/*
  * R99 (0x63) - Mixer Enables
  */
#define WM8962_HPMIXL_ENA                       0x0008  /* HPMIXL_ENA */
#define WM8962_HPMIXL_ENA_MASK                  0x0008  /* HPMIXL_ENA */
#define WM8962_HPMIXL_ENA_SHIFT                      3  /* HPMIXL_ENA */
#define WM8962_HPMIXL_ENA_WIDTH                      1  /* HPMIXL_ENA */
#define WM8962_HPMIXR_ENA                       0x0004  /* HPMIXR_ENA */
#define WM8962_HPMIXR_ENA_MASK                  0x0004  /* HPMIXR_ENA */
#define WM8962_HPMIXR_ENA_SHIFT                      2  /* HPMIXR_ENA */
#define WM8962_HPMIXR_ENA_WIDTH                      1  /* HPMIXR_ENA */
#define WM8962_SPKMIXL_ENA                      0x0002  /* SPKMIXL_ENA */
#define WM8962_SPKMIXL_ENA_MASK                 0x0002  /* SPKMIXL_ENA */
#define WM8962_SPKMIXL_ENA_SHIFT                     1  /* SPKMIXL_ENA */
#define WM8962_SPKMIXL_ENA_WIDTH                     1  /* SPKMIXL_ENA */
#define WM8962_SPKMIXR_ENA                      0x0001  /* SPKMIXR_ENA */
#define WM8962_SPKMIXR_ENA_MASK                 0x0001  /* SPKMIXR_ENA */
#define WM8962_SPKMIXR_ENA_SHIFT                     0  /* SPKMIXR_ENA */
#define WM8962_SPKMIXR_ENA_WIDTH                     1  /* SPKMIXR_ENA */

/*
  * R105 (0x69) - Speaker Mixer (1)
  */
#define WM8962_SPKMIXL_TO_SPKOUTL_PGA           0x0080  /* SPKMIXL_TO_SPKOUTL_PGA */
#define WM8962_SPKMIXL_TO_SPKOUTL_PGA_MASK      0x0080  /* SPKMIXL_TO_SPKOUTL_PGA */
#define WM8962_SPKMIXL_TO_SPKOUTL_PGA_SHIFT          7  /* SPKMIXL_TO_SPKOUTL_PGA */
#define WM8962_SPKMIXL_TO_SPKOUTL_PGA_WIDTH          1  /* SPKMIXL_TO_SPKOUTL_PGA */
#define WM8962_DACL_TO_SPKMIXL                  0x0020  /* DACL_TO_SPKMIXL */
#define WM8962_DACL_TO_SPKMIXL_MASK             0x0020  /* DACL_TO_SPKMIXL */
#define WM8962_DACL_TO_SPKMIXL_SHIFT                 5  /* DACL_TO_SPKMIXL */
#define WM8962_DACL_TO_SPKMIXL_WIDTH                 1  /* DACL_TO_SPKMIXL */
#define WM8962_DACR_TO_SPKMIXL                  0x0010  /* DACR_TO_SPKMIXL */
#define WM8962_DACR_TO_SPKMIXL_MASK             0x0010  /* DACR_TO_SPKMIXL */
#define WM8962_DACR_TO_SPKMIXL_SHIFT                 4  /* DACR_TO_SPKMIXL */
#define WM8962_DACR_TO_SPKMIXL_WIDTH                 1  /* DACR_TO_SPKMIXL */
#define WM8962_MIXINL_TO_SPKMIXL                0x0008  /* MIXINL_TO_SPKMIXL */
#define WM8962_MIXINL_TO_SPKMIXL_MASK           0x0008  /* MIXINL_TO_SPKMIXL */
#define WM8962_MIXINL_TO_SPKMIXL_SHIFT               3  /* MIXINL_TO_SPKMIXL */
#define WM8962_MIXINL_TO_SPKMIXL_WIDTH               1  /* MIXINL_TO_SPKMIXL */
#define WM8962_MIXINR_TO_SPKMIXL                0x0004  /* MIXINR_TO_SPKMIXL */
#define WM8962_MIXINR_TO_SPKMIXL_MASK           0x0004  /* MIXINR_TO_SPKMIXL */
#define WM8962_MIXINR_TO_SPKMIXL_SHIFT               2  /* MIXINR_TO_SPKMIXL */
#define WM8962_MIXINR_TO_SPKMIXL_WIDTH               1  /* MIXINR_TO_SPKMIXL */
#define WM8962_IN4L_TO_SPKMIXL                  0x0002  /* IN4L_TO_SPKMIXL */
#define WM8962_IN4L_TO_SPKMIXL_MASK             0x0002  /* IN4L_TO_SPKMIXL */
#define WM8962_IN4L_TO_SPKMIXL_SHIFT                 1  /* IN4L_TO_SPKMIXL */
#define WM8962_IN4L_TO_SPKMIXL_WIDTH                 1  /* IN4L_TO_SPKMIXL */
#define WM8962_IN4R_TO_SPKMIXL                  0x0001  /* IN4R_TO_SPKMIXL */
#define WM8962_IN4R_TO_SPKMIXL_MASK             0x0001  /* IN4R_TO_SPKMIXL */
#define WM8962_IN4R_TO_SPKMIXL_SHIFT                 0  /* IN4R_TO_SPKMIXL */
#define WM8962_IN4R_TO_SPKMIXL_WIDTH                 1  /* IN4R_TO_SPKMIXL */

 /*
  * R106 (0x6A) - Speaker Mixer (2)
  */
#define WM8962_SPKMIXR_TO_SPKOUTR_PGA           0x0080  /* SPKMIXR_TO_SPKOUTR_PGA */
#define WM8962_SPKMIXR_TO_SPKOUTR_PGA_MASK      0x0080  /* SPKMIXR_TO_SPKOUTR_PGA */
#define WM8962_SPKMIXR_TO_SPKOUTR_PGA_SHIFT          7  /* SPKMIXR_TO_SPKOUTR_PGA */
#define WM8962_SPKMIXR_TO_SPKOUTR_PGA_WIDTH          1  /* SPKMIXR_TO_SPKOUTR_PGA */
#define WM8962_DACL_TO_SPKMIXR                  0x0020  /* DACL_TO_SPKMIXR */
#define WM8962_DACL_TO_SPKMIXR_MASK             0x0020  /* DACL_TO_SPKMIXR */
#define WM8962_DACL_TO_SPKMIXR_SHIFT                 5  /* DACL_TO_SPKMIXR */
#define WM8962_DACL_TO_SPKMIXR_WIDTH                 1  /* DACL_TO_SPKMIXR */
#define WM8962_DACR_TO_SPKMIXR                  0x0010  /* DACR_TO_SPKMIXR */
#define WM8962_DACR_TO_SPKMIXR_MASK             0x0010  /* DACR_TO_SPKMIXR */
#define WM8962_DACR_TO_SPKMIXR_SHIFT                 4  /* DACR_TO_SPKMIXR */
#define WM8962_DACR_TO_SPKMIXR_WIDTH                 1  /* DACR_TO_SPKMIXR */
#define WM8962_MIXINL_TO_SPKMIXR                0x0008  /* MIXINL_TO_SPKMIXR */
#define WM8962_MIXINL_TO_SPKMIXR_MASK           0x0008  /* MIXINL_TO_SPKMIXR */
#define WM8962_MIXINL_TO_SPKMIXR_SHIFT               3  /* MIXINL_TO_SPKMIXR */
#define WM8962_MIXINL_TO_SPKMIXR_WIDTH               1  /* MIXINL_TO_SPKMIXR */
#define WM8962_MIXINR_TO_SPKMIXR                0x0004  /* MIXINR_TO_SPKMIXR */
#define WM8962_MIXINR_TO_SPKMIXR_MASK           0x0004  /* MIXINR_TO_SPKMIXR */
#define WM8962_MIXINR_TO_SPKMIXR_SHIFT               2  /* MIXINR_TO_SPKMIXR */
#define WM8962_MIXINR_TO_SPKMIXR_WIDTH               1  /* MIXINR_TO_SPKMIXR */
#define WM8962_IN4L_TO_SPKMIXR                  0x0002  /* IN4L_TO_SPKMIXR */
#define WM8962_IN4L_TO_SPKMIXR_MASK             0x0002  /* IN4L_TO_SPKMIXR */
#define WM8962_IN4L_TO_SPKMIXR_SHIFT                 1  /* IN4L_TO_SPKMIXR */
#define WM8962_IN4L_TO_SPKMIXR_WIDTH                 1  /* IN4L_TO_SPKMIXR */
#define WM8962_IN4R_TO_SPKMIXR                  0x0001  /* IN4R_TO_SPKMIXR */
#define WM8962_IN4R_TO_SPKMIXR_MASK             0x0001  /* IN4R_TO_SPKMIXR */
#define WM8962_IN4R_TO_SPKMIXR_SHIFT                 0  /* IN4R_TO_SPKMIXR */
#define WM8962_IN4R_TO_SPKMIXR_WIDTH                 1  /* IN4R_TO_SPKMIXR */

 /*
  * R107 (0x6B) - Speaker Mixer (3)
  */
#define WM8962_SPKMIXL_MUTE                     0x0100  /* SPKMIXL_MUTE */
#define WM8962_SPKMIXL_MUTE_MASK                0x0100  /* SPKMIXL_MUTE */
#define WM8962_SPKMIXL_MUTE_SHIFT                    8  /* SPKMIXL_MUTE */
#define WM8962_SPKMIXL_MUTE_WIDTH                    1  /* SPKMIXL_MUTE */
#define WM8962_MIXINL_SPKMIXL_VOL               0x0080  /* MIXINL_SPKMIXL_VOL */
#define WM8962_MIXINL_SPKMIXL_VOL_MASK          0x0080  /* MIXINL_SPKMIXL_VOL */
#define WM8962_MIXINL_SPKMIXL_VOL_SHIFT              7  /* MIXINL_SPKMIXL_VOL */
#define WM8962_MIXINL_SPKMIXL_VOL_WIDTH              1  /* MIXINL_SPKMIXL_VOL */
#define WM8962_MIXINR_SPKMIXL_VOL               0x0040  /* MIXINR_SPKMIXL_VOL */
#define WM8962_MIXINR_SPKMIXL_VOL_MASK          0x0040  /* MIXINR_SPKMIXL_VOL */
#define WM8962_MIXINR_SPKMIXL_VOL_SHIFT              6  /* MIXINR_SPKMIXL_VOL */
#define WM8962_MIXINR_SPKMIXL_VOL_WIDTH              1  /* MIXINR_SPKMIXL_VOL */
#define WM8962_IN4L_SPKMIXL_VOL_MASK            0x0038  /* IN4L_SPKMIXL_VOL - [5:3] */
#define WM8962_IN4L_SPKMIXL_VOL_SHIFT                3  /* IN4L_SPKMIXL_VOL - [5:3] */
#define WM8962_IN4L_SPKMIXL_VOL_WIDTH                3  /* IN4L_SPKMIXL_VOL - [5:3] */
#define WM8962_IN4R_SPKMIXL_VOL_MASK            0x0007  /* IN4R_SPKMIXL_VOL - [2:0] */
#define WM8962_IN4R_SPKMIXL_VOL_SHIFT                0  /* IN4R_SPKMIXL_VOL - [2:0] */
#define WM8962_IN4R_SPKMIXL_VOL_WIDTH                3  /* IN4R_SPKMIXL_VOL - [2:0] */

 /*
  * R108 (0x6C) - Speaker Mixer (4)
  */
#define WM8962_SPKMIXR_MUTE                     0x0100  /* SPKMIXR_MUTE */
#define WM8962_SPKMIXR_MUTE_MASK                0x0100  /* SPKMIXR_MUTE */
#define WM8962_SPKMIXR_MUTE_SHIFT                    8  /* SPKMIXR_MUTE */
#define WM8962_SPKMIXR_MUTE_WIDTH                    1  /* SPKMIXR_MUTE */
#define WM8962_MIXINL_SPKMIXR_VOL               0x0080  /* MIXINL_SPKMIXR_VOL */
#define WM8962_MIXINL_SPKMIXR_VOL_MASK          0x0080  /* MIXINL_SPKMIXR_VOL */
#define WM8962_MIXINL_SPKMIXR_VOL_SHIFT              7  /* MIXINL_SPKMIXR_VOL */
#define WM8962_MIXINL_SPKMIXR_VOL_WIDTH              1  /* MIXINL_SPKMIXR_VOL */
#define WM8962_MIXINR_SPKMIXR_VOL               0x0040  /* MIXINR_SPKMIXR_VOL */
#define WM8962_MIXINR_SPKMIXR_VOL_MASK          0x0040  /* MIXINR_SPKMIXR_VOL */
#define WM8962_MIXINR_SPKMIXR_VOL_SHIFT              6  /* MIXINR_SPKMIXR_VOL */
#define WM8962_MIXINR_SPKMIXR_VOL_WIDTH              1  /* MIXINR_SPKMIXR_VOL */
#define WM8962_IN4L_SPKMIXR_VOL_MASK            0x0038  /* IN4L_SPKMIXR_VOL - [5:3] */
#define WM8962_IN4L_SPKMIXR_VOL_SHIFT                3  /* IN4L_SPKMIXR_VOL - [5:3] */
#define WM8962_IN4L_SPKMIXR_VOL_WIDTH                3  /* IN4L_SPKMIXR_VOL - [5:3] */
#define WM8962_IN4R_SPKMIXR_VOL_MASK            0x0007  /* IN4R_SPKMIXR_VOL - [2:0] */
#define WM8962_IN4R_SPKMIXR_VOL_SHIFT                0  /* IN4R_SPKMIXR_VOL - [2:0] */
#define WM8962_IN4R_SPKMIXR_VOL_WIDTH                3  /* IN4R_SPKMIXR_VOL - [2:0] */

 /*
  * R109 (0x6D) - Speaker Mixer (5)
  */
#define WM8962_DACL_SPKMIXL_VOL                 0x0080  /* DACL_SPKMIXL_VOL */
#define WM8962_DACL_SPKMIXL_VOL_MASK            0x0080  /* DACL_SPKMIXL_VOL */
#define WM8962_DACL_SPKMIXL_VOL_SHIFT                7  /* DACL_SPKMIXL_VOL */
#define WM8962_DACL_SPKMIXL_VOL_WIDTH                1  /* DACL_SPKMIXL_VOL */
#define WM8962_DACR_SPKMIXL_VOL                 0x0040  /* DACR_SPKMIXL_VOL */
#define WM8962_DACR_SPKMIXL_VOL_MASK            0x0040  /* DACR_SPKMIXL_VOL */
#define WM8962_DACR_SPKMIXL_VOL_SHIFT                6  /* DACR_SPKMIXL_VOL */
#define WM8962_DACR_SPKMIXL_VOL_WIDTH                1  /* DACR_SPKMIXL_VOL */
#define WM8962_DACL_SPKMIXR_VOL                 0x0020  /* DACL_SPKMIXR_VOL */
#define WM8962_DACL_SPKMIXR_VOL_MASK            0x0020  /* DACL_SPKMIXR_VOL */
#define WM8962_DACL_SPKMIXR_VOL_SHIFT                5  /* DACL_SPKMIXR_VOL */
#define WM8962_DACL_SPKMIXR_VOL_WIDTH                1  /* DACL_SPKMIXR_VOL */
#define WM8962_DACR_SPKMIXR_VOL                 0x0010  /* DACR_SPKMIXR_VOL */
#define WM8962_DACR_SPKMIXR_VOL_MASK            0x0010  /* DACR_SPKMIXR_VOL */
#define WM8962_DACR_SPKMIXR_VOL_SHIFT                4  /* DACR_SPKMIXR_VOL */
#define WM8962_DACR_SPKMIXR_VOL_WIDTH                1  /* DACR_SPKMIXR_VOL */

#endif /* __MIXER_H */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/deva/ctrl/mx/nto/arm/dll.le.v7.mx6_wm8962/wm8962.h $ $Rev: 753817 $")
#endif
