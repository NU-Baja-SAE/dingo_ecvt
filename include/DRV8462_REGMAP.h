/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*******************************************************************************
 *
 *  drv8xx2-RegMap.h - Declaration file for utility functions and global variables
 *  DRV84x2_DRV82x2_DRV89x2EVM_FIRMWARE
 *  02/10/2022
 *
 ******************************************************************************/
#ifndef DRV84x2_REGMAP_H_
#define DRV84x2_REGMAP_H_

#include <Arduino.h>

// MACROs

// SPI Protocol
#define SPI_ADDRESS_MASK 0x3F00 // Mask for SPI register address bits
#define SPI_ADDRESS_POS 8       // Position for SPI register address bits
#define SPI_DATA_MASK 0x00FF    // Mask for SPI register data bits
#define SPI_DATA_POS 0          // Position for SPI register data bits
#define SPI_RW_BIT_MASK 0x4000  // Mask for SPI register read write indication bit
#define SPI_CLK 1000000         // 1 MHz

// DRV8xx2 SPI Register Addresses
#define SPI_FAULT                               (uint8_t)(0x00)         // FAULT Register
#define SPI_DIAG1                               (uint8_t)(0x01)         // DIAG1 Register
#define SPI_DIAG2                               (uint8_t)(0x02)         // DIAG2 Register
#define SPI_DIAG3                               (uint8_t)(0x03)         // DIAG3 Register
#define SPI_CTRL1                               (uint8_t)(0x04)         // CTRL1 Register
#define SPI_CTRL2                               (uint8_t)(0x05)         // CTRL2 Register
#define SPI_CTRL3                               (uint8_t)(0x06)         // CTRL3 Register
#define SPI_CTRL4                               (uint8_t)(0x07)         // CTRL4 Register
#define SPI_CTRL5                               (uint8_t)(0x08)         // CTRL5 Register
#define SPI_CTRL6                               (uint8_t)(0x09)         // CTRL6 Register
#define SPI_CTRL7                               (uint8_t)(0x0A)         // CTRL7 Register
#define SPI_CTRL8                               (uint8_t)(0x0B)         // CTRL8 Register
#define SPI_CTRL9                               (uint8_t)(0x0C)         // CTRL9 Register
#define SPI_CTRL10                              (uint8_t)(0x0D)         // CTRL10 Register
#define SPI_CTRL11                              (uint8_t)(0x0E)         // CTRL11 Register
#define SPI_CTRL12                              (uint8_t)(0x0F)         // CTRL12 Register
#define SPI_CTRL13                              (uint8_t)(0x10)         // CTRL13 Register
#define SPI_INDEX1                              (uint8_t)(0x11)         // INDEX1 Register
#define SPI_INDEX2                              (uint8_t)(0x12)         // INDEX2 Register
#define SPI_INDEX3                              (uint8_t)(0x13)         // INDEX3 Register
#define SPI_INDEX4                              (uint8_t)(0x14)         // INDEX4 Register
#define SPI_INDEX5                              (uint8_t)(0x15)         // INDEX5 Register
#define SPI_CUSTOM_CTRL1                        (uint8_t)(0x16)         // CUSTOM_CTRL1 Register
#define SPI_CUSTOM_CTRL2                        (uint8_t)(0x17)         // CUSTOM_CTRL2 Register
#define SPI_CUSTOM_CTRL3                        (uint8_t)(0x18)         // CUSTOM_CTRL3 Register
#define SPI_CUSTOM_CTRL4                        (uint8_t)(0x19)         // CUSTOM_CTRL4 Register
#define SPI_CUSTOM_CTRL5                        (uint8_t)(0x1A)         // CUSTOM_CTRL5 Register
#define SPI_CUSTOM_CTRL6                        (uint8_t)(0x1B)         // CUSTOM_CTRL6 Register
#define SPI_CUSTOM_CTRL7                        (uint8_t)(0x1C)         // CUSTOM_CTRL7 Register
#define SPI_CUSTOM_CTRL8                        (uint8_t)(0x1D)         // CUSTOM_CTRL8 Register
#define SPI_CUSTOM_CTRL9                        (uint8_t)(0x1E)         // CUSTOM_CTRL9 Register
#define SPI_ATQ_CTRL1                           (uint8_t)(0x1F)         // ATQ_CTRL1 Register
#define SPI_ATQ_CTRL2                           (uint8_t)(0x20)         // ATQ_CTRL2 Register
#define SPI_ATQ_CTRL3                           (uint8_t)(0x21)         // ATQ_CTRL3 Register
#define SPI_ATQ_CTRL4                           (uint8_t)(0x22)         // ATQ_CTRL4 Register
#define SPI_ATQ_CTRL5                           (uint8_t)(0x23)         // ATQ_CTRL5 Register
#define SPI_ATQ_CTRL6                           (uint8_t)(0x24)         // ATQ_CTRL6 Register
#define SPI_ATQ_CTRL7                           (uint8_t)(0x25)         // ATQ_CTRL7 Register
#define SPI_ATQ_CTRL8                           (uint8_t)(0x26)         // ATQ_CTRL8 Register
#define SPI_ATQ_CTRL9                           (uint8_t)(0x27)         // ATQ_CTRL9 Register
#define SPI_ATQ_CTRL10                          (uint8_t)(0x28)         // ATQ_CTRL10 Register
#define SPI_ATQ_CTRL11                          (uint8_t)(0x29)         // ATQ_CTRL11 Register
#define SPI_ATQ_CTRL12                          (uint8_t)(0x2A)         // ATQ_CTRL12 Register
#define SPI_ATQ_CTRL13                          (uint8_t)(0x2B)         // ATQ_CTRL13 Register
#define SPI_ATQ_CTRL14                          (uint8_t)(0x2C)         // ATQ_CTRL14 Register
#define SPI_ATQ_CTRL15                          (uint8_t)(0x2D)         // ATQ_CTRL15 Register
#define SPI_ATQ_CTRL16                          (uint8_t)(0x2E)         // ATQ_CTRL16 Register
#define SPI_ATQ_CTRL17                          (uint8_t)(0x2F)         // ATQ_CTRL17 Register
#define SPI_ATQ_CTRL18                          (uint8_t)(0x30)         // ATQ_CTRL18 Register
#define SPI_SILENTSTEP_CTRL1                    (uint8_t)(0x31)         // SILENTSTEP_CTRL1 Register
#define SPI_SILENTSTEP_CTRL2                    (uint8_t)(0x32)         // SILENTSTEP_CTRL2 Register
#define SPI_SILENTSTEP_CTRL3                    (uint8_t)(0x33)         // SILENTSTEP_CTRL3 Register
#define SPI_SILENTSTEP_CTRL4                    (uint8_t)(0x34)         // SILENTSTEP_CTRL4 Register
#define SPI_SILENTSTEP_CTRL5                    (uint8_t)(0x35)         // SILENTSTEP_CTRL5 Register
#define SPI_CTRL14                              (uint8_t)(0x3C)         // CTRL14 Register

// Bit Mask definitions for Registers

// Register 0x00: FAULT Register
#define OL_MASK               (0x01)         // Indicates open load condition
#define OT_MASK               (0x02)         // Logic OR of OTW and OTSD
#define STL_MASK              (0x04)         // Indicates motor stall
#define OCP_MASK              (0x08)         // Indicates over current fault condition
#define CPUV_MASK             (0x10)         // Indicates charge pump under voltage fault condition
#define UVLO_MASK             (0x20)         // Indicates an supply under voltage lockout fault condition
#define SPI_ERROR_MASK        (0x40)         // Indication SPI communication error
#define FAULT_MASK            (0x80)         // Inverse value of the nFAULT pin

// Register 0x01 : DIAG1 Register
#define OCP_HS1_A_MASK        (0x01)        // Indicates over current fault on the high-side FET of AOUT1
#define OCP_LS1_A_MASK        (0x02)        // Indicates over current fault on the low-side FET of AOUT1
#define OCP_HS2_A_MASK        (0x04)        // Indicates over current fault on the high-side FET of AOUT2
#define OCP_LS2_A_MASK        (0x08)        // Indicates over current fault on the low-side FET of AOUT2
#define OCP_HS1_B_MASK        (0x10)        // Indicates over current fault on the high-side FET of BOUT1
#define OCP_LS1_B_MASK        (0x20)        // Indicates over current fault on the low-side FET of BOUT1
#define OCP_HS2_B_MASK        (0x40)        // Indicates over current fault on the high-side FET of BOUT2
#define OCP_LS2_B_MASK        (0x80)        // Indicates over current fault on the low-side FET of BOUT2

// Register 0x02 : DIAG2 Register
#define OL_A_MASK             (0x01)        // Indicates open-load detection on AOUT
#define OL_B_MASK             (0x02)        // Indicates open-load detection on BOUT
#define ATQ_LRN_DONE_MASK     (0x04)        // Indicates auto torque learning was successful
#define STALL_MASK            (0x08)        // Indicates motor stall condition
#define STL_LRN_OK_MASK       (0x10)        // Indicates stall detection learning was successful
#define OTS_MASK              (0x20)        // Indicates over temperature shutdown
#define OTW_MASK              (0x40)        // Indicates over temperature warning
#define STSL_MASK             (0x80)        // Indicates motor standstill

// Register 0x03 : DIAG3 Register
#define RSVD0_MASK            (0x01)        // Reserved bit
#define RSVD1_MASK            (0x02)        // Reserved bit
#define NPOR_MASK             (0x04)        // Indicates a VCC UVLO event
#define SILENTSTEP_ERROR_MASK (0x08)        // Indicates Silent Step operation error
#define ATQ_CNT_UFLW_MASK     (0x10)        // Indicates ATQ_CNT is less than ATQ_LL
#define ATQ_CNT_OFLW_MASK     (0x20)        // Indicates ATQ_CNT is more than ATQ_UL
#define NHOME_MASK            (0x40)        // Indicates indexer is at home position of step table
#define SILENTSTEP_ON_MASK    (0x80)        // Indicates that device is working with silentstep decay mode

// Register 0x04 : CTRL1 Register
#define DECAY_MASK            (0x07)        // Bridge decay setting
#define TOFF_MASK             (0x18)        // Current regulation TOFF setting
#define RSVD2_MASK            (0x20)        // Reserved bit
#define SR_MASK               (0x40)        // Output driver rise and fall time selection
#define EN_OUT_MASK           (0x80)        // Hi-Z outputs bit OR-ed with DRVOFF

// Register 0x05 : CTRL2 Register
#define MICROSTEP_MODE_MASK   (0x0F)        // Microstep setting
#define SPI_STEP_MASK         (0x10)        // Enable SPI step control mode
#define SPI_DIR_MASK          (0x20)        // Enable SPI direction control mode
#define STEP_MASK             (0x40)        // Step control bit if SPI_STEP is enabled
#define DIR_MASK              (0x80)        // Direction control bit if SPI_DIR is enabled

// Register 0x06 : CTRL3 Register
#define TW_REP_MASK           (0x01)        // Report OTW on nFAULT
#define OTSD_MODE_MASK        (0x02)        // OTSD latch fault setting
#define OCP_MODE_MASK         (0x04)        // OCP latch fault setting
#define TOCP_MASK             (0x08)        // OCP deglitch time setting
#define LOCK_MASK             (0x70)        // Lock SPI registers
#define CLR_FLT_MASK          (0x80)        // Clear all fault bits

// Register 0x07 : CTRL4 Register
#define STEP_FREQ_TOL_MASK    (0x03)        // Programs the filter setting on the STEP frequency input.
#define RSVD3_MASK            (0x04)        // Reserved bit
#define STL_REP_MASK          (0x08)        // Report stall detection on nFAULT
#define EN_STL_MASK           (0x10)        // Enable stall detection
#define DIS_STL_MASK          (0xEF)        // Disable stall detection - clears EN_STL bit
#define STL_LRN_MASK          (0x20)        // Learn stall count threshold
#define TBLANK_TIME_MASK      (0x80)        // Controls the current sense blanking time

// Register 0x08 : CTRL5 Register
#define STALL_TH_MASK         (0xFF)        // Stall Threshold Lower 8-bits

// Register 0x09 : CTRL6 Register
#define STALL_TH_MSB_MASK     (0x0F)        // Stall Threshold Upper 4-bits
#define TRQ_SCALE_MASK        (0x80)        // Torque scaling 0: x1 and 1: x8
#define DIS_SSC_MASK          (0x40)        // Spread-spectrum disable for CP and OSC
#define RC_RIPPLE             (0xC0)        // Controls the current ripple in smart tune ripple control decay mode

// Register 0x0A : CTRL7 Register
#define TRQ_COUNT_MASK        (0xFF)        // Torque Count Lower 8-bits

// Register 0x0B : CTRL8 Register
#define TRQ_COUNT_MASK_MSB    (0x0F)        // Torque Count Upper 4-bits
#define REV_ID_MASK           (0xF0)        // Silicon Revision ID

// Register 0x0C : CTRL9 Register
#define OLD_MASK              (0x80)        // Open Load Detection Enable

// Register 0x10 : CTRL13 Register
#define VREF_MASK             (0x02)        // Voltage Reference Setting


#endif // DRV8462_REGMAP_H_
