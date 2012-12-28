/* vim: set noet ts=8 sts=8 sw=8 : */

/*
 * Copyright © 2010 Saleem Abdulrasool <compnerd@compnerd.org>.
 * Copyright © 2010-2011 Genesi USA, Inc. <matt@genesi-usa.com>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SIIHDMI_H
#define SIIHDMI_H

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>

#include <linux/ioport.h>

/* TPI registers */
#define SIIHDMI_TPI_REG_VIDEO_MODE_DATA_BASE		(0x00)
#define SIIHDMI_TPI_REG_PIXEL_CLOCK_LSB			(0x00)
#define SIIHDMI_TPI_REG_PIXEL_CLOCK_MSB			(0x01)
#define SIIHDMI_TPI_REG_VFREQ_LSB			(0x02)
#define SIIHDMI_TPI_REG_VFREQ_MSB			(0x03)
#define SIIHDMI_TPI_REG_PIXELS_LSB			(0x04)
#define SIIHDMI_TPI_REG_PIXELS_MSB			(0x05)
#define SIIHDMI_TPI_REG_LINES_LSB			(0x06)
#define SIIHDMI_TPI_REG_LINES_MSB			(0x07)
#define SIIHDMI_TPI_REG_INPUT_BUS_PIXEL_REPETITION	(0x08)
#define SIIHDMI_TPI_REG_AVI_INPUT_FORMAT		(0x09)
#define SIIHDMI_TPI_REG_AVI_OUTPUT_FORMAT		(0x0a)
#define SIIHDMI_TPI_REG_YC_INPUT_MODE			(0x0b)
#define SIIHDMI_TPI_REG_AVI_DBYTE0			(0x0c)
#define SIIHDMI_TPI_REG_AVI_DBYTE1			(0x0d)
#define SIIHDMI_TPI_REG_AVI_DBYTE2			(0x0e)
#define SIIHDMI_TPI_REG_AVI_DBYTE3			(0x0f)
#define SIIHDMI_TPI_REG_AVI_DBYTE4			(0x10)
#define SIIHDMI_TPI_REG_AVI_DBYTE5			(0x11)
#define SIIHDMI_TPI_REG_AVI_INFO_END_TOP_BAR_LSB	(0x12)
#define SIIHDMI_TPI_REG_AVI_INFO_END_TOP_BAR_MSB	(0x13)
#define SIIHDMI_TPI_REG_AVI_INFO_START_BOTTOM_BAR_LSB	(0x14)
#define SIIHDMI_TPI_REG_AVI_INFO_START_BOTTOM_BAR_MSB	(0x15)
#define SIIHDMI_TPI_REG_AVI_INFO_END_LEFT_BAR_LSB	(0x16)
#define SIIHDMI_TPI_REG_AVI_INFO_END_LEFT_BAR_MSB	(0x17)
#define SIIHDMI_TPI_REG_AVI_INFO_END_RIGHT_BAR_LSB	(0x18)
#define SIIHDMI_TPI_REG_AVI_INFO_END_RIGHT_BAR_MSB	(0x19)
#define SIIHDMI_TPI_REG_SYS_CTRL			(0x1a)
#define SIIHDMI_TPI_REG_DEVICE_ID			(0x1b)
#define SIIHDMI_TPI_REG_DEVICE_REVISION			(0x1c)
#define SIIHDMI_TPI_REG_TPI_REVISION			(0x1d)
#define SIIHDMI_TPI_REG_PWR_STATE			(0x1e)
#define SIIHDMI_TPI_REG_I2S_ENABLE_MAPPING		(0x1f)
#define SIIHDMI_TPI_REG_I2S_INPUT_CONFIGURATION		(0x20)
#define SIIHDMI_TPI_REG_I2S_STREAM_HEADER_SETTINGS_BASE	(0x21)
#define SIIHDMI_TPI_REG_I2S_CHANNEL_STATUS		(0x21)
#define SIIHDMI_TPI_REG_I2S_CATEGORY_CODE		(0x22)
#define SIIHDMI_TPI_REG_I2S_SOURCE_CHANNEL		(0x23)
#define SIIHDMI_TPI_REG_I2S_ACCURACY_SAMPLING_FREQUENCY	(0x24)
#define SIIHDMI_TPI_REG_I2S_ORIGINAL_FREQ_SAMPLE_LENGTH	(0x25)
#define SIIHDMI_TPI_REG_I2S_AUDIO_CONFIGURATION_BASE	(0x26)
#define SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL	(0x26)
#define SIIHDMI_TPI_REG_I2S_AUDIO_SAMPLING_HBR		(0x27)
#define SIIHDMI_TPI_REG_I2S_AUDIO_RESERVED		(0x28)
#define SIIHDMI_TPI_REG_HDCP_QUERY_DATA			(0x29)
#define SIIHDMI_TPI_REG_HDCP_CONTROL_DATA		(0x2a)
#define SIIHDMI_TPI_REG_HDCP_CONTROL_DATA		(0x2a)
#define SIIHDMI_TPI_REG_HDCP_BKSV_1			(0x2b)
#define SIIHDMI_TPI_REG_HDCP_BKSV_2			(0x2c)
#define SIIHDMI_TPI_REG_HDCP_BKSV_3			(0x2d)
#define SIIHDMI_TPI_REG_HDCP_BKSV_4			(0x2e)
#define SIIHDMI_TPI_REG_HDCP_BKSV_5			(0x2f)
#define SIIHDMI_TPI_REG_HDCP_REVISION			(0x30)
#define SIIHDMI_TPI_REG_HDCP_KSV_V_STAR_VALUE		(0x31)
#define SIIHDMI_TPI_REG_HDCP_V_STAR_VALUE_1		(0x32)
#define SIIHDMI_TPI_REG_HDCP_V_STAR_VALUE_2		(0x33)
#define SIIHDMI_TPI_REG_HDCP_V_STAR_VALUE_3		(0x34)
#define SIIHDMI_TPI_REG_HDCP_V_STAR_VALUE_4		(0x35)
#define SIIHDMI_TPI_REG_HDCP_AKSV_1			(0x36)
#define SIIHDMI_TPI_REG_HDCP_AKSV_2			(0x37)
#define SIIHDMI_TPI_REG_HDCP_AKSV_3			(0x38)
#define SIIHDMI_TPI_REG_HDCP_AKSV_4			(0x39)
#define SIIHDMI_TPI_REG_HDCP_AKSV_5			(0x3a)

#define SIIHDMI_TPI_REG_IER				(0x3c)
#define SIIHDMI_TPI_REG_ISR				(0x3d)

#define SIIHDMI_INTERNAL_REG_SET_PAGE			(0xbc)
#define SIIHDMI_INTERNAL_REG_SET_OFFSET			(0xbd)
#define SIIHDMI_INTERNAL_REG_ACCESS			(0xbe)

#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_SELECT		(0xbf)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_TYPE		(0xc0)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_VERSION		(0xc1)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_LEN		(0xc2)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE0		(0xc3)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE1		(0xc4)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE2		(0xc5)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE3		(0xc6)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE4		(0xc7)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE5		(0xc8)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE6		(0xc9)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE7		(0xca)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE8		(0xcb)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE9		(0xcc)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE10		(0xcd)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE11		(0xce)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE12		(0xcf)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE13		(0xd0)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE14		(0xd1)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE15		(0xd2)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE16		(0xd3)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE17		(0xd4)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE18		(0xd5)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE19		(0xd6)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE20		(0xd7)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE21		(0xd8)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE22		(0xd9)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE23		(0xda)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE24		(0xdb)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE25		(0xdc)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE26		(0xdd)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE27		(0xde)

#define SIIHDMI_TPI_REG_RQB				(0xc7)


#define SIIHDMI_CPI_REG_DEVICE_ID			(0x80)
#define SIIHDMI_CPI_REG_CEC_VERSION			(0x81)
#define SIIHDMI_CPI_REG_CEC_SUBSYSTEM			(0x82)
#define SIIHDMI_CPI_REG_CEC_REVISION			(0x83)

#define SIIHDMI_CPI_REG_DEBUG_LOW_PERIOD		(0x84)
#define SIIHDMI_CPI_REG_DEBUG_LOW_DURATION		(0x85)
#define SIIHDMI_CPI_REG_DEBUG_SNOOP_OWNER		(0x86)
#define SIIHDMI_CPI_REG_DEBUG_FIFO_CONTROL		(0x87)

#define SIIHDMI_CPI_REG_CEC_TX_INIT			(0x88)
#define SIIHDMI_CPI_REG_CEC_TX_DEST			(0x89)

#define SIIHDMI_CPI_REG_CPI_CALIBRATION			(0x8e) // for 9022/9024 only!

#define SIIHDMI_CPI_REG_CEC_TX_OPCODE			(0x8f)
#define SIIHDMI_CPI_REG_CEC_TX_OPERAND0			(0x90)
#define SIIHDMI_CPI_REG_CEC_TX_CONTROL_1		(0x9f)
#define SIIHDMI_CPI_REG_CEC_TX_CONTROL_2		(0xa0)

#define SIIHDMI_CPI_REG_CEC_TX_CAPTURE_ID0		(0xa2)
#define SIIHDMI_CPI_REG_CEC_TX_CAPTURE_ID1		(0xa3)

#define SIIHDMI_CPI_REG_CEC_INT_ENABLE0			(0xa4)
#define SIIHDMI_CPI_REG_CEC_INT_ENABLE1			(0xa5)

#define SIIHDMI_CPI_REG_CEC_INT_STATUS0			(0xa6)
#define SIIHDMI_CPI_REG_CEC_INT_STATUS1			(0xa7)

#define SIIHDMI_CPI_REG_CEC_RX_CONTROL			(0xac)
#define SIIHDMI_CPI_REG_CEC_RX_COUNT			(0xad)
#define SIIHDMI_CPI_REG_CEC_RX_CMD_HEADER		(0xae)

#define SIIHDMI_CPI_REG_CEC_RX_OPCODE			(0xaf)
#define SIIHDMI_CPI_REG_CEC_RX_OPERAND0			(0xb0)

// 0xc0..0xdf
#define SIIHDMI_CPI_REG_CEC_ABORT_OPCODES0		(0xc0)

#define SIIHDMI_CPI_REG_CEC_DISCOVERY_CTRL		(0xe0)
#define SIIHDMI_CPI_REG_CEC_DISCOVERY_STATUS1		(0xe1)
#define SIIHDMI_CPI_REG_CEC_DISCOVERY_STATUS2		(0xe2)

#define SIIHDMI_CPI_REG_CDC_ARB				(0xe3)
#define SIIHDMI_CPI_REG_CDC_OPCODE			(0xe4)

enum cec_address {
	CEC_ADDRESS_TV,
	CEC_ADDRESS_RECORDING_1,
	CEC_ADDRESS_RECORDING_2,
	CEC_ADDRESS_TUNER_1,
	CEC_ADDRESS_PLAYBACK_1,
	CEC_ADDRESS_AUDIO_SYSTEM,
	CEC_ADDRESS_TUNER_2,
	CEC_ADDRESS_TUNER_3,
	CEC_ADDRESS_PLAYBACK_2,
	CEC_ADDRESS_RECORDING_3,
	CEC_ADDRESS_TUNER_4,
	CEC_ADDRESS_PLAYBACK_3,
	CEC_ADDRESS_RESERVED_1,
	CEC_ADDRESS_RESERVED_2,
	CEC_ADDRESS_FREE_USE,
	CEC_ADDRESS_BROADCAST_UNREGISTERED,
};


#define CEC_DEST_TV					(0)
#define CEC_DEST_RECORDING_DEVICE_1			(1)
#define CEC_DEST_RECORDING_DEVICE_2			(2)
#define CEC_DEST_TUNER_1				(3)



/* driver constants */
#define SIIHDMI_DEVICE_ID_902x				(0xb0)
#define SIIHDMI_BASE_TPI_REVISION			(0x29)
#define SIIHDMI_CTRL_INFO_FRAME_DRAIN_TIME		(0x80)
#define SIIHDMI_RESOLUTION_STABILIZE_TIME		(50)

#define SIIHDMI_HOTPLUG_HANDLER_TIMEOUT			(0x32)

#define SIIHDMI_VERSION_FLAG_VIRTUAL			(1 << 7)

/* Input Bus and Pixel Repetition */
#define SIIHDMI_PIXEL_REPETITION_DUAL			(1 << 0)
#define SIIHDMI_PIXEL_REPETITION_QUAD			(3 << 0)
#define SIIHDMI_INPUT_BUS_EDGE_SELECT_RISING		(1 << 4)
#define SIIHDMI_INPUT_BUS_SELECT_FULL_PIXEL_WIDTH	(1 << 5)
#define SIIHDMI_INPUT_BUS_TMDS_CLOCK_RATIO_1X		(1 << 6)
#define SIIHDMI_INPUT_BUS_TMDS_CLOCK_RATIO_2X		(2 << 6)
#define SIIHDMI_INPUT_BUS_TMDS_CLOCK_RATIO_4X		(3 << 6)

/* Input Format */
#define SIIHDMI_INPUT_COLOR_SPACE_RGB			(0 << 0)
#define SIIHDMI_INPUT_COLOR_SPACE_YUV_444		(1 << 0)
#define SIIHDMI_INPUT_COLOR_SPACE_YUV_422		(2 << 0)
#define SIIHDMI_INPUT_COLOR_SPACE_BLACK			(3 << 0)
#define SIIHDMI_INPUT_VIDEO_RANGE_EXPANSION_AUTO	(0 << 2)
#define SIIHDMI_INPUT_VIDEO_RANGE_EXPANSION_ON		(1 << 2)
#define SIIHDMI_INPUT_VIDEO_RANGE_EXPANSION_OFF		(2 << 2)
#define SIIHDMI_INPUT_COLOR_DEPTH_8BIT			(0 << 6)
#define SIIHDMI_INPUT_COLOR_DEPTH_10BIT			(2 << 6)
#define SIIHDMI_INPUT_COLOR_DEPTH_12BIT			(3 << 6)
#define SIIHDMI_INPUT_COLOR_DEPTH_16BIT			(1 << 6)

/* Output Format */
#define SIIHDMI_OUTPUT_FORMAT_HDMI_RGB			(0 << 0)
#define SIIHDMI_OUTPUT_FORMAT_HDMI_YUV_444		(1 << 0)
#define SIIHDMI_OUTPUT_FORMAT_HDMI_YUV_422		(2 << 0)
#define SIIHDMI_OUTPUT_FORMAT_DVI_RGB			(3 << 0)
#define SIIHDMI_OUTPUT_VIDEO_RANGE_COMPRESSION_AUTO	(0 << 2)
#define SIIHDMI_OUTPUT_VIDEO_RANGE_COMPRESSION_ON	(1 << 2)
#define SIIHDMI_OUTPUT_VIDEO_RANGE_COMPRESSION_OFF	(2 << 2)
#define SIIHDMI_OUTPUT_COLOR_STANDARD_BT601		(0 << 4)
#define SIIHDMI_OUTPUT_COLOR_STANDARD_BT709		(1 << 4)
#define SIIHDMI_OUTPUT_DITHERING			(1 << 5)
#define SIIHDMI_OUTPUT_COLOR_DEPTH_8BIT			(0 << 6)
#define SIIHDMI_OUTPUT_COLOR_DEPTH_10BIT		(2 << 6)
#define SIIHDMI_OUTPUT_COLOR_DEPTH_12BIT		(3 << 6)
#define SIIHDMI_OUTPUT_COLOR_DEPTH_16BIT		(1 << 6)

/* System Control */
#define SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI	(1 << 0)
#define SIIHDMI_SYS_CTRL_DDC_BUS_OWNER_HOST		(1 << 1)
#define SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED		(1 << 1)
#define SIIHDMI_SYS_CTRL_DDC_BUS_REQUEST		(1 << 2)
#define SIIHDMI_SYS_CTRL_AV_MUTE_HDMI			(1 << 3)
#define SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN		(1 << 4)
#define SIIHDMI_SYS_CTRL_DYNAMIC_LINK_INTEGRITY		(1 << 6)

/* Device Power State Control Data */
#define SIIHDMI_POWER_STATE_D0				(0 << 0)
#define SIIHDMI_POWER_STATE_D1				(1 << 0)
#define SIIHDMI_POWER_STATE_D2				(2 << 0)
#define SIIHDMI_POWER_STATE_D3				(3 << 0)
#define SIIHDMI_WAKEUP_STATE_COLD			(1 << 2)

/* Audio Frequency Sampling Length */
#define SIIHDMI_AUDIO_HANDLING_PASS_BASIC_AUDIO		(0 << 0)
#define SIIHDMI_AUDIO_HANDLING_PASS_ALL_AUDIO_MODES	(1 << 0)
#define SIIHDMI_AUDIO_HANDLING_DOWN_SAMPLE		(2 << 0)
#define SIIHDMI_AUDIO_HANDLING_UNCHECKED		(3 << 0)

/* Audio Interface Control */
#define SIIHDMI_AUDIO_MUTE				(1 << 4)
#define SIIHDMI_AUDIO_DISABLE				(0 << 6)
#define SIIHDMI_AUDIO_I2S_ENABLE			(2 << 6)
#define SIIHDMI_AUDIO_SPDIF_ENABLE			(1 << 6)

/* Audio Sampling HBR */
#define SIIHDMI_AUDIO_SAMPLING_HBR_ENABLE		(1 << 2)

#define SIIHDMI_AUDIO_SAMPLING_FREQUENCY_AUTO		(0 << 3)
#define SIIHDMI_AUDIO_SAMPLING_FREQUENCY_32_KHZ		(1 << 3)
#define SIIHDMI_AUDIO_SAMPLING_FREQUENCY_44_1_KHZ	(2 << 3)
#define SIIHDMI_AUDIO_SAMPLING_FREQUENCY_48_KHZ		(3 << 3)
#define SIIHDMI_AUDIO_SAMPLING_FREQUENCY_88_2_KHZ	(4 << 3)
#define SIIHDMI_AUDIO_SAMPLING_FREQUENCY_96_KHZ		(5 << 3)
#define SIIHDMI_AUDIO_SAMPLING_FREQUENCY_176_4_KHZ	(6 << 3)
#define SIIHDMI_AUDIO_SAMPLING_FREQUENCY_192_KHZ	(7 << 3)

#define SIIHDMI_AUDIO_SAMPLING_DEPTH_AUTO		(0 << 6)
#define SIIHDMI_AUDIO_SAMPLING_DEPTH_16_BIT		(1 << 6)
#define SIIHDMI_AUDIO_SAMPLING_DEPTH_20_BIT		(2 << 6)
#define SIIHDMI_AUDIO_SAMPLING_DEPTH_24_BIT		(3 << 6)

/* I²S Enable and Mapping */
#define SIIHDMI_I2S_MAPPING_SELECT_SD_CHANNEL_0		(0 << 0)
#define SIIHDMI_I2S_MAPPING_SELECT_SD_CHANNEL_1		(1 << 0)
#define SIIHDMI_I2S_MAPPING_SELECT_SD_CHANNEL_2		(2 << 0)
#define SIIHDMI_I2S_MAPPING_SELECT_SD_CHANNEL_3		(3 << 0)
#define SIIHDMI_I2S_MAPPING_SWAP_CHANNELS		(1 << 3)
#define SIIHDMI_I2S_ENABLE_BASIC_AUDIO_DOWNSAMPLING	(1 << 4)
#define SIIHDMI_I2S_MAPPING_SELECT_FIFO_0		(0 << 5)
#define SIIHDMI_I2S_MAPPING_SELECT_FIFO_1		(1 << 5)
#define SIIHDMI_I2S_MAPPING_SELECT_FIFO_2		(2 << 5)
#define SIIHDMI_I2S_MAPPING_SELECT_FIFO_3		(3 << 5)
#define SIIHDMI_I2S_ENABLE_SELECTED_FIFO		(1 << 7)

/* I²S Input Configuration */
#define SIIHDMI_I2S_WS_TO_SD_FIRST_BIT_SHIFT		(1 << 0)
#define SIIHDMI_I2S_SD_LSB_FIRST			(1 << 1)
#define SIIHDMI_I2S_SD_RIGHT_JUSTIFY_DATA		(1 << 2)
#define SIIHDMI_I2S_WS_POLARITY_HIGH			(1 << 3)
#define SIIHDMI_I2S_MCLK_MULTIPLIER_128			(0 << 4)
#define SIIHDMI_I2S_MCLK_MULTIPLIER_256			(1 << 4)
#define SIIHDMI_I2S_MCLK_MULTIPLIER_384			(2 << 4)
#define SIIHDMI_I2S_MCLK_MULTIPLIER_512			(3 << 4)
#define SIIHDMI_I2S_MCLK_MULTIPLIER_768			(4 << 4)
#define SIIHDMI_I2S_MCLK_MULTIPLIER_1024		(5 << 4)
#define SIIHDMI_I2S_MCLK_MULTIPLIER_1152		(6 << 4)
#define SIIHDMI_I2S_MCLK_MULTIPLIER_192			(7 << 4)
#define SIIHDMI_I2S_SCK_SAMPLE_RISING_EDGE		(1 << 7)

/* Interrupt Enable */
#define SIIHDMI_IER_HOT_PLUG_EVENT			(1 << 0)
#define SIIHDMI_IER_RECEIVER_SENSE_EVENT		(1 << 1)
#define SIIHDMI_IER_CTRL_BUS_EVENT			(1 << 2)
#define SIIHDMI_IER_CPI_EVENT				(1 << 3)
#define SIIHDMI_IER_AUDIO_EVENT				(1 << 4)
#define SIIHDMI_IER_SECURITY_STATUS_CHANGE		(1 << 5)
#define SIIHDMI_IER_HDCP_VALUE_READY			(1 << 6)
#define SIIHDMI_IER_HDCP_AUTHENTICATION_STATUS_CHANGE	(1 << 7)

/* Interrupt Status */
#define SIIHDMI_ISR_HOT_PLUG_EVENT			(1 << 0)
#define SIIHDMI_ISR_RECEIVER_SENSE_EVENT		(1 << 1)
#define SIIHDMI_ISR_CTRL_BUS_EVENT			(1 << 2)
#define SIIHDMI_ISR_DISPLAY_ATTACHED			(1 << 2)
#define SIIHDMI_ISR_CPI_EVENT				(1 << 3)
#define SIIHDMI_ISR_RECEIVER_SENSE			(1 << 3)
#define SIIHDMI_ISR_AUDIO_EVENT				(1 << 4)
#define SIIHDMI_ISR_SECURITY_STATUS_CHANGED		(1 << 5)
#define SIIHDMI_ISR_HDCP_VALUE_READY			(1 << 6)
#define SIIHDMI_ISR_HDCP_AUTHENTICATION_STATUS_CHANGED	(1 << 7)

/* Request/Grant/Black Mode */
#define SIIHDMI_RQB_DDC_BUS_REQUEST			(1 << 0)
#define SIIHDMI_RQB_DDC_BUS_GRANTED			(1 << 1)
#define SIIHDMI_RQB_I2C_ACCESS_DDC_BUS			(1 << 2)
#define SIIHDMI_RQB_FORCE_VIDEO_BLANK			(1 << 5)
#define SIIHDMI_RQB_TPI_MODE_DISABLE			(1 << 7)

/*
 * SII HDMI chips have two ways to send HDMI InfoFrames:
 *   a) AVI InfoFrame is sent via a dedicated TPI register block
 *   b) Other InfoFrames are sent via a misc TPI register block with a header to
 *      identifty the InfoFrame data
 */

/* InfoFrame blocks */
#define SIIHDMI_TPI_REG_AVI_INFO_FRAME_BASE		(SIIHDMI_TPI_REG_AVI_DBYTE0)
#define SIIHDMI_TPI_REG_AVI_INFO_FRAME_LENGTH		(SIIHDMI_TPI_REG_AVI_INFO_END_RIGHT_BAR_MSB - SIIHDMI_TPI_REG_AVI_DBYTE0 + 1)

#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_BASE		(SIIHDMI_TPI_REG_MISC_INFO_FRAME_SELECT)
#define SIIHDMI_TPI_REG_MISC_INFO_FRAME_LENGTH		(SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE27 - SIIHDMI_TPI_REG_MISC_INFO_FRAME_SELECT + 1)

#define SIIHDMI_TPI_REG_AUDIO_INFO_FRAME_BASE		(SIIHDMI_TPI_REG_MISC_INFO_FRAME_SELECT)
#define SIIHDMI_TPI_REG_AUDIO_INFO_FRAME_LENGTH		(SIIHDMI_TPI_REG_MISC_INFO_FRAME_DBYTE10 - SIIHDMI_TPI_REG_MISC_INFO_FRAME_SELECT + 1)

/*
 * AVI InfoFrame is handled specially.  The type, version, and length fields (1
 * byte each) are skipped in transmission.  We must offset into the standard
 * structure to skip these fields.
 */
#define SIIHDMI_AVI_INFO_FRAME_OFFSET			(0x03)


enum siihdmi_info_frame_type {
	SIIHDMI_INFO_FRAME_NONE,
	SIIHDMI_INFO_FRAME_SPD_ACP,
	SIIHDMI_INFO_FRAME_AUDIO,
	SIIHDMI_INFO_FRAME_MPEG_GBD,
	SIIHDMI_INFO_FRAME_GENERIC1_ISRC1,
	SIIHDMI_INFO_FRAME_GENERIC2_ISRC2,
};

struct __packed info_frame_buffer_header {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	unsigned info_frame     : 3;
	unsigned                : 1;
	unsigned                : 2;
	unsigned repeat         : 1;
	unsigned enable         : 1;
#else
	unsigned enable         : 1;
	unsigned repeat         : 1;
	unsigned                : 2;
	unsigned                : 1;
	unsigned info_frame     : 3;
#endif
};

struct siihdmi_spd_info_frame {
	struct info_frame_buffer_header header;
//	struct spd_info_frame           info_frame;
	u8                              padding[2];
};

struct siihdmi_audio_info_frame {
	struct info_frame_buffer_header header;
//	struct audio_info_frame         info_frame;
};

struct siihdmi_platform_data {
	/* reset function */
	void (*reset)(void);

	/* DRM fixed id */
	char *drm_name;
	int encon_id;

	/* hotplug IRQ */
	//struct resource hotplug;

	/* maximum pixel clock rate - should be in crtc or drm_mode_config_funcs damn it*/
	int max_pixclock;

	/* HDMI SPD InfoFrame Data */
	char *vendor;
	char *description;
};

struct siihdmi_tx {
	struct i2c_client *client;

	struct drm_connector connector;
	struct imx_drm_connector *imx_drm_connector;
	struct drm_encoder encoder;
	struct imx_drm_encoder *imx_drm_encoder;
	struct drm_display_mode mode;

	struct device *dev;

	u32 interface_pix_fmt;
	unsigned int gpio_reset, gpio_enable1, gpio_enable2, gpio_irq;

	struct fb_info *info;
	struct notifier_block nb;

	struct {
		bool enabled;
		struct work_struct handler;
	} hotplug;

	struct {
		u8 *data;
		u32 length;
#if defined(CONFIG_SYSFS)
		struct bin_attribute attributes;
#endif
	} edid;


	struct {
		bool available;
#if defined(CONFIG_SYSFS)
		struct bin_attribute attributes;
#endif
	} audio;

	struct {
		enum {
			SINK_TYPE_DVI,
			SINK_TYPE_HDMI,
		} type;

		enum {
			SCANNING_EXACT,
			SCANNING_UNDERSCANNED,
			SCANNING_OVERSCANNED,
		} scanning;

		struct fb_videomode preferred_mode;
		struct fb_videomode current_mode;
	} sink;
};

#endif

