/*
 * asm-arm/arch-lpc32xx/lpc32xx_mac.h
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef LPC32XX_MAC_H
#define LPC32XX_MAC_H

#define _BIT(n) (1 << (n))

/**********************************************************************
* Ethernet MAC controller Register offsets
**********************************************************************/

/* MAC registers */
#define ENET_MAC1(x)		(x + 0x000)
#define ENET_MAC2(x)		(x + 0x004)
#define ENET_IPGT(x)		(x + 0x008)
#define ENET_IPGR(x)		(x + 0x00C)
#define ENET_CLRT(x)		(x + 0x010)
#define ENET_MAXF(x)		(x + 0x014)
#define ENET_SUPP(x)		(x + 0x018)
#define ENET_TEST(x)		(x + 0x01C)
#define ENET_MCFG(x)		(x + 0x020)
#define ENET_MCMD(x)		(x + 0x024)
#define ENET_MADR(x)		(x + 0x028)
#define ENET_MWTD(x)		(x + 0x02C)
#define ENET_MRDD(x)		(x + 0x030)
#define ENET_MIND(x)		(x + 0x034)
#define ENET_SA0(x)		(x + 0x040)
#define ENET_SA1(x)		(x + 0x044)
#define ENET_SA2(x)		(x + 0x048)

/* Control registers */
#define ENET_COMMAND(x)		(x + 0x100)
#define ENET_STATUS(x)		(x + 0x104)
#define ENET_RXDESCRIPTOR(x)	(x + 0x108)
#define ENET_RXSTATUS(x)	(x + 0x10C)
#define ENET_RXDESCRIPTORNUMBER(x) (x + 0x110)
#define ENET_RXPRODUCEINDEX(x)	(x + 0x114)
#define ENET_RXCONSUMEINDEX(x)	(x + 0x118)
#define ENET_TXDESCRIPTOR(x)	(x + 0x11C)
#define ENET_TXSTATUS(x)	(x + 0x120)
#define ENET_TXDESCRIPTORNUMBER(x) (x + 0x124)
#define ENET_TXPRODUCEINDEX(x)	(x + 0x128)
#define ENET_TXCONSUMEINDEX(x)	(x + 0x12C)
#define ENET_TSV0(x)		(x + 0x158)
#define ENET_TSV1(x)		(x + 0x15C)
#define ENET_RSV(x)		(x + 0x160)
#define ENET_FLOWCONTROLCOUNTER(x) (x + 0x170)
#define ENET_FLOWCONTROLSTATUS(x) (x + 0x174)

/* RX filter registers */
#define ENET_RXFILTER_CTRL(x)	(x + 0x200)
#define ENET_RXFILTERWOLSTATUS(x) (x + 0x204)
#define ENET_RXFILTERWOLCLEAR(x) (x + 0x208)
#define ENET_HASHFILTERL(x)	(x + 0x210)
#define ENET_HASHFILTERH(x)	(x + 0x214)

/* RX filter registers */
#define ENET_INTSTATUS(x)	(x + 0xFE0)
#define ENET_INTENABLE(x)	(x + 0xFE4)
#define ENET_INTCLEAR(x)	(x + 0xFE8)
#define ENET_INTSET(x)		(x + 0xFEC)
#define ENET_POWERDOWN(x)	(x + 0xFF4)

/* Structure of a TX/RX descriptor */
struct txrx_desc_t
{
  volatile u32 packet;
  volatile u32 control;
};

/* Structure of a RX status entry */
struct rx_status_t
{
  volatile u32 statusinfo;
  volatile u32 statushashcrc;
};

/**********************************************************************
* mac1 register definitions
**********************************************************************/
/* Set this to allow receive frames to be received. Internally the
   MAC synchronize this control bit to the incoming receive stream */
#define MAC1_RECV_ENABLE               _BIT(0)
/* When enabled (set to ’1’), the MAC will pass all frames regardless
   of type (normal vs. Control). When disabled, the MAC does not pass
   valid Control frames */
#define MAC1_PASS_ALL_RX_FRAMES        _BIT(1)
/* When enabled (set to ’1’), the MAC acts upon received PAUSE Flow
   Control frames. When disabled, received PAUSE Flow Control frames
   are ignored */
#define MAC1_RX_FLOW_CONTROL           _BIT(2)
/* When enabled (set to ’1’), PAUSE Flow Control frames are allowed
   to be transmitted. When disabled, Flow Control frames are blocked */
#define MAC1_TX_FLOW_CONTROL           _BIT(3)
/* Setting this bit will cause the MAC Transmit interface to be
   looped back to the MAC Receive interface. Clearing this bit
   results in normal operation */
#define MAC1_LOOPBACK                  _BIT(4)
/* Setting this bit will put the Transmit Function logic in reset */
#define MAC1_RESET_TX                  _BIT(8)
/* Setting this bit resets the MAC Control Sublayer / Transmit logic.
   The MCS logic implements flow control */
#define MAC1_RESET_MCS_TX              _BIT(9)
/* Setting this bit will put the Ethernet receive logic in reset */
#define MAC1_RESET_RX                  _BIT(10)
/* Setting this bit resets the MAC Control Sublayer / Receive logic.
   The MCS logic implements flow control */
#define MAC1_RESET_MCS_RX              _BIT(11)
/* Setting this bit will cause a reset to the random number generator
   within the Transmit Function */
#define MAC1_SIMULATION_RESET          _BIT(14)
/* Setting this bit will put all modules within the MAC in reset
   except the Host Interface */
#define MAC1_SOFT_RESET                _BIT(15)

/**********************************************************************
* mac2 register definitions
**********************************************************************/
/* When enabled (set to ’1’), the MAC operates in Full-Duplex mode.
   When disabled the MAC operates in Half-Duplex mode */
#define MAC2_FULL_DUPLEX               _BIT(0)
/* When enabled (set to ’1’), both transmit and receive frame lengths
   are compared to the Length/Type field. If the Length/Type field
   represents a length then the check is performed. Mismatches are
   reported in the StatusInfo word for each received frame */
#define MAC2_FRAME_LENGTH_CHECKING     _BIT(1)
/* When enabled (set to ’1’), frames of any length are transmitted
   and received */
#define MAC2_HUGH_LENGTH_CHECKING      _BIT(2)
/* This bit determines the number of bytes, if any, of proprietary
   header information that exist on the front of IEEE 802.3 frames.
   When 1, four bytes of header (ignored by the CRC function) are
   added. When 0, there is no proprietary header */
#define MAC2_DELAYED_CRC               _BIT(3)
/* Set this bit to append a CRC to every frame whether padding was
   required or not. Must be set if PAD/CRC ENABLE is set. Clear this
   bit if frames presented to the MAC contain a CRC */
#define MAC2_CRC_ENABLE                _BIT(4)
/* Set this bit to have the MAC pad all short frames. Clear this bit
   if frames presented to the MAC have a valid length. This bit is used
   in conjunction with AUTO PAD ENABLE and VLAN PAD ENABLE */
#define MAC2_PAD_CRC_ENABLE            _BIT(5)
/* Set this bit to cause the MAC to pad all short frames to 64 bytes
   and append a valid CRC. Note: This bit is ignored if
   MAC2_PAD_CRC_ENABLE is cleared */
#define MAC2_VLAN_PAD_ENABLE           _BIT(6)
/* Set this bit to cause the MAC to automatically detect the type of
   frame, either tagged or un-tagged, by comparing the two octets
   following the source address with 0x8100 (VLAN Protocol ID) and
   pad accordingly. Table 14–273 - Pad Operation provides a description
   of the pad function based on the configuration of this register.
   Note: This bit is ignored if PAD / CRC ENABLE is cleared */
#define MAC2_AUTO_DETECT_PAD_ENABLE    _BIT(7)
/* When enabled (set to ’1’), the MAC will verify the content of the
   preamble to ensure it contains 0x55 and is error-free. A packet
   with an incorrect preamble is discarded. When disabled, no preamble
   checking is performed */
#define MAC2_PURE_PREAMBLE_ENFORCEMENT _BIT(8)
/* When enabled (set to ’1’), the MAC only allows receive packets
   which contain preamble fields less than 12 bytes in length. When
   disabled, the MAC allows any length preamble as per the Standard */
#define MAC2_LONG_PREAMBLE_ENFORCEMENT _BIT(9)
/* When enabled (set to ’1’), the MAC will immediately retransmit
   following a collision rather than using the Binary Exponential
   Backoff algorithm as specified in the Standard */
#define MAC2_NO_BACKOFF                _BIT(12)
/* When enabled (set to ’1’), after the MAC incidentally causes a
   collision during back pressure, it will immediately retransmit
   without backoff, reducing the chance of further collisions and
   ensuring transmit packets get sent */
#define MAC2_BACK_PRESSURE             _BIT(13)
/* When enabled (set to ’1’) the MAC will defer to carrier indefinitely
   as per the Standard. When disabled, the MAC will abort when the
   excessive deferral limit is reached */
#define MAC2_EXCESS_DEFER              _BIT(14)

/**********************************************************************
* ipgt register definitions
**********************************************************************/
/* This is a programmable field representing the nibble time offset
   of the minimum possible period between the end of any transmitted
   packet to the beginning of the next. In Full-Duplex mode, the
   register value should be the desired period in nibble times minus 3.
   In Half-Duplex mode, the register value should be the desired
   period in nibble times minus 6. In Full-Duplex the recommended
   setting is 0x15 (21d), which represents the minimum IPG of 960 ns
   (in 100 Mbps mode) or 9.6 ?s (in 10 Mbps mode). In Half-Duplex the
   recommended setting is 0x12 (18d), which also represents the minimum
   IPG of 960 ns (in 100 Mbps mode) or 9.6 ?s (in 10 Mbps mode) */
#define IPGT_LOAD(n)                   ((n) & 0x7F)

/**********************************************************************
* ipgr register definitions
**********************************************************************/
/* This is a programmable field representing the Non-Back-to-Back
   Inter-Packet-Gap. The recommended value is 0x12 (18d), which
   represents the minimum IPG of 960 ns (in 100 Mbps mode) or 9.6 ?s
   (in 10 Mbps mode) */
#define IPGR_LOAD_PART2(n)             ((n) & 0x7F)
/* This is a programmable field representing the optional carrierSense
   window referenced in IEEE 802.3/4.2.3.2.1 'Carrier Deference'. If
   carrier is detected during the timing of IPGR1, the MAC defers to
   carrier. If, however, carrier becomes active after IPGR1, the MAC
   continues timing IPGR2 and transmits, knowingly causing a collision,
   thus ensuring fair access to medium. Its range of values is 0x0 to
   IPGR2. The recommended value is 0xC (12d) */
#define IPGR_LOAD_PART1(n)             (((n) & 0x7F) << 8)

/**********************************************************************
* clrt register definitions
**********************************************************************/
/* This is a programmable field specifying the number of
   retransmission attempts following a collision before aborting the
   packet due to excessive collisions. The Standard specifies the
   attemptLimit to be 0xF (15d). See IEEE 802.3/4.2.3.2.5. */
#define CLRT_LOAD_RETRY_MAX(n)         ((n) & 0xF)
/* This is a programmable field representing the slot time or
   collision window during which collisions occur in properly
   configured networks. The default value of 0x37 (55d) represents a
   56 byte window following the preamble and SFD. */
#define CLRT_LOAD_COLLISION_WINDOW(n)  (((n) & 0x3F) << 8)

/**********************************************************************
* maxf register definitions
**********************************************************************/
/* This field resets to the value 0x0600, which represents a maximum
   receive frame of 1536 octets. An untagged maximum size Ethernet
   frame is 1518 octets. A tagged frame adds four octets for a total
   of 1522 octets. If a shorter maximum length restriction is desired,
   program this 16 bit field. */
#define MAXF_LOAD_MAX_FRAME_LEN(n)     ((n) & 0xFFFF)

/**********************************************************************
* supp register definitions
**********************************************************************/
/* This bit configures the Reduced MII logic for the current operating
   speed. When set, 100 Mbps mode is selected. When cleared, 10 Mbps
   mode is selected */
#define SUPP_SPEED                     _BIT(8)
/* Reset Reduced MII Logic */
#define SUPP_RESET_RMII                _BIT(11)

/**********************************************************************
* test register definitions
**********************************************************************/
/* This bit reduces the effective PAUSE quanta from 64 byte-times to
   1 byte-time. */
#define TEST_SHORTCUT_PAUSE_QUANTA     _BIT(0)
/* This bit causes the MAC Control sublayer to inhibit transmissions,
   just as if a PAUSE Receive Control frame with a nonzero pause time
   parameter was received. */
#define TEST_PAUSE                     _BIT(1)
/* Setting this bit will cause the MAC to assert backpressure on the
   link. Backpressure causes preamble to be transmitted, raising
   carrier sense. A transmit packet from the system will be sent
   during backpressure. */
#define TEST_BACKPRESSURE              _BIT(2)

/**********************************************************************
* mcfg register definitions
**********************************************************************/
/* Set this bit to cause the MII Management hardware to perform read
   cycles across a range of PHYs. When set, the MII Management
   hardware will perform read cycles from address 1 through the value
   set in PHY ADDRESS[4:0]. Clear this bit to allow continuous reads
   of the same PHY. */
#define MCFG_SCAN_INCREMENT            _BIT(0)
/* Set this bit to cause the MII Management hardware to perform
   read/write cycles without the 32 bit preamble field. Clear this bit
   to cause normal cycles to be performed. Some PHYs support
   suppressed preamble. */
#define MCFG_SUPPRESS_PREAMBLE         _BIT(1)
/* This field is used by the clock divide logic in creating the MII
   Management Clock (MDC) which IEEE 802.3u defines to be no faster
   than 2.5 MHz. Some PHYs support clock rates up to 12.5 MHz,
   however. Refer to Table 14–280 below for the definition of values
   for this field. */
#define MCFG_CLOCK_SELECT(n)           (((n) & 0x7) << 2)
/* MCFG_CLOCK_SELECT macro load values */
#define MCFG_CLOCK_HOST_DIV_4          0
#define MCFG_CLOCK_HOST_DIV_6          2
#define MCFG_CLOCK_HOST_DIV_8          3
#define MCFG_CLOCK_HOST_DIV_10         4
#define MCFG_CLOCK_HOST_DIV_14         5
#define MCFG_CLOCK_HOST_DIV_20         6
#define MCFG_CLOCK_HOST_DIV_28         7
/* This bit resets the MII Management hardware */
#define MCFG_RESET_MII_MGMT            _BIT(15)

/**********************************************************************
* mcmd register definitions
**********************************************************************/
/* This bit causes the MII Management hardware to perform a single
   Read cycle. The Read data is returned in Register MRDD (MII Mgmt
   Read Data). */
#define MCMD_READ                      _BIT(0)
/* This bit causes the MII Management hardware to perform Read cycles
   continuously. This is useful for monitoring Link Fail for example */
#define MCMD_SCAN                      _BIT(1)

/**********************************************************************
* madr register definitions
**********************************************************************/
/* This field represents the 5 bit Register Address field of Mgmt
   cycles. Up to 32 registers can be accessed. */
#define MADR_REGISTER_ADDRESS(n)       ((n) & 0x1F)
/* This field represents the 5 bit PHY Address field of Mgmt
   cycles. Up to 31 PHYs can be addressed (0 is reserved). */
#define MADR_PHY_0ADDRESS(n)           (((n) & 0x1F) << 8)

/**********************************************************************
* mwtd register definitions
**********************************************************************/
/* When written, an MII Mgmt write cycle is performed using the 16 bit
   data and the pre-configured PHY and Register addresses from the
   MII Mgmt Address register (MADR). */
#define MWDT_WRITE(n)                  ((n) & 0xFFFF)

/**********************************************************************
* mrdd register definitions
**********************************************************************/
/* Read mask for MUU read */
#define MRDD_READ_MASK                 0xFFFF

/**********************************************************************
* mind register definitions
**********************************************************************/
/* When ’1’ is returned - indicates MII Mgmt is currently performing
   an MII Mgmt Read or Write cycle. */
#define MIND_BUSY                      _BIT(0)
/* When ’1’ is returned - indicates a scan operation (continuous MII
   Mgmt Read cycles) is in progress. */
#define MIND_SCANNING                  _BIT(1)
/* When ’1’ is returned - indicates MII Mgmt Read cycle has not
   completed and the Read Data is not yet valid. */
#define MIND_NOT_VALID                 _BIT(2)
/* When ’1’ is returned - indicates that an MII Mgmt link fail has
   occurred.*/
#define MIND_MII_LINK_FAIL             _BIT(3)

/**********************************************************************
* command register definitions
**********************************************************************/
/* Enable receive */
#define COMMAND_RXENABLE               _BIT(0)
/* Enable transmit */
#define COMMAND_TXENABLE               _BIT(1)
/* When a ’1’ is written, all datapaths and the host registers are
   reset. The MAC needs to be reset separately. */
#define COMMAND_REG_RESET              _BIT(3)
/* When a ’1’ is written, the transmit datapath is reset. */
#define COMMAND_TXRESET                _BIT(4)
/* When a ’1’ is written, the receive datapath is reset. */
#define COMMAND_RXRESET                _BIT(5)
/* When set to ’1’, passes runt frames smaller than 64 bytes to
   memory unless they have a CRC error. If ’0’ runt frames are
   filtered out. */
#define COMMAND_PASSRUNTFRAME          _BIT(6)
/* When set to ’1’, disables receive filtering i.e. all frames
   received are written to memory. */
#define COMMAND_PASSRXFILTER           _BIT(7)
/* Enable IEEE 802.3 / clause 31 flow control sending pause
   frames in full duplex and continuous preamble in half duplex. */
#define COMMAND_TXFLOWCONTROL          _BIT(8)
/* When set to ’1’, RMII mode is selected; if ’0’, MII mode is
   selected. */
#define COMMAND_RMII                   _BIT(9)
/* When set to ’1’, indicates full duplex operation. */
#define COMMAND_FULLDUPLEX             _BIT(10)

/**********************************************************************
* status register definitions
**********************************************************************/
/* If 1, the receive channel is active. If 0, the receive channel is
   inactive. */
#define STATUS_RXACTIVE                _BIT(0)
/* If 1, the transmit channel is active. If 0, the transmit channel is
   inactive. */
#define STATUS_TXACTIVE                _BIT(1)

/**********************************************************************
* tsv0 register definitions
**********************************************************************/
/* The attached CRC in the packet did not match the internally
   generated CRC. */
#define TSV0_CRC_ERROR                 _BIT(0)
/* Indicates the frame length field does not match the actual
   number of data items and is not a type field. */
#define TSV0_LENGTH_CHECK_ERROR        _BIT(1)
/* Indicates that frame type/length field was larger tha 1500 bytes. */
#define TSV0_LENGTH_OUT_OF_RANGE       _BIT(2)
/* Transmission of packet was completed. */
#define TSV0_DONE                      _BIT(3)
/* Packet’s destination was a multicast address. */
#define TSV0_MULTICAST                 _BIT(4)
/* Packet’s destination was a broadcast address. */
#define TSV0_BROADCAST                 _BIT(5)
/* Packet was deferred for at least one attempt, but less than
   an excessive defer. */
#define TSV0_PACKET_DEFER              _BIT(6)
/* Packet was deferred in excess of 6071 nibble times in
   100 Mbps or 24287 bit times in 10 Mbps mode. */
#define TSV0_ESCESSIVE_DEFER           _BIT(7)
/* Packet was aborted due to exceeding of maximum allowed
   number of collisions. */
#define TSV0_ESCESSIVE_COLLISION       _BIT(8)
/* Collision occurred beyond collision window, 512 bit times. */
#define TSV0_LATE_COLLISION            _BIT(9)
/* Byte count in frame was greater than can be represented
   in the transmit byte count field in TSV1. */
#define TSV0_GIANT                     _BIT(10)
/* Host side caused buffer underrun. */
#define TSV0_UNDERRUN                  _BIT(11)
/* Macro: The total number of bytes transferred including
   collided attempts. */
#define TSV0_TOTAL_BYTES(n)            (((n) >> 12) & 0xFFFF)
/* The frame was a control frame. */
#define TSV0_CONTROL_FRAME             _BIT(28)
/* The frame was a control frame with a valid PAUSE opcode. */
#define TSV0_PAUSE                     _BIT(29)
/* Carrier-sense method backpressure was previously applied. */
#define TSV0_BACKPRESSURE              _BIT(30)
/* Frame’s length/type field contained 0x8100 which is the
   VLAN protocol identifier. */
#define TSV0_VLAN                      _BIT(31)

/**********************************************************************
* tsv1 register definitions
**********************************************************************/
/* Macro: The total number of bytes in the frame, not counting the
   collided bytes. */
#define TSV1_TRANSMIT_BYTE_COUNT(n)    ((n) & 0xFFFF)
/* Macro: Number of collisions the current packet incurred during
   transmission attempts. The maximum number of collisions
   (16) cannot be represented. */
#define TSV1_COLLISION_COUNT(n)        (((n) >> 16) & 0xF)

/**********************************************************************
* rsv register definitions
**********************************************************************/
/* Macro: Indicates length of received frame. */
#define RSV_RECEIVED_BYTE_COUNT(n)     ((n) & 0xFFFF)
/* Indicates that a packet was dropped. */
#define RSV_RXDV_EVENT_IGNORED         _BIT(16)
/* Indicates that the last receive event seen was not long
   enough to be a valid packet. */
#define RSV_RXDV_EVENT_PREVIOUSLY_SEEN _BIT(17)
/* Indicates that at some time since the last receive statistics,
   a carrier event was detected. */
#define RSV_CARRIER_EVNT_PREVIOUS_SEEN _BIT(18)
/* Indicates that MII data does not represent a valid receive
   code. */
#define RSV_RECEIVE_CODE_VIOLATION     _BIT(19)
/* The attached CRC in the packet did not match the internally
   generated CRC. */
#define RSV_CRC_ERROR                  _BIT(20)
/* Indicates the frame length field does not match the actual
   number of data items and is not a type field. */
#define RSV_LENGTH_CHECK_ERROR         _BIT(21)
/* Indicates that frame type/length field was larger than 1518 bytes */
#define RSV_LENGTH_OUT_OF_RANGE        _BIT(22)
/* The packet had valid CRC and no symbol errors. */
#define RSV_RECEIVE_OK                 _BIT(23)
/* The packet destination was a multicast address. */
#define RSV_MULTICAST                  _BIT(24)
/* The packet destination was a boardcase address. */
#define RSV_BROADCAST                  _BIT(25)
/* Indicates that after the end of packet another 1-7 bits were
   received. A single nibble, called dribble nibble, is formed
   but not sent out. */
#define RSV_DRIBBLE_NIBBLE             _BIT(26)
/* The frame was a control frame. */
#define RSV_CONTROL_FRAME              _BIT(27)
/* The frame was a control frame with a valid PAUSE opcode. */
#define RSV_PAUSE                      _BIT(28)
/* The current frame was recognized as a Control Frame but
   contains an unknown opcode. */
#define RSV_UNSUPPORTED_OPCODE         _BIT(29)
/* Frame’s length/type field contained 0x8100 which is the
   VLAN protocol identifier. */
#define RSV_VLAN                       _BIT(30)

/**********************************************************************
* flowcontrolcounter register definitions
**********************************************************************/
/* Macro: In full duplex mode the MirrorCounter specifies the number
   of cycles before re-issuing the Pause control frame. */
#define FCCR_MIRRORCOUNTER(n)          ((n) & 0xFFFF)
/* Macro: In full-duplex mode the PauseTimer specifies the value
   that is inserted into the pause timer field of a pause flow
   control frame. In half duplex mode the PauseTimer
   specifies the number of backpressure cycles. */
#define FCCR_PAUSETIMER(n)             (((n) >> 16) & 0xFFFF)

/**********************************************************************
* flowcontrolstatus register definitions
**********************************************************************/
/* Macro: In full duplex mode this register represents the current
   value of the datapath’s mirror counter which counts up to
   the value specified by the MirrorCounter field in the
   FlowControlCounter register. In half duplex mode the
   register counts until it reaches the value of the PauseTimer
   bits in the FlowControlCounter register. */
#define FCCR_MIRRORCOUNTERCURRENT(n)   ((n) & 0xFFFF)

/**********************************************************************
* rxfliterctrl, rxfilterwolstatus, and rxfilterwolclear shared
* register definitions
**********************************************************************/
/* Unicast frame control */
#define RXFLTRW_ACCEPTUNICAST          _BIT(0)
/* Broadcase frame control. */
#define RXFLTRW_ACCEPTUBROADCAST       _BIT(1)
/* Multicast frame control */
#define RXFLTRW_ACCEPTUMULTICAST       _BIT(2)
/* Imperfect unicast frame control */
#define RXFLTRW_ACCEPTUNICASTHASH      _BIT(3)
/* Imperfect multicast frame control */
#define RXFLTRW_ACCEPTUMULTICASTHASH   _BIT(4)
/* Perfect frame control */
#define RXFLTRW_ACCEPTPERFECT          _BIT(5)

/**********************************************************************
* rxfliterctrl register definitions
**********************************************************************/
/* When set to ’1’, the result of the magic packet filter will
   generate a WoL interrupt when there is a match. */
#define RXFLTRWSTS_MAGICPACKETENWOL    _BIT(12)
/* When set to ’1’, the result of the perfect address
   matching filter and the imperfect hash filter will
   generate a WoL interrupt when there is a match. */
#define RXFLTRWSTS_RXFILTERENWOL       _BIT(13)

/**********************************************************************
* rxfilterwolstatus register definitions
**********************************************************************/
/* When the value is ’1’, the receive filter caused WoL. */
#define RXFLTRWSTS_RXFILTERWOL         _BIT(7)
/* When the value is ’1’, the magic packet filter caused WoL. */
#define RXFLTRWSTS_MAGICPACKETWOL      _BIT(8)

/**********************************************************************
* rxfilterwolclear register definitions
**********************************************************************/
/* When a ’1’ is written to one of these bits (7 and/or 8),
   the corresponding status bit in the rxfilterwolstatus
   register is cleared. */
#define RXFLTRWCLR_RXFILTERWOL         RXFLTRWSTS_RXFILTERWOL
#define RXFLTRWCLR_MAGICPACKETWOL      RXFLTRWSTS_MAGICPACKETWOL

/**********************************************************************
* intstatus, intenable, intclear, and Intset shared register
* definitions
**********************************************************************/
/* Interrupt trigger on receive buffer overrun or descriptor underrun
   situations. */
#define MACINT_RXOVERRUNINTEN          _BIT(0)
/* Enable for interrupt trigger on receive errors. */
#define MACINT_RXERRORONINT            _BIT(1)
/* Enable for interrupt triggered when all receive descriptors have
   been processed i.e. on the transition to the situation where
   ProduceIndex == ConsumeIndex. */
#define MACINT_RXFINISHEDINTEN         _BIT(2)
/* Enable for interrupt triggered when a receive descriptor has
   been processed while the Interrupt bit in the Control field of the
   descriptor was set. */
#define MACINT_RXDONEINTEN             _BIT(3)
/* Enable for interrupt trigger on transmit buffer or descriptor
   underrun situations. */
#define MACINT_TXUNDERRUNINTEN         _BIT(4)
/* Enable for interrupt trigger on transmit errors. */
#define MACINT_TXERRORINTEN            _BIT(5)
/* Enable for interrupt triggered when all transmit descriptors
   have been processed i.e. on the transition to the situation
   where ProduceIndex == ConsumeIndex. */
#define MACINT_TXFINISHEDINTEN         _BIT(6)
/* Enable for interrupt triggered when a descriptor has been
   transmitted while the Interrupt bit in the Control field of the
   descriptor was set. */
#define MACINT_TXDONEINTEN             _BIT(7)
/* Enable for interrupt triggered by the SoftInt bit in the IntStatus
   register, caused by software writing a 1 to the SoftIntSet bit in
   the IntSet register. */
#define MACINT_SOFTINTEN               _BIT(12)
/* Enable for interrupt triggered by a Wakeup event detected by
   the receive filter. */
#define MACINT_WAKEUPINTEN             _BIT(13)

/**********************************************************************
* powerdown register definitions
**********************************************************************/
/* If true, all AHB accesses will return a read/write error,
   except accesses to the PowerDown register. */
#define POWERDOWN_MACAHB               _BIT(31)

#endif /* LPC32XX_MAC_H */
