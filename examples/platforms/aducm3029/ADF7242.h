
#ifndef _ADF7242_H_
#define _ADF7242_H_

#include <system_ADuCM3029.h>

/* ADF7242 registers */
#define REG_EXT_CTRL                0x100
#define REG_TX_FSK_TEST             0x101
#define REG_FSK_PREAMBLE            0x102
#define REG_CCA1                    0x105
#define REG_CCA2                    0x106
#define REG_BUFFERCFG               0x107
#define REG_PKT_CFG                 0x108
#define REG_DELAYCFG0               0x109
#define REG_DELAYCFG1               0x10A
#define REG_DELAYCFG2               0x10B
#define REG_SYNC_WORD0              0x10C
#define REG_SYNC_WORD1              0x10D
#define REG_SYNC_WORD2              0x10E
#define REG_SYNC_CONFIG             0x10F
#define REG_RC_CFG                  0x13E
#define REG_RC_VAR44                0x13F
#define REG_CH_FREQ0                0x300
#define REG_CH_FREQ1                0x301
#define REG_CH_FREQ2                0x302
#define REG_TX_FD                   0x304
#define REG_DM_CFG0                 0x305
#define REG_TX_M                    0x306
#define REG_RX_M                    0x307
#define REG_RRB                     0x30C
#define REG_LRB                     0x30D
#define REG_DR0                     0x30E
#define REG_DR1                     0x30F
#define REG_PRAMPG                  0x313
#define REG_TXPB                    0x314
#define REG_RXPB                    0x315
#define REG_TMR_CFG0                0x316
#define REG_TMR_CFG1                0x317
#define REG_TMR_RLD0                0x318
#define REG_TMR_RLD1                0x319
#define REG_TMR_CTRL                0x31A
#define REG_PD_AUX                  0x31E
#define REG_GP_CFG                  0x32C
#define REG_GP_OUT                  0x32D
#define REG_GP_IN                   0x32E
#define REG_SYNT                    0x335
#define REG_CAL_CFG                 0x33D
#define REG_PA_BIAS                 0x36E
#define REG_SYNT_CAL                0x371
#define REG_IIRF_CFG                0x389
#define REG_CDR_CFG                 0x38A
#define REG_DM_CFG1                 0x38B
#define REG_AGCSTAT                 0x38E
#define REG_RXCAL0                  0x395
#define REG_RXCAL1                  0x396
#define REG_RXFE_CFG                0x39B
#define REG_PA_RR                   0x3A7
#define REG_PA_CFG                  0x3A8
#define REG_EXTPA_CFG               0x3A9
#define REG_EXTPA_MSC               0x3AA
#define REG_ADC_RBK                 0x3AE
#define REG_AGC_CFG1                0x3B2
#define REG_AGC_MAX                 0x3B4
#define REG_AGC_CFG2                0x3B6
#define REG_AGC_CFG3                0x3B7
#define REG_AGC_CFG4                0x3B8
#define REG_AGC_CFG5                0x3B9
#define REG_AGC_CFG6                0x3BA
#define REG_AGC_CFG7                0x3BC
#define REG_OCL_CFG0                0x3BF
#define REG_OCL_CFG1                0x3C4
#define REG_IRQ1_EN0                0x3C7
#define REG_IRQ1_EN1                0x3C8
#define REG_IRQ2_EN0                0x3C9
#define REG_IRQ2_EN1                0x3CA
#define REG_IRQ1_SRC0               0x3CB
#define REG_IRQ1_SRC1               0x3CC
#define REG_OCL_BW0                 0x3D2
#define REG_OCL_BW1                 0x3D3
#define REG_OCL_BW2                 0x3D4
#define REG_OCL_BW3                 0x3D5
#define REG_OCL_BW4                 0x3D6
#define REG_OCL_BWS                 0x3D7
#define REG_OCL_CFG13               0x3E0
#define REG_GP_DRV                  0x3E3
#define REG_BM_CFG                  0x3E6
#define REG_SFD_15_4                0x3F4
#define REG_PREAMBLE_NUM_VALIDATE   0x3F3
#define REG_AFC_CFG                 0x3F7
#define REG_AFC_KI_KP               0x3F8
#define REG_AFC_RANGE               0x3F9
#define REG_AFC_READ                0x3FA
#define REG_PAN_ID0                 0x112
#define REG_AUTO_STATUS             0x122
#define REG_802_15_4_STATE          0x312
#define REG_AUTO_CFG                0x11F

/* ADF7242 commands */
#define CMD_SPI_NOP                     (0xFF)
#define CMD_SPI_PKT_WR                  (0x10)
#define CMD_SPI_PKT_RD                  (0x30)
#define CMD_SPI_MEM_WR(x)               (0x18 + (x >> 8))
#define CMD_SPI_MEM_RD(x)               (0x38 + (x >> 8))
#define CMD_SPI_MEMR_WR(x)              (0x08 + (x >> 8))
#define CMD_SPI_MEMR_RD(x)              (0x28 + (x >> 8))
#define CMD_SPI_PRAM_WR                 (0x1E)
#define CMD_SPI_PRAM_RD                 (0x3E)

#define CMD_RC_SLEEP                    (0xB1)
#define CMD_RC_IDLE                     (0xB2)
#define CMD_RC_PHY_RDY                  (0xB3)
#define CMD_RC_RX                       (0xB4)
#define CMD_RC_TX                       (0xB5)
#define CMD_RC_MEAS                     (0xB6)
#define CMD_RC_CCA                      (0xB7)
#define CMD_RC_PC_RESET                 (0xC7)
#define CMD_RC_RESET                    (0xC8)
#define CMD_RC_CSMACA                   (0xC1)

/* ADF7242 status masks */
#define RC_PHY_RDY_MASK          (0x03)
#define RC_READY_MASK            (0x20)
#define RC_RX_MASK               (0x04)
#define RC_TX_MASK               (0x05)
/* ADF7242 802.15.4 status */
#define DS_15d4_STATE_EXTRACT_DATA   (0x04)

/* ADF7242 IRQ */
#define IRQ_CCA_COMPLETE       (1 << 0)
#define IRQ_SFD_RX             (1 << 1)
#define IRQ_SFD_TX             (1 << 2)
#define IRQ_RX_PKT_RCVD        (1 << 3)
#define IRQ_TX_PKT_SENT        (1 << 4)
#define IRQ_FRAME_VALID        (1 << 5)
#define IRQ_ADDRESS_VALID      (1 << 6)
#define IRQ_CSMA_CA            (1 << 7)

/* REG_FFILT_CFG */
#define ACCEPT_BEACON_FRAMES   (0x01)
#define ACCEPT_DATA_FRAMES     (1 << 1)
#define ACCEPT_ACK_FRAMES      (1 << 2)
#define ACCEPT_MACCMD_FRAMES   (1 << 3)
#define ACCEPT_RESERVED_FRAMES (1 << 4)
#define ACCEPT_ALL_ADDRESS     (1 << 5)

/* REG_AUTO_CFG */
#define AUTO_ACK_FRAMEPEND     (1 << 0)
#define IS_PANCOORD            (1 << 1)
#define RX_AUTO_ACK_EN         (1 << 3)
#define CSMA_CA_TURNAROUND  (1 << 4)

#define AUTO_TX_TURNAROUND     (1 << 3)
#define ADDON_EN               (1 << 4)
#define PKT_CFG_RESERVED       (1 << 2)
#define AUTO_FCS_OFF            (1 << 0)

#define RX_BUFF_MODE_CONTINUOUS         (0x01)

#define CALL_FUNC(func, cond, expect, error) \
    if (eResult == cond) { \
        if (func != expect) { \
            eResult = error; \
        } \
    }

#define PRAM_PAGESIZE           256

#define BBRAM_SIZE      64
#define MCR_REG_SIZE      256



#define extpa_bias_mode         1
#define extpa_bias_mode_offset  0
#define extpa_bias_src          0
#define extpa_bias_src_offset   3
#define pa_pwr_offset           4

/* Defines receiver sensitivity value */
#define RX_SENSITIVITY          (-95)
/* Defines RSSI upper limit value */
#define RSSI_UL                 (-20)
/* Defines RSSI lower limit value */
#define RSSI_LL                 (-95)
/* Maximum energy detection value */
#define MAX_ED_VAL              0xFF

typedef struct {
  uint16_t pan_id;
  uint16_t short_addr;
  uint8_t ieee_addr[8];
  uint8_t ffilt_cfg;
  uint8_t auto_cfg;
  uint8_t auto_tx1;     //Max CCA retries and MAC retries
  uint8_t auto_tx2;     //Min and Max BE
}ieee_802_15_4_mode_reg_t;

typedef enum {
  IEEE_802_15_4_packet_mode = 0x00,
  IEEE_802_15_4_SPORT_mode = 0x02,
  GFSK_FSK_SPORT_mode = 0x03,
  GFSK_FSK_packet_mode = 0x04
}ADF7242_phy_mode_t;

typedef enum {
  IEEE_802_15_4_AUTO_CSMACA = 0x01,
  IEEE_802_15_4_RX_AUTO_ACK = 0x02,
  IEEE_802_15_4_RX_ON_WHEN_IDLE = 0x04
}ieee_802_15_4_auto_oper_mode_t;

typedef struct {
  ADF7242_phy_mode_t   phy_mode;
  uint32_t   channelFreq;
  uint8_t   ieee_802_15_4_oper_mode; //support of ieee_802_15_4_auto_oper_mode_t
  int8_t txPower;
}ADF7242_trx_config_t;


typedef enum {
  ADF_SUCCESS = 0,
  ADF_SPI_INIT_FAILED,
  ADF_IRQ_INIT_FAILED,
  ADF_SPI_ERROR
}ADF7242_Result_t;


/* Status */

typedef enum {
    SPI_NOT_READY = 0,
    SPI_READY,
} rc_status_spi_ready_t;

typedef enum {
    NO_PENDING_IRQ = 0,
    PENDING_IRQ,
} rc_status_irq_status_t;

typedef enum {
    RC_NOT_READY = 0,
    RC_READY,
} rc_status_rc_ready_t;

typedef enum {
    CHANNEL_BUSY = 0,
    CHANNEL_IDLE,
} rc_status_cca_result_t;

typedef enum {
    RC_STATUS_RESERVED = 0,
    RC_STATUS_IDLE,
    RC_STATUS_MEAS,
    RC_STATUS_PHY_RDY,
    RC_STATUS_RX,
    RC_STATUS_TX,
} rc_status_rc_status_t;


typedef union {
    struct {
        rc_status_rc_status_t       rc_status:4;
        rc_status_cca_result_t      cca_result:1;
        rc_status_rc_ready_t        rc_ready:1;
        rc_status_irq_status_t      irq_status:1;
        rc_status_spi_ready_t       spi_ready:1;
    } bits;
    uint8_t         byte;
}ADF7242_Status_t;

typedef enum {
  TX_SUCCESS = 0,
  TX_SUCCESS_DATA_PEND,
  TX_FAILURE_CSMACA,
  TX_FAILURE_NOACK,
  TX_FAILURE_CFG
}tx_auto_status_t;

/*----------------------------------------------------------------------------*/

extern ieee_802_15_4_mode_reg_t ieee_802_15_4_mode_regs;
extern ADF7242_trx_config_t TRX_config_params;

ADF7242_Result_t SPI_init(void);
ADF7242_Result_t IRQ_init(void);
void ADF7242_irq_handler(void);
void rf_spi_write_fast(uint8_t *pTxbuf, uint8_t txlen);
ADF7242_Result_t rf_spi_transaction(uint8_t *pTxbuf, uint16_t txlen, 
                                        uint8_t *pRxbuf, uint16_t rxlen);
ADF7242_Result_t ADF7242_init(void);
int8_t ADF7242_tx(uint8_t *pTxbuf, uint16_t txlen);
void ADF7242_tx_done(uint8_t status);
ADF7242_Result_t ADF7242_radio_rx_on(void);
void ADF7242_rx_done(uint8_t *pRx_buf, uint16_t rxlen, int8_t rssi, uint8_t lqi);

void ADF7242_update_802_15_4_mode_regs(void);
void ADF7242_SetPALevel(uint8_t power);
void ADF7242_goto_sleep_state(void);
void ADF7242_wakeup_from_sleep(void);
void ADF7242_set_channel(uint32_t channel);
/*---------------------------------------------------------------------------*/

void Enable_TRX_IRQ(void);
void Disable_TRX_IRQ(void);
int8_t PhyWakeUp(void);
void disable_CS3(void);
void adi_initpinmux(void);
uint8_t  ADF7242_is_trx_busy(void);
void ADF7242_Reset(void);

#endif /* _ADF7242_H_ */
