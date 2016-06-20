
/* fec.c - GMAC lwIP driver */

/*
 * Copyright (c) 2015, Wind River Systems, Inc. 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at 
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License.
 */
#ifdef CONFIG_BOARD_FRDM_K64F


#include <lwip/mem.h>
#include <lwip/pbuf.h>
#include <lwip/netif.h>
#include <netif/etharp.h>
#include <lwip/ethip6.h>
#include <lwip/timers.h>

#include <string.h>
#include <nanokernel.h>
#include <microkernel/event.h>
#include <arch/arm/cortex_m/nvic.h>

#include <microkernel.h>
#include "zephyr.h"

#include "fec.h"


#include <pinmux.h>

#include "pinmux-map-k64f.h"

struct fec_config fec_conf = {.event = 0 };


#define RXBD 8
#define TXBD 4

#define RX_INDEX_NEXT(x) (((x) + 1) % RXBD)
#define RX_INDEX_PREV(x) (((x) + RXBD - 1) % RXBD)

#define TX_INDEX_NEXT(x) (((x) + 1) % TXBD)
#define TX_INDEX_PREV(x) (((x) + TXBD - 1) % TXBD)

#define MONITOR_INTERVAL 100 	/* how often to check PHY (in ms) */
#define MONITOR_PHY      0

/* FEC register offsets*/
#define FEC_EIR        0x0004
#define FEC_EIMR       0x0008
#define FEC_RDAR       0x0010
#define FEC_TDAR       0x0014
#define FEC_ECR        0x0024
#define FEC_MMFR       0x0040
#define FEC_MSCR       0x0044

#define FEC_RCR        0x0084
#define FEC_TCR        0x00C4

#define FEC_PALR       0x00E4
#define FEC_PAUR       0x00E8
#define FEC_GAUR       0x0120
#define FEC_GALR       0x0124

#define FEC_TFWR       0x0144
#define FEC_RDSR       0x0180
#define FEC_TDSR       0x0184
#define FEC_MRBR       0x0188

/*FEC register fields*/
/* EIR/EIMR */
#define FEC_EI_BABR      (1<<30)
#define FEC_EI_BABT      (1<<29)
#define FEC_EI_GRA       (1<<28)
#define FEC_EI_TXF       (1<<27)
#define FEC_EI_TXB       (1<<26)
#define FEC_EI_RXF       (1<<25)
#define FEC_EI_RXB       (1<<24)
#define FEC_EI_MII       (1<<23)
#define FEC_EI_EBERR     (1<<22)
#define FEC_EI_LC        (1<<21)
#define FEC_EI_RL        (1<<20)
#define FEC_EI_UN        (1<<19)
#define FEC_EI_PLR       (1<<18)
#define FEC_EI_WAKEUP    (1<<17)
#define FEC_EI_TS_AVAIL  (1<<16)
#define FEC_EI_TS_TIMER  (1<<15)

/* RDAR */
#define FEC_RDA_RDAR     (1<<24)

/* TDAR */
#define FEC_TDA_TDAR     (1<<24)


/* ECR */
#define FEC_EC_DBSWP     (1<<8)
#define FEC_EC_STOPEN    (1<<7)
#define FEC_EC_DBGEN     (1<<6)
#define FEC_EC_EN1588    (1<<4)
#define FEC_EC_SLEEP     (1<<3)
#define FEC_EC_MAGICEN   (1<<2)
#define FEC_EC_ETHEREN   (1<<1)
#define FEC_EC_RESET     (1<<0)

/* MMFR */
#define FEC_MMF_SET_ST(x)   (((x) & 0x00000003) << 30)
#define FEC_MMF_SET_OP(x)   (((x) & 0x00000003) << 28)
#define FEC_MMF_SET_PA(x)   (((x) & 0x0000001F) << 23)
#define FEC_MMF_SET_RA(x)   (((x) & 0x0000001F) << 18)
#define FEC_MMF_SET_TA(x)   (((x) & 0x00000003) << 16)
#define FEC_MMF_SET_DATA(x) ((x) & 0x0000FFFF)
#define FEC_MMF_GET_DATA(reg) ((reg) & 0x0000FFFF)

/* MSCR */
#define FEC_MSC_SET_HOLDTIME(x)  (((x) & 0x00000007) << 8)
#define FEC_MSC_DIS_PRE  (1<<7)
#define FEC_MSC_SET_MII_SPEED(x) (((x) & 0x0000003F) << 1)

/* RCR */
#define FEC_RC_GRS       (1<<31)
#define FEC_RC_NLC       (1<<30)
#define FEC_RC_SET_MAX_FL(x) (((x) & 0x00003FFF) << 16)
#define FEC_RC_GET_MAX_FL(reg) (((reg) >> 16) & 0x00003FFF)
#define FEC_RC_CFEN      (1<<15)
#define FEC_RC_CRCFWD    (1<<14)
#define FEC_RC_PAUFWD    (1<<13)
#define FEC_RC_PADEN     (1<<12)
#define FEC_RC_RMII_10T  (1<<9)
#define FEC_RC_RMII_MODE (1<<8)
#define FEC_RC_FCE       (1<<5)
#define FEC_RC_BC_REJ    (1<<4)
#define FEC_RC_PROM      (1<<3)
#define FEC_RC_MII_MODE  (1<<2)
#define FEC_RC_DRT       (1<<1)
#define FEC_RC_LOOP      (1<<0)

/* TCR */
#define FEC_TC_CRCFWD    (1<<9)
#define FEC_TC_ADDINS    (1<<8)

#define FEC_TC_RFC_PAUSE (1<<4)
#define FEC_TC_TFC_PAUSE (1<<3)
#define FEC_TC_FDEN      (1<<2)
#define FEC_TC_GTS       (1<<0)

/* PALR */

/* PAUR */

/* TFWR */
#define FEC_TFW_STRFWD   (1<<8)
#define FEC_TFW_SET_TFWR(x) ((x) & 0x0000003F)
#define FEC_TFW_GET_TFWR(reg) ((reg) & 0x0000003F)
/* RDSR */

/* TDSR */

/* MRBR */


typedef struct fec_bd {
  u16_t data_len;
  volatile u16_t flags;
#define BD_EMPTY    (1<<15)  /* RX flag */
#define BD_READY    (1<<15)  /* TX flag */
#define BD_WRAP     (1<<13)  /* last descriptor */
#define BD_LAST     (1<<11)  /* last buffer in frame */
#define BD_TC       (1<<10)  /* transmit CRC */
  u8_t  *data_ptr;
  } fec_bd_t;

/* Assumes a single FEC present. Move this to fecif_t if not true. */
fec_bd_t rxbd[RXBD] __attribute__ ((aligned (16)));
fec_bd_t txbd[TXBD] __attribute__ ((aligned (16)));
struct pbuf *rx_pbuf[RXBD];
struct pbuf *tx_pbuf[TXBD];

typedef struct fecif {
  struct eth_addr *ethaddr;
  u32_t *imm; /* FEC base */
#if defined(CONFIG_MICROKERNEL) && NO_SYS
  kevent_t event;
#endif
#if !NO_SYS
  sys_sem_t irqsem;
#endif
  u32_t flags;
#define FEC_F_LINK	1
#define FEC_F_10T	2
#define FEC_F_HD	4
  int tx_free; /* free TX descriptors */
  int tx_head; /* first free TX descriptor */
  int rx_refill; /* where to start scanning for empty descriptors */
  int rx_head; /* first RX descriptor to check */
} fecif_t;


static void fecDisable(struct netif *netif);
static void fecEnable(struct netif *netif);
static void fec_service_tx(struct netif *netif);
static void fec_service_rx(struct netif *netif);

static inline u32_t CSR_READ_4(fecif_t *fec, u16_t addr) {
  volatile u32_t *reg = (u32_t*)(((u8_t*)fec->imm) + addr);
  return reg[0];
}

static inline void CSR_WRITE_4(fecif_t *fec, u16_t addr, u32_t value) {
  volatile u32_t *reg = (u32_t*)(((u8_t*)fec->imm) + addr);
  reg[0] = value;
}

static inline void CSR_SET_4(fecif_t *fec, u16_t addr, u32_t value) {
  volatile u32_t *reg = (u32_t*)(((u8_t*)fec->imm) + addr);
  reg[0] |= value;
}

static inline void CSR_CLEAR_4(fecif_t *fec, u16_t addr, u32_t value) {
  volatile u32_t *reg = (u32_t*)(((u8_t*)fec->imm) + addr);
  reg[0] &= ~value;
}


static err_t rxbd_fill()
{
  int i;

  for (i = 0; i < RXBD ; i++) {
    rx_pbuf[i] = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
    if (NULL == rx_pbuf[i])
      return ERR_MEM;
    /* Buffer must be 16-byte aligned */
    rx_pbuf[i]->payload = (void *)((((u32_t)rx_pbuf[i]->payload) + 15) & ~15);
    rxbd[i].data_ptr = rx_pbuf[i]->payload;
    rxbd[i].flags = BD_EMPTY;
    rxbd[i].data_len =  0;
  }
  
  rxbd[RXBD - 1].flags |= BD_WRAP;
  
  return ERR_OK;

}

static void rxbd_refill(fecif_t *fec)
{

  while (NULL == rx_pbuf[fec->rx_refill]) {
    rx_pbuf[fec->rx_refill] = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
    if (NULL == rx_pbuf[fec->rx_refill])
      return;
    /* Buffer must be 16-byte aligned */
    rx_pbuf[fec->rx_refill]->payload = (void *)((((u32_t)rx_pbuf[fec->rx_refill]->payload) + 15) & ~15);
    rxbd[fec->rx_refill].data_ptr = rx_pbuf[fec->rx_refill]->payload;
    rxbd[fec->rx_refill].flags &= BD_WRAP;
    rxbd[fec->rx_refill].flags |= BD_EMPTY;
    rxbd[fec->rx_refill].data_len =  0;
    
    fec->rx_refill = RX_INDEX_NEXT(fec->rx_refill);
    
  };
  
}

static err_t txbd_fill()
{
  int i;

  for (i = 0; i < TXBD ; i++) {
    txbd[i].flags = 0; /* do not set ready flag */
    tx_pbuf[i] = NULL;
  }
  
  txbd[TXBD - 1].flags |= BD_WRAP ;
  
  return ERR_OK;

  
}

static err_t fec_mii_read(struct netif *netif, u8_t phy, u8_t reg, u16_t *value)
{
  fecif_t *fec;
  int i;
  u32_t eir;

  fec = (fecif_t *)netif->state;

  CSR_WRITE_4(fec, FEC_MSCR, 0);
  CSR_WRITE_4(fec, FEC_MMFR, FEC_MMF_SET_ST(1) | FEC_MMF_SET_OP(2) | FEC_MMF_SET_PA(phy) | FEC_MMF_SET_RA(reg) | FEC_MMF_SET_TA(2));

 /*  DO NOT disable preamble; speed < 30 gives bad reads (155MHz clock?); check HOLDTIME value */
  CSR_WRITE_4(fec, FEC_MSCR, FEC_MSC_SET_HOLDTIME(2) | FEC_MSC_SET_MII_SPEED(30));
  
  for (i = 0; i < 10000; i++) {
    eir = CSR_READ_4(fec, FEC_EIR);
    if (eir & FEC_EI_MII)
      break;
  }
  
  if (eir & FEC_EI_MII) {
    CSR_WRITE_4(fec, FEC_EIR, FEC_EI_MII);
    /* reuse eir variable for data */
    eir = CSR_READ_4(fec, FEC_MMFR);
    *value = FEC_MMF_GET_DATA(eir);
//    printk("read %d:%d = %x i=%d\n", phy, reg, *value, i);
    return ERR_OK;
  } else {
    CSR_WRITE_4(fec, FEC_MSCR, 0);
//    printk("read failed %d:%d = %x i=%d\n", phy, reg, 0xFFFF, i);
    return ERR_TIMEOUT;
  }
}

/* TODO: use this to set ANAR and maybe register 09h if Gigabit PHY present (?) */
static err_t fec_mii_write(struct netif *netif, u8_t phy, u8_t reg, u16_t value)
{
  fecif_t *fec;
  int i;
  u32_t eir;

  fec = (fecif_t *)netif->state;

  CSR_WRITE_4(fec, FEC_MSCR, 0);

  CSR_WRITE_4(fec, FEC_MMFR, FEC_MMF_SET_ST(1) | FEC_MMF_SET_OP(1) | FEC_MMF_SET_PA(phy) | FEC_MMF_SET_RA(reg) | FEC_MMF_SET_TA(2) | \
    FEC_MMF_SET_DATA(value));

 /*  DO NOT disable preamble; speed < 30 gives bad reads (155MHz clock?); check HOLDTIME value */
  CSR_WRITE_4(fec, FEC_MSCR, FEC_MSC_SET_HOLDTIME(2) | FEC_MSC_SET_MII_SPEED(30));
  
  for (i = 0; i < 10000; i++) {
    eir = CSR_READ_4(fec, FEC_EIR);
    if (eir & FEC_EI_MII)
      break;
  }
  
  if (eir & FEC_EI_MII) {
    CSR_WRITE_4(fec, FEC_EIR, FEC_EI_MII);
//    printk("write %d:%d = %x i=%d\n", phy, reg, value, i);
    return ERR_OK;
  } else {
    CSR_WRITE_4(fec, FEC_MSCR, 0);
//    printk("write failed %d:%d = %x i=%d\n", phy, reg, value, i);
    return ERR_TIMEOUT;
  }
}


//TODO: what if link established via parallel detection instead of AN?

static void fec_monitor(void *arg)
{
  struct netif *netif = arg;
  fecif_t *fec = netif->state;
  u16_t bmsr, anar, anlpar;
  int status, rescan = 0;
  u32_t newflags = 0;

//  printk("MONITOR flags=%x fflags=%x\n", netif->flags, fec->flags);

  if (ERR_OK != fec_mii_read(netif, MONITOR_PHY, 1, &bmsr)) 
    goto end;

  if (!(bmsr & 4)) { /* Link */
  status = fec_mii_read(netif, MONITOR_PHY, 1, &bmsr);
  /*Read again*/
  if ((ERR_OK == status) && (bmsr & 4)) { /*Link*/
//    printk("Glitch\n");
    newflags |= FEC_F_LINK;
    rescan = 1;
    } else {
//    printk("Down\n");
    }
  } else {
//  printk("Up\n");
  newflags |= FEC_F_LINK;
  }

  if ((fec->flags & FEC_F_LINK) && !(newflags & FEC_F_LINK))
    goto down;
  
  if (rescan || ((newflags & FEC_F_LINK) && !(fec->flags & FEC_F_LINK))) { /* Link went down-up */
    printk("Link up\n");

  if ( (ERR_OK != fec_mii_read(netif, MONITOR_PHY, 4, &anar)) ||
       (ERR_OK != fec_mii_read(netif, MONITOR_PHY, 5, &anlpar))) /* Can't determine speed*/
       goto down;

    if (anar & anlpar & (1<<8))
      newflags |= 0;
    else if (anar & anlpar & (1<<7))
      newflags |= FEC_F_HD;
    else if (anar & anlpar & (1<<6))
      newflags |= FEC_F_10T;
    else if (anar & anlpar & (1<<5))
      newflags |= FEC_F_10T | FEC_F_HD;

    if (rescan && (newflags != fec->flags)) {/*Speed/duplex change*/
      fec->flags = newflags;
      netif_set_link_down(netif);
      fecDisable(netif);
      fecEnable(netif);
      netif_set_link_up(netif);
    } else {
      fec->flags = newflags;
      fecEnable(netif);
      netif_set_link_up(netif);
    }
  }
  
  goto end;

down:
  printk("Link down\n");
  fec->flags = 0;
  netif_set_link_down(netif);
  fecDisable(netif);
  
end:
  sys_timeout(MONITOR_INTERVAL, fec_monitor, arg);  
}

/* TODO: return status in fecDisable() and fecEnable() */
static void fecDisable(struct netif *netif)
{
  fecif_t *fec;
  int i;

  fec = (fecif_t *)netif->state;

#if FEC_USE_IRQ
  CSR_WRITE_4(fec, FEC_EIMR, 0);
#endif

  CSR_SET_4(fec, FEC_TCR, FEC_TC_GTS);

  /* Wait for GRA interrupt */
  for (i = 0; i < 10000; i++)
    if (FEC_EI_GRA & CSR_READ_4(fec, FEC_EIR))
      break;

  /* Clear interrupt */  
  CSR_WRITE_4(fec, FEC_EIR, FEC_EI_GRA);
  /* Disable FEC */
  CSR_CLEAR_4(fec, FEC_ECR, FEC_EC_ETHEREN);
  
  fec_service_rx(netif);
  fec_service_tx(netif);


  /* Discard unsent frames */
  for (i = 0; i < TXBD ; i++)
    if (NULL != tx_pbuf[i])
      pbuf_free(tx_pbuf[i]);
  txbd_fill();

  for (i = 0; i < RXBD ; i++)
    if (NULL != rx_pbuf[i])
      pbuf_free(rx_pbuf[i]);
  rxbd_fill();

  CSR_CLEAR_4(fec, FEC_TCR, FEC_TC_GTS);

}

static void fecEnable(struct netif *netif)
{
  fecif_t *fec;
  u32_t ten, hd;
  int i;

  fec = (fecif_t *)netif->state;

  /*Link is 10BaseT ?*/
  ten = !!(fec->flags & FEC_F_10T);

  /*Link is half-duplex ?*/
  hd = !!(fec->flags & FEC_F_HD);

  fec->tx_free = TXBD;
  fec->tx_head = 0;
  fec->rx_refill = 0;
  fec->rx_head = 0;

  CSR_SET_4(fec, FEC_ECR, FEC_EC_RESET);

  for (i = 0 ; i < 1000; i++) {
    if (CSR_READ_4(fec, FEC_ECR) & FEC_EC_RESET)
      continue;
    break;
  }
  
  if (1000 == i) /*Can't reset?*/
    return;

  CSR_SET_4(fec, FEC_ECR, FEC_EC_DBSWP);
  
  /* Set the source address for the controller */
  CSR_WRITE_4(fec, FEC_PALR, 0
    | (fec->ethaddr->addr[0] <<24)
    | (fec->ethaddr->addr[1] <<16)
    | (fec->ethaddr->addr[2] <<8)
    | (fec->ethaddr->addr[3] <<0));
 
  CSR_WRITE_4(fec, FEC_PAUR, 0
    | (fec->ethaddr->addr[4] <<24)
    | (fec->ethaddr->addr[5] <<16));

#if LWIP_IPV6_MLD || LWIP_IGMP
  /* Accept all multicast frames. Should implement igmp_mac_filter/mld_mac_filter instead */
  CSR_WRITE_4(fec, FEC_GAUR, 0xFFFFFFFF);
  CSR_WRITE_4(fec, FEC_GALR, 0xFFFFFFFF);
#endif

  /* Set receive buffer size, must be 16 byte aligned, can add 15 bytes for alignment to buffer so substract that */
  CSR_WRITE_4(fec, FEC_MRBR, (PBUF_POOL_BUFSIZE - 15) & ~ 15);

  CSR_WRITE_4(fec, FEC_TFWR, FEC_TFW_STRFWD);

  /* Set RX, TX rings */
  CSR_WRITE_4(fec, FEC_RDSR, (u32_t)rxbd);
  CSR_WRITE_4(fec, FEC_TDSR, (u32_t)txbd);

  /*RCR/TCR*/ 
  CSR_WRITE_4(fec, FEC_RCR, FEC_RC_SET_MAX_FL(1518)|(ten?FEC_RC_RMII_10T:0)|FEC_RC_RMII_MODE|FEC_RC_MII_MODE|(hd?FEC_RC_DRT:0));
  CSR_WRITE_4(fec, FEC_TCR, (hd?0:FEC_TC_FDEN));

    CSR_WRITE_4(fec, FEC_EIR, (0xFFFFFFFF)); /*Clear interrupts*/

  rxbd_refill(fec);
  CSR_SET_4(fec, FEC_ECR, FEC_EC_ETHEREN);

  CSR_WRITE_4(fec, FEC_RDAR, FEC_RDA_RDAR); /* enable RX*/

#if FEC_USE_IRQ
  CSR_WRITE_4(fec, FEC_EIMR, FEC_EI_TXF|FEC_EI_RXF);
#endif

}


static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  int num_desc, start, end, idx, aligned = 1;
  fecif_t *fec;
  struct pbuf *pp;

  fec = (fecif_t *)netif->state;
  
  if (!(fec->flags & FEC_F_LINK)) {
    /* we are getting dupes, lost and out of order packets if we would allow it because fecEnable() resets pointers */
//    printk("OUT with no link\n");
    return ERR_IF;
  }

	if (PBUF_REF == p->type) {
		/* Since we are allocating a new pbuf anyway, align it */
		struct pbuf *q = pbuf_alloc(PBUF_RAW_TX, p->tot_len + 7, PBUF_RAM);
		if (q != NULL) {
			q->payload = (void *)((((u32_t)q->payload) + 7) & ~7);
			/* Set the correct length because pbuf_copy() will not */
			q->len = q->tot_len = p->tot_len;
			if (ERR_OK != pbuf_copy(q, p)) {
				pbuf_free(q);
				q = NULL;
			}
		}
		if (q == NULL) {
                        printk("fec low_level_output: cannot queue copy of PBUF_REF packet (OOM)\n");
			pbuf_free(p);
			return ERR_MEM;
		}
		p = q;
	} else {

  /* Check if the pbuf can be sent as-is (payload fields aligned) , if not - make a copy */
  /* We need to allocate +7 bytes and align it */  

  pp = p;
  do {
//    printk("TX payload %p\n",pp->payload);
    if (((u32_t)pp->payload) & 7) {
      aligned = 0;
      break;
      }
    pp = pp->next;
  } while (pp->tot_len != pp->len);

  if (aligned)
    pbuf_ref(p);
  else {
    pp = pbuf_alloc(PBUF_RAW, p->tot_len + 7, PBUF_RAM);
    if (NULL==pp) {
      printk("pbuf_alloc failed\n");
      return ERR_MEM;
    }
    /*Buffer needs to be 8 byte aligned*/
    pp->payload = (void *)((((u32_t)pp->payload) + 7) & ~7);
    /* Set the correct length because pbuf_copy() will not */
    pp->len = pp->tot_len = p->tot_len;
    pbuf_copy(pp, p);
    p = pp;
  };
  
  }; /*(PBUF_REF == p->type)*/

  num_desc = pbuf_clen(p);
//  printk("TX aligned %d num_desc %d free %d\n", aligned, num_desc, fec->tx_free);

  if (num_desc > fec->tx_free) {
    printk("pbuf_clen(p) > tx_free\n");
    pbuf_free(p);
    return ERR_MEM;
  }

  start = end = fec->tx_head;

  do {
    fec->tx_free--;
    tx_pbuf[end] = p;
    txbd[end].data_len = p->len;
    txbd[end].data_ptr = p->payload;
    if (p->tot_len != p->len) { /* packet queues something something*/
      end = TX_INDEX_NEXT(end);
      p = p->next;    
      } else break;
  } while(1); 
  
  idx = end;
  do {
    txbd[idx].flags &= BD_WRAP ; /* keep wrap flag */
    if (idx == end)
      txbd[idx].flags  |= BD_LAST | BD_TC;
    txbd[idx].flags  |= BD_READY;
    
    if (idx != start) {
      idx = TX_INDEX_PREV(idx);
    } else break;
  } while (1);

  fec->tx_head = TX_INDEX_NEXT(end);

  CSR_WRITE_4(fec, FEC_TDAR, FEC_TDA_TDAR);
  return ERR_OK;
}

static void fec_service_tx(struct netif *netif)
{
  int end, prev, clean = 0;
  fecif_t *fec;

  fec = (fecif_t *)netif->state;
  
  end = fec->tx_head;
  
  do {
    end = TX_INDEX_PREV(end);
    if (!clean && /* short-circuit the rest */
        (txbd[end].flags & BD_LAST) &&
        (NULL != tx_pbuf[end]) && /* has pbuf */
        !(txbd[end].flags & BD_READY))  /* !ready means transmitted */
      clean = 1; /*can clean descriptors from now on*/
    } while(!clean && tx_pbuf[end]);

    while (clean && tx_pbuf[end]) {
      prev = TX_INDEX_PREV(end);
      if ( (NULL==tx_pbuf[prev]) || (txbd[prev].flags & BD_LAST) ) /* previous BD is tail or not used: we're head */
        pbuf_free(tx_pbuf[end]);
      tx_pbuf[end] = NULL;
      fec->tx_free++;
      end = prev;
      }
}

static void fec_service_rx(struct netif *netif)
{
  int end;
  fecif_t *fec;
  struct pbuf *p;

  fec = (fecif_t *)netif->state;

  end = fec->rx_head;

  while (!(rxbd[end].flags & BD_EMPTY) && (NULL != rx_pbuf[end])) {
    if (!(rxbd[end].flags & BD_LAST)) {
      /*keep looking*/
      end = RX_INDEX_NEXT(end);
      continue;
      }

    p = rx_pbuf[fec->rx_head];
    p->tot_len = p->len = rxbd[fec->rx_head].data_len;
    rx_pbuf[fec->rx_head] = NULL;

    while (fec->rx_head != end) {
      fec->rx_head = RX_INDEX_NEXT(fec->rx_head);
      if (fec->rx_head == end) /* last BD contains total packet length */
        rx_pbuf[fec->rx_head]->tot_len =rx_pbuf[fec->rx_head]->len = rxbd[fec->rx_head].data_len - p->tot_len;
      else
        rx_pbuf[fec->rx_head]->tot_len = rx_pbuf[fec->rx_head]->len = rxbd[fec->rx_head].data_len;
      pbuf_cat(p, rx_pbuf[fec->rx_head]);
      rx_pbuf[fec->rx_head] = NULL;
    };
    
    fec->rx_head = RX_INDEX_NEXT(fec->rx_head);

    netif->input(p, netif);

    /*keep looking*/
    end = RX_INDEX_NEXT(end);
  }
  
}

#if !NO_SYS && FEC_USE_IRQ
static
#endif
void fec_service(struct netif *netif)
{
  fecif_t *fec;
  u32_t eir;

  fec = (fecif_t *)netif->state;

  eir = CSR_READ_4(fec, FEC_EIR);

  if (eir & FEC_EI_TXF) {
    fec_service_tx(netif);
    CSR_WRITE_4(fec, FEC_EIR, FEC_EI_TXF);
    }

  if (eir & FEC_EI_RXF) {
    fec_service_rx(netif);
    CSR_WRITE_4(fec, FEC_EIR, FEC_EI_RXF);
    }

  /* refill RX ring, write RDAR */
  /* rxbd_refill() should start refilling where the FEC looks for the next frame */
  /* if this is done only on RXF interrupts, driver won't receive after RX overflow */
  rxbd_refill(fec);
  if (fec->flags & FEC_F_LINK)
    CSR_WRITE_4(fec, FEC_RDAR, FEC_RDA_RDAR);

#if FEC_USE_IRQ
  irq_enable(83);
  irq_enable(84);
#endif

}

#if !NO_SYS && FEC_USE_IRQ
static void fec_thread(void *arg)
{
	struct netif *netif = arg;
	fecif_t *state = netif->state;

	do {
		sys_arch_sem_wait(&state->irqsem, 0);
		fec_service(netif);
	} while(1);
}
#endif

static err_t low_level_init (struct netif *netif)
{
  fecif_t *fec;
  int i;

  fec = (fecif_t *)netif->state;

/*We probably don't need all of this, maybe just reset and setup PHY */  
  if (ERR_OK != rxbd_fill())
    return ERR_MEM;

  if (ERR_OK != txbd_fill())
    return ERR_MEM;

  fec->tx_free = TXBD;
  fec->rx_refill = 0;
  fec->tx_head = fec->rx_head = 0;

  CSR_SET_4(fec, FEC_ECR, FEC_EC_RESET);

  for (i = 0 ; i < 100; i++) {
    if (CSR_READ_4(fec, FEC_ECR) & FEC_EC_RESET)
      continue;
    break;
  }
  
  if (100 == i) /*Can't reset?*/
    return ERR_IF;

  return ERR_OK;
}

#if FEC_USE_IRQ
static void fec_tx_irqhandler(void *arg)
{
//printk("fec_irqhandler\n");
	fecif_t *state = arg;
	irq_disable(83);
#if defined(CONFIG_MICROKERNEL) && NO_SYS
	isr_event_send(state->event);
#endif
#if !NO_SYS
	sys_sem_signal(&state->irqsem);
#endif
}

static void fec_rx_irqhandler(void *arg)
{
//printk("fec_irqhandler\n");
	fecif_t *state = arg;
	irq_disable(84);
#if defined(CONFIG_MICROKERNEL) && NO_SYS
	isr_event_send(state->event);
#endif
#if !NO_SYS
	sys_sem_signal(&state->irqsem);
#endif
}
#endif


#include <sys_io.h>
#include <pinmux.h>
//#include <pinmux/pinmux.h>
//#include <k64f_drivers/k64f_pinmux.h>

/*pins to be configured as RMII*/
static uint8_t enet_pin_set[] = {
	K64F_PIN_ID(PCR_PORT_A,5 ),
	K64F_PIN_ID(PCR_PORT_A,12),
	K64F_PIN_ID(PCR_PORT_A,13),
	K64F_PIN_ID(PCR_PORT_A,14),
	K64F_PIN_ID(PCR_PORT_A,15),
	K64F_PIN_ID(PCR_PORT_A,16),
	K64F_PIN_ID(PCR_PORT_A,17),
	K64F_PIN_ID(PCR_PORT_A,18),
	K64F_PIN_ID(PCR_PORT_B,0 ),
	K64F_PIN_ID(PCR_PORT_B,1 ),
};

static void fec_pinmux_config(void)
{
  
  int i;
  
	/*open clock gate*/
#define CONFIG_SIM_BASE_ADDR (0x40047000)
#define K64F_SCGC_2_ADDR (CONFIG_SIM_BASE_ADDR + 0x102c)
#define K64F_CLOCK_GATE_ENET (1<<0)

	sys_write32(sys_read32(K64F_SCGC_2_ADDR)|K64F_CLOCK_GATE_ENET, K64F_SCGC_2_ADDR);

	/*set pinmux*/
	struct device *pinmux;
	
	pinmux = device_get_binding(PINMUX_NAME);
	
	for (i=0; i<sizeof(enet_pin_set)/sizeof(uint8_t);i++) {
	  	uint32_t muxv = _k64f_get_mux_config(enet_pin_set[i], e_K64F_PORT_FUNCTION_RMII);
  		if (muxv == _K64F_MUX_INVALID) {
  			//DBG_PRINT("%s bad ENET pin mapping %d\n", __FUNCTION__, pin);
  			//return ERR_VAL;
  			return;
  		}
  		else {
  			if (pinmux_pin_set(pinmux, K64F_PIN_ID_2_INDEX(enet_pin_set[i]), (muxv<<8))) {
  				//DBG_PRINT("%s pinmux setting failed for ENET pin %d\n",__FUNCTION__, pin);
  			//	return ERR_VAL;
  			return;
  			} else {
  				//DBG_PRINT("%s pinmux setting success for ENET pin %d\n",__FUNCTION__, pin);

  			}
  		} 
  		
  		/*
		if (pinmux_set_pin(pinmux, enet_pin_set[i], e_K64F_PORT_FUNCTION_RMII)) {
		    //DBG_PRINT("failed\n");
		    return ERR_VAL;
	    } else {
		    //DBG_PRINT("success\n");
	    }*/
	}
}
err_t fec_init(struct netif *netif)
{
  fecif_t *fecif;
#if FEC_USE_IRQ && defined(CONFIG_MICROKERNEL) && NO_SYS
  struct fec_config *config = netif->state;
#endif

  if (NULL==netif)
    return ERR_MEM;


#if defined(CONFIG_SOC_FSL_FRDM_K64F)

/*Assumes configuration is saved in flash */
#ifdef NV_FRDM
 if ((0x01 != __vars_frdm_k64f[0])||(0xfe != __vars_frdm_k64f[1])) {
 _DEBUG_PRINT("NO VARS @0x420\n");
#if 0
 while(1);
#else
 netif->hwaddr[0]=8;
 netif->hwaddr[5]=8;
#endif
 } else {
 memcpy(&netif->hwaddr[0],&__vars_frdm_k64f[2],6);
 }
#else
 netif->hwaddr[0]=8;
 netif->hwaddr[5]=8;
#endif
#endif


  fec_pinmux_config();

  fecif = mem_malloc(sizeof(fecif_t));

  if (NULL == fecif)
    return ERR_MEM;

  netif->state = fecif;
  netif->name[0] = 'f';
  netif->name[1] = 'e';
  netif->mtu = 1500;
  netif->hwaddr_len = 6;

  netif->linkoutput = low_level_output;
#if LWIP_IPV4
  netif->output = etharp_output;
#endif
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif

  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP;

  fecif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);
  
  fecif->imm = (u32_t*)0x400C0000;

  if (ERR_OK != low_level_init(netif))
    return ERR_IF;

#if FEC_USE_IRQ

#if defined(CONFIG_MICROKERNEL) && NO_SYS
	fecif->event = config->event;
#endif
#if !NO_SYS
	sys_sem_new(&fecif->irqsem, 0);
	sys_thread_new("fec", fec_thread, netif, 1024, 10);
#endif
	printk("Connecting irq 83\n");
	if (irq_connect(83, 4, fec_tx_irqhandler, fecif) == -1)
		return ERR_IF;
	printk("Connecting irq 84\n");
	if (irq_connect(84, 3, fec_rx_irqhandler, fecif) == -1)
		return ERR_IF;
	printk("Connected irqs succesfully\n");
	irq_enable(83);
	irq_enable(84);
	/*Interrupts are unmasked in fecEnable*/
#endif

  sys_timeout(MONITOR_INTERVAL, fec_monitor, netif);  

  return ERR_OK;
}

#endif /*  CONFIG_BOARD_FRDM_K64F */
