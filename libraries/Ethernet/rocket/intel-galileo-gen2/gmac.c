/* gmac.c - GMAC lwIP driver */

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
#ifdef CONFIG_BOARD_GALILEO

#include <lwip/mem.h>
#include <lwip/pbuf.h>
#include <lwip/netif.h>
#include <netif/etharp.h>
#include <lwip/ethip6.h>
#include <lwip/timers.h>

#include <string.h>
#include <nanokernel.h>

#include <drivers/pci/pci.h>
#include <drivers/pci/pci_mgr.h>
#include <microkernel/event.h>

#ifndef GMAC_DEBUG
#define GMAC_DEBUG LWIP_DBG_LEVEL_SERIOUS
#endif

//#undef LWIP_DEBUGF
//#define LWIP_DEBUGF(dummy, str)  printf(str)


#include "gmac.h"

struct gmac_config gmac_conf = { .pci.bus = 0, .pci.dev = 20, .pci.function = 6 , .event = 0 };


#define MONITOR_INTERVAL	2000	/* how often to check PHY (in ms) */
#define MONITOR_PHY			1

#define GMAC_TIMEOUT		1000000

#define RXBD	8
#define TXBD	8

#define RX_INDEX_NEXT(x) (((x) + 1) % RXBD)
#define RX_INDEX_PREV(x) (((x) + RXBD - 1) % RXBD)

#define TX_INDEX_NEXT(x) (((x) + 1) % TXBD)
#define TX_INDEX_PREV(x) (((x) + TXBD - 1) % TXBD)

#define DESC_ALIGN 16

#define GMAC_CONF	0x00
#define GMAC_FFILT	0x04

#define GMAC_MIIADDR	0x10
#define GMAC_MIIDATA	0x14

#define GMAC_ADDRH	0x40
#define GMAC_ADDRL	0x44

#define GMAC_DMA_BUSMOD	0x1000
#define GMAC_DMA_TPD	0x1004
#define GMAC_DMA_RPD	0x1008
#define GMAC_DMA_RDL	0x100C
#define GMAC_DMA_TDL	0x1010
#define GMAC_DMA_STATUS	0x1014
#define GMAC_DMA_OPMOD	0x1018
#define GMAC_DMA_INTEN	0x101C

#define FIELD_SET(name, value) (((value) & name##_MASK) << (name##_SHIFT))
#define FIELD_GET(value, name) (((value) >> name##_SHIFT) & (name##_MASK))

#define BIT_CONF_FES	14
#define BIT_CONF_DM	11
#define BIT_CONF_ACS	7
#define BIT_CONF_TE	3
#define BIT_CONF_RE	2

#define CONF_FES	(1<<BIT_CONF_FES)
#define CONF_DM		(1<<BIT_CONF_DM)
#define CONF_ACS	(1<<BIT_CONF_ACS)
#define CONF_TE		(1<<BIT_CONF_TE)
#define CONF_RE		(1<<BIT_CONF_RE)

#define BIT_FFILT_PM	4

#define FFILT_PM	(1<<BIT_FFILT_PM)

#define BIT_MIIADDR_GW	1
#define BIT_MIIADDR_GB	0

#define MIIADDR_PA_MASK		0x1F
#define MIIADDR_PA_SHIFT	11
#define MIIADDR_GR_MASK		0x1F
#define MIIADDR_GR_SHIFT	6
#define MIIADDR_CR_MASK		0x0F
#define MIIADDR_CR_SHIFT	2
#define MIIADDR_GW		(1<<BIT_MIIADDR_GW)
#define MIIADDR_GB		(1<<BIT_MIIADDR_GB)

#define MIIDATA_GD_MASK		0xFFFF
#define MIIDATA_GD_SHIFT	0

#define BIT_DMA_BUSMOD_SWR	0

#define DMA_BUSMOD_SWR		(1<<BIT_DMA_BUSMOD_SWR)

#define BIT_DMA_STATUS_NI	16
#define BIT_DMA_STATUS_RI	6
#define BIT_DMA_STATUS_TI	0

#define DMA_STATUS_NI		(1<<BIT_DMA_STATUS_NI)
#define DMA_STATUS_RI		(1<<BIT_DMA_STATUS_RI)
#define DMA_STATUS_TI		(1<<BIT_DMA_STATUS_TI)

#define BIT_DMA_OPMOD_RSF	25
#define BIT_DMA_OPMOD_TSF	21
#define BIT_DMA_OPMOD_ST	13
#define BIT_DMA_OPMOD_SR	1

#define DMA_OPMOD_RSF		(1<<BIT_DMA_OPMOD_RSF)
#define DMA_OPMOD_TSF		(1<<BIT_DMA_OPMOD_TSF)
#define DMA_OPMOD_ST		(1<<BIT_DMA_OPMOD_ST)
#define DMA_OPMOD_SR		(1<<BIT_DMA_OPMOD_SR)

#define BIT_DMA_INTEN_NIE	16
#define BIT_DMA_INTEN_RIE	6
#define BIT_DMA_INTEN_TIE	0

#define DMA_INTEN_NIE		(1<<BIT_DMA_INTEN_NIE)
#define DMA_INTEN_RIE		(1<<BIT_DMA_INTEN_RIE)
#define DMA_INTEN_TIE		(1<<BIT_DMA_INTEN_TIE)

/* DMA Tx Desc status descriptor */

#define TX_DESC_DEFERRED       (1 << 0)     /* Status : Tx deferred */
#define TX_DESC_UNDERFLOW_ERR  (1 << 1)     /* Underflow Err */
#define TX_DESC_EXCE_DEF_ERR   (1 << 2)     /* Excessive Tx deferral Err */
#define TX_DESC_COL_COUNT(x)   (((x) >> 3) & 0xf) /* Collision count */
#define TX_DESC_VLAN_FRAME     (1 << 7)     /* Status : VLAN frame */
#define TX_DESC_EXCE_COL       (1 << 8)     /* Excessive collisions Err */
#define TX_DESC_LATE_COL       (1 << 9)     /* Late collision Err */
#define TX_DESC_NO_CARRIER     (1 << 10)    /* No carrier Err */
#define TX_DESC_LOSS_CARRIER   (1 << 11)    /* Loss carrier Err */
#define TX_DESC_PAYLOAD_ERR    (1 << 12)    /* Addr/Payload csum Err */
#define TX_DESC_FRAME_FLUSH    (1 << 13)    /* Frame flushed Err */
#define TX_DESC_JAB_TIMEOUT    (1 << 14)    /* Jabber timeout Err */
#define TX_DESC_ERR_SUMMARY    (1 << 15)    /* Tx Error summary */
#define TX_DESC_HEADER_ERR     (1 << 16)    /* IP header csum Err */
#define TX_DESC_TIMESTAMP_ST   (1 << 17)
#define TX_DESC_CHAINED        (1 << 20)
#define TX_DESC_RING_END       (1 << 21)
#define TX_DESC_CSUM_INSERT(x) ((x) << 22)
#define TX_DESC_TIMESTAMP_EN   (1 << 25)
#define TX_DESC_PADDING_DIS    (1 << 26)
#define TX_DESC_CRC_DIS        (1 << 27)
#define TX_DESC_FIRST          (1 << 28)
#define TX_DESC_LAST           (1 << 29)
#define TX_DESC_INT            (1 << 30)
#define TX_DESC_OWNBYDMA       (1 << 31)

/* DMA Tx Desc control descriptor */

#define TX_BUFFER1_SIZE(x)       (((x) & 0x1FFF) << 0)
#define TX_BUFFER2_SIZE(x)       (((x) & 0x1FFF) << 16)

/* DMA Rx Desc status descriptor */

#define RX_DESC_PAYLOAD_CSUM_ERR (1 << 0)
#define RX_DESC_CRC_ERR          (1 << 1)     /* CRC error */
#define RX_DESC_DRIBBLING        (1 << 2)     /* Dribbling error */
#define RX_DESC_GMII_ERR         (1 << 3)     /* GMII Rx Error */
#define RX_DESC_RX_WATCHDOG      (1 << 4)     /* Rx watchdog error */
#define RX_DESC_FRAMEETHER       (1 << 5)
#define RX_DESC_LATE_COL         (1 << 6)     /* late collision error */
#define RX_DESC_IPC_CSUM_ERR     (1 << 7)     /* IPC csum Err/Giant frame */
#define RX_DESC_LAST             (1 << 8)
#define RX_DESC_FIRST            (1 << 9)
#define RX_DESC_VLAN_TAG         (1 << 10)    /* Status : VLAN frame tagged */
#define RX_DESC_OVERFLOW_ERR     (1 << 11)    /* overflow error */
#define RX_DESC_LENGTH_ERR       (1 << 12)    /* length_error */
#define RX_DESC_SAFILTER_FAIL    (1 << 13)    /* Source Address filter fail */
#define RX_DESC_DESC_ERR         (1 << 14)    /* Descriptor error */
#define RX_DESC_ERROR_SUMMARY    (1 << 15)    /* Rx Error summary */
#define RX_DESC_FRMLEN_MASK     (((x) & 0x3FFF) << 16)
#define RX_DESC_FRMLEN_LEN_MASK  (0x3FFF)
#define RX_DESC_FRMLEN_LEN_SHIFT (16)
#define RX_DESC_DAFILTER_FAIL    (1 << 30)    /* Dest Address filter fail */
#define RX_DESC_OWNBYDMA         (1 << 31)

#define RX_DESC_ERR_FRAME  (RX_DESC_ERROR_SUMMARY | RX_DESC_PAYLOAD_CSUM_ERR | \
                             RX_DESC_CRC_ERR)

/* DMA Rx Desc control descriptor */

#define RX_DESC_CTL_CHAINED      (1 << 14)
#define RX_DESC_CTL_RINGEND      (1 << 15)
#define RX_BUFFER1_SIZE(x)       (((x) & 0x1FFF) << 0)
#define RX_BUFFER2_SIZE(x)       (((x) & 0x1FFF) << 16)
#define RX_DESC_CTL_INT_DIS      (1 << 31)

struct gmacTxDesc
    {
    volatile u32_t txStatus;
    volatile u32_t txControl;
    volatile u32_t pointer;
    u32_t nextDesc;
    };

struct gmacRxDesc
    {
    volatile u32_t rxStatus;
    volatile u32_t rxControl;
    volatile u32_t pointer;
    u32_t nextDesc;
    };

struct gmac_if {
	struct pci_dev_info	pci_dev;
#if defined(CONFIG_MICROKERNEL)
	kevent_t		event;
#endif
	u32_t			flags;
#define ETH_LINK	1
#define ETH_10T		2
#define ETH_HD		4
	void* 			gmac_rx_desc_mem;
	struct gmacRxDesc	*gmac_rx;
	void* 			gmac_tx_desc_mem;
	struct gmacTxDesc	*gmac_tx;
	struct pbuf		*rx_pbuf[RXBD];
	struct pbuf		*tx_pbuf[TXBD];
	int			rx_head; /* first descriptor that was used by DMA engine */
	int			rx_refill; /* first descriptor to check if empty (no buffer) */
	int			tx_free;  /* number of free TX descriptors */
	int 			tx_head;  /* first free TX descriptor */
};

static inline u32_t CSR_READ_4(struct gmac_if *gmac, u16_t addr) {
	return sys_read32(gmac->pci_dev.addr + addr);
}

static inline void CSR_WRITE_4(struct gmac_if *gmac, u16_t addr, u32_t value) {
	sys_write32(value, gmac->pci_dev.addr + addr);
}

static inline void CSR_SET_BIT(struct gmac_if *gmac, u16_t addr, int bit) {
	sys_set_bit(gmac->pci_dev.addr + addr, bit);
}

static inline void CSR_CLEAR_BIT(struct gmac_if *gmac, u16_t addr, int bit) {
	sys_clear_bit(gmac->pci_dev.addr + addr, bit);
}


static inline int CSR_TEST_BIT(struct gmac_if *gmac, u16_t addr, int bit) {
	return sys_test_bit(gmac->pci_dev.addr + addr, bit);
}


static void gmacEnable( struct gmac_if *state)
{
	u32_t conf = CSR_READ_4(state, GMAC_CONF) | CONF_RE | CONF_TE;

	if (state->flags & ETH_10T)
		conf &= ~ CONF_FES;
	else
		conf |= CONF_FES;
	if (state->flags & ETH_HD)
		conf &= ~CONF_DM;
	else
		conf |= CONF_DM;

	CSR_WRITE_4(state, GMAC_CONF, conf);

}

static void gmacDisable( struct gmac_if *state)
{
	CSR_WRITE_4(state, GMAC_CONF, CSR_READ_4(state, GMAC_CONF) & ~CONF_RE & ~CONF_TE);
}

static err_t mii_read(struct netif *netif, u8_t phy, u8_t reg, u16_t *value)
{
	struct gmac_if *state = netif->state;
	u32_t addr;
	int i;

	*value = 0xFFFF;

	if (CSR_TEST_BIT(state, GMAC_MIIADDR, BIT_MIIADDR_GB)) {/* MII interface is busy */
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_WARNING,
			("gmac mii_read: Busy\n"));
		return ERR_IF;
	}


	addr = FIELD_SET(MIIADDR_CR, 2) | FIELD_SET(MIIADDR_GR, reg) | FIELD_SET(MIIADDR_PA, phy) | MIIADDR_GB;
	CSR_WRITE_4(state, GMAC_MIIADDR, addr);

	for (i = 0; CSR_TEST_BIT(state, GMAC_MIIADDR, BIT_MIIADDR_GB) && (i < GMAC_TIMEOUT); i++);

	if (CSR_TEST_BIT(state, GMAC_MIIADDR, BIT_MIIADDR_GB)) {
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_WARNING,
			("gmac mii_read: Timeout\n"));
		return ERR_TIMEOUT;
	}

	*value = FIELD_GET(CSR_READ_4(state, GMAC_MIIDATA), MIIDATA_GD);
	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL,
		("gmac mii_read: OK %d:%d = %04x i=%d\n", phy, reg, *value, i));

	return ERR_OK;
}

static void gmac_monitor(void *arg)
{
	struct netif *netif = arg;
	struct gmac_if *state = netif->state;
	u16_t bmsr, anar, anlpar;
	int status, rescan = 0;
	u32_t newflags = 0;

	/*
	 * Do not hard code the PHY address in the driver. There are 2 GMACs on the
	 * Quark, with only one that has a PHY attached.
	 */
	 
  	if (ERR_OK != mii_read(netif, MONITOR_PHY, 1, &bmsr)) {
  		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_WARNING,
			("gmac monitor: Cannot read BMSR\n"));
  		goto down;
	}

	if (!(bmsr & 4)) { /* No link */
		status = mii_read(netif, MONITOR_PHY, 1, &bmsr);
		/*Read again*/
		if ((ERR_OK == status) && (bmsr & 4)) { /* Link went up-down-up */
			LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL,
				("gmac monitor: Glitch\n"));
			newflags |= ETH_LINK;
			rescan = 1;
		} else {
			LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL,
				("gmac monitor: Down\n"));
		}
	} else {
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL,
			("gmac monitor: Up\n"));
		newflags |= ETH_LINK;
	}

	if ((state->flags & ETH_LINK) && !(newflags & ETH_LINK))
		goto down;
  
	if (rescan || ((newflags & ETH_LINK) && !(state->flags & ETH_LINK))) { /* Link went down-up or up-down-up */
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_ALL,
			("gmac monitor: Link up\n"));

		if ( (ERR_OK != mii_read(netif, MONITOR_PHY, 4, &anar)) ||
			(ERR_OK != mii_read(netif, MONITOR_PHY, 5, &anlpar))) { /* Can't determine speed */
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_WARNING,
			("gmac monitor: Cannot read ANAR/ANLPAR\n"));
			goto down;
		}

		if (anar & anlpar & (1<<8))
			newflags |= 0;
		else if (anar & anlpar & (1<<7))
			newflags |= ETH_HD;
		else if (anar & anlpar & (1<<6))
			newflags |= ETH_10T;
		else if (anar & anlpar & (1<<5))
			newflags |= ETH_10T | ETH_HD;

		if (rescan && (newflags != state->flags)) {/* Maybe speed/duplex changed */
			netif_set_link_down(netif);
			gmacDisable(state);
			}
		state->flags = newflags;
		gmacEnable(state);
		netif_set_link_up(netif);
	}
  
goto end;

down:
	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_ALL,
		("gmac monitor: Link down\n"));
	state->flags = 0;
	netif_set_link_down(netif);
	gmacDisable(state);
  
end:
	sys_timeout(MONITOR_INTERVAL, gmac_monitor, arg);  
}

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	int num_desc, start, end, idx;
	struct gmac_if *state = netif->state;
	u32_t status;

	if (!(state->flags & ETH_LINK)) {
		return ERR_IF;
	}


	if (PBUF_REF == p->type) {
		struct pbuf *q = pbuf_alloc(PBUF_RAW_TX, p->tot_len, PBUF_RAM);
		if (q != NULL)
			if (ERR_OK != pbuf_copy(q, p)) {
				pbuf_free(q);
				q = NULL;
			}
		if (q == NULL) {
			LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_WARNING,
				("gmac low_level_output: cannot queue copy of PBUF_REF packet (OOM)\n"));
			pbuf_free(p);
			return ERR_MEM;
		}
		p = q;
	} else
	pbuf_ref(p);

	num_desc = pbuf_clen(p);

	if (num_desc > state->tx_free) {
		/*we could maybe try coalesce the pbuf */
		printk("pbuf_clen(p) > tx_free\n");
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_WARNING,
			("gmac low_level_output: pbuf_clen(p) > tx_free\n"));
		pbuf_free(p);
		return ERR_MEM;
	}

	start = end = state->tx_head;

	do { /* Set size and buffer */
		state->tx_free--;
		state->tx_pbuf[end] = p;
		state->gmac_tx[end].txControl = TX_BUFFER1_SIZE(p->len);
		state->gmac_tx[end].pointer = (u32_t)p->payload;
		if (p->tot_len != p->len) { /* packet queues something something*/
			end = TX_INDEX_NEXT(end);
			p = p->next;    
		} else break;
	} while(1); 

	idx = end;
	do { /* go backwards and set status, so when first buffer is passed to DMA, all the others are correct */
		status = TX_DESC_OWNBYDMA | TX_DESC_CHAINED | TX_DESC_INT;
		if (idx == end)
			status |=  TX_DESC_LAST;
		if (idx != start) {
			state->gmac_tx[idx].txStatus  = status;
			idx = TX_INDEX_PREV(idx);
		} else  {
			state->gmac_tx[idx].txStatus  = status | TX_DESC_FIRST;
			break;
		}
	} while (1);

	state->tx_head = TX_INDEX_NEXT(end);
	CSR_SET_BIT(state, GMAC_DMA_OPMOD, BIT_DMA_OPMOD_ST);
	CSR_WRITE_4(state, GMAC_DMA_TPD, 1);
	return ERR_OK;
}

static void rxbd_refill (struct gmac_if *state)
{
	 while (NULL == state->rx_pbuf[state->rx_refill]) {
	 	state->rx_pbuf[state->rx_refill] = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
	 	if (NULL == state->rx_pbuf[state->rx_refill]) {
	 		return;
	 		}
		state->gmac_rx[state->rx_refill].rxControl = RX_BUFFER1_SIZE(state->rx_pbuf[state->rx_refill]->len) | RX_DESC_CTL_CHAINED;
		state->gmac_rx[state->rx_refill].pointer   = (u32_t)state->rx_pbuf[state->rx_refill]->payload;
		state->gmac_rx[state->rx_refill].rxStatus = RX_DESC_OWNBYDMA;

		state->rx_refill = RX_INDEX_NEXT(state->rx_refill);
		};
  

}

static void gmac_service_tx(struct netif *netif)
{
	int end, prev, clean = 0;
	struct gmac_if *state = netif->state;

	end = state->tx_head;

	/*TODO: check state of the packet (in TDES0), maybe update SNMP counters */

	do {
		end = TX_INDEX_PREV(end);
		if (!clean && /* short-circuit the rest */
			(state->gmac_tx[end].txStatus & TX_DESC_LAST) &&
			(NULL != state->tx_pbuf[end]) && /* has pbuf */
			!(state->gmac_tx[end].txStatus & TX_DESC_OWNBYDMA))  /* !ready means transmitted */
			clean = 1; /*can clean descriptors from now on*/
    } while(!clean && state->tx_pbuf[end]);

    while (clean && state->tx_pbuf[end]) {
      prev = TX_INDEX_PREV(end);
		if ( (NULL==state->tx_pbuf[prev]) || (state->gmac_tx[prev].txStatus & TX_DESC_LAST)) {
			/* previous BD is tail or not used: we're head */
			LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL,
				("gmac service_tx: sent %d bytes\n", state->tx_pbuf[end]->tot_len));
        pbuf_free(state->tx_pbuf[end]);
		}
      state->tx_pbuf[end] = NULL;
      state->tx_free++;
      end = prev;
      }
}

void gmac_service_rx(struct netif *netif)
{
	struct gmac_if *state = netif->state;
	int end;
	struct pbuf *p;
	u32_t status;
	end = state->rx_head;
	/* Should check RX_DESC_FIRST on first descriptor, maybe */
	while (!(state->gmac_rx[end].rxStatus & RX_DESC_OWNBYDMA) && (NULL != state->rx_pbuf[end])) {

		if (!(state->gmac_rx[end].rxStatus & RX_DESC_LAST)) {
			end = RX_INDEX_NEXT(end);
			continue;
		}

		p = state->rx_pbuf[state->rx_head];
		p->tot_len = p->len = (state->gmac_rx[state->rx_head].rxStatus >> RX_DESC_FRMLEN_LEN_SHIFT) & RX_DESC_FRMLEN_LEN_MASK;
		state->rx_pbuf[state->rx_head] = NULL;
		while (state->rx_head != end) {
			state->rx_head = RX_INDEX_NEXT(state->rx_head);
			state->rx_pbuf[state->rx_head]->len =
				(state->gmac_rx[state->rx_head].rxStatus >> RX_DESC_FRMLEN_LEN_SHIFT) & RX_DESC_FRMLEN_LEN_MASK;

			/* descriptor contains frame length received so far, substract previous buffers length */
			state->rx_pbuf[state->rx_head]->len -= p->tot_len;
			state->rx_pbuf[state->rx_head]->tot_len = state->rx_pbuf[state->rx_head]->len;
			pbuf_cat(p, state->rx_pbuf[state->rx_head]);
			state->rx_pbuf[state->rx_head] = NULL;
		}
		status = state->gmac_rx[state->rx_head].rxStatus;
		state->rx_head = RX_INDEX_NEXT(state->rx_head);

		if ( status & RX_DESC_ERROR_SUMMARY) {
			LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_WARNING,
				("gmac service_rx: error %08x\n", status));
			/* we could check status when we reach RX_DESC_LAST above and drop the packet, but it would complicate refill */
			pbuf_free(p);
		} else {
			LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL,
				("gmac service_rx: received %d bytes\n", p->tot_len));
			netif->input(p, netif);
		}

		end = RX_INDEX_NEXT(end);
	}
}

/* It seems that you need to ack both RI/TI and NI */
void gmac_service(struct netif *netif)
{
	struct gmac_if *state = netif->state;
	u32_t sr = CSR_READ_4(state, GMAC_DMA_STATUS);

   
	if (sr & DMA_STATUS_RI) {
		gmac_service_rx(netif);
		CSR_WRITE_4(state, GMAC_DMA_STATUS, DMA_STATUS_NI | DMA_STATUS_RI);
		}

	/* refill RX ring , poll RX */
	rxbd_refill(state);
	CSR_WRITE_4(state, GMAC_DMA_RPD, 1);

sr = CSR_READ_4(state, GMAC_DMA_STATUS);

	if (sr & DMA_STATUS_TI) {
		gmac_service_tx(netif);
		CSR_WRITE_4(state, GMAC_DMA_STATUS, DMA_STATUS_NI | DMA_STATUS_TI);
		}
	/*
	 * Handle RU/OVF and other error conditions over here if needed.
	 */

#if GMAC_USE_IRQ
	irq_enable(state->pci_dev.irq);
#endif

}

#if GMAC_USE_IRQ
void gmac_irqhandler(void *arg)
{
	struct gmac_if *state = arg;
	irq_disable(state->pci_dev.irq);
#if defined(CONFIG_MICROKERNEL)
	isr_event_send(state->event);
#endif	
}
#endif

err_t gmac_init(struct netif *netif)
{
	struct gmac_if *state = mem_calloc(1, sizeof(struct gmac_if));
	struct gmac_config *config = netif->state;
	u32_t mach, macl;
	int i;
	err_t ret = ERR_IF;

	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_ALL, ("gmac init : Start\n"));

	if (state == NULL) {
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_LEVEL_SERIOUS, ("gmac init: state OOM\n"));
		return ERR_MEM;
		}

	memcpy(&state->pci_dev, &config->pci, sizeof(struct pci_dev_info));

	pci_bus_scan_init();
	if (!pci_bus_scan(&state->pci_dev)) {
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_SERIOUS, ("gmac init: No device\n"));
		goto err;
	}

	/* Can test for class/ VID/ PID here too */

	if ((state->pci_dev.mem_type != BAR_SPACE_MEM) || (state->pci_dev.size !=  0x2000)) {
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_SERIOUS, ("gmac init: Bad BAR\n"));
		goto err;
	}

	pci_show(&state->pci_dev);
	pci_enable_regs(&state->pci_dev);
	pci_enable_bus_master(&state->pci_dev);

	netif->state = state;
	netif->name[0] = 'g';
	netif->name[1] = 'm';
	netif->mtu = 1500;
	netif->hwaddr_len = 6;

	netif->linkoutput = low_level_output;
#if LWIP_IPV4
	netif->output = etharp_output;
#endif
#if LWIP_IPV6
	netif->output_ip6 = ethip6_output;
#endif
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
	mach = CSR_READ_4(state, GMAC_ADDRH);
	macl = CSR_READ_4(state, GMAC_ADDRL);
	netif->hwaddr[4] = (mach >> 0)  & 0xff; 
	netif->hwaddr[5] = (mach >> 8)  & 0xff;
	netif->hwaddr[0] = (macl >> 0)  & 0xff; 
	netif->hwaddr[1] = (macl >> 8)  & 0xff;
	netif->hwaddr[2] = (macl >> 16) & 0xff; 
	netif->hwaddr[3] = (macl >> 24) & 0xff;

	/* Can test for validity (LSB of hwaddr[0]) and replace with hardcoded value */

	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_ALL,
		("gmac init: MAC %"X8_F":%"X8_F":%"X8_F":%"X8_F":%"X8_F":%"X8_F"\n",
		netif->hwaddr[0],netif->hwaddr[1],netif->hwaddr[2],
		netif->hwaddr[3],netif->hwaddr[4],netif->hwaddr[5]));

	/* Reset GMAC */
	CSR_SET_BIT(state, GMAC_DMA_BUSMOD, BIT_DMA_BUSMOD_SWR);
	while (CSR_TEST_BIT(state, GMAC_DMA_BUSMOD, BIT_DMA_BUSMOD_SWR))
		;

	/*Restore MAC address  */
	CSR_WRITE_4(state, GMAC_ADDRH, mach);
	CSR_WRITE_4(state, GMAC_ADDRL, macl);

	state->rx_head = 0;
	state->rx_refill = 0;
	state->tx_head = 0;
	state->tx_free = TXBD;
	state->gmac_rx_desc_mem = mem_malloc(sizeof(struct gmacRxDesc) * RXBD + (DESC_ALIGN - 1));
	ret = ERR_MEM;

	if (state->gmac_rx_desc_mem == NULL) {
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_SERIOUS, ("gmac init: RX desc OOM\n"));
		goto err;
	}

	state->gmac_rx = (struct gmacRxDesc*)((((u32_t)state->gmac_rx_desc_mem) + DESC_ALIGN - 1) &~(DESC_ALIGN - 1));
	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL, ("gmac_init: RX descriptors @ %p\n",state->gmac_rx));
	for (i = 0; i < RXBD; i++) {
		state->rx_pbuf[i] = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
		if (NULL == state->rx_pbuf[i]) {
			LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_LEVEL_SERIOUS, ("gmac init: RX pbuf OOM\n"));
			goto err;
		}
		state->gmac_rx[i].rxControl = RX_BUFFER1_SIZE(state->rx_pbuf[i]->len) | RX_DESC_CTL_CHAINED;
		state->gmac_rx[i].pointer   = (u32_t)state->rx_pbuf[i]->payload;
		state->gmac_rx[i].rxStatus = RX_DESC_OWNBYDMA;
		state->gmac_rx[i].nextDesc = (u32_t)&state->gmac_rx[RX_INDEX_NEXT(i)];
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL,
			("status = %08x control = %08x pointer = %08x next=%08x\n",
			state->gmac_rx[i].rxStatus,
			state->gmac_rx[i].rxControl,
			state->gmac_rx[i].pointer,
			state->gmac_rx[i].nextDesc
			));
	}

	state->gmac_tx_desc_mem = mem_malloc(sizeof(struct gmacTxDesc) * TXBD + (DESC_ALIGN - 1));

	if (state->gmac_tx_desc_mem == NULL) {
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_LEVEL_SERIOUS, ("gmac init: TX desc OOM\n"));
		goto err;
	}
	state->gmac_tx = (struct gmacTxDesc*)((((u32_t)state->gmac_tx_desc_mem) + DESC_ALIGN - 1) &~(DESC_ALIGN - 1));
	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL, ("gmac_init: TX descriptors @ %p\n",state->gmac_tx));
	for (i = 0; i < TXBD; i++) {
		state->tx_pbuf[i] = NULL;
		state->gmac_tx[i].txControl = 0;
		state->gmac_tx[i].pointer   = (u32_t)NULL;
		state->gmac_tx[i].txStatus = TX_DESC_CHAINED;
		state->gmac_tx[i].nextDesc = (u32_t)&state->gmac_tx[TX_INDEX_NEXT(i)];
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_STATE | LWIP_DBG_LEVEL_ALL,
			("status = %08x control = %08x pointer = %08x next=%08x\n",
			state->gmac_tx[i].txStatus,
			state->gmac_tx[i].txControl,
			state->gmac_tx[i].pointer,
			state->gmac_tx[i].nextDesc
			));
	}

#if LWIP_IPV6_MLD || LWIP_IGMP
	CSR_SET_BIT(state, GMAC_FFILT, BIT_FFILT_PM ); /* Accept all multicast. Should implement igmp_mac_filter/mld_mac_filter instead.*/
#endif

	CSR_WRITE_4(state, GMAC_DMA_RDL, (u32_t)state->gmac_rx);
	CSR_WRITE_4(state, GMAC_DMA_TDL, (u32_t)state->gmac_tx);
	CSR_WRITE_4(state, GMAC_DMA_OPMOD, DMA_OPMOD_RSF | DMA_OPMOD_TSF | DMA_OPMOD_SR);
#if 0
	CSR_SET_BIT(state, GMAC_CONF, BIT_CONF_ACS); /* ACS: strip padding and CRC from frame - doesn't seem to work */
#endif

	CSR_WRITE_4(state, GMAC_DMA_RPD, 1);

#if GMAC_USE_IRQ

#if defined(CONFIG_MICROKERNEL)
	state->event = config->event;
#endif
	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_ALL, ("gmac init: Connecting irq %d\n", state->pci_dev.irq));
	if (irq_connect(state->pci_dev.irq, 1, gmac_irqhandler, state) == -1) {
		LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_SERIOUS, ("gmac init: Connecting irq failed\n"));
		ret = ERR_IF;
		goto err;
	}
	irq_enable(state->pci_dev.irq);
	/* It seems that RIE (or TIE) alone won't trigger an interrupt, needs to add NIE */
	CSR_WRITE_4(state, GMAC_DMA_INTEN, DMA_INTEN_NIE | DMA_INTEN_RIE | DMA_INTEN_TIE);
#endif

	sys_timeout(MONITOR_INTERVAL, gmac_monitor, netif);

	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_ALL, ("gmac init: Success\n"));
	return ERR_OK;
err:
	for (i = 0; i < RXBD; i++)
		if (state->rx_pbuf[i])
			pbuf_free(state->rx_pbuf[i]);
		else /* pbufs are allocated sequentially */
			break;
	if (state->gmac_rx_desc_mem)
		mem_free(state->gmac_rx_desc_mem);
	if (state->gmac_tx_desc_mem)
		mem_free(state->gmac_tx_desc_mem);
	mem_free(state);
	LWIP_DEBUGF(GMAC_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_ALL, ("gmac init: Failed %d\n", ret));
	return ret;
}

#endif /* CONFIG_BOARD_GALILEO */


