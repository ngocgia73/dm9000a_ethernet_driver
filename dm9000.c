#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/dm9000.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/io.h>

#define CARD_NAME 	"dm9000"
#define DRV_VERSION 	"1.31"



enum dm9000_type
{
	TYPE_DM9000E,
	TYPE_DM9000A,
	TYPE_DM9000B
};

typedef struct board_info {
	void __iomem 	*io_addr; 	// register I/O base address
	void __iomem 	*io-data; 	// data I/O address 
	// TODO :
	u8 		imr_all; 	// interrupt mask register 
	u16 		tx_pkt_cnt;
	u16 		queue_pkt_len;
	u16 		irq; 		// for normal interrupt
	u16 		queue_ip_summed;
	int 		ip_summed;
	int 		irq_wake; 	// for wake interrupt
	unsigned int 	wake_supported :1;
	unsigned int 	flags; 		// to address that 8bit or 16bit or 32bit

	enum dm9000_type type;

	enum dm9000_type type;
	void (*inblk)(void __iomem *port, void *data, int length);
	void (*outblk)(void __iomem *port, void *data, int length);
	void (*dumpblk)(void  __iomem *port, int length);

	struct device *dev; 		// parent device

	struct resource *addr_res; 	// resource found
	struct resource *data_res;
	struct resource *addr_req; 	// resource requested
	struct resource *data_req;

	struct resource *irq_res;

	struct delayed_work phy_poll;
	struct net_device *ndev;

	// for MII
	u32 msg_enable;
	struct mii_if_info mii;

	spinlock_t 	lock;

}board_info_t;

//
// reset dm9000 device
//
static void dm9000_reset(board_info_t *db)
{
	printk(KERN_INFO "resetting device\n");

	// datasheet page 14
	// write address of register NCR to index register
	writeb(DM9000_NCR, db->io_addr);
	udelay(200);

	// write data
	// bit 0 of reg NCR(00H) is RST
	writeb(NCR_RST, db->io_data);
	udelay(200);
}

/*
 * Initialize dm9000 board
 */
static void dm9000_init_dm9000(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	// interrupt mask register
	unsigned int imr;
	// network control register
	unsigned int ncr;

	dm9000_dbg(db, 1, "entering %s\n", __func__);

	/* I/O mode */
	db->io_mode = ior(db, DM9000_ISR) >> 6;	/* ISR bit7:6 keeps I/O mode */

	/* Checksum mode */
	if (dev->hw_features & NETIF_F_RXCSUM)
		iow(db, DM9000_RCSR,
			(dev->features & NETIF_F_RXCSUM) ? RCSR_CSUM : 0);

	iow(db, DM9000_GPCR, GPCR_GEP_CNTL);	/* Let GPIO0 output */

	ncr = (db->flags & DM9000_PLATF_EXT_PHY) ? NCR_EXT_PHY : 0;

	/* if wol is needed, then always set NCR_WAKEEN otherwise we end
	 * up dumping the wake events if we disable this. There is already
	 * a wake-mask in DM9000_WCR */
	if (db->wake_supported)
		ncr |= NCR_WAKEEN;

	iow(db, DM9000_NCR, ncr);

	/* Program operating register */
	iow(db, DM9000_TCR, 0);	        /* TX Polling clear */
	iow(db, DM9000_BPTR, 0x3f);	/* Less 3Kb, 200us */
	iow(db, DM9000_FCR, 0xff);	/* Flow Control */
	iow(db, DM9000_SMCR, 0);        /* Special Mode */
	/* clear TX status */
	iow(db, DM9000_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);
	iow(db, DM9000_ISR, ISR_CLR_STATUS); /* Clear interrupt status */

	/* Set address filter table */
	dm9000_hash_table_unlocked(dev);
i
	// datasheet page 27
	// register IMR (FFH)
	imr = IMR_PAR | IMR_PTM | IMR_PRM;
	if (db->type != TYPE_DM9000E)
		imr |= IMR_LNKCHNG;

	db->imr_all = imr;

	/* Enable TX/RX interrupt mask */
	iow(db, DM9000_IMR, imr);

	/* Init Driver variable */
	db->tx_pkt_cnt = 0;
	db->queue_pkt_len = 0;
	dev->trans_start = jiffies;
}

//
// dm9000 had 2 register 
// INDEX regiser : store address of data
// DATA register : real data
// before read or write data need write data to INDEX register
//

//
// read a byte from I/O port 
//
static void ior(board_info_t *db, int reg)
{
	writeb(reg, db->io_addr);
	return readb(db->io_data);
}

//
// write a byte to I/O port
//
static void iow(board_info_t *db, int reg, int value)
{
	writeb(reg, db->io_addr);
	writeb(value, db->io_data);
}

//
// routine for sending block to chip
//
static void dm9000_outblk_8bit(void __iomem *reg, void *data, int count)
{
	writesb(reg, data, count);
}

static void dm9000_outblk_16bit(void __iomem *reg, void *data, int count)
{
	writesw(reg, data, (count+1) >> 1);
}

static void dm9000_outblk_132bit(void __iomem *reg, void *data, int count)
{
	writesl(reg, data, (count+3) >> 2);
}

//
// routine for receive data from dm9000 chip to memory
//

static void dm9000_inblk_8bit(void __iomem *reg, void *data, int count)
{
	readsb(reg, data, count);
}

static void dm9000_inblk_16bit(void __iomem *reg, void *data, int count)
{
	readsw(reg, data, (count+1) >> 1);
}

static void dm9000_inblk_8bit(void __iomem *reg, void *data, int count)
{
	readsl(reg, data, (count+3) >> 2);
}

//
// for read dummy data 
//
static void dm9000_dumpblk_8bit(void __iomem *reg, int count)
{
	int i;
	int tmp;
	for (i = 0; i< count; i++)
	{
		tmp = readb(reg);
	}
}

static void dm9000_dumpblk_16bit(void __iomem *reg, int count)
{
	int i;
	int tmp;
	// at least 2 byte for every time read data
	count = (count+1) >> 1;
	for (i = 0; i< count; i++)
	{
		tmp = readw(reg);
	}
}

static void dm9000_dumpblk_32bit(void __iomem *reg, int count)
{
	int i;
	int tmp;
	// at least 4 byte for every time read data
	count = (count + 3) >> 2;
	for (i = 0; i< count; i++)
	{
		tmp = readl(reg);
	}
}

static void dm9000_set_io(struct board_info_t *db, int byte_width)
{
	switch(byte_width)
	{
		case 1:
			db->dumpblk = dm9000_dumpblk_8bit;
			db->outblk  = dm9000_outblk_8bit;
			db->inblk   = dm9000_inblk_8bit;
			break;
		case 3:
		case 2:
			db->dumpblk = dm9000_dumpblk_16bit;
			db->outblk  = dm9000_outblk_16bit;
			db->inblk   = dm9000_inblk_16bit;
			break;
		case 4:
		default:
			db->dumpblk = dm9000_dumpblk_32bit;
			db->outblk  = dm9000_outblk_32bit;
			db->inblk   = dm9000_inblk_32bit;
			break;
	}
}

//
// make sure safely exit 
//
static void dm9000_release_board(struct platform_device *pdev, struct board_info_t *db)
{
	// unmap our resource
	iounmap(db->io_addr);
	iounmap(db->io_data);

	// release the resource
	release_resource(db->addr_req);
	kfree(db->addr_req);

	release_resource(db->data_req);
	kfree(db->data_req);
}

static unsigned char dm9000_type_to_char(enum dm9000_type type)
{
	switch (type)
	{
		case TYPE_DM9000E:
			return 'e';
		case TYPE_DM9000A:
			return 'a';
		case TYPE_DM9000B:
			return 'b';
		default:
			return '?'''
	}
}


static void dm9000_poll_work(struct work_struct *w)
{
	// TODO:
}

/*
 * DM9000 interrupt handler
 * receive the packet to upper layer, free the transmitted packet
 */

static void dm9000_handle_tx_done(struct net_device *ndev, board_info_t *db)
{
	int tx_status = ior(db, DM9000_NSR);	/* Got TX status */

	if (tx_status & (NSR_TX2END | NSR_TX1END)) {
		/* One packet sent complete */
		db->tx_pkt_cnt--;
		ndev->stats.tx_packets++;

		/* Queue packet check & send */
		if (db->tx_pkt_cnt > 0)
			dm9000_send_packet(ndev, db->queue_ip_summed,
					   db->queue_pkt_len);
		netif_wake_queue(ndev);
	}
}

struct dm9000_rxhdr {
	u8	RxPktReady;
	u8	RxStatus;
	__le16	RxLen;
} __packed;

//
// received a package and pass it to upper layer
//
static void dm9000_handle_rx(struct net_device *ndev)
{

	board_info_t *db = netdev_priv(ndev);
	struct dm9000_rxhdr rxhdr;
	struct sk_buff *skb;
	u8 rxbyte, *rdptr;
	bool GoodPackage;
	int RxLen;

	// check package ready or not
	do
	{
		// dummy read
		ior(db, DM9000_MRCMDX);
		//  get most update data
		rxbyte = ready(db->io_data);
		// status check
		if(rxbyte & DM9000_PKT_ERR)
		{
			// stop device
			iow(db, DM9000_RCR, 0x0);
			// strop IRQ request
			iow(db, DM9000_ISR, IMR_PAR);
			return ;
		}
		if(!(rxbyte & DM9000_PKT_RDY))
			return;

		// A packet ready now  & Get status/length
		// get status and length from dm9000 first
		// then will receive data
		GoodPackage = true;
		
		writeb(DM9000_MRCMD, db->io_addr);
		(db->inblk)(db->io_data, &rxhdr, sizeof(rxhdr));

		RxLen = le16_to_cpu(rxhdr.RxLen);
		
		// Packet Status check
		if (RxLen < 0x40)
		{
			GoodPackage = false;

		}
		if (rxhdr.RxStatus & (RSR_FOE | RSR_CE | RSR_AE |
				 RSR_PLE | RSR_RWTO |
				 RSR_LCS | RSR_RF))
		{
			GoodPackage = false;
			if (rxhdr.RxStatus & RSR_FOE)
				ndev->stats.rx_fifo_errors++;
			if (rxhdr.RxStatus & RSR_CE)
				ndev->stats.rx_crc_errors++;
			if (rxhdr.RxStatus & RSR_RF)
				ndev->stats.rx_length_errors++;
		}

		// move data from dm9000
		if( GoodPackage && ((skb = dev_alloc_skb(RxLen + 4)) != NULL))
		{
			skb_reserve(skb, 2);
			rdptr = (u8 *) skb_put(skb, RxLen - 4);

			// Read received packet from RX SRAM 
			(db->inblk)(db->io_data, rdptr, RxLen);
			ndev->stats.rx_bytes += RxLen;
			
			// determine the packet's protocol ID
			if (ndev->features & NETIF_F_RXCSUM)
			{
				if ((((rxbyte & 0x1c) << 3) & rxbyte) == 0)
					skb->ip_summed = CHECKSUM_UNNECESSARY;
				else
					skb_checksum_none_assert(skb);
			}
			// pass package from device driver to upper layer
			netif_rx(skb);
			dev->stats.rx_packets++;
		}
		else
		{
			// need to dump the packet's data
			// handle for bad package
			// need get package from DM9000 chip to NULL
			(db->dumpblk)(db->io_data, RxLen);
		}
	}
	while(rxbyte & DM9000_PKT_RDY);
}

//
// handle interrupt function 
//
static irqreturn_t dm9000_interrupt(int irq, void *data)
{
	struct net_device *ndev = data;
	board_info_t *db = netdev_priv(ndev);
	unsigned long flags;
	u8 reg_save;
	// use spin lock will be disable irq in irq context
	// use spin_lock_irqsave because we don't make sure at this time irq disabled or not
	spin_lock_irqsave(&db->lock, flags);

	// save previous register address
	register_save =  readb(db->io_addr);

	// disable all irq
	iow(db, DM9000_IMR, IMR_PAR);
	// Got DM9000 interrupt status
	irq_status = ior(db, DM9000_ISR);
	// Clear ISR status . more infomation in dm9000a datasheet
	iow(db, DM9000_ISR, int_status);

	// comming package
	if(irq_status & ISR_PRS)
	{
		dm9000_handle_rx(ndev);
	}
	// transmit irq check
	if(irq_status & ISR_PTS)
	{
		dm9000_handle_tx_done(ndev, db);
	}
	if(db->type != TYPE_DM9000E)
	{
		if(irq_status & ISR_LNKCHNG)
		{
			// will call function dm9000_poll_work after 1 jiffies
			schedule_delayed_work(&db->phy_poll, 1);
		}
	}
	// re-enable interrupt mask
	iow(db, DM9000_IMR, db->imr_all);
	// restore previous register address
	writeb(reg_save, db->io_addr);

	spin_unlock_irq(&db->lock, flags);

	return IRQ_HANDLED;

}

//
// open inferface 
//
#define PHY_UP 		0
#define PHY_DOWN 	1

static int dm9000_open(struct net_device *ndev)
{
	// access private data
	board_info_t *db = netdev_priv(ndev); 
	unsigned long irqflags = db->irq_res->flags & IRQF_TRIGGER_MASK;

	// if there is no IRQ specified, default to something that may work,
	// and tell the user that this is a problem 
	if(irqflags == IRQF_TRIGGER_NONE)
	{
		// just notify to user
		printk(KERN_INFO "no IRQ resource flag set\n");
	}
	irqflags |= IRQF_SHARED;
	// reg 1F : bit 0 -> indicate state of PHY
	// bit 0 = 0 : power up PHY
	// bit 0 = 1 : power down PHY
	// datasheet dm9000a page 22
	iow(db, DM9000_GPR, PHY_UP); 
	mdelay(1); // need by dm9000b;

	// initialize dm9000 board
	dm9000_reset(db);
	dm9000_init_dm9000(ndev);
	ret = request_irq(ndev->irq, dm9000_interrupt, irqflags , ndev->name, ndev);
	if(ret == 1)
	{
		printk(KERN_ERR "can not request interrupt\n");
		return -EAGAIN;
	}	
	// for mii (media independent interface)
	// can use different type of PHY device without redesign or replace MAC hardware 
	mii_check_media(&db->mii, netif_msg_link(db), 1);
      	// Allow upper layers to call the device hard_start_xmit routine
	netif_start_queue(ndev);	
}

static void dm9000_shutdown(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
	// reset device
	//dm9000_phy_write(dev, 0, MII_BMCR, BMCR_RESET); /* PHY RESET */ 
	iow(db, DM9000_GPR, 0x01);      /* Power-Down PHY */
	iow(db, DM9000_IMR, IMR_PAR);   /* Disable all interrupt */
	iow(db, DM9000_RCR, 0x00);      /* Disable RX */

static int dm9000_stop(struct net_device *ndev)
{
	board_info_t *db = netdev_priv(ndev);

	if(netif_msg_ifdown(db))
	{
		// msg_enable -> NETIF_MSG_IFDOWN
		printk(KERN_INFO "shuting down\n");

	}
	cancel_delayed_work_sync(&db->phy_poll);
	// stop transmitted packets
	netif_stop_queue(ndev);

	netif_carrier_off(ndev);
	// free irq
	free_irq(ndev->irq, ndev);
	dm9000_shutdown(ndev);

	return 0;
}

static void dm9000_send_packet(struct net_device *ndev,
			       int ip_summed,
			       u16 pkt_len)
{
	board_info_t *dm = to_dm9000_board(ndev);

	/* The DM9000 is not smart enough to leave fragmented packets alone. */
	if (dm->ip_summed != ip_summed) {
		if (ip_summed == CHECKSUM_NONE)
			iow(dm, DM9000_TCCR, 0);
		else
			iow(dm, DM9000_TCCR, TCCR_IP | TCCR_UDP | TCCR_TCP);
		dm->ip_summed = ip_summed;
	}

	/* Set TX length to DM9000 */
	iow(dm, DM9000_TXPLL, pkt_len);
	iow(dm, DM9000_TXPLH, pkt_len >> 8);

	/* Issue TX polling command */
	iow(dm, DM9000_TCR, TCR_TXREQ);	/* Cleared after TX complete */
}


static int dm9000_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{

	// datashet  page 42 : package transmition 
	// step 1: placed data to sram of dm9000 
	// step 2: send length package 
	unsigned long flags;
	board_info_t *db = netdev_priv(ndev);

	if (db->tx_pkt_cnt > 1)
		return NETDEV_TX_BUSY;

	spin_lock_irqsave(&db->lock, flags);

	/* Move data to DM9000 TX RAM */
	writeb(DM9000_MWCMD, db->io_addr);

	(db->outblk)(db->io_data, skb->data, skb->len);
	ndev->stats.tx_bytes += skb->len;

	db->tx_pkt_cnt++;
	/* TX control: First packet immediately send, second packet queue */
	if (db->tx_pkt_cnt == 1) {
		dm9000_send_packet(ndev, skb->ip_summed, skb->len);
	} else {
		/* Second packet */
		db->queue_pkt_len = skb->len;
		db->queue_ip_summed = skb->ip_summed;
		netif_stop_queue(ndev);
	}

	spin_unlock_irqrestore(&db->lock, flags);

	/* free this SKB */
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

static const struct net_device_ops dm9000_netdev_ops = {
	.ndo_open 	= 	dm9000_open,
	.ndo_stop 	= 	dm9000_stop,
	.ndo_start_xmit = 	dm9000_start_xmit
	// add more method here
}
// handle probe function
static int __devinit dm9000_custom_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "probe network driver function called \n");
	// TODO:
	int ret = 0;
	int iosize;
	// this structure we defined and assign it in file arch/arm/mach-s5pv210/mach-mini210.c
	struct dm9000_plat_data *pdata = pdev->dev.platform_device;
	struct net_device *ndev = NULL;
	int ret = 0;
	int i;
	u32 id_val;
	board_info_t *db; 	// point to board information structure

	// init network device 
	ndev = alloc_etherdev(sizeof(board_info_t));
	if (!ndev)
	{
		printk(KERN_INFO "can not allocate device\n");
		return -ENOMEM;
	}

	// for init device
	SET_NETDEV_DEV(ndev, &pdev->dev);
	db = netdev_priv(ndev);
	db->dev = &pdev->dev;
	db->ndev = ndev;
	
	// init spinlock
	spin_lock_init(&db->lock);
	// init timer 
	INIT_DELAYED_WORK(&db->phy_poll, dm9000_poll_work);
	
	db->addr_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	db->data_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	db->irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	// check error
	if(db->addr_res == NULL || db->data_res == NULL || db->irq_res == NULL)
	{
		printk(KERN_ERR "insufficient resource\n");
		ret = -ENOMEM;
		goto __OUT;
	}
	// get interupt number
	db->irq_wake = platform_get_irq(pdev, 1);
	printk(KERN_INFO "db->irq_wake = %d\n",db->irq_wake);
	if(db->irq_wake >= 0)
	{
		// request interrupt 
		printk(KERN_INFO "irq_wake = %d\n",db->irq_wake);
		ret = request_irq(db->irq_wake, dm9000_wol_interrupt,
				IRQF_SHARED, dev_name(db->dev), (void *)ndev);
		// check and make sure irq really active
		if(ret)
		{
			printk(KERN_ERR "can not request irq : ret = %d\n",ret);
		}
		else
		{
			// make sure irq really work
			// test to see if irq is really wakeup capable
			ret = irq_set_irq_wake(db->irq_wake, 1);
			if(ret)
			{
				printk(KERN_ERR "irq %d can not set wakeup: ret = %d\n",db->irq_wake, ret);
				ret = 0;
			}
			else
			{
				irq_set_irq_wake(db->irq_wake, 0);
				db->wake_supported = 1;
			}
		}
	}

	iosize = resource_size(db->addr_res);
	db>addr_req = request_mem_region(db->addr_res->start, iosize, pdev->name);
	if(db->addr_req == NULL)
	{
		printk(KERN_ERR "can not request address reg area \n");
		ret = -EIO;
		goto __OUT;
	}

	db->io_addr = ioremap(db->addr_res->start, iosize);
	if(db->io_addr ==NULL)
	{
		printk(KERN_ERR "failed to remap address reg\n");
		ret = -EINVAL;
		goto __OUT;
	}

	iosize = resource_size(db->data_res);
	db->data_req = request_mem_region(db->data_res->start, iosize, pdev->name);
	if(db->data_req == NULL)
	{
		printk(KERN_ERR "can not request data res area\n");
		ret = -EIO;
		goto __OUT;
	}
	db->io_data = ioremap(db->data_res->start, iosize);
	if(db->io_data == NULL)
	{
		printk(KERN_ERR "failed to remap data reg\n");
		ret = -EINVAL;
		goto __OUT;
	}

	// fill in parameter for net-dev structure
	ndev->base_addr = (unsigned long) db->io_addr;
	ndev->irq = db->irq_res->start;
	printk(KERN_INFO "ndev->irq = %d\n",ndev->irq);

	// set for io routine (8bit or 16bit or 32bit)
	dm9000_set_io(db, iosize);
	//
	// check to see if anything is being over-ridden
	//
	if(pdata != NULL)
	{
		if(pdata->flag & DM9000_PLATF_8BITONLY)
			dm9000_set_io(db, 1);

		if(pdata->flag & DM9000_PLATF_16BITONLY)
			dm9000_set_io(db, 2);

		if(pdata->flag & DM9000_PLATF_32BITONLY)
			dm9000_set_io(db, 4);
		//
		// check to see  if there are any IO routine over-rides
		//
		if(pdata->inblk != NULL)
			db->inblk = pdata->inblk;
		
		if(pdata->outblk != NULL)
			db->outblk = pdata->outblk;

		if(pdata->dumpblk != NULL)
			db->dumpblk = pdata->dumpblk;

		db->flags = pdata->flags;
	}

	// reset device
	dm9000_reset(db);

	// get product ID and vender ID
	// try multiple times , DM9000 sometimes get the wrong read
	for (i = 0 ; i<8; i++)
	{
		//
		// 32<->24<->16<->8<->0
		//  PIDH PIDL VIDH VIDL
		//
		id_val  = ior(db, DM9000_VIDL);
		id_val |= (u32)ior(db, DM9000_VIDH) << 8;
		id_val |= (u32)ior(db, DM9000_PIDL) << 16;
		id_val |= (u32)ior(db, DM9000_PIDH) << 24;
		
		if(id_val == DM9000_ID)
			break;
		printk(KERN_ERR "read wrong ID. try it again\n");
	}

	if(id_val != DM9000_ID)
	{
		printk(KERN_ERR "wrong ID 0x%08x\n",id_val);
		ret = -ENODEV;
		goto __OUT;
	}

	// identify what type of dm9000 we are using
	id_val = ior(db, DM9000_CHIPR);
	printk(KERN_INFO "dm9000 revision 0x%02x\n",id_val);
	switch(id_val)
	{
		case CHIPR_DM9000A: 
			db->type = TYPE_DM9000A;
			break;
		case CHIPR_DM9000B:
			db->type = TYPE_DM9000B;
			break;
		default:
			db->type = TYPE_DM9000E;
			break;
	}

	/* dm9000a/b are capable of hardware checksum offload */
	if (db->type == TYPE_DM9000A || db->type == TYPE_DM9000B)
	{
		ndev->hw_features = NETIF_F_RXCSUM | NETIF_F_IP_CSUM;
		ndev->features |= ndev->hw_features;
	}

	// driver system function
	ether_setup(ndev);

	ndev->netdev_ops 	= 	&dm9000_netdev_ops;
	ndev->watchdog_timeo 	= 	msecs_to_jiffies(watchdog);
	//ndev->ethtool_ops       =  	&dm9000_ethtool_ops;
	
	// for MII
	// permit to  print link up/down messages when plug cable network
	db->msg_enable 		= 	NETIF_MSG_LINK;
	// add more config for MII here

	// TODO : identify mac address and where store MAC address ?

	platform_set_drvdata(pdev, ndev);
	// register net device driver
	ret = register_netdev(ndev);
	if(ret == 0)
	{
		printk(KERN_INFO "%s: dm9000%c at %p, %p IRQ %d MAC : %p MS (%s)\n",
				ndev->name, dm9000_type_to_char(db->type),
				db->io_addr, db->io_data, ndev->irq,
				ndev->dev_addr, mac_src);
	}

	return 0;
__OUT:
	dm9000_release_board(pdev, db);
	free_netdev(ndev);
	return ret;	

}

// handle remove function 
static int __devexit dm9000_custom_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "remove network driver function called\n");
	// TODO :
	struct net_device *ndev = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);
	unregister_netdev(ndev);
	dm9000_release_board(pdev, netdev_priv(ndev));
	free_netdev(ndev);
	return 0;
}

// handle suspend function 
static int dm9000_drv_suspend(struct device *dev)
{
	// TODO:
	printk(KERN_INFO "call function dm9000_drv_suspend"); 
	return 0;
}

// handle resume function
static int dm9000_drv_resume(struct device *dev)
{
	// TODO:
	printk(KERN_INFO "call function dm9000_drv_resume");
	return 0 ;
}

static const struct dev_pm_ops dm9000_drv_pm_ops = {
	.suspend 	= 	dm9000_drv_suspend,
	.resume 	= 	dm9000_drv_resume,
};

static struct platform_driver dm9000_driver = {
	.driver = 
	{
		.name 	= 	"dm9000",
		.owner 	= 	THIS_MODULE,
		.pm 	= 	&dm9000_drv_pm_ops,
	},
	.probe 	= 	dm9000_custom_probe,
	.remove = 	__devexit_p(dm9000_custom_remove),
}

static int __init dm9000_custom_init(void)
{
	printK(KERN_INFO "%s ethernet driver , v%s",CARD_NAME, DRV_VERSION);
	return platform_driver_register(&dm9000_driver);
}

static void __exit dm9000_custom_exit(void)
{
	printk(KERN_INFO "remove dm9000 enthernet driver");
	platform_driver_unregister(&dm9000_driver);
}

module_init(dm9000_custom_init);
module_exit(dm9000_custom_exit);


MODULE_AUTHOR("giann custom from exits driver");
MODULE_DESCRIPTION("DM9000 network driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dm9000");
