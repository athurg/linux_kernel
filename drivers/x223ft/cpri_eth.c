/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : 
 ::  ::  ::       ::           :::      FileName   : cpri_eth.c
 ::   :: ::       ::             ::     Generate   : 
 ::    ::::       ::       ::      ::   Update     : 2010-11-05 17:07:00
::::    :::     ::::::      ::::::::    Version    : 0.0.2

Description
*/
#include <linux/etherdevice.h>
#include <linux/irq.h>
#include <linux/crc32.h>
#include <g410sd/g410sd_hw.h>

#define MAX_MAC_PACK_LEN   512
#define SPD	0x133
#define EPD	0x144

struct net_local {
	struct net_device_stats stats;
	char rxbuf[MAX_MAC_PACK_LEN];
	char tx_flag;
};
//dev_cpri是本驱动虚拟出的网卡设备
static struct net_device *dev_cpri;

static irqreturn_t net_interrupt(int irq, void *dev_id);
static int net_open(struct net_device *dev);
static int net_close(struct net_device *dev);
static void net_timeout(struct net_device *dev);
static int set_mac_address(struct net_device *dev, void *addr);
static struct net_device_stats *net_get_stats(struct net_device *dev);
static int net_send_packet(struct sk_buff *skb, struct net_device *dev);
static void net_rx_packet(char *buf, unsigned int length);


static int net_open(struct net_device *dev)
{
	int ret;

	ret = request_irq(dev->irq, net_interrupt, 0, dev->name, dev);
	if (ret != 0) {
		printk("%s: fail request_rx_irq.\n", dev->name);
	} else {
		netif_start_queue(dev);
	}

	return ret;
}

static int net_close(struct net_device *dev)
{
	netif_stop_queue(dev);

	free_irq(dev->irq, dev);

	return 0;
}

static void net_timeout(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	lp->stats.tx_errors++;

	netif_wake_queue(dev);
}

static int net_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	unsigned long crc;
	unsigned int i=0,len;

	netif_stop_queue(dev);

	crc = ether_crc(skb->len, skb->data);

	__raw_writew(SPD, CPRI_TX_FIFO);

	len = skb->len;
	for(i=0; i<skb->len; i++){
		__raw_writew(skb->data[i], CPRI_TX_FIFO);
	}

	lp->stats.tx_bytes += skb->len;
	dev->trans_start = jiffies;

	__raw_writew((crc>>24) & 0xFF, CPRI_TX_FIFO);
	__raw_writew((crc>>16) & 0xFF, CPRI_TX_FIFO);
	__raw_writew((crc>> 8) & 0xFF, CPRI_TX_FIFO);
	__raw_writew( crc      & 0xFF, CPRI_TX_FIFO);

	__raw_writew(EPD, CPRI_TX_FIFO);

	dev_kfree_skb(skb);

	lp->tx_flag = 1;
	return 0;
}

static irqreturn_t net_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct net_local *lp = netdev_priv(dev);
	unsigned int i,len;
	unsigned long crc=0;
	unsigned int flag;

	if (0xFF00 & __raw_readw(CPRI_VER)) {
		printk("CPRI_ETH: Invalid FPGA CPRI core\n");
		return IRQ_HANDLED;
	}

	flag = __raw_readw(CPRI_INTERRUPT_FLAG);

	if ((flag & CPRI_INTERRUPT_TX) && (lp->tx_flag) ){
		lp->tx_flag = 0;
		netif_wake_queue(dev);
	}

	//check if a package is received
	if (!(flag & CPRI_INTERRUPT_RX)) {
		return IRQ_HANDLED;
	}

	if (SPD != __raw_readw(CPRI_RX1_FIFO)) {
		goto err_rx_package;
	}

	len = __raw_readw(CPRI_RX1_LEN) - 2 - 4 ; //2=epd+spd   4 =crc
	for (i=0; i<len; i++){
		lp->rxbuf[i] = 0xFF & __raw_readw(CPRI_RX1_FIFO);
	}

	for (i=0; i<4; i++) {
		crc = (crc<<8) | (0xFF & __raw_readw(CPRI_RX1_FIFO));
	}

	if (EPD != __raw_readw(CPRI_RX1_FIFO)) {
		goto err_rx_package;
	}

	if (crc != ether_crc(len, lp->rxbuf)){
		goto err_rx_package;
	}

	net_rx_packet(lp->rxbuf, len);
	return IRQ_HANDLED;

err_rx_package:
	lp->stats.rx_errors++;
	return IRQ_HANDLED;
}


static void net_rx_packet(char *buf, unsigned int len)
{
	struct sk_buff *skb;
	struct net_local *lp = netdev_priv(dev_cpri);

	if (len == 0)	return;

	skb = dev_alloc_skb(len);
	if (skb == NULL) {
		lp->stats.rx_dropped++;
		return;
	}

	skb_reserve(skb, 2);

	//request memory for skb->data
	skb_put(skb, len);

	skb->dev = dev_cpri;
	memcpy(skb->data, buf, len);
	skb->protocol=eth_type_trans(skb, dev_cpri);

	// put skb to protocol
	netif_rx(skb);

	dev_cpri->last_rx = jiffies;
	lp->stats.rx_packets++;
	lp->stats.rx_bytes += len;
}

//获取网卡统计状态函数
static struct net_device_stats* net_get_stats(struct net_device *dev)
{
	struct net_local *lp;

	lp = netdev_priv(dev);
	return &(lp->stats);
}

//设置MAC地址函数
static int set_mac_address(struct net_device *dev, void *p)
{
	int i;
	struct sockaddr *addr = p;

	if (netif_running(dev)){
		return -EBUSY;
	}

	//更新设备结构体中的MAC地址
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	//更新FPGA中的MAC地址
	__raw_writew((dev->dev_addr[0]<<8)|dev->dev_addr[1], CPRI_MAC0);
	__raw_writew((dev->dev_addr[2]<<8)|dev->dev_addr[3], CPRI_MAC1);
	__raw_writew((dev->dev_addr[4]<<8)|dev->dev_addr[5], CPRI_MAC2);

	printk("%s: Setting MAC address to ", dev->name);
	for (i = 0; i < dev->addr_len; i++)
		printk(" %2.2x", dev->dev_addr[i]);
	printk(".\n");

	return 0;
}

static int __init cpri_eth_init(void)
{
	int ret = 0;
	unsigned int irq;
	struct sockaddr sock_addr={
		.sa_data = {0x00,0xE0,0x01,0x00,0x01,0x01},
	};

	//为cpri网络设备及其私有设备分配内存空间
	dev_cpri = alloc_etherdev(sizeof(struct net_local));
	if (!dev_cpri){
		return -ENOMEM;
	}


	//初始化网络设备结构体
	irq = CPRI_IRQ;
	set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);
	dev_cpri->irq  = irq;
	dev_cpri->mtu  = MAX_MAC_PACK_LEN - ( 7 + 7 + 4 + 1 + 1); // DA + SA + FCS(CRC) + SPD + EPD

	dev_cpri->open            = net_open;
	dev_cpri->stop            = net_close;
	dev_cpri->get_stats       = net_get_stats;
	dev_cpri->tx_timeout      = net_timeout;
	dev_cpri->set_mac_address = set_mac_address;
	dev_cpri->hard_start_xmit = net_send_packet;
	dev_cpri->watchdog_timeo  = HZ;

	set_mac_address(dev_cpri, &sock_addr);


	// 注册设备
	ret = register_netdev(dev_cpri);
	if (ret){
		printk("BSP: %s Fail to Register net device", __FUNCTION__);
		goto error;
	}

	//初始化私有设备结构体
	memset(netdev_priv(dev_cpri), 0, sizeof(struct net_local));

	printk("BSP: Register CPRI net device succesful\n");
	return ret;

error:
	free_netdev(dev_cpri);
	return ret;
}

static void __exit cpri_eth_exit(void)
{
	unregister_netdev(dev_cpri);
	free_netdev(dev_cpri);
}

module_init(cpri_eth_init);
module_exit(cpri_eth_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("CPRI Ethernet Interface");
MODULE_LICENSE("GPL");

