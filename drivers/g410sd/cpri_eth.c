/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : 
 ::  ::  ::       ::           :::      FileName   : cpri_eth.c
 ::   :: ::       ::             ::     Generate   : 
 ::    ::::       ::       ::      ::   Update     : 2010-10-27 14:03:09
::::    :::     ::::::      ::::::::    Version    : 0.0.2

Description
	远端机单收单发、近端机六收单发，远近端机通过FPGA的编号寄存器区分。
	近端机编号为0，远端机编号为1～6；
*/
#include <linux/etherdevice.h>
#include <linux/irq.h>
#include <linux/crc32.h>
#include <g410sd/g410sd_hw.h>

#define MAX_MAC_PACK_LEN   352

struct net_local {
	struct net_device_stats stats;

	//xx_buf size is word size, MAX_MAC_PACK_LEN is byte size
	short rxbuf[MAX_MAC_PACK_LEN*3];
	short txbuf[MAX_MAC_PACK_LEN/2];

	int txflag;
	unsigned int tx_word_len;
	char level;//级联编号，0代表近端机，1～6依次代表第1～6级远端机
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
static void net_rx_packet(short *buf, unsigned int fifo_addr, unsigned int length);


static int net_open(struct net_device *dev)
{
	int ret;
	struct net_local *lp = netdev_priv(dev);

	lp->txflag = 0;

	lp->level = __raw_readw(CPRI_LEVEL);
	if (lp->level)
		printk("CPRI_ETH: we are DRU %d", lp->level);
	else
		printk("CPRI_ETH: we are RAU");

	ret = request_irq(dev->irq, net_interrupt, 0, dev->name, dev);
	if (ret != 0) {
		printk("%s: fail request_irq.\n", dev->name);
	} else {
		netif_start_queue(dev);
	}

	return ret;
}

static int net_close(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	netif_stop_queue(dev);

	free_irq(dev->irq, dev);

	lp->txflag = 0;
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

	netif_stop_queue(dev);

	// copy skb data to our memory buffer
	memset(lp->txbuf, 0, MAX_MAC_PACK_LEN);
	memcpy(lp->txbuf, skb->data, skb->len);

	// pad byte to word
	if (skb->len%2)		skb->len++;

	lp->tx_word_len  = skb->len/2 + 2;
	lp->txflag = 1;
	lp->stats.tx_bytes += skb->len;
	dev->trans_start = jiffies;

	// add FCS(CRC) checksum value
	crc = ether_crc(skb->len, (char *)lp->txbuf);
	lp->txbuf[lp->tx_word_len-2] = crc;
	lp->txbuf[lp->tx_word_len-1] = crc>>16;

	dev_kfree_skb(skb);

	return 0;
}

static irqreturn_t net_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct net_local *lp = netdev_priv(dev);
	int i;

	//验证逻辑IR接口是否就位
	if (0xFF00 & __raw_readw(CPRI_VER)) {
		printk("CPRI_ETH: Invalid FPGA CPRI core\n");
		return IRQ_HANDLED;
	}

	// 处理发送数据
	if (lp->txflag != 0) {
		for (i=0; i< lp->tx_word_len; i++) {
			__raw_writew(lp->txbuf[i], CPRI_TX_FIFO);
		}

		lp->txflag = 0;
		lp->stats.tx_packets++;
		netif_wake_queue(dev);
	}

	// 处理接收数据
	net_rx_packet(lp->rxbuf, CPRI_RX1_FIFO, __raw_readw(CPRI_RX1_LEN));

	if (lp->level == 0) { //近端机需要收六个包
		net_rx_packet(lp->rxbuf +   MAX_MAC_PACK_LEN/2, CPRI_RX2_FIFO, __raw_readw(CPRI_RX2_LEN));
		net_rx_packet(lp->rxbuf + 2*MAX_MAC_PACK_LEN/2, CPRI_RX3_FIFO, __raw_readw(CPRI_RX3_LEN));
		net_rx_packet(lp->rxbuf + 3*MAX_MAC_PACK_LEN/2, CPRI_RX4_FIFO, __raw_readw(CPRI_RX4_LEN));
		net_rx_packet(lp->rxbuf + 4*MAX_MAC_PACK_LEN/2, CPRI_RX5_FIFO, __raw_readw(CPRI_RX5_LEN));
		net_rx_packet(lp->rxbuf + 5*MAX_MAC_PACK_LEN/2, CPRI_RX6_FIFO, __raw_readw(CPRI_RX6_LEN));
	}

	return IRQ_HANDLED;
}


static void net_rx_packet(short *buf, unsigned int fifo_addr, unsigned int len)
{
	int i;
	unsigned long crc;
	struct sk_buff *skb;
	struct net_local *lp = netdev_priv(dev_cpri);

	if (len == 0)	return;
	
	//len now means word_len
	for (i=0; i<len; i++) {
		buf[i] = __raw_readw(fifo_addr);
	}

	len -= 2; //skip FCS

	crc  = buf[i-2];
	crc += buf[i-1] << 16;

	// switch 'len' to bytes
	len *=2;
	if (crc != ether_crc(len, (char *)buf)) {
		lp->stats.rx_errors++;
		return;
	}

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
	dev_cpri->mtu  = MAX_MAC_PACK_LEN - ( 7 + 7 + 4); // DA + SA + FCS(CRC)

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

