/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : 
 ::  ::  ::       ::           :::      FileName   : cpri_eth.c
 ::   :: ::       ::             ::     Generate   : 
 ::    ::::       ::       ::      ::   Update     : 2010-10-09 16:28:58
::::    :::     ::::::      ::::::::    Version    : 0.0.2

Description
	None
*/
#include <linux/etherdevice.h>
#include <linux/irq.h>
#include <linux/crc32.h>
#include <g410sd/g410sd_hw.h>

#define MAX_TRX_BUFF_LEN   10534
#define SPD             0x01FB//MAC帧首字节标示位
#define EPD             0x01FD//MAC帧末字节标示位

//net_local是本网卡设备私有信息结构体
struct net_local {
	struct net_device_stats stats;
	unsigned short txbuf[MAX_TRX_BUFF_LEN];
	unsigned short rxbuf[MAX_TRX_BUFF_LEN];
	int txflag;
	int txlen;
};
//dev_cpri是本驱动虚拟出的网卡设备
static struct net_device *dev_cpri;

static int net_open(struct net_device *dev);
static int net_close(struct net_device *dev);
static irqreturn_t net_interrupt(int irq, void *dev_id);
static int net_send_packet(struct sk_buff *skb, struct net_device *dev);
static void net_rx_packet(struct net_device *dev, unsigned short *buf, int len);

static void net_timeout(struct net_device *dev);
static int set_mac_address(struct net_device *dev, void *addr);
static struct net_device_stats *net_get_stats(struct net_device *dev);

static int net_open(struct net_device *dev)
{
	int ret;
	struct net_local *lp = netdev_priv(dev);

	lp->txflag = 0;

	ret = request_irq(dev->irq, net_interrupt, 0, dev->name, dev);
	if (ret != 0) {
		printk("%s: fail request_irq.\n", dev->name);
		return ret;
	}

	netif_start_queue(dev);

	return 0;
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
	unsigned int i=0, j;
	unsigned long crc;

	//先禁用发送序列（表示当前正在处理序列），本次发送完后再激活
	netif_stop_queue(dev);

	crc = ether_crc(skb->len, skb->data);

	//MAC帧头（SPD、LEN）
	lp->txbuf[i++] = SPD;
	lp->txbuf[i++] = skb->len;
	lp->txbuf[i++] = skb->len>>8;

	//MAC帧数据
	for (j=0; j<skb->len; j++)
		lp->txbuf[i++] = skb->data[j];

	//MAC帧CRC校验数据
	lp->txbuf[i++] = crc;
	lp->txbuf[i++] = crc>>8;
	lp->txbuf[i++] = crc>>16;
	lp->txbuf[i++] = crc>>24;

	//MAC帧尾
	lp->txbuf[i++] = EPD;

	//添加统计信息
	lp->txlen = i;
	lp->txflag = 1;
	lp->stats.tx_bytes += skb->len;
	dev->trans_start = jiffies;

	dev_kfree_skb(skb);

	return 0;
}

static irqreturn_t net_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct net_local *lp = netdev_priv(dev);
	unsigned int i=0;
	unsigned int sign=0, length=0,crc=0;

	//验证逻辑IR接口是否就位
	if (0xFF00 & __raw_readw(CPRI_VER)) {
		printk("Interrupt err: invalid IP core version\n");
		return IRQ_HANDLED;
	}

	// 处理发送数据
	if (lp->txflag != 0) {
		for (i=0; i<lp->txlen; i+=2) {
			__raw_writew(lp->txbuf[i], CPRI_TBUFL);
			__raw_writew(lp->txbuf[i+1], CPRI_TBUFH);
		}
		//清空发送缓冲区待发标志，发送长度置零
		lp->txflag = 0;
		lp->txlen = 0;
		lp->stats.tx_packets++;//统计已发包数量
		netif_wake_queue(dev);//激活发送序列
	}

	// 处理接收数据

	//检查MAC帧头
	for (i=0; i<2; i++){
		sign <<=8;
		sign += __raw_readw(CPRI_RBUF);
	}
	if (sign != SPD) {
		lp->stats.rx_frame_errors++;
		return IRQ_HANDLED;
	}

	//检查MAC数据包长度
	length = __raw_readw(CPRI_RBUF);
	length = (length<<8) + __raw_readw(CPRI_RBUF);
	if ((~length) != __raw_readw(CPRI_RCNTN)) {
		lp->stats.rx_errors++;
		return IRQ_HANDLED;
	}

	//读取FIFO数据并执行CRC校验
	for (i=0; i<length; i++)
		lp->rxbuf[i] = __raw_readw(CPRI_RBUF);

	for (i=0; i<4; i++){
		crc <<=8;
		crc += __raw_readw(CPRI_RBUF);
	}

	if (crc != ether_crc(length, (char *)lp->rxbuf)) {
		lp->stats.rx_errors++;
	} else {
		net_rx_packet(dev, lp->rxbuf, length);
	}
	return IRQ_HANDLED;
}


static void net_rx_packet(struct net_device *dev, unsigned short *buf, int len)
{
	struct net_local *lp = netdev_priv(dev);
	struct sk_buff *skb;

	//申请并初始化sk_buff结构体
	skb = dev_alloc_skb(len+2);
	if (skb == NULL) {
		lp->stats.rx_dropped++;
		return;
	}

	skb_reserve(skb, 2);

	//为sk_buff结构体的data成员指向的数据空间申请内存
	skb_put(skb, len);
	skb->dev = dev;
	memcpy(skb->data, buf, len);

	//根据MAC地址确定MAC包类型是MULTICAST/OTHERHOST/BROADCAST还是发送给本机的MAC包
	skb->protocol=eth_type_trans(skb, dev);

	//将解析后的MAC包发给上层协议栈
	netif_rx(skb);

	//更新最后接收时间、接收计数器、等统计信息
	dev->last_rx = jiffies;
	lp->stats.rx_packets++;
	lp->stats.rx_bytes += len;
}

//获取网卡统计状态函数
static struct net_device_stats* net_get_stats(struct net_device *dev)
{
	struct net_local *dev_priv;

	dev_priv = netdev_priv(dev);
	return &(dev_priv->stats);
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

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------

static int __init cpri_eth_init(void)
{
	unsigned int irq;
	int ret = 0;

	//为cpri网络设备及其私有设备分配内存空间
	dev_cpri = alloc_etherdev(sizeof(struct net_local));
	if (!dev_cpri){
		return -ENOMEM;
	}


	//初始化网络设备结构体
	irq = CPRI_IRQ;
	set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);
	dev_cpri->irq  = irq;
	dev_cpri->mtu  = MAX_TRX_BUFF_LEN-(2+2+14+4+2); //spd+epd+mac+crc+len

	dev_cpri->open            = net_open;
	dev_cpri->stop            = net_close;
	dev_cpri->get_stats       = net_get_stats;
	dev_cpri->tx_timeout      = net_timeout;
	dev_cpri->set_mac_address = set_mac_address;
	dev_cpri->hard_start_xmit = net_send_packet;
	dev_cpri->watchdog_timeo  = HZ;

	dev_cpri->dev_addr[0] = 0x00;
	dev_cpri->dev_addr[1] = 0xE0;
	dev_cpri->dev_addr[2] = 0x01;
	dev_cpri->dev_addr[3] = 0x00;
	dev_cpri->dev_addr[4] = 0x01;
	dev_cpri->dev_addr[5] = 0x01;


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

