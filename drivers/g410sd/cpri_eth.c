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
#define SPD             0x01FB//MAC֡���ֽڱ�ʾλ
#define EPD             0x01FD//MAC֡ĩ�ֽڱ�ʾλ

//net_local�Ǳ������豸˽����Ϣ�ṹ��
struct net_local {
	struct net_device_stats stats;
	unsigned short txbuf[MAX_TRX_BUFF_LEN];
	unsigned short rxbuf[MAX_TRX_BUFF_LEN];
	int txflag;
	int txlen;
};
//dev_cpri�Ǳ�����������������豸
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

	//�Ƚ��÷������У���ʾ��ǰ���ڴ������У������η�������ټ���
	netif_stop_queue(dev);

	crc = ether_crc(skb->len, skb->data);

	//MAC֡ͷ��SPD��LEN��
	lp->txbuf[i++] = SPD;
	lp->txbuf[i++] = skb->len;
	lp->txbuf[i++] = skb->len>>8;

	//MAC֡����
	for (j=0; j<skb->len; j++)
		lp->txbuf[i++] = skb->data[j];

	//MAC֡CRCУ������
	lp->txbuf[i++] = crc;
	lp->txbuf[i++] = crc>>8;
	lp->txbuf[i++] = crc>>16;
	lp->txbuf[i++] = crc>>24;

	//MAC֡β
	lp->txbuf[i++] = EPD;

	//���ͳ����Ϣ
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

	//��֤�߼�IR�ӿ��Ƿ��λ
	if (0xFF00 & __raw_readw(CPRI_VER)) {
		printk("Interrupt err: invalid IP core version\n");
		return IRQ_HANDLED;
	}

	// ����������
	if (lp->txflag != 0) {
		for (i=0; i<lp->txlen; i+=2) {
			__raw_writew(lp->txbuf[i], CPRI_TBUFL);
			__raw_writew(lp->txbuf[i+1], CPRI_TBUFH);
		}
		//��շ��ͻ�����������־�����ͳ�������
		lp->txflag = 0;
		lp->txlen = 0;
		lp->stats.tx_packets++;//ͳ���ѷ�������
		netif_wake_queue(dev);//���������
	}

	// �����������

	//���MAC֡ͷ
	for (i=0; i<2; i++){
		sign <<=8;
		sign += __raw_readw(CPRI_RBUF);
	}
	if (sign != SPD) {
		lp->stats.rx_frame_errors++;
		return IRQ_HANDLED;
	}

	//���MAC���ݰ�����
	length = __raw_readw(CPRI_RBUF);
	length = (length<<8) + __raw_readw(CPRI_RBUF);
	if ((~length) != __raw_readw(CPRI_RCNTN)) {
		lp->stats.rx_errors++;
		return IRQ_HANDLED;
	}

	//��ȡFIFO���ݲ�ִ��CRCУ��
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

	//���벢��ʼ��sk_buff�ṹ��
	skb = dev_alloc_skb(len+2);
	if (skb == NULL) {
		lp->stats.rx_dropped++;
		return;
	}

	skb_reserve(skb, 2);

	//Ϊsk_buff�ṹ���data��Աָ������ݿռ������ڴ�
	skb_put(skb, len);
	skb->dev = dev;
	memcpy(skb->data, buf, len);

	//����MAC��ַȷ��MAC��������MULTICAST/OTHERHOST/BROADCAST���Ƿ��͸�������MAC��
	skb->protocol=eth_type_trans(skb, dev);

	//���������MAC�������ϲ�Э��ջ
	netif_rx(skb);

	//����������ʱ�䡢���ռ���������ͳ����Ϣ
	dev->last_rx = jiffies;
	lp->stats.rx_packets++;
	lp->stats.rx_bytes += len;
}

//��ȡ����ͳ��״̬����
static struct net_device_stats* net_get_stats(struct net_device *dev)
{
	struct net_local *dev_priv;

	dev_priv = netdev_priv(dev);
	return &(dev_priv->stats);
}

//����MAC��ַ����
static int set_mac_address(struct net_device *dev, void *p)
{
	int i;
	struct sockaddr *addr = p;

	if (netif_running(dev)){
		return -EBUSY;
	}

	//�����豸�ṹ���е�MAC��ַ
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	//����FPGA�е�MAC��ַ
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

	//Ϊcpri�����豸����˽���豸�����ڴ�ռ�
	dev_cpri = alloc_etherdev(sizeof(struct net_local));
	if (!dev_cpri){
		return -ENOMEM;
	}


	//��ʼ�������豸�ṹ��
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


	// ע���豸
	ret = register_netdev(dev_cpri);
	if (ret){
		printk("BSP: %s Fail to Register net device", __FUNCTION__);
		goto error;
	}

	//��ʼ��˽���豸�ṹ��
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

