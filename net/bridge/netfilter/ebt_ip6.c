/*
 *  ebt_ip6
 *
 *	Authors:
 *	Manohar Castelino <manohar.r.castelino@intel.com>
 *	Kuo-Lang Tseng <kuo-lang.tseng@intel.com>
 *	Jan Engelhardt <jengelh@computergmbh.de>
 *
 * Summary:
 * This is just a modification of the IPv4 code written by
 * Bart De Schuymer <bdschuym@pandora.be>
 * with the changes required to support IPv6
 *
 *  Jan, 2008
 */

#include <linux/netfilter_bridge/ebtables.h>
#include <linux/netfilter_bridge/ebt_ip6.h>
#include <linux/ipv6.h>
#include <net/ipv6.h>
#include <linux/in.h>
#include <linux/module.h>
#include <net/dsfield.h>

struct tcpudphdr {
	__be16 src;
	__be16 dst;
};

static int ebt_filter_ip6(const struct sk_buff *skb,
   const struct net_device *in,
   const struct net_device *out, const void *data,
   unsigned int datalen)
{
	const struct ebt_ip6_info *info = (struct ebt_ip6_info *)data;
	const struct ipv6hdr *ih6;
	struct ipv6hdr _ip6h;
	const struct tcpudphdr *pptr;
	struct tcpudphdr _ports;
	struct in6_addr tmp_addr;
	int i;

	ih6 = skb_header_pointer(skb, 0, sizeof(_ip6h), &_ip6h);
	if (ih6 == NULL)
		return EBT_NOMATCH;
	if (info->bitmask & EBT_IP6_TCLASS &&
	   FWINV(info->tclass != ipv6_get_dsfield(ih6), EBT_IP6_TCLASS))
		return EBT_NOMATCH;
	for (i = 0; i < 4; i++)
		tmp_addr.in6_u.u6_addr32[i] = ih6->saddr.in6_u.u6_addr32[i] &
			info->smsk.in6_u.u6_addr32[i];
	if (info->bitmask & EBT_IP6_SOURCE &&
		FWINV((ipv6_addr_cmp(&tmp_addr, &info->saddr) != 0),
			EBT_IP6_SOURCE))
		return EBT_NOMATCH;
	for (i = 0; i < 4; i++)
		tmp_addr.in6_u.u6_addr32[i] = ih6->daddr.in6_u.u6_addr32[i] &
			info->dmsk.in6_u.u6_addr32[i];
	if (info->bitmask & EBT_IP6_DEST &&
	   FWINV((ipv6_addr_cmp(&tmp_addr, &info->daddr) != 0), EBT_IP6_DEST))
		return EBT_NOMATCH;
	if (info->bitmask & EBT_IP6_PROTO) {
		uint8_t nexthdr = ih6->nexthdr;
		int offset_ph;

		offset_ph = ipv6_skip_exthdr(skb, sizeof(_ip6h), &nexthdr);
		if (offset_ph == -1)
			return EBT_NOMATCH;
		if (FWINV(info->protocol != nexthdr, EBT_IP6_PROTO))
			return EBT_NOMATCH;
		if (!(info->bitmask & EBT_IP6_DPORT) &&
		    !(info->bitmask & EBT_IP6_SPORT))
			return EBT_MATCH;
		pptr = skb_header_pointer(skb, offset_ph, sizeof(_ports),
					  &_ports);
		if (pptr == NULL)
			return EBT_NOMATCH;
		if (info->bitmask & EBT_IP6_DPORT) {
			u32 dst = ntohs(pptr->dst);
			if (FWINV(dst < info->dport[0] ||
				  dst > info->dport[1], EBT_IP6_DPORT))
				return EBT_NOMATCH;
		}
		if (info->bitmask & EBT_IP6_SPORT) {
			u32 src = ntohs(pptr->src);
			if (FWINV(src < info->sport[0] ||
				  src > info->sport[1], EBT_IP6_SPORT))
			return EBT_NOMATCH;
		}
		return EBT_MATCH;
	}
	return EBT_MATCH;
}

static int ebt_ip6_check(const char *tablename, unsigned int hookmask,
   const struct ebt_entry *e, void *data, unsigned int datalen)
{
	struct ebt_ip6_info *info = (struct ebt_ip6_info *)data;

	if (datalen != EBT_ALIGN(sizeof(struct ebt_ip6_info)))
		return -EINVAL;
	if (e->ethproto != htons(ETH_P_IPV6) || e->invflags & EBT_IPROTO)
		return -EINVAL;
	if (info->bitmask & ~EBT_IP6_MASK || info->invflags & ~EBT_IP6_MASK)
		return -EINVAL;
	if (info->bitmask & (EBT_IP6_DPORT | EBT_IP6_SPORT)) {
		if (info->invflags & EBT_IP6_PROTO)
			return -EINVAL;
		if (info->protocol != IPPROTO_TCP &&
		    info->protocol != IPPROTO_UDP &&
		    info->protocol != IPPROTO_UDPLITE &&
		    info->protocol != IPPROTO_SCTP &&
		    info->protocol != IPPROTO_DCCP)
			 return -EINVAL;
	}
	if (info->bitmask & EBT_IP6_DPORT && info->dport[0] > info->dport[1])
		return -EINVAL;
	if (info->bitmask & EBT_IP6_SPORT && info->sport[0] > info->sport[1])
		return -EINVAL;
	return 0;
}

static struct ebt_match filter_ip6 =
{
	.name		= EBT_IP6_MATCH,
	.match		= ebt_filter_ip6,
	.check		= ebt_ip6_check,
	.me		= THIS_MODULE,
};

static int __init ebt_ip6_init(void)
{
	return ebt_register_match(&filter_ip6);
}

static void __exit ebt_ip6_fini(void)
{
	ebt_unregister_match(&filter_ip6);
}

module_init(ebt_ip6_init);
module_exit(ebt_ip6_fini);
MODULE_DESCRIPTION("Ebtables: IPv6 protocol packet match");
MODULE_LICENSE("GPL");
