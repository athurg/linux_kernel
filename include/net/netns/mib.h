#ifndef __NETNS_MIB_H__
#define __NETNS_MIB_H__

#include <net/snmp.h>

struct netns_mib {
	DEFINE_SNMP_STAT(struct tcp_mib, tcp_statistics);
	DEFINE_SNMP_STAT(struct ipstats_mib, ip_statistics);
	DEFINE_SNMP_STAT(struct linux_mib, net_statistics);
	DEFINE_SNMP_STAT(struct udp_mib, udp_statistics);
	DEFINE_SNMP_STAT(struct udp_mib, udplite_statistics);
	DEFINE_SNMP_STAT(struct icmp_mib, icmp_statistics);
	DEFINE_SNMP_STAT(struct icmpmsg_mib, icmpmsg_statistics);
};

#endif
