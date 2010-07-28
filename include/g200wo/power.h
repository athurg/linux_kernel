/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : version.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-28 16:03:52
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/

#ifndef __POWER_H__
#define __POWER_H__

#include <asm/ioctl.h>
#include <g200wo/g200wo_hw.h>

#define CMD_GET_POWER_PEND	_IOR(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_GET_POWER_STAT	_IOR(G200WO_IOCTL_MAGIC, 0x02, int)

#define CMD_SET_POWER_P28	_IOW(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_SET_POWER_PID	_IOW(G200WO_IOCTL_MAGIC, 0x02, int)


#define POWER_P28_ON	1
#define POWER_P28_OFF	0

// Signal number kernel will send to user-space
// while power warnning detect
#define SIG_POWER	37

// some macro-functions for userspace program
#define POWER_OVP(a)	((a>>0) & 0x01)
#define POWER_UVP(a)	((a>>1) & 0x01)
#define POWER_OTP(a)	((a>>2) & 0x01)

#endif // __POWER_H__
