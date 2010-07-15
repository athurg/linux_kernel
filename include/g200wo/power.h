/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : version.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010.06.09
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/

#ifndef __POWER_H__
#define __POWER_H__

#include <asm/ioctl.h>

//define of Local OSC command
#define CMD_GET_POWER_PEND	_IOR(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_GET_POWER_STAT	_IOR(G200WO_IOCTL_MAGIC, 0x02, int)

#define CMD_SET_POWER_P28	_IOW(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_SET_POWER_PID	_IOW(G200WO_IOCTL_MAGIC, 0x02, int)

#define SIG_POWER               37 //SIGRTMIN is different in Kernel and Libc, so don't use it.

#define POWER_P28_ON	1
#define POWER_P28_OFF	0

#endif // __POWER_H__
