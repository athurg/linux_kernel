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

#define SIG_POWER               37 //SIGRTMIN is different in Kernel and Libc, so don't use it.

#define CMD_POWER_TYPE          MAGIC_POWER    //magic number
#define POWER_GET_PEND          _IO(CMD_POWER_TYPE, 1)
#define POWER_GET_STAT          _IO(CMD_POWER_TYPE, 2)
#define POWER_P28_ON            _IO(CMD_POWER_TYPE, 3)
#define POWER_P28_OFF           _IO(CMD_POWER_TYPE, 4)
#define POWER_SET_PID           _IO(CMD_POWER_TYPE, 5)

#define POWER_STAT_P28ON        0x08
#define POWER_STAT_OTP          0x04
#define POWER_STAT_UVP          0x02
#define POWER_STAT_OVP          0x01

#endif // __POWER_H__
