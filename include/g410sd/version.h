/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G410SD
 ::  ::  ::       ::           :::      FileName  : version.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-28 16:02:07
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/

#ifndef __VERSION_H__
#define __VERSION_H__

struct version_elem
{
	unsigned char hard;   //hardware version
	unsigned char cpld;   //cpld version
	unsigned char uboot;  //uboot verision
	unsigned char kernel; //kernel verision
};


#endif // __VERSION_H__
