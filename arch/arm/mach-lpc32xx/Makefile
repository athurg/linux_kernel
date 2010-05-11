#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y				:= timer-lpc32xx.o irq-lpc32xx.o arch-lpc32xx.o
obj-y				+= serial-lpc32xx.o clocks-lpc32xx.o
obj-y				+= dma-lpc32xx.o
obj-m				:= 
obj-n				:=
obj-				:=

obj-$(CONFIG_MACH_PHY3250)	+= board-phy3250.o

obj-$(CONFIG_MACH_ARM9DIMM3250)	+= board-arm9dimm3250.o
obj-$(CONFIG_MACH_EA3250)	+= board-ea3250.o
obj-$(CONFIG_MACH_NTS3250)	+= board-nts3250.o

