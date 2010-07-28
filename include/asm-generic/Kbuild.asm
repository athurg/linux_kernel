ifneq ($(wildcard $(srctree)/arch/$(SRCARCH)/include/asm/kvm.h \
      		  $(srctree)/include/asm-$(SRCARCH)/kvm.h),)
header-n  += kvm.h
endif

ifneq ($(wildcard $(srctree)/arch/$(SRCARCH)/include/asm/a.out.h \
      		  $(srctree)/include/asm-$(SRCARCH)/a.out.h),)
unifdef-n += a.out.h
endif
unifdef-n += auxvec.h
unifdef-n += byteorder.h
unifdef-n += errno.h
unifdef-n += fcntl.h
unifdef-n += ioctl.h
unifdef-n += ioctls.h
unifdef-n += ipcbuf.h
unifdef-n += mman.h
unifdef-n += msgbuf.h
unifdef-n += param.h
unifdef-n += poll.h
unifdef-n += posix_types.h
unifdef-n += ptrace.h
unifdef-n += resource.h
unifdef-n += sembuf.h
unifdef-n += setup.h
unifdef-n += shmbuf.h
unifdef-n += sigcontext.h
unifdef-n += siginfo.h
unifdef-n += signal.h
unifdef-n += socket.h
unifdef-n += sockios.h
unifdef-n += stat.h
unifdef-n += statfs.h
unifdef-n += termbits.h
unifdef-n += termios.h
unifdef-n += types.h
unifdef-n += unistd.h
