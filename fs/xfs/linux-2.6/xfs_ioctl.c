/*
 * Copyright (c) 2000-2005 Silicon Graphics, Inc.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write the Free Software Foundation,
 * Inc.,  51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include "xfs.h"
#include "xfs_fs.h"
#include "xfs_bit.h"
#include "xfs_log.h"
#include "xfs_inum.h"
#include "xfs_trans.h"
#include "xfs_sb.h"
#include "xfs_ag.h"
#include "xfs_dir2.h"
#include "xfs_alloc.h"
#include "xfs_dmapi.h"
#include "xfs_mount.h"
#include "xfs_bmap_btree.h"
#include "xfs_alloc_btree.h"
#include "xfs_ialloc_btree.h"
#include "xfs_attr_sf.h"
#include "xfs_dir2_sf.h"
#include "xfs_dinode.h"
#include "xfs_inode.h"
#include "xfs_btree.h"
#include "xfs_ialloc.h"
#include "xfs_rtalloc.h"
#include "xfs_itable.h"
#include "xfs_error.h"
#include "xfs_rw.h"
#include "xfs_acl.h"
#include "xfs_attr.h"
#include "xfs_bmap.h"
#include "xfs_buf_item.h"
#include "xfs_utils.h"
#include "xfs_dfrag.h"
#include "xfs_fsops.h"
#include "xfs_vnodeops.h"
#include "xfs_quota.h"
#include "xfs_inode_item.h"

#include <linux/capability.h>
#include <linux/dcache.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/pagemap.h>

/*
 * xfs_find_handle maps from userspace xfs_fsop_handlereq structure to
 * a file or fs handle.
 *
 * XFS_IOC_PATH_TO_FSHANDLE
 *    returns fs handle for a mount point or path within that mount point
 * XFS_IOC_FD_TO_HANDLE
 *    returns full handle for a FD opened in user space
 * XFS_IOC_PATH_TO_HANDLE
 *    returns full handle for a path
 */
STATIC int
xfs_find_handle(
	unsigned int		cmd,
	void			__user *arg)
{
	int			hsize;
	xfs_handle_t		handle;
	xfs_fsop_handlereq_t	hreq;
	struct inode		*inode;

	if (copy_from_user(&hreq, arg, sizeof(hreq)))
		return -XFS_ERROR(EFAULT);

	memset((char *)&handle, 0, sizeof(handle));

	switch (cmd) {
	case XFS_IOC_PATH_TO_FSHANDLE:
	case XFS_IOC_PATH_TO_HANDLE: {
		struct path path;
		int error = user_lpath((const char __user *)hreq.path, &path);
		if (error)
			return error;

		ASSERT(path.dentry);
		ASSERT(path.dentry->d_inode);
		inode = igrab(path.dentry->d_inode);
		path_put(&path);
		break;
	}

	case XFS_IOC_FD_TO_HANDLE: {
		struct file	*file;

		file = fget(hreq.fd);
		if (!file)
		    return -EBADF;

		ASSERT(file->f_path.dentry);
		ASSERT(file->f_path.dentry->d_inode);
		inode = igrab(file->f_path.dentry->d_inode);
		fput(file);
		break;
	}

	default:
		ASSERT(0);
		return -XFS_ERROR(EINVAL);
	}

	if (inode->i_sb->s_magic != XFS_SB_MAGIC) {
		/* we're not in XFS anymore, Toto */
		iput(inode);
		return -XFS_ERROR(EINVAL);
	}

	switch (inode->i_mode & S_IFMT) {
	case S_IFREG:
	case S_IFDIR:
	case S_IFLNK:
		break;
	default:
		iput(inode);
		return -XFS_ERROR(EBADF);
	}

	/* now we can grab the fsid */
	memcpy(&handle.ha_fsid, XFS_I(inode)->i_mount->m_fixedfsid,
			sizeof(xfs_fsid_t));
	hsize = sizeof(xfs_fsid_t);

	if (cmd != XFS_IOC_PATH_TO_FSHANDLE) {
		xfs_inode_t	*ip = XFS_I(inode);
		int		lock_mode;

		/* need to get access to the xfs_inode to read the generation */
		lock_mode = xfs_ilock_map_shared(ip);

		/* fill in fid section of handle from inode */
		handle.ha_fid.fid_len = sizeof(xfs_fid_t) -
					sizeof(handle.ha_fid.fid_len);
		handle.ha_fid.fid_pad = 0;
		handle.ha_fid.fid_gen = ip->i_d.di_gen;
		handle.ha_fid.fid_ino = ip->i_ino;

		xfs_iunlock_map_shared(ip, lock_mode);

		hsize = XFS_HSIZE(handle);
	}

	/* now copy our handle into the user buffer & write out the size */
	if (copy_to_user(hreq.ohandle, &handle, hsize) ||
	    copy_to_user(hreq.ohandlen, &hsize, sizeof(__s32))) {
		iput(inode);
		return -XFS_ERROR(EFAULT);
	}

	iput(inode);
	return 0;
}


/*
 * Convert userspace handle data into inode.
 *
 * We use the fact that all the fsop_handlereq ioctl calls have a data
 * structure argument whose first component is always a xfs_fsop_handlereq_t,
 * so we can pass that sub structure into this handy, shared routine.
 *
 * If no error, caller must always iput the returned inode.
 */
STATIC int
xfs_vget_fsop_handlereq(
	xfs_mount_t		*mp,
	struct inode		*parinode,	/* parent inode pointer    */
	xfs_fsop_handlereq_t	*hreq,
	struct inode		**inode)
{
	void			__user *hanp;
	size_t			hlen;
	xfs_fid_t		*xfid;
	xfs_handle_t		*handlep;
	xfs_handle_t		handle;
	xfs_inode_t		*ip;
	xfs_ino_t		ino;
	__u32			igen;
	int			error;

	/*
	 * Only allow handle opens under a directory.
	 */
	if (!S_ISDIR(parinode->i_mode))
		return XFS_ERROR(ENOTDIR);

	hanp = hreq->ihandle;
	hlen = hreq->ihandlen;
	handlep = &handle;

	if (hlen < sizeof(handlep->ha_fsid) || hlen > sizeof(*handlep))
		return XFS_ERROR(EINVAL);
	if (copy_from_user(handlep, hanp, hlen))
		return XFS_ERROR(EFAULT);
	if (hlen < sizeof(*handlep))
		memset(((char *)handlep) + hlen, 0, sizeof(*handlep) - hlen);
	if (hlen > sizeof(handlep->ha_fsid)) {
		if (handlep->ha_fid.fid_len !=
		    (hlen - sizeof(handlep->ha_fsid) -
		            sizeof(handlep->ha_fid.fid_len)) ||
		    handlep->ha_fid.fid_pad)
			return XFS_ERROR(EINVAL);
	}

	/*
	 * Crack the handle, obtain the inode # & generation #
	 */
	xfid = (struct xfs_fid *)&handlep->ha_fid;
	if (xfid->fid_len == sizeof(*xfid) - sizeof(xfid->fid_len)) {
		ino  = xfid->fid_ino;
		igen = xfid->fid_gen;
	} else {
		return XFS_ERROR(EINVAL);
	}

	/*
	 * Get the XFS inode, building a Linux inode to go with it.
	 */
	error = xfs_iget(mp, NULL, ino, 0, XFS_ILOCK_SHARED, &ip, 0);
	if (error)
		return error;
	if (ip == NULL)
		return XFS_ERROR(EIO);
	if (ip->i_d.di_gen != igen) {
		xfs_iput_new(ip, XFS_ILOCK_SHARED);
		return XFS_ERROR(ENOENT);
	}

	xfs_iunlock(ip, XFS_ILOCK_SHARED);

	*inode = VFS_I(ip);
	return 0;
}

STATIC int
xfs_open_by_handle(
	xfs_mount_t		*mp,
	void			__user *arg,
	struct file		*parfilp,
	struct inode		*parinode)
{
	int			error;
	int			new_fd;
	int			permflag;
	struct file		*filp;
	struct inode		*inode;
	struct dentry		*dentry;
	xfs_fsop_handlereq_t	hreq;

	if (!capable(CAP_SYS_ADMIN))
		return -XFS_ERROR(EPERM);
	if (copy_from_user(&hreq, arg, sizeof(xfs_fsop_handlereq_t)))
		return -XFS_ERROR(EFAULT);

	error = xfs_vget_fsop_handlereq(mp, parinode, &hreq, &inode);
	if (error)
		return -error;

	/* Restrict xfs_open_by_handle to directories & regular files. */
	if (!(S_ISREG(inode->i_mode) || S_ISDIR(inode->i_mode))) {
		iput(inode);
		return -XFS_ERROR(EINVAL);
	}

#if BITS_PER_LONG != 32
	hreq.oflags |= O_LARGEFILE;
#endif
	/* Put open permission in namei format. */
	permflag = hreq.oflags;
	if ((permflag+1) & O_ACCMODE)
		permflag++;
	if (permflag & O_TRUNC)
		permflag |= 2;

	if ((!(permflag & O_APPEND) || (permflag & O_TRUNC)) &&
	    (permflag & FMODE_WRITE) && IS_APPEND(inode)) {
		iput(inode);
		return -XFS_ERROR(EPERM);
	}

	if ((permflag & FMODE_WRITE) && IS_IMMUTABLE(inode)) {
		iput(inode);
		return -XFS_ERROR(EACCES);
	}

	/* Can't write directories. */
	if ( S_ISDIR(inode->i_mode) && (permflag & FMODE_WRITE)) {
		iput(inode);
		return -XFS_ERROR(EISDIR);
	}

	if ((new_fd = get_unused_fd()) < 0) {
		iput(inode);
		return new_fd;
	}

	dentry = d_alloc_anon(inode);
	if (dentry == NULL) {
		iput(inode);
		put_unused_fd(new_fd);
		return -XFS_ERROR(ENOMEM);
	}

	/* Ensure umount returns EBUSY on umounts while this file is open. */
	mntget(parfilp->f_path.mnt);

	/* Create file pointer. */
	filp = dentry_open(dentry, parfilp->f_path.mnt, hreq.oflags);
	if (IS_ERR(filp)) {
		put_unused_fd(new_fd);
		return -XFS_ERROR(-PTR_ERR(filp));
	}
	if (inode->i_mode & S_IFREG) {
		/* invisible operation should not change atime */
		filp->f_flags |= O_NOATIME;
		filp->f_op = &xfs_invis_file_operations;
	}

	fd_install(new_fd, filp);
	return new_fd;
}

/*
 * This is a copy from fs/namei.c:vfs_readlink(), except for removing it's
 * unused first argument.
 */
STATIC int
do_readlink(
	char __user		*buffer,
	int			buflen,
	const char		*link)
{
        int len;

	len = PTR_ERR(link);
	if (IS_ERR(link))
		goto out;

	len = strlen(link);
	if (len > (unsigned) buflen)
		len = buflen;
	if (copy_to_user(buffer, link, len))
		len = -EFAULT;
 out:
	return len;
}


STATIC int
xfs_readlink_by_handle(
	xfs_mount_t		*mp,
	void			__user *arg,
	struct inode		*parinode)
{
	struct inode		*inode;
	xfs_fsop_handlereq_t	hreq;
	__u32			olen;
	void			*link;
	int			error;

	if (!capable(CAP_SYS_ADMIN))
		return -XFS_ERROR(EPERM);
	if (copy_from_user(&hreq, arg, sizeof(xfs_fsop_handlereq_t)))
		return -XFS_ERROR(EFAULT);

	error = xfs_vget_fsop_handlereq(mp, parinode, &hreq, &inode);
	if (error)
		return -error;

	/* Restrict this handle operation to symlinks only. */
	if (!S_ISLNK(inode->i_mode)) {
		error = -XFS_ERROR(EINVAL);
		goto out_iput;
	}

	if (copy_from_user(&olen, hreq.ohandlen, sizeof(__u32))) {
		error = -XFS_ERROR(EFAULT);
		goto out_iput;
	}

	link = kmalloc(MAXPATHLEN+1, GFP_KERNEL);
	if (!link)
		goto out_iput;

	error = -xfs_readlink(XFS_I(inode), link);
	if (error)
		goto out_kfree;
	error = do_readlink(hreq.ohandle, olen, link);
	if (error)
		goto out_kfree;

 out_kfree:
	kfree(link);
 out_iput:
	iput(inode);
	return error;
}

STATIC int
xfs_fssetdm_by_handle(
	xfs_mount_t		*mp,
	void			__user *arg,
	struct inode		*parinode)
{
	int			error;
	struct fsdmidata	fsd;
	xfs_fsop_setdm_handlereq_t dmhreq;
	struct inode		*inode;

	if (!capable(CAP_MKNOD))
		return -XFS_ERROR(EPERM);
	if (copy_from_user(&dmhreq, arg, sizeof(xfs_fsop_setdm_handlereq_t)))
		return -XFS_ERROR(EFAULT);

	error = xfs_vget_fsop_handlereq(mp, parinode, &dmhreq.hreq, &inode);
	if (error)
		return -error;

	if (IS_IMMUTABLE(inode) || IS_APPEND(inode)) {
		error = -XFS_ERROR(EPERM);
		goto out;
	}

	if (copy_from_user(&fsd, dmhreq.data, sizeof(fsd))) {
		error = -XFS_ERROR(EFAULT);
		goto out;
	}

	error = -xfs_set_dmattrs(XFS_I(inode), fsd.fsd_dmevmask,
				 fsd.fsd_dmstate);

 out:
	iput(inode);
	return error;
}

STATIC int
xfs_attrlist_by_handle(
	xfs_mount_t		*mp,
	void			__user *arg,
	struct inode		*parinode)
{
	int			error;
	attrlist_cursor_kern_t	*cursor;
	xfs_fsop_attrlist_handlereq_t al_hreq;
	struct inode		*inode;
	char			*kbuf;

	if (!capable(CAP_SYS_ADMIN))
		return -XFS_ERROR(EPERM);
	if (copy_from_user(&al_hreq, arg, sizeof(xfs_fsop_attrlist_handlereq_t)))
		return -XFS_ERROR(EFAULT);
	if (al_hreq.buflen > XATTR_LIST_MAX)
		return -XFS_ERROR(EINVAL);

	/*
	 * Reject flags, only allow namespaces.
	 */
	if (al_hreq.flags & ~(ATTR_ROOT | ATTR_SECURE))
		return -XFS_ERROR(EINVAL);

	error = xfs_vget_fsop_handlereq(mp, parinode, &al_hreq.hreq, &inode);
	if (error)
		goto out;

	kbuf = kmalloc(al_hreq.buflen, GFP_KERNEL);
	if (!kbuf)
		goto out_vn_rele;

	cursor = (attrlist_cursor_kern_t *)&al_hreq.pos;
	error = xfs_attr_list(XFS_I(inode), kbuf, al_hreq.buflen,
					al_hreq.flags, cursor);
	if (error)
		goto out_kfree;

	if (copy_to_user(al_hreq.buffer, kbuf, al_hreq.buflen))
		error = -EFAULT;

 out_kfree:
	kfree(kbuf);
 out_vn_rele:
	iput(inode);
 out:
	return -error;
}

STATIC int
xfs_attrmulti_attr_get(
	struct inode		*inode,
	char			*name,
	char			__user *ubuf,
	__uint32_t		*len,
	__uint32_t		flags)
{
	char			*kbuf;
	int			error = EFAULT;

	if (*len > XATTR_SIZE_MAX)
		return EINVAL;
	kbuf = kmalloc(*len, GFP_KERNEL);
	if (!kbuf)
		return ENOMEM;

	error = xfs_attr_get(XFS_I(inode), name, kbuf, (int *)len, flags);
	if (error)
		goto out_kfree;

	if (copy_to_user(ubuf, kbuf, *len))
		error = EFAULT;

 out_kfree:
	kfree(kbuf);
	return error;
}

STATIC int
xfs_attrmulti_attr_set(
	struct inode		*inode,
	char			*name,
	const char		__user *ubuf,
	__uint32_t		len,
	__uint32_t		flags)
{
	char			*kbuf;
	int			error = EFAULT;

	if (IS_IMMUTABLE(inode) || IS_APPEND(inode))
		return EPERM;
	if (len > XATTR_SIZE_MAX)
		return EINVAL;

	kbuf = kmalloc(len, GFP_KERNEL);
	if (!kbuf)
		return ENOMEM;

	if (copy_from_user(kbuf, ubuf, len))
		goto out_kfree;

	error = xfs_attr_set(XFS_I(inode), name, kbuf, len, flags);

 out_kfree:
	kfree(kbuf);
	return error;
}

STATIC int
xfs_attrmulti_attr_remove(
	struct inode		*inode,
	char			*name,
	__uint32_t		flags)
{
	if (IS_IMMUTABLE(inode) || IS_APPEND(inode))
		return EPERM;
	return xfs_attr_remove(XFS_I(inode), name, flags);
}

STATIC int
xfs_attrmulti_by_handle(
	xfs_mount_t		*mp,
	void			__user *arg,
	struct file		*parfilp,
	struct inode		*parinode)
{
	int			error;
	xfs_attr_multiop_t	*ops;
	xfs_fsop_attrmulti_handlereq_t am_hreq;
	struct inode		*inode;
	unsigned int		i, size;
	char			*attr_name;

	if (!capable(CAP_SYS_ADMIN))
		return -XFS_ERROR(EPERM);
	if (copy_from_user(&am_hreq, arg, sizeof(xfs_fsop_attrmulti_handlereq_t)))
		return -XFS_ERROR(EFAULT);

	error = xfs_vget_fsop_handlereq(mp, parinode, &am_hreq.hreq, &inode);
	if (error)
		goto out;

	error = E2BIG;
	size = am_hreq.opcount * sizeof(xfs_attr_multiop_t);
	if (!size || size > 16 * PAGE_SIZE)
		goto out_vn_rele;

	error = ENOMEM;
	ops = kmalloc(size, GFP_KERNEL);
	if (!ops)
		goto out_vn_rele;

	error = EFAULT;
	if (copy_from_user(ops, am_hreq.ops, size))
		goto out_kfree_ops;

	attr_name = kmalloc(MAXNAMELEN, GFP_KERNEL);
	if (!attr_name)
		goto out_kfree_ops;


	error = 0;
	for (i = 0; i < am_hreq.opcount; i++) {
		ops[i].am_error = strncpy_from_user(attr_name,
				ops[i].am_attrname, MAXNAMELEN);
		if (ops[i].am_error == 0 || ops[i].am_error == MAXNAMELEN)
			error = -ERANGE;
		if (ops[i].am_error < 0)
			break;

		switch (ops[i].am_opcode) {
		case ATTR_OP_GET:
			ops[i].am_error = xfs_attrmulti_attr_get(inode,
					attr_name, ops[i].am_attrvalue,
					&ops[i].am_length, ops[i].am_flags);
			break;
		case ATTR_OP_SET:
			ops[i].am_error = mnt_want_write(parfilp->f_path.mnt);
			if (ops[i].am_error)
				break;
			ops[i].am_error = xfs_attrmulti_attr_set(inode,
					attr_name, ops[i].am_attrvalue,
					ops[i].am_length, ops[i].am_flags);
			mnt_drop_write(parfilp->f_path.mnt);
			break;
		case ATTR_OP_REMOVE:
			ops[i].am_error = mnt_want_write(parfilp->f_path.mnt);
			if (ops[i].am_error)
				break;
			ops[i].am_error = xfs_attrmulti_attr_remove(inode,
					attr_name, ops[i].am_flags);
			mnt_drop_write(parfilp->f_path.mnt);
			break;
		default:
			ops[i].am_error = EINVAL;
		}
	}

	if (copy_to_user(am_hreq.ops, ops, size))
		error = XFS_ERROR(EFAULT);

	kfree(attr_name);
 out_kfree_ops:
	kfree(ops);
 out_vn_rele:
	iput(inode);
 out:
	return -error;
}

STATIC int
xfs_ioc_space(
	struct xfs_inode	*ip,
	struct inode		*inode,
	struct file		*filp,
	int			ioflags,
	unsigned int		cmd,
	void			__user *arg)
{
	xfs_flock64_t		bf;
	int			attr_flags = 0;
	int			error;

	if (inode->i_flags & (S_IMMUTABLE|S_APPEND))
		return -XFS_ERROR(EPERM);

	if (!(filp->f_mode & FMODE_WRITE))
		return -XFS_ERROR(EBADF);

	if (!S_ISREG(inode->i_mode))
		return -XFS_ERROR(EINVAL);

	if (copy_from_user(&bf, arg, sizeof(bf)))
		return -XFS_ERROR(EFAULT);

	if (filp->f_flags & (O_NDELAY|O_NONBLOCK))
		attr_flags |= XFS_ATTR_NONBLOCK;
	if (ioflags & IO_INVIS)
		attr_flags |= XFS_ATTR_DMI;

	error = xfs_change_file_space(ip, cmd, &bf, filp->f_pos,
					      NULL, attr_flags);
	return -error;
}

STATIC int
xfs_ioc_bulkstat(
	xfs_mount_t		*mp,
	unsigned int		cmd,
	void			__user *arg)
{
	xfs_fsop_bulkreq_t	bulkreq;
	int			count;	/* # of records returned */
	xfs_ino_t		inlast;	/* last inode number */
	int			done;
	int			error;

	/* done = 1 if there are more stats to get and if bulkstat */
	/* should be called again (unused here, but used in dmapi) */

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (XFS_FORCED_SHUTDOWN(mp))
		return -XFS_ERROR(EIO);

	if (copy_from_user(&bulkreq, arg, sizeof(xfs_fsop_bulkreq_t)))
		return -XFS_ERROR(EFAULT);

	if (copy_from_user(&inlast, bulkreq.lastip, sizeof(__s64)))
		return -XFS_ERROR(EFAULT);

	if ((count = bulkreq.icount) <= 0)
		return -XFS_ERROR(EINVAL);

	if (bulkreq.ubuffer == NULL)
		return -XFS_ERROR(EINVAL);

	if (cmd == XFS_IOC_FSINUMBERS)
		error = xfs_inumbers(mp, &inlast, &count,
					bulkreq.ubuffer, xfs_inumbers_fmt);
	else if (cmd == XFS_IOC_FSBULKSTAT_SINGLE)
		error = xfs_bulkstat_single(mp, &inlast,
						bulkreq.ubuffer, &done);
	else	/* XFS_IOC_FSBULKSTAT */
		error = xfs_bulkstat(mp, &inlast, &count,
			(bulkstat_one_pf)xfs_bulkstat_one, NULL,
			sizeof(xfs_bstat_t), bulkreq.ubuffer,
			BULKSTAT_FG_QUICK, &done);

	if (error)
		return -error;

	if (bulkreq.ocount != NULL) {
		if (copy_to_user(bulkreq.lastip, &inlast,
						sizeof(xfs_ino_t)))
			return -XFS_ERROR(EFAULT);

		if (copy_to_user(bulkreq.ocount, &count, sizeof(count)))
			return -XFS_ERROR(EFAULT);
	}

	return 0;
}

STATIC int
xfs_ioc_fsgeometry_v1(
	xfs_mount_t		*mp,
	void			__user *arg)
{
	xfs_fsop_geom_v1_t	fsgeo;
	int			error;

	error = xfs_fs_geometry(mp, (xfs_fsop_geom_t *)&fsgeo, 3);
	if (error)
		return -error;

	if (copy_to_user(arg, &fsgeo, sizeof(fsgeo)))
		return -XFS_ERROR(EFAULT);
	return 0;
}

STATIC int
xfs_ioc_fsgeometry(
	xfs_mount_t		*mp,
	void			__user *arg)
{
	xfs_fsop_geom_t		fsgeo;
	int			error;

	error = xfs_fs_geometry(mp, &fsgeo, 4);
	if (error)
		return -error;

	if (copy_to_user(arg, &fsgeo, sizeof(fsgeo)))
		return -XFS_ERROR(EFAULT);
	return 0;
}

/*
 * Linux extended inode flags interface.
 */

STATIC unsigned int
xfs_merge_ioc_xflags(
	unsigned int	flags,
	unsigned int	start)
{
	unsigned int	xflags = start;

	if (flags & FS_IMMUTABLE_FL)
		xflags |= XFS_XFLAG_IMMUTABLE;
	else
		xflags &= ~XFS_XFLAG_IMMUTABLE;
	if (flags & FS_APPEND_FL)
		xflags |= XFS_XFLAG_APPEND;
	else
		xflags &= ~XFS_XFLAG_APPEND;
	if (flags & FS_SYNC_FL)
		xflags |= XFS_XFLAG_SYNC;
	else
		xflags &= ~XFS_XFLAG_SYNC;
	if (flags & FS_NOATIME_FL)
		xflags |= XFS_XFLAG_NOATIME;
	else
		xflags &= ~XFS_XFLAG_NOATIME;
	if (flags & FS_NODUMP_FL)
		xflags |= XFS_XFLAG_NODUMP;
	else
		xflags &= ~XFS_XFLAG_NODUMP;

	return xflags;
}

STATIC unsigned int
xfs_di2lxflags(
	__uint16_t	di_flags)
{
	unsigned int	flags = 0;

	if (di_flags & XFS_DIFLAG_IMMUTABLE)
		flags |= FS_IMMUTABLE_FL;
	if (di_flags & XFS_DIFLAG_APPEND)
		flags |= FS_APPEND_FL;
	if (di_flags & XFS_DIFLAG_SYNC)
		flags |= FS_SYNC_FL;
	if (di_flags & XFS_DIFLAG_NOATIME)
		flags |= FS_NOATIME_FL;
	if (di_flags & XFS_DIFLAG_NODUMP)
		flags |= FS_NODUMP_FL;
	return flags;
}

STATIC int
xfs_ioc_fsgetxattr(
	xfs_inode_t		*ip,
	int			attr,
	void			__user *arg)
{
	struct fsxattr		fa;

	xfs_ilock(ip, XFS_ILOCK_SHARED);
	fa.fsx_xflags = xfs_ip2xflags(ip);
	fa.fsx_extsize = ip->i_d.di_extsize << ip->i_mount->m_sb.sb_blocklog;
	fa.fsx_projid = ip->i_d.di_projid;

	if (attr) {
		if (ip->i_afp) {
			if (ip->i_afp->if_flags & XFS_IFEXTENTS)
				fa.fsx_nextents = ip->i_afp->if_bytes /
							sizeof(xfs_bmbt_rec_t);
			else
				fa.fsx_nextents = ip->i_d.di_anextents;
		} else
			fa.fsx_nextents = 0;
	} else {
		if (ip->i_df.if_flags & XFS_IFEXTENTS)
			fa.fsx_nextents = ip->i_df.if_bytes /
						sizeof(xfs_bmbt_rec_t);
		else
			fa.fsx_nextents = ip->i_d.di_nextents;
	}
	xfs_iunlock(ip, XFS_ILOCK_SHARED);

	if (copy_to_user(arg, &fa, sizeof(fa)))
		return -EFAULT;
	return 0;
}

STATIC void
xfs_set_diflags(
	struct xfs_inode	*ip,
	unsigned int		xflags)
{
	unsigned int		di_flags;

	/* can't set PREALLOC this way, just preserve it */
	di_flags = (ip->i_d.di_flags & XFS_DIFLAG_PREALLOC);
	if (xflags & XFS_XFLAG_IMMUTABLE)
		di_flags |= XFS_DIFLAG_IMMUTABLE;
	if (xflags & XFS_XFLAG_APPEND)
		di_flags |= XFS_DIFLAG_APPEND;
	if (xflags & XFS_XFLAG_SYNC)
		di_flags |= XFS_DIFLAG_SYNC;
	if (xflags & XFS_XFLAG_NOATIME)
		di_flags |= XFS_DIFLAG_NOATIME;
	if (xflags & XFS_XFLAG_NODUMP)
		di_flags |= XFS_DIFLAG_NODUMP;
	if (xflags & XFS_XFLAG_PROJINHERIT)
		di_flags |= XFS_DIFLAG_PROJINHERIT;
	if (xflags & XFS_XFLAG_NODEFRAG)
		di_flags |= XFS_DIFLAG_NODEFRAG;
	if (xflags & XFS_XFLAG_FILESTREAM)
		di_flags |= XFS_DIFLAG_FILESTREAM;
	if ((ip->i_d.di_mode & S_IFMT) == S_IFDIR) {
		if (xflags & XFS_XFLAG_RTINHERIT)
			di_flags |= XFS_DIFLAG_RTINHERIT;
		if (xflags & XFS_XFLAG_NOSYMLINKS)
			di_flags |= XFS_DIFLAG_NOSYMLINKS;
		if (xflags & XFS_XFLAG_EXTSZINHERIT)
			di_flags |= XFS_DIFLAG_EXTSZINHERIT;
	} else if ((ip->i_d.di_mode & S_IFMT) == S_IFREG) {
		if (xflags & XFS_XFLAG_REALTIME)
			di_flags |= XFS_DIFLAG_REALTIME;
		if (xflags & XFS_XFLAG_EXTSIZE)
			di_flags |= XFS_DIFLAG_EXTSIZE;
	}

	ip->i_d.di_flags = di_flags;
}

STATIC void
xfs_diflags_to_linux(
	struct xfs_inode	*ip)
{
	struct inode		*inode = VFS_I(ip);
	unsigned int		xflags = xfs_ip2xflags(ip);

	if (xflags & XFS_XFLAG_IMMUTABLE)
		inode->i_flags |= S_IMMUTABLE;
	else
		inode->i_flags &= ~S_IMMUTABLE;
	if (xflags & XFS_XFLAG_APPEND)
		inode->i_flags |= S_APPEND;
	else
		inode->i_flags &= ~S_APPEND;
	if (xflags & XFS_XFLAG_SYNC)
		inode->i_flags |= S_SYNC;
	else
		inode->i_flags &= ~S_SYNC;
	if (xflags & XFS_XFLAG_NOATIME)
		inode->i_flags |= S_NOATIME;
	else
		inode->i_flags &= ~S_NOATIME;
}

#define FSX_PROJID	1
#define FSX_EXTSIZE	2
#define FSX_XFLAGS	4
#define FSX_NONBLOCK	8

STATIC int
xfs_ioctl_setattr(
	xfs_inode_t		*ip,
	struct fsxattr		*fa,
	int			mask)
{
	struct xfs_mount	*mp = ip->i_mount;
	struct xfs_trans	*tp;
	unsigned int		lock_flags = 0;
	struct xfs_dquot	*udqp = NULL, *gdqp = NULL;
	struct xfs_dquot	*olddquot = NULL;
	int			code;

	xfs_itrace_entry(ip);

	if (mp->m_flags & XFS_MOUNT_RDONLY)
		return XFS_ERROR(EROFS);
	if (XFS_FORCED_SHUTDOWN(mp))
		return XFS_ERROR(EIO);

	/*
	 * If disk quotas is on, we make sure that the dquots do exist on disk,
	 * before we start any other transactions. Trying to do this later
	 * is messy. We don't care to take a readlock to look at the ids
	 * in inode here, because we can't hold it across the trans_reserve.
	 * If the IDs do change before we take the ilock, we're covered
	 * because the i_*dquot fields will get updated anyway.
	 */
	if (XFS_IS_QUOTA_ON(mp) && (mask & FSX_PROJID)) {
		code = XFS_QM_DQVOPALLOC(mp, ip, ip->i_d.di_uid,
					 ip->i_d.di_gid, fa->fsx_projid,
					 XFS_QMOPT_PQUOTA, &udqp, &gdqp);
		if (code)
			return code;
	}

	/*
	 * For the other attributes, we acquire the inode lock and
	 * first do an error checking pass.
	 */
	tp = xfs_trans_alloc(mp, XFS_TRANS_SETATTR_NOT_SIZE);
	code = xfs_trans_reserve(tp, 0, XFS_ICHANGE_LOG_RES(mp), 0, 0, 0);
	if (code)
		goto error_return;

	lock_flags = XFS_ILOCK_EXCL;
	xfs_ilock(ip, lock_flags);

	/*
	 * CAP_FOWNER overrides the following restrictions:
	 *
	 * The user ID of the calling process must be equal
	 * to the file owner ID, except in cases where the
	 * CAP_FSETID capability is applicable.
	 */
	if (current->fsuid != ip->i_d.di_uid && !capable(CAP_FOWNER)) {
		code = XFS_ERROR(EPERM);
		goto error_return;
	}

	/*
	 * Do a quota reservation only if projid is actually going to change.
	 */
	if (mask & FSX_PROJID) {
		if (XFS_IS_PQUOTA_ON(mp) &&
		    ip->i_d.di_projid != fa->fsx_projid) {
			ASSERT(tp);
			code = XFS_QM_DQVOPCHOWNRESV(mp, tp, ip, udqp, gdqp,
						capable(CAP_FOWNER) ?
						XFS_QMOPT_FORCE_RES : 0);
			if (code)	/* out of quota */
				goto error_return;
		}
	}

	if (mask & FSX_EXTSIZE) {
		/*
		 * Can't change extent size if any extents are allocated.
		 */
		if (ip->i_d.di_nextents &&
		    ((ip->i_d.di_extsize << mp->m_sb.sb_blocklog) !=
		     fa->fsx_extsize)) {
			code = XFS_ERROR(EINVAL);	/* EFBIG? */
			goto error_return;
		}

		/*
		 * Extent size must be a multiple of the appropriate block
		 * size, if set at all.
		 */
		if (fa->fsx_extsize != 0) {
			xfs_extlen_t	size;

			if (XFS_IS_REALTIME_INODE(ip) ||
			    ((mask & FSX_XFLAGS) &&
			    (fa->fsx_xflags & XFS_XFLAG_REALTIME))) {
				size = mp->m_sb.sb_rextsize <<
				       mp->m_sb.sb_blocklog;
			} else {
				size = mp->m_sb.sb_blocksize;
			}

			if (fa->fsx_extsize % size) {
				code = XFS_ERROR(EINVAL);
				goto error_return;
			}
		}
	}


	if (mask & FSX_XFLAGS) {
		/*
		 * Can't change realtime flag if any extents are allocated.
		 */
		if ((ip->i_d.di_nextents || ip->i_delayed_blks) &&
		    (XFS_IS_REALTIME_INODE(ip)) !=
		    (fa->fsx_xflags & XFS_XFLAG_REALTIME)) {
			code = XFS_ERROR(EINVAL);	/* EFBIG? */
			goto error_return;
		}

		/*
		 * If realtime flag is set then must have realtime data.
		 */
		if ((fa->fsx_xflags & XFS_XFLAG_REALTIME)) {
			if ((mp->m_sb.sb_rblocks == 0) ||
			    (mp->m_sb.sb_rextsize == 0) ||
			    (ip->i_d.di_extsize % mp->m_sb.sb_rextsize)) {
				code = XFS_ERROR(EINVAL);
				goto error_return;
			}
		}

		/*
		 * Can't modify an immutable/append-only file unless
		 * we have appropriate permission.
		 */
		if ((ip->i_d.di_flags &
				(XFS_DIFLAG_IMMUTABLE|XFS_DIFLAG_APPEND) ||
		     (fa->fsx_xflags &
				(XFS_XFLAG_IMMUTABLE | XFS_XFLAG_APPEND))) &&
		    !capable(CAP_LINUX_IMMUTABLE)) {
			code = XFS_ERROR(EPERM);
			goto error_return;
		}
	}

	xfs_trans_ijoin(tp, ip, lock_flags);
	xfs_trans_ihold(tp, ip);

	/*
	 * Change file ownership.  Must be the owner or privileged.
	 * If the system was configured with the "restricted_chown"
	 * option, the owner is not permitted to give away the file,
	 * and can change the group id only to a group of which he
	 * or she is a member.
	 */
	if (mask & FSX_PROJID) {
		/*
		 * CAP_FSETID overrides the following restrictions:
		 *
		 * The set-user-ID and set-group-ID bits of a file will be
		 * cleared upon successful return from chown()
		 */
		if ((ip->i_d.di_mode & (S_ISUID|S_ISGID)) &&
		    !capable(CAP_FSETID))
			ip->i_d.di_mode &= ~(S_ISUID|S_ISGID);

		/*
		 * Change the ownerships and register quota modifications
		 * in the transaction.
		 */
		if (ip->i_d.di_projid != fa->fsx_projid) {
			if (XFS_IS_PQUOTA_ON(mp)) {
				olddquot = XFS_QM_DQVOPCHOWN(mp, tp, ip,
							&ip->i_gdquot, gdqp);
			}
			ip->i_d.di_projid = fa->fsx_projid;

			/*
			 * We may have to rev the inode as well as
			 * the superblock version number since projids didn't
			 * exist before DINODE_VERSION_2 and SB_VERSION_NLINK.
			 */
			if (ip->i_d.di_version == XFS_DINODE_VERSION_1)
				xfs_bump_ino_vers2(tp, ip);
		}

	}

	if (mask & FSX_EXTSIZE)
		ip->i_d.di_extsize = fa->fsx_extsize >> mp->m_sb.sb_blocklog;
	if (mask & FSX_XFLAGS) {
		xfs_set_diflags(ip, fa->fsx_xflags);
		xfs_diflags_to_linux(ip);
	}

	xfs_trans_log_inode(tp, ip, XFS_ILOG_CORE);
	xfs_ichgtime(ip, XFS_ICHGTIME_CHG);

	XFS_STATS_INC(xs_ig_attrchg);

	/*
	 * If this is a synchronous mount, make sure that the
	 * transaction goes to disk before returning to the user.
	 * This is slightly sub-optimal in that truncates require
	 * two sync transactions instead of one for wsync filesystems.
	 * One for the truncate and one for the timestamps since we
	 * don't want to change the timestamps unless we're sure the
	 * truncate worked.  Truncates are less than 1% of the laddis
	 * mix so this probably isn't worth the trouble to optimize.
	 */
	if (mp->m_flags & XFS_MOUNT_WSYNC)
		xfs_trans_set_sync(tp);
	code = xfs_trans_commit(tp, 0);
	xfs_iunlock(ip, lock_flags);

	/*
	 * Release any dquot(s) the inode had kept before chown.
	 */
	XFS_QM_DQRELE(mp, olddquot);
	XFS_QM_DQRELE(mp, udqp);
	XFS_QM_DQRELE(mp, gdqp);

	if (code)
		return code;

	if (DM_EVENT_ENABLED(ip, DM_EVENT_ATTRIBUTE)) {
		XFS_SEND_NAMESP(mp, DM_EVENT_ATTRIBUTE, ip, DM_RIGHT_NULL,
				NULL, DM_RIGHT_NULL, NULL, NULL, 0, 0,
				(mask & FSX_NONBLOCK) ? DM_FLAGS_NDELAY : 0);
	}

	return 0;

 error_return:
	XFS_QM_DQRELE(mp, udqp);
	XFS_QM_DQRELE(mp, gdqp);
	xfs_trans_cancel(tp, 0);
	if (lock_flags)
		xfs_iunlock(ip, lock_flags);
	return code;
}

STATIC int
xfs_ioc_fssetxattr(
	xfs_inode_t		*ip,
	struct file		*filp,
	void			__user *arg)
{
	struct fsxattr		fa;
	unsigned int		mask;

	if (copy_from_user(&fa, arg, sizeof(fa)))
		return -EFAULT;

	mask = FSX_XFLAGS | FSX_EXTSIZE | FSX_PROJID;
	if (filp->f_flags & (O_NDELAY|O_NONBLOCK))
		mask |= FSX_NONBLOCK;

	return -xfs_ioctl_setattr(ip, &fa, mask);
}

STATIC int
xfs_ioc_getxflags(
	xfs_inode_t		*ip,
	void			__user *arg)
{
	unsigned int		flags;

	flags = xfs_di2lxflags(ip->i_d.di_flags);
	if (copy_to_user(arg, &flags, sizeof(flags)))
		return -EFAULT;
	return 0;
}

STATIC int
xfs_ioc_setxflags(
	xfs_inode_t		*ip,
	struct file		*filp,
	void			__user *arg)
{
	struct fsxattr		fa;
	unsigned int		flags;
	unsigned int		mask;

	if (copy_from_user(&flags, arg, sizeof(flags)))
		return -EFAULT;

	if (flags & ~(FS_IMMUTABLE_FL | FS_APPEND_FL | \
		      FS_NOATIME_FL | FS_NODUMP_FL | \
		      FS_SYNC_FL))
		return -EOPNOTSUPP;

	mask = FSX_XFLAGS;
	if (filp->f_flags & (O_NDELAY|O_NONBLOCK))
		mask |= FSX_NONBLOCK;
	fa.fsx_xflags = xfs_merge_ioc_xflags(flags, xfs_ip2xflags(ip));

	return -xfs_ioctl_setattr(ip, &fa, mask);
}

STATIC int
xfs_ioc_getbmap(
	struct xfs_inode	*ip,
	int			ioflags,
	unsigned int		cmd,
	void			__user *arg)
{
	struct getbmap		bm;
	int			iflags;
	int			error;

	if (copy_from_user(&bm, arg, sizeof(bm)))
		return -XFS_ERROR(EFAULT);

	if (bm.bmv_count < 2)
		return -XFS_ERROR(EINVAL);

	iflags = (cmd == XFS_IOC_GETBMAPA ? BMV_IF_ATTRFORK : 0);
	if (ioflags & IO_INVIS)
		iflags |= BMV_IF_NO_DMAPI_READ;

	error = xfs_getbmap(ip, &bm, (struct getbmap __user *)arg+1, iflags);
	if (error)
		return -error;

	if (copy_to_user(arg, &bm, sizeof(bm)))
		return -XFS_ERROR(EFAULT);
	return 0;
}

STATIC int
xfs_ioc_getbmapx(
	struct xfs_inode	*ip,
	void			__user *arg)
{
	struct getbmapx		bmx;
	struct getbmap		bm;
	int			iflags;
	int			error;

	if (copy_from_user(&bmx, arg, sizeof(bmx)))
		return -XFS_ERROR(EFAULT);

	if (bmx.bmv_count < 2)
		return -XFS_ERROR(EINVAL);

	/*
	 * Map input getbmapx structure to a getbmap
	 * structure for xfs_getbmap.
	 */
	GETBMAP_CONVERT(bmx, bm);

	iflags = bmx.bmv_iflags;

	if (iflags & (~BMV_IF_VALID))
		return -XFS_ERROR(EINVAL);

	iflags |= BMV_IF_EXTENDED;

	error = xfs_getbmap(ip, &bm, (struct getbmapx __user *)arg+1, iflags);
	if (error)
		return -error;

	GETBMAP_CONVERT(bm, bmx);

	if (copy_to_user(arg, &bmx, sizeof(bmx)))
		return -XFS_ERROR(EFAULT);

	return 0;
}

int
xfs_ioctl(
	xfs_inode_t		*ip,
	struct file		*filp,
	int			ioflags,
	unsigned int		cmd,
	void			__user *arg)
{
	struct inode		*inode = filp->f_path.dentry->d_inode;
	xfs_mount_t		*mp = ip->i_mount;
	int			error;

	xfs_itrace_entry(XFS_I(inode));
	switch (cmd) {

	case XFS_IOC_ALLOCSP:
	case XFS_IOC_FREESP:
	case XFS_IOC_RESVSP:
	case XFS_IOC_UNRESVSP:
	case XFS_IOC_ALLOCSP64:
	case XFS_IOC_FREESP64:
	case XFS_IOC_RESVSP64:
	case XFS_IOC_UNRESVSP64:
		/*
		 * Only allow the sys admin to reserve space unless
		 * unwritten extents are enabled.
		 */
		if (!xfs_sb_version_hasextflgbit(&mp->m_sb) &&
		    !capable(CAP_SYS_ADMIN))
			return -EPERM;

		return xfs_ioc_space(ip, inode, filp, ioflags, cmd, arg);

	case XFS_IOC_DIOINFO: {
		struct dioattr	da;
		xfs_buftarg_t	*target =
			XFS_IS_REALTIME_INODE(ip) ?
			mp->m_rtdev_targp : mp->m_ddev_targp;

		da.d_mem = da.d_miniosz = 1 << target->bt_sshift;
		da.d_maxiosz = INT_MAX & ~(da.d_miniosz - 1);

		if (copy_to_user(arg, &da, sizeof(da)))
			return -XFS_ERROR(EFAULT);
		return 0;
	}

	case XFS_IOC_FSBULKSTAT_SINGLE:
	case XFS_IOC_FSBULKSTAT:
	case XFS_IOC_FSINUMBERS:
		return xfs_ioc_bulkstat(mp, cmd, arg);

	case XFS_IOC_FSGEOMETRY_V1:
		return xfs_ioc_fsgeometry_v1(mp, arg);

	case XFS_IOC_FSGEOMETRY:
		return xfs_ioc_fsgeometry(mp, arg);

	case XFS_IOC_GETVERSION:
		return put_user(inode->i_generation, (int __user *)arg);

	case XFS_IOC_FSGETXATTR:
		return xfs_ioc_fsgetxattr(ip, 0, arg);
	case XFS_IOC_FSGETXATTRA:
		return xfs_ioc_fsgetxattr(ip, 1, arg);
	case XFS_IOC_FSSETXATTR:
		return xfs_ioc_fssetxattr(ip, filp, arg);
	case XFS_IOC_GETXFLAGS:
		return xfs_ioc_getxflags(ip, arg);
	case XFS_IOC_SETXFLAGS:
		return xfs_ioc_setxflags(ip, filp, arg);

	case XFS_IOC_FSSETDM: {
		struct fsdmidata	dmi;

		if (copy_from_user(&dmi, arg, sizeof(dmi)))
			return -XFS_ERROR(EFAULT);

		error = xfs_set_dmattrs(ip, dmi.fsd_dmevmask,
				dmi.fsd_dmstate);
		return -error;
	}

	case XFS_IOC_GETBMAP:
	case XFS_IOC_GETBMAPA:
		return xfs_ioc_getbmap(ip, ioflags, cmd, arg);

	case XFS_IOC_GETBMAPX:
		return xfs_ioc_getbmapx(ip, arg);

	case XFS_IOC_FD_TO_HANDLE:
	case XFS_IOC_PATH_TO_HANDLE:
	case XFS_IOC_PATH_TO_FSHANDLE:
		return xfs_find_handle(cmd, arg);

	case XFS_IOC_OPEN_BY_HANDLE:
		return xfs_open_by_handle(mp, arg, filp, inode);

	case XFS_IOC_FSSETDM_BY_HANDLE:
		return xfs_fssetdm_by_handle(mp, arg, inode);

	case XFS_IOC_READLINK_BY_HANDLE:
		return xfs_readlink_by_handle(mp, arg, inode);

	case XFS_IOC_ATTRLIST_BY_HANDLE:
		return xfs_attrlist_by_handle(mp, arg, inode);

	case XFS_IOC_ATTRMULTI_BY_HANDLE:
		return xfs_attrmulti_by_handle(mp, arg, filp, inode);

	case XFS_IOC_SWAPEXT: {
		error = xfs_swapext((struct xfs_swapext __user *)arg);
		return -error;
	}

	case XFS_IOC_FSCOUNTS: {
		xfs_fsop_counts_t out;

		error = xfs_fs_counts(mp, &out);
		if (error)
			return -error;

		if (copy_to_user(arg, &out, sizeof(out)))
			return -XFS_ERROR(EFAULT);
		return 0;
	}

	case XFS_IOC_SET_RESBLKS: {
		xfs_fsop_resblks_t inout;
		__uint64_t	   in;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		if (copy_from_user(&inout, arg, sizeof(inout)))
			return -XFS_ERROR(EFAULT);

		/* input parameter is passed in resblks field of structure */
		in = inout.resblks;
		error = xfs_reserve_blocks(mp, &in, &inout);
		if (error)
			return -error;

		if (copy_to_user(arg, &inout, sizeof(inout)))
			return -XFS_ERROR(EFAULT);
		return 0;
	}

	case XFS_IOC_GET_RESBLKS: {
		xfs_fsop_resblks_t out;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		error = xfs_reserve_blocks(mp, NULL, &out);
		if (error)
			return -error;

		if (copy_to_user(arg, &out, sizeof(out)))
			return -XFS_ERROR(EFAULT);

		return 0;
	}

	case XFS_IOC_FSGROWFSDATA: {
		xfs_growfs_data_t in;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		if (copy_from_user(&in, arg, sizeof(in)))
			return -XFS_ERROR(EFAULT);

		error = xfs_growfs_data(mp, &in);
		return -error;
	}

	case XFS_IOC_FSGROWFSLOG: {
		xfs_growfs_log_t in;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		if (copy_from_user(&in, arg, sizeof(in)))
			return -XFS_ERROR(EFAULT);

		error = xfs_growfs_log(mp, &in);
		return -error;
	}

	case XFS_IOC_FSGROWFSRT: {
		xfs_growfs_rt_t in;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		if (copy_from_user(&in, arg, sizeof(in)))
			return -XFS_ERROR(EFAULT);

		error = xfs_growfs_rt(mp, &in);
		return -error;
	}

	case XFS_IOC_FREEZE:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		if (inode->i_sb->s_frozen == SB_UNFROZEN)
			freeze_bdev(inode->i_sb->s_bdev);
		return 0;

	case XFS_IOC_THAW:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		if (inode->i_sb->s_frozen != SB_UNFROZEN)
			thaw_bdev(inode->i_sb->s_bdev, inode->i_sb);
		return 0;

	case XFS_IOC_GOINGDOWN: {
		__uint32_t in;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		if (get_user(in, (__uint32_t __user *)arg))
			return -XFS_ERROR(EFAULT);

		error = xfs_fs_goingdown(mp, in);
		return -error;
	}

	case XFS_IOC_ERROR_INJECTION: {
		xfs_error_injection_t in;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		if (copy_from_user(&in, arg, sizeof(in)))
			return -XFS_ERROR(EFAULT);

		error = xfs_errortag_add(in.errtag, mp);
		return -error;
	}

	case XFS_IOC_ERROR_CLEARALL:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		error = xfs_errortag_clearall(mp, 1);
		return -error;

	default:
		return -ENOTTY;
	}
}
