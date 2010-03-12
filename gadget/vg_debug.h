/*
 * Copyright (C) 2010 Paul Wolneykien.
 *
 * This file is part of vgadget -- the versatile gadget driver.
 *
 * Vgadget is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of
 * the License, or (at your option) any later version.
 *
 * Vgadget is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with vgadget driver.
 * If not, see <http://www.gnu.org/licenses/>.
 */


/* Debug stuff */

#define xprintk(f,level,fmt,args...) \
	dev_printk(level , &(f)->gadget->dev , fmt , ## args)

#ifdef DEBUG
#define DBG(dev,fmt,args...) \
	xprintk(dev , KERN_DEBUG , fmt , ## args)
#define MDBG(fmt,args...) \
	printk(KERN_DEBUG DRIVER_NAME ": " fmt , ## args)
static void dump_msg(struct vg_dev *vg, const char *label,
		     const u8 *buf, unsigned int length);
#else
#define DBG(dev,fmt,args...) \
	do { } while (0)
#define MDBG(fmt,args...) \
	do { } while (0)
static void inline dump_msg(struct vg_dev *vg, const char *label,
			    const u8 *buf, unsigned int length);
#endif /* DEBUG */

#ifdef VERBOSE
#define VDBG	DBG
#else
#define VDBG(dev,fmt,args...) \
	do { } while (0)
#define VLDBG(lun,fmt,args...) \
	do { } while (0)
#endif /* VERBOSE */

#define ERROR(dev,fmt,args...) \
	xprintk(dev , KERN_ERR , fmt , ## args)

#define MERROR(fmt,args...) \
        printk(KERN_ERR DRIVER_NAME ": " fmt , ## args)

#define WARN(dev,fmt,args...) \
	xprintk(dev , KERN_WARNING , fmt , ## args)

#define INFO(dev,fmt,args...) \
	xprintk(dev , KERN_INFO , fmt , ## args)

#define MINFO(fmt,args...) \
	printk(KERN_INFO DRIVER_NAME ": " fmt , ## args)
