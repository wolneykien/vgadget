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



/*
 * A gadget device object
 */

/* DMA pool parameters */

#define DMA_POOL_NAME "vgadget"
#define DMA_POOL_SIZE 8
#define DMA_POOL_BUF_SIZE PAGE_SIZE

/* A gadget device structure */
struct vg_dev {
        /* Low-level device link */
	struct usb_gadget	*gadget;
        struct dma_pool         *dma_pool;

        /* State and control */
        u8			config;
        int                     intf_index;
        u8                      intf_config[2];
	volatile unsigned int	req_tag;
//        struct semaphore        mutex;
        unsigned long		flags;
        int			pid;
        struct completion       main_event;
        struct completion       main_exit;

        /* Endpoints */
	struct usb_ep		*ep0;
	struct usb_ep		*bulk_out;
	struct usb_ep		*bulk_in;
        struct usb_ep		*bulk_status_in;

        /* Character devices */
        struct cdev             cons_dev;
        struct cdev             fifo_dev;
};

/* Constatns for state flags */
#define REGISTERED              0x00
#define RUNNING                 0x01
#define RECONFIGURATION         0x02
#define INTF_RECONFIGURATION    0x03
#define SUSPENDED               0x04
#define CONS_REGISTERED         0x05
#define FIFO_REGISTERED         0x06
