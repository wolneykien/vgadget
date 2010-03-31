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


/* A thread control structure */
struct vg_thread_ctl {
	struct completion	completion;
	int			pid;
        unsigned long		state;
        struct semaphore        mutex;
        struct semaphore        limit;
        wait_queue_head_t       wait;
};

/* A request queue entry */
struct vg_req_entry {
  struct usb_request *req;
  struct vg_req_entry *next;
}

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
	volatile unsigned int	req_tag;
        unsigned long		flags;

        /* Endpoints */
	struct usb_ep		*ep0;
	struct usb_ep		*bulk_out;
	struct usb_ep		*bulk_in;
        struct usb_ep		*bulk_status_in;

        /* Read/send-ahead processes */
        struct vg_thread_ctl    cmd_read;
        struct vg_req_entry     *cmd_queue;
        struct semaphore        cmd_queue_sem;
        struct vg_thread_ctl    file_send;
};

/* Constatns for state flags */
#define REGISTERED		0
#define CLEAR_BULK_HALTS	1
#define SUSPENDED		2

/* The type of gadget device function */
typedef void (*vg_dev_proc_t)(struct vg_dev *);


/* Lifecycle prototypes */
static int vg_alloc(struct vg_dev **vg);
static void vg_free(struct vg_dev *vg);

/* Exception handling prototypes */
static void raise_exception(struct vg_dev *vg, enum vg_state new_state);
static int inline exception_in_progress(struct vg_dev *vg);
