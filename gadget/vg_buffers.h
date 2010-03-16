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
 * Buffer queue
 */

/* Number of buffers we will use.
 * 2 is enough for double-buffering.
 */
#define VG_NUM_BUFFERS	2

/* The size of the buffer */
#define VG_BUF_SIZE 16384

/* The set of buffer states */
enum vg_buffer_state {
	BUF_STATE_EMPTY = 0,
	BUF_STATE_FULL,
	BUF_STATE_BUSY
};

/* A buffer head structure */
struct vg_buffhd {
	volatile enum vg_buffer_state	state;
	struct vg_buffhd		*next;
	unsigned int			bulk_out_intended_length;
	struct usb_request		*req;
	volatile int			req_busy;
};

/* A buffer queue structure */
struct vg_buffer_queue {
	struct vg_buffhd	buffhds[VG_NUM_BUFFERS];
	struct vg_buffhd	*next_buffhd_to_fill;
	struct vg_buffhd	*next_buffhd_to_drain;
};

/* Allocation and free prototypes */
static int vg_allocate_requests(struct vg_buffer_queue *bufq, struct usb_ep *ep);
static void vg_free_requests(struct vg_buffer_queue *bufq, struct usb_ep *ep);
