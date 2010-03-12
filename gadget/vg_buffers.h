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
int vg_allocate_requests(struct vg_buffer_queue *bufq, struct usb_ep *ep);
int vg_allocate_buffers(struct vg_buffer_queue *bufq, struct usb_ep *ep);
int vg_free_buffers(struct vg_buffer_queue *bufq, struct usb_ep *ep);
void vg_free_requests(struct vg_buffer_queue *bufq, struct usb_ep *ep);
