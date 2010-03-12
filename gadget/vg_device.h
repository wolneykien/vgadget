/*
 * A gadget device object
 */


/* A thread control structure */
struct vg_thread_ctl {
	wait_queue_head_t	thread_wqh;
	int			thread_wakeup_needed;
	struct completion	thread_notifier;
	int			thread_pid;
	struct task_struct	*thread_task;
	sigset_t		thread_signal_mask;
};

/* The set of gadget states */
enum vg_state {
	VG_STATE_TERMINATED = -7,
	VG_STATE_EXIT,
	VG_STATE_DISCONNECT,
	VG_STATE_CONFIG_CHANGE,
	VG_STATE_INTERFACE_CHANGE,
	VG_STATE_RESET,
        VG_STATE_ABORT_BULK_OUT,
	VG_STATE_IDLE = 0,
	VG_STATE_RUNNING,
        VG_STATE_COMMAND_PHASE,
	VG_STATE_DATA_PHASE,
	VG_STATE_STATUS_PHASE,
};

/* A gadget device structure */
struct vg_dev {
        /* Locks and semaphores */
	spinlock_t		lock;
	struct rw_semaphore	filesem;

        /* Low-level device link */
	struct usb_gadget	*gadget;

        /* State and control */
        u8			config;
        u8			new_config;
	u32			tag;
	volatile unsigned int	req_tag;
	unsigned int		exception_req_tag;
	enum vg_state		state;
	unsigned long		flags;

        /* Endpoints */
	struct usb_ep		*ep0;
        struct usb_request	*ep0_req;
	struct usb_ep		*bulk_out;
	struct usb_ep		*bulk_in;
        struct usb_ep		*bulk_status_in;

        /* Buffer queues */
        struct vg_buffer_queue  out_bufq;
        struct vg_buffer_queue  in_bufq;
        struct vg_buffer_queue  status_in_bufq;

        /* Thread control */
        struct vg_thread_ctl    thread_ctl;
};

/* Constatns for state flags */
#define REGISTERED		0
#define CLEAR_BULK_HALTS	1
#define SUSPENDED		2

/* The type of gadget device function */
typedef void (*vg_dev_proc_t)(struct vg_dev *);


/* Lifecycle prototypes */
int vg_alloc(struct vg_dev **vg);
void vg_free(struct vg_dev *vg);

/* Exception handling prototypes */
void raise_exception(struct vg_dev *vg, enum vg_state new_state);
int inline exception_in_progress(struct vg_dev *vg);
