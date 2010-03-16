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
        /* Low-level device link */
	struct usb_gadget	*gadget;

        /* State and control */
        u8			config;
        u8			new_config;
	u32			tag;
	volatile unsigned int	req_tag;
	unsigned int		exception_req_tag;
	enum vg_state		state;
       	rwlock_t		state_lock;
        unsigned long           irq_state;
        struct semaphore        exception_sem;
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
static int vg_alloc(struct vg_dev **vg);
static void vg_free(struct vg_dev *vg);

/* Exception handling prototypes */
static void raise_exception(struct vg_dev *vg, enum vg_state new_state);
static int inline exception_in_progress(struct vg_dev *vg);
