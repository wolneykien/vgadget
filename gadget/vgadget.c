/*
 * vgadget.c -- Versatile gadget driver
 *
 * Copyright (C) 2010 Paul Wolneykien
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/config.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#include <linux/bitops.h>
#include <linux/blkdev.h>
#include <linux/compiler.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pagemap.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/utsname.h>
#include <linux/wait.h>

#include <linux/usb_ch9.h>
#include <linux/usb_gadget.h>

/*
 * Local inclusions
 */
#include "vg_buffers.h"
#include "vg_device.h"
#include "vg_debug.h"


/*
 * Module information definitions
 */

/* String constants */
#define DRIVER_DESC		"Versatile Gadget"
#define DRIVER_NAME		"vgadget"
#define DRIVER_VERSION		"09 March 2010"
static const char longname[] = DRIVER_DESC;
static const char shortname[] = DRIVER_NAME;

/* Claim module description */
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Paul Wolneykien");
MODULE_LICENSE("Dual BSD/GPL");


/* Default Id values */
#define DRIVER_VENDOR_ID        0x0
#define DRIVER_PRODUCT_ID       0x0

/* The structure for module parameters */
static struct {
	unsigned short	vendor;
	unsigned short	product;
	unsigned short	release;
        char            *vendor_name;
        char            *product_name;
	char		*serial;
        char            *filename;
} mod_data = {					// Default values
	.vendor		= DRIVER_VENDOR_ID,
	.product	= DRIVER_PRODUCT_ID,
};



/*
 * Gadget USB device descriptors
 */

/* Numeric descriptor constants */
#define STRING_MANUFACTURER	1
#define STRING_PRODUCT		2
#define STRING_SERIAL		3
#define STRING_CONFIG		4

/* There is only one configuration. */
#define	CONFIG_VALUE		1

/* Endpoint 0 buffer size */
#define EP0_BUFSIZE 1024

/* The gadget USB device descriptor */
static struct usb_device_descriptor
device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,

	/* The next three values can be overridden by module parameters */
	.idVendor =		__constant_cpu_to_le16(DRIVER_VENDOR_ID),
	.idProduct =		__constant_cpu_to_le16(DRIVER_PRODUCT_ID),
	.bcdDevice =		__constant_cpu_to_le16(0xffff),

	.iManufacturer =	STRING_MANUFACTURER,
	.iProduct =		STRING_PRODUCT,
	.iSerialNumber =	STRING_SERIAL,
	.bNumConfigurations =	1,
};

/* The gadget USB configuration descriptor */
static struct usb_config_descriptor
config_desc = {
	.bLength =		sizeof config_desc,
	.bDescriptorType =	USB_DT_CONFIG,

	/* wTotalLength computed by usb_gadget_config_buf() */
	.bNumInterfaces =	1,
	.bConfigurationValue =	CONFIG_VALUE,
	.bmAttributes =		USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower =		1,	// self-powered
};

/* The gadget USB On-The-Go descriptor */
static struct usb_otg_descriptor
otg_desc = {
	.bLength =		sizeof(otg_desc),
	.bDescriptorType =	USB_DT_OTG,

	.bmAttributes =		USB_OTG_SRP,
};

/* Transport type constant */
#define USB_PR_BULK             0x50 // Bulk-only

/* The gadget USB interface descritor */
static struct usb_interface_descriptor
intf_desc = {
	.bLength =		sizeof intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_PR_BULK,
	.bInterfaceProtocol =	USB_PR_BULK,
};

/* String descriptors */
static char manufacturer[32] = "N/A";
static char product[32] = "N/A";
static char serial[12] = "N/A";
static struct usb_string		strings[] = {
	{STRING_MANUFACTURER,	manufacturer},
	{STRING_PRODUCT,	product},
	{STRING_SERIAL,		serial},
	{STRING_CONFIG,		"Self-powered"},
	{}
};
static struct usb_gadget_strings	stringtab = {
	.language	= 0x0409,		// en-us
	.strings	= strings,
};



/*
 * Endpoint descriptors
 */

/* Full-speed output endpoint */
static struct usb_endpoint_descriptor
fs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};

/* Full-speed input endpoint */
static struct usb_endpoint_descriptor
fs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};

/* Full-speed status input endpoint */
static struct usb_endpoint_descriptor
fs_bulk_status_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};


/* The set of full-speed endpoint descriptors */
static const struct usb_descriptor_header *fs_function[] = {
	(struct usb_descriptor_header *) &otg_desc,
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_status_in_desc,
	NULL
};

/* The number of full-speed endpoints */
#define FS_FUNCTION_PRE_EP_ENTRIES	3


#ifdef	CONFIG_USB_GADGET_DUALSPEED

/* Maximum packet size */
#define MAX_PACKET_SIZE 512

/* High-speed USB device qualifier */
static struct usb_device_qualifier
dev_qualifier = {
	.bLength =		sizeof dev_qualifier,
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,

	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,

	.bNumConfigurations =	1,
};

/* The addresses of the following endpoints are set from the corresponding
 * full-speed descriptors during setup. */

/* High-speed output endpoint */
static struct usb_endpoint_descriptor
hs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(MAX_PACKET_SIZE),
	.bInterval =		1,
};

/* High-speed input endpoint */
static struct usb_endpoint_descriptor
hs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(MAX_PACKET_SIZE),
};

/* High-speed status input endpoint */
static struct usb_endpoint_descriptor
hs_bulk_status_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(MAX_PACKET_SIZE),
};

/* The set of high-speed endpoint descriptors */
static const struct usb_descriptor_header *hs_function[] = {
	(struct usb_descriptor_header *) &otg_desc,
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_status_in_desc,
	NULL
};

/* The number of high-speed endpoints */
#define HS_FUNCTION_PRE_EP_ENTRIES	3

#endif	/* !CONFIG_USB_GADGET_DUALSPEED */




/*
 * The versatile gadget driver object.
 */

/* Lifecycle prototypes */
static int __init vg_bind(struct usb_gadget *gadget);
static void vg_unbind(struct usb_gadget *gadget);
static void vg_disconnect(struct usb_gadget *gadget);
static int vg_setup(struct usb_gadget *gadget,
		    const struct usb_ctrlrequest *ctrl);
static void vg_suspend(struct usb_gadget *gadget);
static void vg_resume(struct usb_gadget *gadget);

/* The gandet driver instance structure */
static struct usb_gadget_driver
vg_driver = {
#ifdef CONFIG_USB_GADGET_DUALSPEED
	.speed		= USB_SPEED_HIGH,
#else
	.speed		= USB_SPEED_FULL,
#endif
	.function	= (char *) longname,
	.bind		= vg_bind,
	.unbind		= vg_unbind,
	.disconnect	= vg_disconnect,
	.setup		= vg_setup,
	.suspend	= vg_suspend,
	.resume		= vg_resume,

	.driver		= {
		.name		= (char *) shortname,
		/* TODO: implement general driver lifecycle handlers. */
		// .release = ...
		// .suspend = ...
		// .resume = ...
	},
};




/*
 * Module initialization section
 */

/* Bind module parameters */
module_param_named(vendor, mod_data.vendor, ushort, S_IRUGO);
MODULE_PARM_DESC(vendor, "USB Vendor ID");

module_param_named(product, mod_data.product, ushort, S_IRUGO);
MODULE_PARM_DESC(product, "USB Product ID");

module_param_named(release, mod_data.release, ushort, S_IRUGO);
MODULE_PARM_DESC(release, "USB release number");

module_param_named(vendor_name, mod_data.vendor_name, charp, S_IRUGO);
MODULE_PARM_DESC(note, "Vendor name");

module_param_named(product_name, mod_data.product_name, charp, S_IRUGO);
MODULE_PARM_DESC(product_name, "Product name");

module_param_named(serial, mod_data.serial, charp, S_IRUGO);
MODULE_PARM_DESC(serial, "Gadget serial number");

module_param_named(filename, mod_data.filename, charp, S_IRUGO);
MODULE_PARM_DESC(filename, "File to read from");

/* The gadget device object */
static struct vg_dev *the_vg;

/* Initializes and registeres the module  */
static int __init vg_init(void)
{
	int rc;

	MDBG("Process the module parameters\n");
	snprintf(manufacturer, sizeof manufacturer, "%s",
		 mod_data.vendor_name);
	snprintf(product, sizeof product, "%s",
		 mod_data.product_name);
	snprintf(serial, sizeof product, "%s",
		 mod_data.serial);

	MDBG("Allocate the gadget device\n");
	rc = vg_alloc(&the_vg);
	if (rc == 0) {
	  MDBG("Register the gadget device driver\n");
	  rc = usb_gadget_register_driver(&vg_driver);
	  if (rc == 0) {
	    set_bit(REGISTERED, &the_vg->flags);
	    /* Tell the thread to start working */
	    MDBG("Tell the thread to start working\n");
	    complete(&the_vg->thread_ctl.thread_notifier);
	  } else {
	    vg_free(the_vg);
	    MERROR("Unable to register driver\n");
	  }
	} else {
	  MERROR("Unable to allocate the gadget device object\n");
	}

	return rc;
}
/* Set up the module initialization handler */
module_init(vg_init);


/* Unregisteres the module, frees up the allocated resources */
static void __exit vg_cleanup(void)
{
	struct vg_dev	*vg = the_vg;

	/* Unregister the driver iff the thread hasn't already done so */
	if (test_and_clear_bit(REGISTERED, &vg->flags)) {
	  MDBG("Unregister the gadget device driver\n");
	  usb_gadget_unregister_driver(&vg_driver);
	}

	/* Wait for the thread to finish up */
	MDBG("Wait for the thread to finish up\n");
	wait_for_completion(&vg->thread_ctl.thread_notifier);

	vg_free(vg);
}
/* Set up the module exit handler */
module_exit(vg_cleanup);



/*
 * Implementation section. The dagdet device
 */

/* Allocates memory and scheduler objects used by this module */
static int __init vg_alloc(struct vg_dev **vg)
{
        int rc;

	MDBG("Allocate the gadget device object\n");
	*vg = kmalloc(sizeof *vg, GFP_KERNEL);
	if (vg) {
	  memset(*vg, 0, sizeof *vg);
	  MDBG("Allocate the gadget device locks\n");
	  spin_lock_init(&(*vg)->lock);
	  init_rwsem(&(*vg)->filesem);
	  MDBG("Allocate and init the gadget device thread wait queue\n");
	  init_waitqueue_head(&(*vg)->thread_ctl.thread_wqh);
	  init_completion(&(*vg)->thread_ctl.thread_notifier);
	  rc = 0;
	} else {
	  rc = -ENOMEM;
	}

	return rc;
}

/* Frees memory used by this module */
static void vg_free(struct vg_dev *vg)
{
        MDBG("Free the gadget device object\n");
	kfree(vg);
}

/* Completion procedure prototype */
static void bulk_complete(struct usb_ep *ep, struct usb_request *req);

/* Allocates a buffer queue request objects */
static int vg_allocate_requests(struct vg_buffer_queue *bufq,
				struct usb_ep *ep)
{
  int i;
  int rc = 0;

  MDBG("Allocate %d requests for %s\n", VG_NUM_BUFFERS, ep->name);
  for (i = 0; rc == 0 && i < VG_NUM_BUFFERS; ++i) {
    bufq->buffhds[i].req_busy = 0;
    if ((bufq->buffhds[i].req =
	 usb_ep_alloc_request(ep, GFP_ATOMIC)) != NULL) {
      rc = 0;
    } else {
      MERROR("Can't allocate request for %s\n", ep->name);
      rc = -ENOMEM;
    }

    if (rc == 0) {
      bufq->buffhds[i].req->context = &bufq->buffhds[i];
      bufq->buffhds[i].req->complete = bulk_complete;
    }
  }

  return rc;
}

/* Allocates a buffer queue */
static int vg_allocate_buffers(struct vg_buffer_queue *bufq,
			       struct usb_ep *ep)
{
  int i;
  int rc = 0;

  if ((rc = vg_allocate_requests(bufq, ep)) != 0) {
    MERROR("Unable to allocate request objects\n");
  }
  
  if (rc == 0) {
    MDBG("Allocate %d buffers for %s\n", VG_NUM_BUFFERS, ep->name);
    for (i = 0; rc == 0 && i < VG_NUM_BUFFERS; ++i) {
      bufq->buffhds[i].req->buf =
	usb_ep_alloc_buffer(ep,
			    VG_BUF_SIZE,
			    &bufq->buffhds[i].req->dma,
			    GFP_KERNEL);
      if (bufq->buffhds[i].req->buf) {
	if ((i + 1) < VG_NUM_BUFFERS) {
	  bufq->buffhds[i].next = &bufq->buffhds[i + 1];
	} else {
	  bufq->buffhds[i].next = NULL;
	}
	rc = 0;
      } else {
	rc = -ENOMEM;
	MERROR("Unable to allocate buffer for %s\n", ep->name);
      }
    }
  }

  return rc;
}

/* Frees a buffer queue request objects */
static void vg_free_requests(struct vg_buffer_queue *bufq,
			     struct usb_ep *ep)
{
  int i;

  MDBG("Free %d requests for %s\n", VG_NUM_BUFFERS, ep->name);
  for (i = 0; i < VG_NUM_BUFFERS; ++i) {
    if (bufq->buffhds[i].req) {
      usb_ep_free_request(ep, bufq->buffhds[i].req);
      bufq->buffhds[i].req = NULL;
    }
  }
}

/* Frees a buffer queue */
static void vg_free_buffers(struct vg_buffer_queue *bufq,
			    struct usb_ep *ep)
{
  int i;

  MDBG("Free %d buffers for %s\n", VG_NUM_BUFFERS, ep->name);
  for (i = 0; i < VG_NUM_BUFFERS; ++i) {
    usb_ep_free_buffer(ep,
		       bufq->buffhds[i].req->buf,
		       bufq->buffhds[i].req->dma,
		       VG_BUF_SIZE);
    bufq->buffhds[i].next = NULL;
  }
  vg_free_requests(bufq, ep);
}

/* Cancels all the pending transfers */
static void vg_dequeue_all(struct vg_buffer_queue *bufq,
			   struct usb_ep *ep)
{
  int i;

  for (i = 0; i < VG_NUM_BUFFERS; ++i) {
    if (bufq->buffhds[i].req_busy) {
      MDBG("Dequeue a request number %d for %s\n", i, ep->name);
      usb_ep_dequeue(ep, bufq->buffhds[i].req);
    }
  }
}

/* Indicates are all transfers are idle */
static int vg_no_transfers(struct vg_buffer_queue *bufq)
{
  int i;

  for (i = 0; i < VG_NUM_BUFFERS; ++i) {
    if (bufq->buffhds[i].req_busy) {
      return 0;
    }
  }

  return 1;
}

/* Resets the queue pointers */
static void vg_reset_queue(struct vg_buffer_queue *bufq)
{
  bufq->next_buffhd_to_fill =
    bufq->next_buffhd_to_drain = &bufq->buffhds[0];
}

/* Validates the gadget device parameters */
static int __init check_parameters(struct vg_dev *vg)
{
  return 0;
}

/* Terminates and unbinds the gadget device */
static void __init vg_terminate(struct vg_dev *vg)
{
  vg->state = VG_STATE_TERMINATED;	// The thread is dead
  vg_unbind(vg->gadget);
}

/* Configure the endpoint automatically */
static int __init autoconfig_endpoint(struct vg_dev *vg,
				       struct usb_ep *ep,
				       struct usb_endpoint_descriptor *desc)
{
  int rc;
  ep = usb_ep_autoconfig(vg->gadget, desc);
  if (ep) {
    /* Claim the endpoint */
    ep->driver_data = vg;
    rc = 0;
  } else {
    rc = -ENOTSUPP;
  }

  return rc;
}



/*
 * Implementation section. Thread management
 */
static void wakeup_thread(struct vg_dev *vg)
{
	/* Tell the main thread that something has happened */
	vg->thread_ctl.thread_wakeup_needed = 1;
	wake_up_all(&vg->thread_ctl.thread_wqh);
}

/* Passes the thread to the sleeep state */
static int sleep_thread(struct vg_dev *vg)
{
	int rc;

	/* Wait until a signal arrives or we are woken up */
	for (;;) {
		try_to_freeze();
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			rc = -EINTR;
			break;
		}
		if (vg->thread_ctl.thread_wakeup_needed)
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	vg->thread_ctl.thread_wakeup_needed = 0;

	return rc;
}



/*
 * Implementation section. Exception handling
 */

/* Indicates if an exception is in progress */
static int inline exception_in_progress(struct vg_dev *vg)
{
	return (vg->state < VG_STATE_IDLE);
}

/* Raises the given exception (state) on the given device */
static void raise_exception(struct vg_dev *vg, enum vg_state new_state)
{
	unsigned long flags = 0;

	/* Do nothing if a higher-priority exception is already in progress.
	 * If a lower-or-equal priority exception is in progress, preempt it
	 * and notify the main thread by sending it a signal. */
	spin_lock_irqsave(&vg->lock, flags);
	if (vg->state >= new_state) {
	  VDBG(vg, "Set the exception state %d\n", new_state);
		vg->exception_req_tag = vg->req_tag;
		vg->state = new_state;
		if (vg->thread_ctl.thread_task) {
		  wakeup_thread(vg);
		}
	}
	spin_unlock_irqrestore(&vg->lock, flags);
}

/* Sets the device state */
static void vg_set_state(struct vg_dev *vg, enum vg_state new_state)
{
  spin_lock_irq(&vg->lock);
  DBG(vg, "New device state: %d\n", new_state);
  vg->state = new_state;
  spin_unlock_irq(&vg->lock);
}

/* Indicates if a normal execution is in progress */
static int inline is_running(struct vg_dev *vg)
{
	return (vg->state > VG_STATE_IDLE);
}

/*
 * Implementation section. The dagdet driver
 */

/* Sets the device SUSPENDED flag */
static void vg_suspend(struct usb_gadget *gadget)
{
	struct vg_dev *vg = get_gadget_data(gadget);

	DBG(vg, "suspend\n");
	set_bit(SUSPENDED, &vg->flags);
}

/* Resets the device SUSPENDED flag */
static void vg_resume(struct usb_gadget *gadget)
{
	struct vg_dev *vg = get_gadget_data(gadget);

	DBG(vg, "resume\n");
	clear_bit(SUSPENDED, &vg->flags);
}


/* Main thread procedure prototype */
static int vg_main_thread(void *vg_);

/* Bind procedure implementation */
static int __init vg_bind(struct usb_gadget *gadget)
{
	struct vg_dev *vg = the_vg;
	int rc;

	vg->gadget = gadget;
	set_gadget_data(gadget, vg);
	vg->ep0 = gadget->ep0;
	vg->ep0->driver_data = vg;

	if ((rc = check_parameters(vg)) != 0) {
	  ERROR(vg, "Invalid parameter(s) passed\n");
	}

	if (rc == 0) {
	  /* Find all the endpoints we will use */
	  DBG(vg, "Endpoint autoconfguration\n");
	  usb_ep_autoconfig_reset(gadget);
	  if ((rc = autoconfig_endpoint(vg,
					vg->bulk_out,
					&fs_bulk_out_desc)) != 0) {
	    ERROR(vg, "Bulk-out endpoint autoconfiguration failed\n");
	  }
	  if (rc == 0) {
	    if ((rc = autoconfig_endpoint(vg,
					  vg->bulk_in,
					  &fs_bulk_in_desc)) != 0) {
	      ERROR(vg, "Bulk-in endpoint autoconfiguration failed\n");
	    }
	  }
	  if (rc == 0) {
	    if ((rc = autoconfig_endpoint(vg,
					  vg->bulk_status_in,
					  &fs_bulk_status_in_desc)) != 0) {
	      ERROR(vg, "Bulk-status-out endpoint autoconfiguration failed\n");
	    }
	  }
	}

	if (rc == 0) {
	  /* Fix up the descriptors */
	  DBG(vg, "Fix up device descriptors\n");
	  device_desc.bMaxPacketSize0 = vg->ep0->maxpacket;
	  device_desc.idVendor = cpu_to_le16(mod_data.vendor);
	  device_desc.idProduct = cpu_to_le16(mod_data.product);
	  device_desc.bcdDevice = cpu_to_le16(mod_data.release);
	}

#ifdef CONFIG_USB_GADGET_DUALSPEED

	if (rc == 0) {
	  /* Assume ep0 uses the same maxpacket value for both speeds */
	  DBG(vg, "Set up high-speed ep0 max packet size\n");
	  dev_qualifier.bMaxPacketSize0 = vg->ep0->maxpacket;

	  /* Assume that all endpoint addresses are the same for both speeds */
	  DBG(vg, "Set up high-speed endpoint addresses\n");
	  hs_bulk_out_desc.bEndpointAddress = fs_bulk_out_desc.bEndpointAddress;
	  hs_bulk_in_desc.bEndpointAddress = fs_bulk_in_desc.bEndpointAddress;
	  hs_bulk_status_in_desc.bEndpointAddress = fs_bulk_status_in_desc.bEndpointAddress;
	}
#endif

	if (rc == 0) {
	  /* Set OTG attributes */
	  if (gadget->is_otg) {
	    DBG(vg, "Set up OTG attributes\n");
	    otg_desc.bmAttributes |= USB_OTG_HNP,
	      config_desc.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	  }
	}

	if (rc == 0) {
	  /* Allocate the request and buffer for endpoint 0 */
	  DBG(vg, "Allocate the request and buffer for endpoint 0\n");
	  rc = -ENOMEM;
	  vg->ep0_req = usb_ep_alloc_request(vg->ep0, GFP_KERNEL);
	  if (vg->ep0_req) {
	    vg->ep0_req->buf =
	      usb_ep_alloc_buffer(vg->ep0, EP0_BUFSIZE,
				  &vg->ep0_req->dma, GFP_KERNEL);
	    if (vg->ep0_req->buf) {
	      vg->ep0_req->complete = ep0_complete;
	      rc = 0;
	    } else {
	      ERROR(vg, "Unable to allocate a buffer for endpoint 0\n");
	    }
	  } else {
	    ERROR(vg, "Unable to allocate a request object for ep0\n");
	  }
	}

	/* Allocate the data buffers */
	if (rc == 0) {
	  DBG(vg, "Allocate the data buffers\n");
	  if (rc == 0) {
	    if ((rc = vg_allocate_buffers(&vg->out_bufq,
					  vg->bulk_out)) != 0) {
	      ERROR(vg, "Unable to allocate buffers for the host-output "
		        "queue\n");
	    }
	  }
	  if (rc == 0) {
	    if ((rc = vg_allocate_buffers(&vg->in_bufq,
					  vg->bulk_in)) != 0) {
	      ERROR(vg, "Unable to allocate buffers for the host-input"
		        " queue\n");
	    }
	  }
	  if (rc == 0) {
	    if ((rc = vg_allocate_buffers(&vg->in_status_bufq,
					  vg->bulk_status_in)) != 0) {
	      ERROR(vg, "Unable to allocate buffer for the host-status-input "
		    "queue\n");
	    }
	  }
	}

	if (rc == 0) {
	  /* This should reflect the actual gadget power source */
	  DBG(vg, "Claim gadget as self-powered\n");
	  usb_gadget_set_selfpowered(gadget);
	}

	if (rc == 0) {
	  /* Setup the main thread */
	  DBG(vg, "Set up the main thread\n");
	  rc = kernel_thread(vg_main_thread,
			     vg,
			     (CLONE_VM | CLONE_FS | CLONE_FILES));
	  if (rc < 0) {
	    vg->thread_ctl.thread_pid = rc;
	    rc = 0;
	  }

	  if (rc == 0) {
	    INFO(vg, DRIVER_DESC ", version: " DRIVER_VERSION "\n");
	    INFO(vg, "VendorID=x%04x, ProductID=x%04x, Release=x%04x\n",
		 mod_data.vendor, mod_data.product, mod_data.release);
	    DBG(vg, "I/O thread pid: %d\n", vg->thread_pid);
	  } else {
	    ERROR(vg, "Error while allocating the main thread\n");
	  }
	}

	return rc;
}

/* Unbind procedure implementation */
static void vg_unbind(struct usb_gadget *gadget)
{
	struct vg_dev *vg = get_gadget_data(gadget);

	clear_bit(REGISTERED, &vg->flags);

	/* If the thread isn't already dead, tell it to exit now */
	if (vg->state != VG_STATE_TERMINATED) {
	  DBG(vg, "Tell the main thread to exit and wait for it\n");
		raise_exception(vg, VG_STATE_EXIT);
		wait_for_completion(&vg->thread_ctl.thread_notifier);

		/* The cleanup routine waits for this completion also */
		complete(&vg->thread_ctl.thread_notifier);
	}

	/* Free the data buffers */
	DBG(vg, "Free data buffers\n");
	vg_free_buffers(&vg->out_bufq, vg->bulk_out);
	vg_free_buffers(&vg->in_bufq, vg->bulk_in);
	vg_free_buffers(&vg->in_status_bufq, vg->bulk_status_in);

	/* Free the request and a buffer for endpoint 0 */
	if (vg->ep0_req) {
	  if (vg->ep0_req->buf) {
	    DBG(vg, "Free ep 0 buffer\n");
	    usb_ep_free_buffer(vg->ep0, vg->ep0_req->buf,
			       vg->ep0_req->dma, EP0_BUFSIZE);
	  }
	  DBG(vg, "Free ep 0 request\n");
	  usb_ep_free_request(vg->ep0, vg->eq0_req);
	}

	set_gadget_data(gadget, NULL);
}

/* Class setup request processor */
static int class_setup_req(struct vg_dev *vg,
			   const struct usb_ctrlrequest *ctrl)
{
	int			value = -EOPNOTSUPP;

	if (!vg->config) {
	  ERROR(vg, "No configuration number specified\n");
	  return value;
	}

	/* Handle Bulk-only class-specific requests */
	switch (ctrl->bRequest) {
	case USB_BULK_RESET_REQUEST:
	  if (ctrl->bRequestType != (USB_DIR_OUT |
				     USB_TYPE_CLASS | USB_RECIP_INTERFACE))
	    break;
	  if (le16_to_cpu(ctrl->wIndex) != 0) {
	    value = -EDOM;
	    break;
	  }

	  /* Raise an exception to stop the current operation
	   * and reinitialize our state. */
	  DBG(vg, "Bulk reset request\n");
	  raise_exception(vg, VG_STATE_RESET);
	  value = DELAYED_STATUS;
	  break;
	}

	if (value == -EOPNOTSUPP) {
	  VDBG(vg,
	       "Unknown class-specific control req "
	       "%02x.%02x v%04x i%04x l%u\n",
	       ctrl->bRequestType,
	       ctrl->bRequest,
	       le16_to_cpu(ctrl->wValue),
	       le16_to_cpu(ctrl->wIndex),
	       le16_to_cpu(ctrl->wLength));
	}

	return value;
}

/* Fill out the configuration descriptor */
static int populate_config_buf(struct usb_gadget *gadget,
			       u8 *buf, u8 type, unsigned index)
{
        struct vg_dev *vg = get_gadget_data(gadget);
	int len;
	const struct usb_descriptor_header **function;

	if (index > 0) {
	  ERROR(vg, "Configuration number out of rage (%d)\n", index);
	  return -EINVAL;
	}

#ifdef CONFIG_USB_GADGET_DUALSPEED
	if (type == USB_DT_OTHER_SPEED_CONFIG
	    && speed == USB_SPEED_FULL) {
	  DBG(vg, "Configure with high-speed functions\n");
	  function = hs_function;
	} else {
	  DBG(vg, "Configure with full-speed functions\n");
	  function = fs_function;
	}
#else
	DBG(vg, "Configure with full-speed functions (default)\n");
	function = fs_function;
#endif

	/* for now, don't advertise srp-only devices */
	if (!gadget->is_otg)
		function++;

	len = usb_gadget_config_buf(&config_desc, buf, EP0_BUFSIZE, function);
	((struct usb_config_descriptor *) buf)->bDescriptorType = type;
	return len;
}

/* Standard setup request processor */
static int standard_setup_req(struct vg_dev *vg,
			      const struct usb_ctrlrequest *ctrl)
{
	struct usb_request	*req = vg->ep0_req;
	int			value = -EOPNOTSUPP;
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wLength = le16_to_cpu(ctrl->wLength);

	/* Usually this just stores reply data in the pre-allocated ep0 buffer,
	 * but config change events will also reconfigure hardware. */
	switch (ctrl->bRequest) {

	case USB_REQ_GET_DESCRIPTOR:
	  VDBG(vg, "Get descriptor request\n");
		if (ctrl->bRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE))
			break;
		switch (wValue >> 8) {

		case USB_DT_DEVICE:
			VDBG(vg, "Get device descriptor\n");
			value = min(wLength, (u16) sizeof device_desc);
			memcpy(req->buf, &device_desc, value);
			break;
#ifdef CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			VDBG(vg, "Get device qualifier\n");
			if (!vg->gadget->is_dualspeed)
				break;
			value = min(wLength, (u16) sizeof dev_qualifier);
			memcpy(req->buf, &dev_qualifier, value);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			VDBG(vg, "Get other-speed config descriptor\n");
			if (!vg->gadget->is_dualspeed)
				break;
#endif
		case USB_DT_CONFIG:
			VDBG(vg, "Get configuration descriptor\n");
			value = populate_config_buf(vg->gadget,
					req->buf,
					wValue >> 8,
					wValue & 0xff);
			if (value >= 0)
				value = min(wLength, (u16) value);
			break;

		case USB_DT_STRING:
			VDBG(vg, "Get string descriptor\n");

			/* wIndex == language code */
			value = usb_gadget_get_string(&stringtab,
					wValue & 0xff, req->buf);
			if (value >= 0)
				value = min(wLength, (u16) value);
			break;
		}
		break;

	/* One config, two speeds */
	case USB_REQ_SET_CONFIGURATION:
	  VDBG(vg, "Set configuration request\n");
		if (ctrl->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE))
			break;
		if (wValue == CONFIG_VALUE || wValue == 0) {
			vg->new_config = wValue;

			/* Raise an exception to wipe out previous transaction
			 * state (queued bufs, etc) and set the new config. */
			raise_exception(vg, VG_STATE_CONFIG_CHANGE);
			value = DELAYED_STATUS;
		}
		break;
	case USB_REQ_GET_CONFIGURATION:
	  VDBG(vg, "Get configuration request\n");
		if (ctrl->bRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE))
			break;
		*(u8 *) req->buf = vg->config;
		value = min(wLength, (u16) 1);
		break;

	case USB_REQ_SET_INTERFACE:
	  VDBG(vg, "Set interface request\n");
		if (ctrl->bRequestType != (USB_DIR_OUT| USB_TYPE_STANDARD |
				USB_RECIP_INTERFACE))
			break;
		if (vg->config && wIndex == 0) {

			/* Raise an exception to wipe out previous transaction
			 * state (queued bufs, etc) and install the new
			 * interface altsetting. */
			raise_exception(vg, VG_STATE_INTERFACE_CHANGE);
			value = DELAYED_STATUS;
		}
		break;
	case USB_REQ_GET_INTERFACE:
	  VDBG(vg, "Get interface\n");
		if (ctrl->bRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
				USB_RECIP_INTERFACE))
			break;
		if (!vg->config)
			break;
		if (wIndex != 0) {
			value = -EDOM;
			break;
		}
		*(u8 *) req->buf = 0;
		value = min(wLength, (u16) 1);
		break;

	default:
		VDBG(vg,
		     "Unknown control req %02x.%02x v%04x i%04x l%u\n",
		     ctrl->bRequestType, ctrl->bRequest,
		     wValue, wIndex, wLength);
	}

	return value;
}

/* Enqueues the request of endpoint zero */
static int ep0_queue(struct vg_dev *vg)
{
	int	rc;

	rc = usb_ep_queue(vg->ep0, vg->ep0_req, GFP_ATOMIC);
	if (rc != 0 && rc != -ESHUTDOWN) {
	  /* We can't do much more than wait for a reset */
	  WARN(vg, "Error in submission: %s --> %d\n",
	       vg->ep0->name, rc);
	}
	return rc;
}

/* Handles a finished request of endpoint zero */
static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct vg_dev *vg = (struct vg_dev *) ep->driver_data;

	if (req->actual > 0)
		dump_msg(vg, "ep0-req", req->buf, req->actual);
	if (req->status || req->actual != req->length)
		DBG(vg, "%s --> %d, %u/%u\n", __FUNCTION__,
				req->status, req->actual, req->length);
	if (req->status == -ECONNRESET)		// Request was cancelled
		usb_ep_fifo_flush(ep);

	if (req->status == 0 && req->context)
		((vg_routine_t) (req->context))(vg);
}

/* Setup the gadget parameters from the given control request */
static int vg_setup(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *ctrl)
{
	struct vg_dev *vg = get_gadget_data(gadget);
	int rc;

	++vg->req_tag;		// Record arrival of a new request
	DBG(vg, "Setup request number %d\n", vg->req_tag);
	vg->ep0_req->context = NULL;
	vg->ep0_req->length = 0;
	dump_msg(vg, "ep0-setup", (u8 *) ctrl, sizeof(*ctrl));

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
	  DBG(vg, "Class setup request\n");
	  rc = class_setup_req(vg, ctrl);
	} else {
	  DBG(vg, "Standard setup request\n");
	  rc = standard_setup_req(vg, ctrl);
	}

	/* Respond with data/status or defer until later? */
	if (rc >= 0 && rc != DELAYED_STATUS) {
		vg->ep0_req->length = rc;
		vg->ep0_req->zero = (rc < le16_to_cpu(ctrl->wLength) &&
				     (rc % gadget->ep0->maxpacket) == 0);
		DBG(vg, "Respond with data/status\n");
		rc = ep0_queue(vg);
	}

	/* Device either stalls (rc < 0) or reports success */
	return rc;
}

/* Acknowledges completion of various requests */
static void vg_disconnect(struct usb_gadget *gadget)
{
	struct vg_dev *vg = get_gadget_data(gadget);

	DBG(vg, "Disconnect or port reset\n");
	raise_exception(vg, VG_STATE_DISCONNECT);
}



/*
 * Implementation section. Bulk endpoint procedures
 */

/* Handles a finished request of a bulk-in endpoint */
static void bulk_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct vg_dev		*vg = (struct vg_dev *) ep->driver_data;
	struct vg_buffhd	*bh = (struct vg_buffhd *) req->context;

	if (req->status || req->actual != req->length)
		DBG(vg, "%s --> %d, %u/%u\n", __FUNCTION__,
				req->status, req->actual, req->length);
	if (req->status == -ECONNRESET)		// Request was cancelled
		usb_ep_fifo_flush(ep);

	/* Hold the lock while we update the request and buffer states */
	spin_lock(&vg->lock);
	bh->req_busy = 0;
	bh->state = BUF_STATE_EMPTY; // TODO: STATE_FULL
	spin_unlock(&vg->lock);
	wakeup_thread(vg); // TODO: submit next buffer?
}

/* Makes bulk-out requests be divisible by the maxpacket size */
static void inline set_bulk_out_req_length(struct vg_dev *vg,
		struct vg_buffhd *bh, unsigned int length)
{
	unsigned int	rem;

	bh->bulk_out_intended_length = length;
	rem = length % vg->bulk_out_maxpacket;
	if (rem > 0)
		length += vg->bulk_out_maxpacket - rem;
	bh->outreq->length = length;
}

/* Initiates a bulk transfer */
static void start_transfer(struct vg_dev *vg, struct usb_ep *ep,
			   struct usb_request *req, volatile int *pbusy,
			   volatile enum vg_buffer_state *state)
{
	int rc;

	if (ep == vg->bulk_out)
	  dump_msg(vg, "bulk-out", req->buf, req->length);
	else if (ep == vg->bulk_in)
	  dump_msg(vg, "bulk-in", req->buf, req->length);
	else if (ep == vg->bulk_status_in)
	  dump_msg(vg, "bulk-status-out", req->buf, req->length);
	else
	  return -EOPNOTSUPP;

	*pbusy = 1;
	*state = BUF_STATE_BUSY;
	rc = usb_ep_queue(ep, req, GFP_KERNEL);
	if (rc != 0) {
	  *pbusy = 0;
	  *state = BUF_STATE_EMPTY;
	  if (rc != -ESHUTDOWN && !(rc == -EOPNOTSUPP &&
				    req->length == 0))
	    WARN(vg, "error in submission: %s --> %d\n",
		 ep->name, rc);
	}

	return rc;
}

/* Sets the endpoint halt status */
static int fsg_set_halt(struct vg_dev *vg, struct usb_ep *ep)
{
	const char *name;

	if (ep == vg->bulk_out)
	  name = "bulk-out";
	else if (ep == vg->bulk_in)
	  name = "bulk-in";
	else if (ep == vg->bulk_status_in)
	  name = "bulk-status-in";
	else
	  name = ep->name;

	DBG(vg, "%s set halt\n", name);
	return usb_ep_set_halt(ep);
}

/* Halts the given bulk endpoint */
static int halt_bulk_in_endpoint(struct vg_dev *vg, struct usb_ep *ep)
{
	int rc;

	rc = vg_set_halt(vg, vg->bulk_in);
	if (rc == -EAGAIN)
	  VDBG(vg, "delayed bulk-in endpoint halt\n");
	while (rc != 0) {
	  if (rc != -EAGAIN) {
	    WARN(vg, "usb_ep_set_halt -> %d\n", rc);
	    rc = 0;
	    break;
	  }

	  /* Wait for a short time and then try again */
	  if (msleep_interruptible(100) != 0)
	    return -EINTR;
	  rc = usb_ep_set_halt(ep);
	}
	
	return rc;
}



/*
 * Implementation section. The main thread and company
 */

/* Read next command */
static int get_next_command(struct vg_dev *vg)
{
	int rc = 1;

	return rc;
}

/* Process the command */
static int process_command(struct vg_dev *vg)
{
	int rc = 1;

	return rc;
}

/* Enables the given endpoint */
static int enable_endpoint(struct vg_dev *vg, struct usb_ep *ep,
			   const struct usb_endpoint_descriptor *d)
{
	int	rc;

	ep->driver_data = vg;
	DBG(vg, "Enable endpoint %s\n", ep->name);
	rc = usb_ep_enable(ep, d);
	if (rc) {
	  ERROR(vg, "Can't enable enpoint %s: %d\n", ep->name, rc);
	}
	return rc;
}

/* Resets the interface setting and re-init endpoints */
static int do_set_interface(struct vg_dev *vg, int altsetting)
{
	int rc = 0;

 	VDBG(vg, "Reset the interface\n");

	/* Deallocate the requests */
	DBG(vg, "Free request objects for all queues\n");
	vg_free_requests(vg->out_bufq, vg->bulk_out);
	vg_free_requests(vg->in_bufq, vg->bulk_in);
	vg_free_requests(vg->in_status_bufq, vg->bulk_status_in);

	/* Disable the endpoints */
	DBG(vg, "Disable all endpoints\n");
	usb_ep_disable(vg->bulk_out);
	usb_ep_disable(vg->bulk_in);
	usb_ep_disable(vg->bulk_status_in);
	vg_set_state(vg, VG_STATE_IDLE);

	if (altsetting < 0) {
	  return rc;
	}

	DBG(vg, "Set interface number %d\n", altsetting);

	/* Enable the endpoints */
	if (vg->gadget->speed == USB_SPEED_HIGH) {
	  DBG(vg, "Enable high-speed endpoints\n");
	  if ((rc = enable_endpoint(vg,
				    vg->bulk_out,
				    &hs_bulk_out_desc)) != 0) {
	    ERROR(vg, "Error while enable bulk-out endpoint\n");
	  }
	  if ((rc = enable_endpoint(vg,
				    vg->bulk_in,
				    &hs_bulk_in_desc)) != 0) {
	    ERROR(vg, "Error while enable bulk-in endpoint\n");
	  }
	  if ((rc = enable_endpoint(vg,
				    vg->bulk_status_in,
				    &hs_bulk_status_in_desc)) != 0) {
	    ERROR(vg, "Error while enable bulk-status-in endpoint\n");
	  }
	} else {
	  DBG(vg, "Enable full-speed endpoints\n");
	  if ((rc = enable_endpoint(vg,
				    vg->bulk_out,
				    &fs_bulk_out_desc)) != 0) {
	    ERROR(vg, "Error while enable bulk-out endpoint\n");
	  }
	  if ((rc = enable_endpoint(vg,
				    vg->bulk_in,
				    &fs_bulk_in_desc)) != 0) {
	    ERROR(vg, "Error while enable bulk-in endpoint\n");
	  }
	  if ((rc = enable_endpoint(vg,
				    vg->bulk_status_in,
				    &fs_bulk_status_in_desc)) != 0) {
	    ERROR(vg, "Error while enable bulk-status-in endpoint\n");
	  }
	}

	/* Allocate the requests */
	DBG(vg, "Allocate request objects for all queues\n");
	if (rc == 0) {
	  if ((rc = vg_allocate_requests(vg->out_bufq, vg->bulk_out)) != 0) {
	    ERROR(vg, "Unable to allocate request for the bulk-out queue\n");
	  }
	}
	if (rc == 0) {
	  if ((rc = vg_allocate_requests(vg->in_bufq, vg->bulk_in)) != 0) {
	    ERROR(vg, "Unable to allocate request for the bulk-in queue\n");
	  }
	}
	if (rc == 0) {
	  if ((rc = vg_allocate_requests(vg->status_in_bufq,
					 vg->bulk_status_in)) != 0) {
	    ERROR(vg, "Unable to allocate request for the bulk-status-in "
		      "queue\n");
	  }
	}

	vg_set_state(vg, VG_STATE_RUNNUNG);

	return rc;
}

/* Change the operational configuration */
static int do_set_config(struct vg_dev *vg, u8 new_config)
{
	int	rc = 0;

	VDBG(vg, "Reset the configuration\n");

	/* Disable the single interface */
	DBG(vg, "Disable the interface\n");
	if (vg->config != 0) {
		vg->config = 0;
		rc = do_set_interface(vg, -1);
	}

	/* Enable the interface */
	if (new_config != 0) {
		vg->config = new_config;
		DBG(vg, "Enable the interface\n");
		if ((rc = do_set_interface(vg, 0)) != 0) {
			vg->config = 0;	// Reset on errors
			ERROR(vg, "Error set up the interface\n");
		} else {
			char *speed;

			switch (vg->gadget->speed) {
			case USB_SPEED_LOW:	speed = "low";	break;
			case USB_SPEED_FULL:	speed = "full";	break;
			case USB_SPEED_HIGH:	speed = "high";	break;
			default: 		speed = "?";	break;
			}
			INFO(vg, "Set up the %s speed config number %d\n",
			     speed,
			     vg->config);
		}
	}
	return rc;
}

/* Handles an exception state of the gadget device */
static void handle_exception(struct vg_dev *vg)
{
	enum vg_state		old_state;
	u8			new_config;
	unsigned int		exception_req_tag;
	int			rc;

	if (!exception_in_progress(vg)) {
	  return 0;
	}

	VDBG(vg, "Handle the exception state %d\n", vg->state);

	/* Cancel all the pending transfers */
	DBG(vg, "Cancell all the pending transfers\n");
	vg_dequeue_all(vg->out_bufq, vg->bulk_out);
	vg_dequeue_all(vg->in_bufq, vg->bulk_in);
	vg_dequeue_all(vg->status_in_bufq, vg->status_in_out);

	/* Wait until everything is idle */
	while (!vg_no_transfers(vg->out_bufq)
	       || !vg_no_transfers(vg->in_bufq)
	       || !vg_no_transfers(vg->status_in_bufq)) {
	  DBG(vg, "Wait until everything is idle\n");
	  if ((rc = sleep_thread(vg)) != 0) {
	    WARN(vg, "Interrupted while handle the exception\n");
	    return rc;
	  }
	}

	/* Clear out the controller's fifos */
	DBG(vg, "Clear out the controller's fifos\n");
	usb_ep_fifo_flush(vg->bulk_out);
	usb_ep_fifo_flush(vg->bulk_in);
	usb_ep_fifo_flush(vg->bulk_status_in);

	/* Reset the queue pointers */
	DBG(vg, "Reset the queue pointers\n");
	vg_reset_queue(vg->out_bufq);
	vg_reset_queue(vg->in_bufq);
	vg_reset_queue(vg->status_in_bufq);

	exception_req_tag = vg->exception_req_tag;
	new_config = vg->new_config;
	old_state = vg->state;

	if (old_state == VG_STATE_ABORT_BULK_OUT) {
	  vg_set_state(VG_STATE_STATUS_PHASE);
	} else {
	  vg_set_state(VG_STATE_IDLE);
	}

	/* Carry out any extra actions required for the exception */
	switch (old_state) {
	default:
		break;

	case VG_STATE_ABORT_BULK_OUT:
	  vg_set_state(VG_STATE_IDLE);
	  break;

	case VG_STATE_RESET:
		/* In case we were forced against our will to halt a
		 * bulk endpoint, clear the halt now.  (The SuperH UDC
		 * requires this.) */
		if (test_and_clear_bit(CLEAR_BULK_HALTS,
				       &vg->flags)) {
		  DBG(vg, "Clear bulk halts\n");
		  usb_ep_clear_halt(vg->bulk_out);
		  usb_ep_clear_halt(vg->bulk_in);
		  usb_ep_clear_halt(vg->bulk_status_in);
		}

		if (vg->ep0_req_tag == exception_req_tag) {
		  DBG(vg, "Enqueue to ep0 to c omplete the status stage\n");
		  ep0_queue(vg);	// Complete the status stage
		}
		break;

	case VG_STATE_INTERFACE_CHANGE:
		rc = do_set_interface(vg, 0);
		if (vg->ep0_req_tag != exception_req_tag)
		  break;
		if (rc != 0)			// STALL on errors
		  vg_set_halt(vg, vg->ep0);
		else				// Complete the status stage
		  ep0_queue(vg);
		break;

	case VG_STATE_CONFIG_CHANGE:
	  rc = do_set_config(vg, new_config);
	  if (vg->ep0_req_tag != exception_req_tag)
	    break;
	  if (rc != 0)			// STALL on errors
	    vg_set_halt(vg, vg->ep0);
	  else				// Complete the status stage
	    ep0_queue(vg);
	  break;

	case VG_STATE_DISCONNECT:
	  do_set_config(vg, -1);		// Unconfigured state
	  break;

	case VG_STATE_EXIT:
	case VG_STATE_TERMINATED:
	  do_set_config(vg, -1);	        // Free resources
	  vg_set_state(VG_STATE_TERMINATED);	// Stop the thread
	  break;
	}

	return rc;
}

/* The main thread function */
static int vg_main_thread(void *vg_)
{
	struct vg_dev *vg = (struct vg_dev *) vg_;

	vg->thread_ctl.thread_task = current;

	/* Release all our userspace resources */
	DBG(vg, "Daemonize the main thread\n");
	daemonize("vgadget");

	/* Arrange for userspace references to be interpreted as kernel
	 * pointers.  That way we can pass a kernel pointer to a routine
	 * that expects a __user pointer and it will work okay. */
	DBG(vg, "Set up for kernel-space reference compatibility\n");
	set_fs(get_ds());

	/* Wait for the gadget registration to finish up */
	DBG(vg, "Wait for the gadget registration to finish up\n");
	wait_for_completion(&vg->thread_ctl.thread_notifier);

	/* The main loop */
	while (vg->state != VG_STATE_TERMINATED) {
	  vg_set_state(vg, VG_STATE_INDLE);
	  DBG(vg, "Put the main thread in a sleep\n");
	  sleep_thread(vg);
	  if (exception_in_progress(vg)) {
	    handle_exception(vg);
	    continue;
	  }
	  vg_set_state(vg, VG_STATE_RUNNING);
	  if (get_next_command(vg)) {
	    vg_set_state(vg, VG_STATE_DATA_PHASE);
	    process_command(vg);
	  }
	}

	VDBG(vg, "Exit off the main thread\n");

	vg->thread_ctl.thread_task = NULL;

	/* Let the unbind and cleanup routines know the thread has exited */
	DBG(vg, "Let the unbind and cleanup routines know the thread "
	        "has exited\n");
	complete_and_exit(&vg->thread_ctl.thread_notifier, 0);
}



#ifdef DEBUG

/*
 * Implementation section. Debug stuff
 */

/* Dumps an USB message */
static void dump_msg(struct vg_dev *vg, const char *label,
		     const u8 *buf, unsigned int length)
{
	unsigned int	start, num, i;
	char		line[52], *p;

	if (length >= 512)
	  return;
	DBG(vg, "%s, length %u:\n", label, length);

	start = 0;
	while (length > 0) {
		num = min(length, 16u);
		p = line;
		for (i = 0; i < num; ++i) {
			if (i == 8)
				*p++ = ' ';
			sprintf(p, " %02x", buf[i]);
			p += 3;
		}
		*p = 0;
		printk(KERN_DEBUG "%6x: %s\n", start, line);
		buf += num;
		start += num;
		length -= num;
	}
}

#else

static void inline dump_msg(struct vg_dev *vg, const char *label,
			    const u8 *buf, unsigned int length)
{}

#endif /* DEBUG */
