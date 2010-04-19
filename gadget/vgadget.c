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

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/string.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <asm/dma-mapping.h>
#include <asm/bitops.h>
#include <asm/atomic.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/poll.h>

/*
 * Local inclusions
 */
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
MODULE_LICENSE("GPL");


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
        unsigned int    maxwrites;
} mod_data = {					// Default values
	.vendor		= DRIVER_VENDOR_ID,
	.product	= DRIVER_PRODUCT_ID,
	.maxwrites      = 16,
};

/* Bulk-only class specific requests */
#define USB_BULK_RESET_REQUEST  0xff

/*
 * Gadget USB device descriptors
 */

/* Numeric descriptor constants */
#define STRING_MANUFACTURER	1
#define STRING_PRODUCT		2
#define STRING_SERIAL		3
#define STRING_CONFIG		4

/* There is only one configuration. */
#define	NUMBER_OF_CONFIGS	1

/* Endpoint 0 buffer size */
#define EP0_BUFSIZE 1024

/* Command-status endpoint buffer size */
#define CONS_BUFSIZE PAGE_SIZE

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
	.bNumInterfaces =	2,
	.bConfigurationValue =	1,
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

/* The gadget CS USB interface descritor */
static struct usb_interface_descriptor
cs_intf_desc = {
	.bLength =		sizeof cs_intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,

	/* The interface number 0 */
	.bInterfaceNumber =     0,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_PR_BULK,
	.bInterfaceProtocol =	USB_PR_BULK,
};

/* The gadget CS USB interface descritor */
static struct usb_interface_descriptor
fifo_intf_desc = {
	.bLength =		sizeof fifo_intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,

	/* The interface number 1 */
	.bInterfaceNumber =     1,
	.bNumEndpoints =	1,
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
	(struct usb_descriptor_header *) &cs_intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_bulk_status_in_desc,
	(struct usb_descriptor_header *) &fifo_intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	NULL
};

/* The number of full-speed endpoints */
#define FS_FUNCTION_PRE_EP_ENTRIES	3


#ifdef	CONFIG_USB_GADGET_DUALSPEED

/* Maximum packet size */
#define MAX_PACKET_SIZE 512

/* High-speed USB device qualifier */
static struct usb_qualifier_descriptor
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
	(struct usb_descriptor_header *) &cs_intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_bulk_status_in_desc,
	(struct usb_descriptor_header *) &fifo_intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	NULL
};

/* The number of high-speed endpoints */
#define HS_FUNCTION_PRE_EP_ENTRIES	3

#endif	/* !CONFIG_USB_GADGET_DUALSPEED */




/*
 * The versatile gadget driver object.
 */

/* Lifecycle prototypes */
static int vg_bind(struct usb_gadget *gadget);
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

/* The character device parameters */
#define CONS_FNAME "usbconsS"
#define CONS_MAJOR 63
#define FIFO_FNAME "usbconsF"
#define FIFO_MAJOR 127

/* Define operations for the character devices */
static ssize_t cmd_read (struct file *, char __user *, size_t, loff_t *);
static ssize_t status_write (struct file *, const char __user *, size_t,
			     loff_t *);
static unsigned int cons_poll(struct file *filp,
			      struct poll_table_struct *wait);
static int cons_open (struct inode *, struct file *);
static int cons_release (struct inode *, struct file *);
static struct file_operations cons_fops = {
	.owner		= THIS_MODULE,
	.read           = cmd_read,
	.write          = status_write,
	.poll           = cons_poll,
	.open		= cons_open,
	.release	= cons_release,
};

static ssize_t fifo_write (struct file *, const char __user *, size_t,
			   loff_t *);
static int fifo_open (struct inode *, struct file *);
static int fifo_release (struct inode *, struct file *);
static int fifo_mmap (struct file *, struct vm_area_struct *);
static ssize_t fifo_sendpage (struct file *, struct page *, int, size_t,
			      loff_t *, int);
static struct file_operations fifo_fops = {
	.owner		= THIS_MODULE,
	.write          = fifo_write,
	.open		= fifo_open,
	.release	= fifo_release,
	.splice_write   = generic_splice_sendpage,
	.mmap           = fifo_mmap,
	.sendpage       = fifo_sendpage,
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
MODULE_PARM_DESC(vendor_name, "Vendor name");

module_param_named(product_name, mod_data.product_name, charp, S_IRUGO);
MODULE_PARM_DESC(product_name, "Product name");

module_param_named(serial, mod_data.serial, charp, S_IRUGO);
MODULE_PARM_DESC(serial, "Gadget serial number");

module_param_named(maxwrites, mod_data.maxwrites, uint, S_IRUGO);
MODULE_PARM_DESC(maxwrites, "Maximal number of buffered writes");

/* The gadget device object */
static struct vg_dev *the_vg;

/* Main process prototype */
static int main_process(void *context);

/* Starts the main process */
static int vg_main_process_start(struct vg_dev *vg)
{
  int rc;

  DBG(vg, "Initialize synchronization objects\n");
  init_completion(&vg->main_event);
  init_completion(&vg->main_exit);
  DBG(vg, "Start the main process\n");
  clear_bit(RECONFIGURATION, &vg->flags);
  clear_bit(INTF_RECONFIGURATION, &vg->flags);
  set_bit(RUNNING, &vg->flags);
  rc = kernel_thread(main_process,
		     vg,
		     (CLONE_VM | CLONE_FS | CLONE_FILES));
  if (rc > 0) {
    the_vg->pid = rc;
    rc = 0;
  }

  return rc;
}

/* Terminates the main process */
static int __exit vg_main_process_terminate(struct vg_dev*vg)
{
  int rc;

  rc = 0;
  DBG(vg, "Terminate the main process");
  clear_bit(RUNNING, &vg->flags);
  complete(&vg->main_event);
  wait_for_completion(&vg->main_exit);

  return rc;
}

/* Device object allocator/deallocator prototypes */
static int vg_alloc(struct vg_dev **vg);
static void vg_free(struct vg_dev *vg);

/* Registers the console character device */
static int cons_chardev_add(struct vg_dev *vg)
{
  int rc;

  //  down(&vg->mutex);
  if ((rc = test_and_set_bit(CONS_REGISTERED, &vg->flags)) == 0) {
    MDBG("Register the console device\n");
    //    MKDEV(CONS_MAJOR, 0);

    if ((rc = register_chrdev(CONS_MAJOR,
			      CONS_FNAME,
			      &cons_fops)) != 0) {
      MERROR("Unable to register the console device\n");
      clear_bit(CONS_REGISTERED, &vg->flags);
    }
  }
  //  up(&vg->mutex);

  return rc;
}

/* Registers the FIFO character device */
static int fifo_chardev_add(struct vg_dev *vg)
{
  int rc;

  //  down(&vg->mutex);
  if ((rc = test_and_set_bit(FIFO_REGISTERED, &vg->flags)) == 0) {
    MDBG("Register the FIFO device\n");
    //    MKDEV(FIFO_MAJOR, 0);

    if ((rc = register_chrdev(FIFO_MAJOR,
			      FIFO_FNAME,
			      &fifo_fops)) != 0) {
      MERROR("Unable to register the FIFO device\n");
      clear_bit(FIFO_REGISTERED, &vg->flags);
    }
  }
  //  up(&vg->mutex);

  return rc;
}

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
	init_completion(&the_vg->bind_complete);
	if (rc == 0) {
	  MDBG("Register the gadget device driver\n");
	  rc = usb_gadget_register_driver(&vg_driver);
	  if (rc == 0) {
	    set_bit(REGISTERED, &the_vg->flags);
	  } else {
	    vg_free(the_vg);
	    MERROR("Unable to register driver\n");
	  }
	} else {
	  MERROR("Unable to allocate the gadget device object\n");
	}

	MDBG("Wait for the gadget device to bind to the "
	     "USB subsystem\n");
	wait_for_completion(&the_vg->bind_complete);

	if (rc == 0) {
	  rc = vg_main_process_start(the_vg);
	}

	MDBG("Module initialization finished (%d)\n", rc);
	return rc;
}
/* Set up the module initialization handler */
module_init(vg_init);

/* Unregisters the console character device */
static void cons_chardev_remove(struct vg_dev *vg)
{
  //  down(&vg->mutex);
  if (test_and_clear_bit(CONS_REGISTERED, &vg->flags)) {
    MDBG("Unregister the console device\n");
    unregister_chrdev(CONS_MAJOR, CONS_FNAME);
  }
  //  up(&vg->mutex);
}

/* Unregisters the FIFO character device */
static void fifo_chardev_remove(struct vg_dev *vg)
{
  //  down(&vg->mutex);
  if (test_and_clear_bit(FIFO_REGISTERED, &vg->flags)) {
    MDBG("Unregister the FIFO device\n");
    unregister_chrdev(FIFO_MAJOR, FIFO_FNAME);
  }
  //  up(&vg->mutex);
}

/* Request device reconfiguration (prototype) */
static int vg_request_reconf(struct vg_dev *vg, u8 new_confn);

/* Unregisteres the module, frees up the allocated resources */
static void __exit vg_cleanup(void)
{
	struct vg_dev	*vg = the_vg;

	/* Turn down the interfaces */
	vg_request_reconf(vg, 0);

	/* Terminate the main process */
	vg_main_process_terminate(the_vg);

	/* Unregister the driver iff the thread hasn't already done so */
	if (test_and_clear_bit(REGISTERED, &vg->flags)) {
	  MDBG("Unregister the gadget device driver\n");
	  usb_gadget_unregister_driver(&vg_driver);
	}

	vg_free(vg);
}
/* Set up the module exit handler */
module_exit(vg_cleanup);



/*
 * Implementation section. The dagdet device
 */

/* Allocates memory and scheduler objects used by this module */
static int vg_alloc(struct vg_dev **vg)
{
        int rc;

	MDBG("Allocate the gadget device object\n");
	*vg = kmalloc(sizeof *vg, GFP_KERNEL);
	if (*vg) {
	  memset(*vg, 0, sizeof *vg);
	  (*vg)->req_tag = 0;
	  (*vg)->flags = 0;
	  //	  sema_init(&(*vg)->mutex, 1);
	  MINFO("Maximum number of buffered FIFO writes: %d\n",
	       mod_data.maxwrites);
	  sema_init(&(*vg)->fifo_wrlim, mod_data.maxwrites);
	  sema_init(&(*vg)->status_wrlim, mod_data.maxwrites);
	  sema_init(&(*vg)->cmd_read_mutex, 1);
	  atomic_set(&(*vg)->statuses_written, 0);
	  init_waitqueue_head(&(*vg)->cons_wait);
	  (*vg)->next_cmd_req = NULL;
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

/* Configure the endpoint automatically */
static __init int autoconfig_endpoint(struct vg_dev *vg,
				      struct usb_ep **ep,
				      struct usb_endpoint_descriptor *desc)
{
  int rc;
  *ep = usb_ep_autoconfig(vg->gadget, desc);
  if (*ep) {
    /* Claim the endpoint */
    (*ep)->driver_data = vg;
    DBG(vg, "Endpoint %s autoconfigured: address = 0x%x, maxpacket %d\n",
	(*ep)->name, desc->bEndpointAddress, desc->wMaxPacketSize);
    rc = 0;
  } else {
    rc = -ENOTSUPP;
  }

  return rc;
}

/* Allocates a new USB request and buffer */
static int allocate_request(struct usb_ep *ep,
			    int size,
			    struct usb_request **req)
{
  int rc;
  struct vg_dev *vg;

  vg = (struct vg_dev *) ep->driver_data;

  MDBG("Allocate a request for endpoint %s\n",
       ep->name);
  *req = usb_ep_alloc_request(ep, GFP_KERNEL);
  if (*req == NULL) {
    rc = -ENOMEM;
    MERROR("Unable to allocate a request for endpoint %s\n", ep->name);
  } else {
    rc = 0;
    (*req)->buf = NULL;
    (*req)->dma = 0;
    (*req)->complete = NULL;
    (*req)->length = 0;
    (*req)->zero = 0;
    (*req)->context = NULL;
  }

  if (rc == 0) {
    MDBG("Allocate a buffer for the request of %d b\n", size);
    if (size <= DMA_POOL_BUF_SIZE) {
      (*req)->buf =
	dma_pool_alloc(vg->dma_pool, GFP_KERNEL, &(*req)->dma);
      if ((*req)->buf != NULL) {
	MDBG("Use pool buffer 0x%lx/0x%lx\n",
	     (unsigned long) (*req)->buf,
	     (unsigned long) (*req)->dma);
      }
    } else {
      MWARNING("Allocate a large coherent DMA buffer not from the pool\n");
      (*req)->buf =
	dma_alloc_coherent(&vg->gadget->dev, size, &(*req)->dma, GFP_KERNEL);
    }
    if ((*req)->buf == NULL) {
      rc = -ENOMEM;
      MERROR("Unable to allocate a buffer for the request\n");
    } else {
      rc = 0;
      (*req)->length = size;
    }
  }

  return rc;
}

/* Frees a given URB request and buffer */
static int free_request(struct usb_ep *ep,
			struct usb_request *req)
{
  int rc;
  struct vg_dev *vg;

  vg = (struct vg_dev *) ep->driver_data;

  if (req->buf != NULL) {
    MDBG("Free the buffer of a request for endpoint %s\n", ep->name);
    //TODO: explicitly state the buffer length
    if (req->length <= DMA_POOL_BUF_SIZE) {
      dma_pool_free(vg->dma_pool, req->buf, req->dma);
    } else {
      dma_free_coherent(&vg->gadget->dev, req->length, req->buf, req->dma);
    }

    req->buf = NULL;
    req->dma = 0;
  }
  
  MDBG("Free a request for endpoint %s\n", ep->name);
  usb_ep_free_request(ep, req);

  rc = 0;
  return rc;
}

/* Sets the size of the request data */
static void set_request_length(struct usb_request *req,
			       int len)
{
  if (req->length <= DMA_POOL_BUF_SIZE) {
    req->length = len; //TODO: explicitly state the buffer length
  }
}

/* Enqueues a URB request */
typedef void (*complete_proc_t)(struct usb_ep *ep,		\
				struct usb_request *req);
static int enqueue_request(struct usb_ep *ep,
			   struct usb_request *req,
			   complete_proc_t complete)
{
  int rc;

  struct vg_dev *vg;

  if (ep == NULL) {
    MERROR("Error while enqueue request: enpoint is NULL\n");
    return -EFAULT;
  }

  vg = ep->driver_data;

  if (vg == NULL) {
    MERROR("Error while enqueue request: device is NULL\n");
    return -EFAULT;
  }

  if (req == NULL) {
    MERROR("Error while enqueue request: request is NULL\n");
    return -EFAULT;
  }

  DBG(vg, "Enqueue a request of size %d/%d b for endpoint %s\n",
      req->actual,
      req->length,
      ep->name);
  if (req->actual != 0) {
    WARNING(vg, "Request actual size is not zero!");
  }
  req->complete = complete;
  rc = usb_ep_queue(ep, req, GFP_ATOMIC);
  if (rc != 0 && rc != -ESHUTDOWN) {
    /* Can't do much more than wait for a reset */
    WARNING(vg, "Error in submission: %s --> %d\n",
	 ep->name, rc);
  }

  return rc;
}

/* Handles a finished request of an endpoint */
static void ep_complete_common(struct usb_ep *ep, struct usb_request *req)
{
  struct vg_dev *vg;

  vg = (struct vg_dev *) ep->driver_data;

  DBG(vg, "Request completed for %s\n", ep->name);

  if (req->status) {
    WARNING(vg, "Request completed with error: %d %u/%u\n",
	    req->status, req->actual, req->length);
  } else {
    DBG(vg, "Request completed: %d %u/%u\n",
	req->status, req->actual, req->length);
  }

  /* Request was cancelled */
  if (req->status == -ECONNRESET) {
    WARNING(vg, "The request for %s was cancelled\n", ep->name);
    DBG(vg, "Flush the endpoint FIFO\n");
    usb_ep_fifo_flush(ep);
  }
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


/* Endpoint zero completion procedure prototype */
static void ep0_complete(struct usb_ep *ep, struct usb_request *req);

/* Bind procedure implementation */
static __init int vg_bind(struct usb_gadget *gadget)
{
	struct vg_dev *vg = the_vg;
	int rc;

	rc = 0;
	vg->gadget = gadget;
	set_gadget_data(gadget, vg);
	vg->ep0 = gadget->ep0;
	vg->ep0->driver_data = vg;

	if (rc == 0) {
	  MDBG("Allocate the DMA pool\n");
	  vg->gadget->dev.coherent_dma_mask = 0xffffffff;
	  if ((vg->dma_pool =
	       dma_pool_create(DMA_POOL_NAME,
			       &gadget->dev, 
			       DMA_POOL_BUF_SIZE,
			       PAGE_SIZE,
			       PAGE_SIZE)) == NULL) {
	    MERROR("Unable to allocate the DMA pool\n");
	    rc = -ENOMEM;
	  }
	}

	if (rc == 0) {
	  /* Find all the endpoints we will use */
	  DBG(vg, "Endpoint autoconfguration\n");
	  usb_ep_autoconfig_reset(gadget);
	  if ((rc = autoconfig_endpoint(vg,
					&vg->bulk_out,
					&fs_bulk_out_desc)) != 0) {
	    ERROR(vg, "Bulk-out endpoint autoconfiguration failed\n");
	  }
	  if (rc == 0) {
	    if ((rc = autoconfig_endpoint(vg,
					  &vg->bulk_in,
					  &fs_bulk_in_desc)) != 0) {
	      ERROR(vg, "Bulk-in endpoint autoconfiguration failed\n");
	    }
	  }
	  if (rc == 0) {
	    if ((rc = autoconfig_endpoint(vg,
					  &vg->bulk_status_in,
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
	  /* This should reflect the actual gadget power source */
	  DBG(vg, "Claim gadget as self-powered\n");
	  usb_gadget_set_selfpowered(gadget);
	}

	complete(&vg->bind_complete);

	return rc;
}

/* Unbind procedure implementation */
static void vg_unbind(struct usb_gadget *gadget)
{
	struct vg_dev *vg = get_gadget_data(gadget);

	clear_bit(REGISTERED, &vg->flags);

	set_gadget_data(gadget, NULL);
	DBG(vg, "Free the DMA pool\n");
	dma_pool_destroy(vg->dma_pool);
	vg->dma_pool = NULL;
}

/* Request device reconfiguration */
static int vg_request_reconf(struct vg_dev *vg, u8 new_confn)
{
  int rc;

  if (test_and_set_bit(RECONFIGURATION, &vg->flags) == 0) {
    vg->config = new_confn;
    complete(&vg->main_event);
    rc = 0;
  } else {
    WARNING(vg, "An other reconfiguration is in progress\n");
    rc = -ENOMEM;
  }

  return rc;
}

/* Request device interface reconfiguration */
static int vg_request_intf_reconf(struct vg_dev *vg,
				  int index,
				  u8 new_confn)
{
  int rc;
  
  if (test_and_set_bit(INTF_RECONFIGURATION, &vg->flags) == 0) {
    vg->intf_index = index;
    vg->intf_config[index] = new_confn;
    complete(&vg->main_event);
    rc = 0;
  } else {
    WARNING(vg, "An other interface reconfiguration is in progress\n");
    rc = -ENOMEM;
  }

  return rc;
}

/* Class setup request processor */
static int class_setup_req(struct vg_dev *vg,
			   const struct usb_ctrlrequest *ctrl,
			   struct usb_request **res_req)
{
	int			value = -EOPNOTSUPP;

	/* Initialize resulting request to NULL */
	*res_req = NULL;

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

	  /* Reinitialize the transport processes */
	  DBG(vg, "Bulk reset request\n");
	  //TODO: may not sleep, restart asynchronously
	  value = 0;
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
	if ((type == USB_DT_OTHER_SPEED_CONFIG				\
	     && gadget->speed == USB_SPEED_FULL)			\
	    || (type != USB_DT_OTHER_SPEED_CONFIG			\
		&& gadget->speed == USB_SPEED_HIGH)) {
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
			      const struct usb_ctrlrequest *ctrl,
			      struct usb_request **res_req)
{
	int			value = -EOPNOTSUPP;
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wLength = le16_to_cpu(ctrl->wLength);

	/* Initialize resulting request to NULL */
	*res_req = NULL;

	/* Parse the request */
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
			allocate_request(vg->ep0, EP0_BUFSIZE, res_req);
			memcpy((*res_req)->buf, &device_desc, value);
			set_request_length(*res_req, value);
			break;
#ifdef CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			VDBG(vg, "Get device qualifier\n");
			if (!vg->gadget->is_dualspeed)
				break;
			value = min(wLength, (u16) sizeof dev_qualifier);
			allocate_request(vg->ep0, EP0_BUFSIZE, res_req);
			memcpy((*res_req)->buf, &dev_qualifier, value);
			set_request_length(*res_req, value);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			VDBG(vg, "Get other-speed config descriptor\n");
			if (!vg->gadget->is_dualspeed)
				break;
#endif
		case USB_DT_CONFIG:
			VDBG(vg, "Get configuration descriptor\n");
			allocate_request(vg->ep0, EP0_BUFSIZE, res_req);
			value = populate_config_buf(vg->gadget,
						    (*res_req)->buf,
						    wValue >> 8,
						    wValue & 0xff);
			if (value >= 0)
				value = min(wLength, (u16) value);
			set_request_length(*res_req, value);
			break;

		case USB_DT_STRING:
			VDBG(vg, "Get string descriptor\n");

			/* wIndex == language code */
			allocate_request(vg->ep0, EP0_BUFSIZE, res_req);
			value = usb_gadget_get_string(&stringtab,
						      wValue & 0xff,
						      (*res_req)->buf);
			if (value >= 0)
				value = min(wLength, (u16) value);
			set_request_length(*res_req, value);
			break;
		}
		break;

	/* One config, two speeds */
	case USB_REQ_SET_CONFIGURATION:
	  VDBG(vg, "Set configuration request\n");
		if (ctrl->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE))
			break;
		if (wValue <= NUMBER_OF_CONFIGS) {
		  value = vg_request_reconf(vg, wValue);
		}
		break;
	case USB_REQ_GET_CONFIGURATION:
	  VDBG(vg, "Get configuration request\n");
		if (ctrl->bRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE))
			break;
		value = min(wLength, (u16) 1);
		allocate_request(vg->ep0, EP0_BUFSIZE, res_req);
		*(u8 *) (*res_req)->buf = vg->config;
		set_request_length(*res_req, value);
		break;

	case USB_REQ_SET_INTERFACE:
	  VDBG(vg, "Set interface request\n");
		if (ctrl->bRequestType != (USB_DIR_OUT| USB_TYPE_STANDARD |
				USB_RECIP_INTERFACE))
			break;
		if (vg->config) {
		  value = vg_request_intf_reconf(vg, wIndex, wValue);
		}
		break;
	case USB_REQ_GET_INTERFACE:
	  VDBG(vg, "Get interface\n");
		if (ctrl->bRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
				USB_RECIP_INTERFACE))
			break;
		if (!vg->config)
			break;
		value = min(wLength, (u16) 1);
		allocate_request(vg->ep0, EP0_BUFSIZE, res_req);
		*(u8 *) (*res_req)->buf = vg->intf_config[wIndex];
		set_request_length(*res_req, value);
		break;

	default:
		VDBG(vg,
		     "Unknown control req %02x.%02x v%04x i%04x l%u\n",
		     ctrl->bRequestType, ctrl->bRequest,
		     wValue, wIndex, wLength);
	}

	return value;
}

/* Handles a finished request of an endpoint zero */
static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
  ep_complete_common(ep, req);
  free_request(ep, req);
}

/* Setup the gadget parameters from the given control request */
static int vg_setup(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *ctrl)
{
	struct vg_dev *vg = get_gadget_data(gadget);
	struct usb_request *res_req;
	int rc;

	++vg->req_tag;		// Record arrival of a new request
	DBG(vg, "Proceess the setup request number %d\n", vg->req_tag);

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
	  DBG(vg, "Class setup request\n");
	  rc = class_setup_req(vg, ctrl, &res_req);
	} else {
	  DBG(vg, "Standard setup request\n");
	  rc = standard_setup_req(vg, ctrl, &res_req);
	}

	/* Send a response if one it ready */
	if (res_req != NULL) {
	  DBG(vg, "Respond with data/status\n");
	  if ((rc = enqueue_request(vg->ep0, res_req, ep0_complete)) != 0) {
	    ERROR(vg, "Unable to submit a response to a setup request\n");
	  }
	}

	/* Device either stalls (rc < 0) or reports success */
	return rc;
}

/* Acknowledges completion of various requests */
static void vg_disconnect(struct usb_gadget *gadget)
{
	struct vg_dev *vg = get_gadget_data(gadget);

	DBG(vg, "Disconnect or port reset\n");
	/* Turn down the interfaces */
	vg_request_reconf(vg, 0);
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



/*
 * Implementation section. Configuration procedures
 */

/* Resets the command-status interface */
static int do_set_cmd_interface(struct vg_dev *vg, int altsetting)
{
  int rc;

  rc = 0;

  /* Disable the endpoints */
  DBG(vg, "Disable command-status endpoints\n");
  usb_ep_disable(vg->bulk_out);
  usb_ep_disable(vg->bulk_status_in);

  /* Unregister the command-status character device */
  cons_chardev_remove(vg);

  if (altsetting < 0) {
    return rc;
  }

  DBG(vg, "Setup the command-status interface\n");
  /* Enable the endpoints */
#ifdef CONFIG_USB_GADGET_DUALSPEED
  if (vg->gadget->speed == USB_SPEED_HIGH) {
    DBG(vg, "Enable high-speed endpoints\n");
    if ((rc = enable_endpoint(vg,
			      vg->bulk_out,
			      &hs_bulk_out_desc)) != 0) {
      ERROR(vg, "Error while enable bulk-out endpoint\n");
    }
    if ((rc = enable_endpoint(vg,
			      vg->bulk_status_in,
			      &hs_bulk_status_in_desc)) != 0) {
      ERROR(vg, "Error while enable bulk-status-in endpoint\n");
    }
  } else {
#endif
    DBG(vg, "Enable full-speed endpoints\n");
    if ((rc = enable_endpoint(vg,
			      vg->bulk_out,
			      &fs_bulk_out_desc)) != 0) {
      ERROR(vg, "Error while enable bulk-out endpoint\n");
    }
    if ((rc = enable_endpoint(vg,
			      vg->bulk_status_in,
			      &fs_bulk_status_in_desc)) != 0) {
      ERROR(vg, "Error while enable bulk-status-in endpoint\n");
    }
#ifdef CONFIG_USB_GADGET_DUALSPEED
  }
#endif

  if (rc == 0) {
    /* Register the command-status character device */
    cons_chardev_add(vg);
  }

  return rc;
}

/* Resets the FIFO interface */
static int do_set_fifo_interface(struct vg_dev *vg, int altsetting)
{
  int rc;

  rc = 0;

  /* Disable the endpoints */
  DBG(vg, "Disable FIFO endpoints\n");
  usb_ep_disable(vg->bulk_in);

  /* Unregister the FIFO character device */
  fifo_chardev_remove(vg);

  if (altsetting < 0) {
    return rc;
  }

  DBG(vg, "Setup the FIFO interface\n");
  /* Enable the endpoints */
#ifdef CONFIG_USB_GADGET_DUALSPEED
  if (vg->gadget->speed == USB_SPEED_HIGH) {
    DBG(vg, "Enable high-speed endpoints\n");
    if ((rc = enable_endpoint(vg,
			      vg->bulk_in,
			      &hs_bulk_in_desc)) != 0) {
      ERROR(vg, "Error while enable bulk-in endpoint\n");
    }
  } else {
#endif
    DBG(vg, "Enable full-speed endpoints\n");
    if ((rc = enable_endpoint(vg,
			      vg->bulk_in,
			      &fs_bulk_in_desc)) != 0) {
      ERROR(vg, "Error while enable bulk-in endpoint\n");
    }
#ifdef CONFIG_USB_GADGET_DUALSPEED
  }
#endif

  if (rc == 0) {
    /* Register the FIFO character device */
    fifo_chardev_add(vg);
  }

  return rc;
}

/* Resets the interface setting and re-init endpoints */
static int do_set_interface(struct vg_dev *vg,
			    int index,
			    int altsetting,
			    int reply)
{
  int rc = 0;
  struct usb_request *req;

  switch (index) {
  case 0:
    rc = do_set_cmd_interface(vg, altsetting);
    break;
  case 1:
    rc = do_set_fifo_interface(vg, altsetting);
    break;
  default:
    ERROR(vg, "No such interface: %d\n", index);
  }

  if (altsetting >= 0 && reply) {
    if ((rc = allocate_request(vg->ep0, EP0_BUFSIZE, &req)) == 0) {
      //      *(u8 *) req->buf = vg->intf_config[index];
      set_request_length(req, 0);
      if ((rc = enqueue_request(vg->ep0, req, ep0_complete)) != 0) {
	ERROR(vg, "Unable to submit a response to an interface "
	      "setup request\n");
      }
    }
  }

  return rc;
}

/* Change the operational configuration */
static int do_set_config(struct vg_dev *vg, int new_config)
{
  int	rc = 0;
  struct usb_request *req;

  VDBG(vg, "Reset the configuration\n");

  /* Disable the single interface */
  DBG(vg, "Disable the interfaces\n");
  vg->config = 0;
  rc |= do_set_interface(vg, 0, -1, 0);
  rc |= do_set_interface(vg, 1, -1, 0);

  if (new_config > 0) {
    char *speed;
    DBG(vg, "Setup the configuration number %d\n", new_config);
    vg->config = new_config;
    DBG(vg, "Enable the interfaces\n");
    rc |= do_set_interface(vg, 0, 0, 0);
    rc |= do_set_interface(vg, 1, 0, 0);
    switch (vg->gadget->speed) {
    case USB_SPEED_LOW:	   speed = "low";       break;
    case USB_SPEED_FULL:   speed = "full";      break;
    case USB_SPEED_HIGH:   speed = "high";      break;
    default:               speed = "?";	        break;
    }
    
    INFO(vg, "Set up the %s speed config number %d\n",
	 speed,
	 vg->config);
  } else {
    vg->config = 0;
  }

  if (new_config > 0 && \
      (rc = allocate_request(vg->ep0, EP0_BUFSIZE, &req)) == 0) {
    //*(u8 *) req->buf = vg->config;
    set_request_length(req, 0);
    if ((rc = enqueue_request(vg->ep0, req, ep0_complete)) != 0) {
      ERROR(vg, "Unable to submit a response to a setup request\n");
    }
  }

  return rc;
}




/*
 * Implementation section. Background processes
 */
static int main_process(void *context)
{
  int rc;
  struct vg_dev *vg;

  vg = (struct vg_dev*) context;

  rc = 0;
  wait_for_completion(&vg->main_event);
  while (test_bit(RUNNING, &vg->flags)) {
    DBG(vg, "Process an event\n");
    if (test_bit(RECONFIGURATION, &vg->flags)) {
      /* Reconfigure the device */
      DBG(vg, "Reconfigure the device\n");
      if (do_set_config(vg, vg->config) != 0) {
	ERROR(vg, "Error on device reconfiguration\n");
      }
      clear_bit(RECONFIGURATION, &vg->flags);
    }
    if (test_bit(INTF_RECONFIGURATION, &vg->flags)) {
      /* Reconfigure an interface */
      DBG(vg, "Reconfigure the device interface #%d\n",
	  vg->intf_index);
      if (do_set_interface(vg,
			   vg->intf_index,
			   vg->intf_config[vg->intf_index],
			   1) != 0) {
	ERROR(vg, "Error on device interface reconfiguration\n");
      }
      clear_bit(INTF_RECONFIGURATION, &vg->flags);
    }
    wait_for_completion(&vg->main_event);
  }

  complete_and_exit(&vg->main_exit, rc);
  return rc;
}



/*
 * Implementation section. Device file operationsn
 */

/* Handles the read-command request completion */
static void cmdread_complete(struct usb_ep *ep, struct usb_request *req)
{
  struct vg_dev *vg;

  vg = (struct vg_dev *) ep->driver_data;
  ep_complete_common(ep, req);
  if (req->actual > 0) {
    DBG(vg, "Send next command available notification\n");
    vg->next_cmd_offs = 0;
    up(&vg->cmd_read_mutex);
    wake_up(&vg->cons_wait);
  } else {
    DBG(vg, "Zero bytes transfered. Re-queue the request\n");
    enqueue_request(ep, req, cmdread_complete);
  }
}

/* Requests a next command from the host */
static int request_next_cmd(struct vg_dev *vg)
{
  int rc;

  vg->next_cmd_req = NULL;
  vg->next_cmd_offs = 0;

  rc = 0;
  DBG(vg, "Allocate a request for the next command\n");
  if (vg->next_cmd_req == NULL) {
    if ((rc = allocate_request(vg->bulk_out,
			       CONS_BUFSIZE,
			       &vg->next_cmd_req)) != 0) {
      ERROR(vg, "Unable to allocate a read-command request/buffer\n");
    }
  }

  if (rc == 0) {
    DBG(vg, "Enqueue the request for the next command\n");
    if ((rc = enqueue_request(vg->bulk_out,
			      vg->next_cmd_req,
			      cmdread_complete)) != 0) {
      ERROR(vg, "Unable to enqueue the read-command request\n");
    }
  }

  return rc;
}

/* Read a command from the host */
static ssize_t cmd_read (struct file *filp,
			 char __user *buf,
			 size_t count,
			 loff_t *offp)
{
  int rc;
  ssize_t len;
  struct vg_dev *vg;

  vg = (struct vg_dev *) filp->private_data;
  DBG(vg, "Read command data from the host\n");
  len = 0;

  if (down_interruptible(&vg->cmd_read_mutex) == 0) {
    if (vg->next_cmd_req != NULL) {
      if (vg->next_cmd_req->status == 0) {
	len = min(count,
		  vg->next_cmd_req->actual - vg->next_cmd_offs);
	if ((rc = copy_to_user(buf,			       
			       vg->next_cmd_req->buf + vg->next_cmd_offs,
			       len)) == 0) {
	  vg->next_cmd_offs += len;
	  if (vg->next_cmd_offs == vg->next_cmd_req->actual) {
	    request_next_cmd(vg);
	  }
	} else {
	  ERROR(vg, "Unable to copy the command data to the user\n");
	  len = rc;
	}
      } else {
	ERROR(vg, "Command-read error (%d)\n",
	      vg->next_cmd_req->status);
	len = -EFAULT;
      }
    } else {
      ERROR(vg, "Expected the next command in the buffer\n");
      len = -EFAULT;
    }
  } else {
    ERROR(vg, "Interrupted. Send the system restart signal\n");
    len = -ERESTARTSYS;
  }

  return len;
}

/* Handles the finished status write request */
static void status_write_complete(struct usb_ep *ep,
				  struct usb_request *req)
{
  struct vg_dev *vg;

  vg = (struct vg_dev *) ep->driver_data;

  ep_complete_common(ep, req);
  free_request(ep, req);
  atomic_dec(&vg->statuses_written);
  up(&vg->status_wrlim);
  wake_up(&vg->cons_wait);
}

/* Read a given status data to the host */
static ssize_t status_write (struct file *filp,
			     const char __user *buf,
			     size_t count,
			     loff_t *offp)
{
  ssize_t len;
  struct vg_dev *vg;

  vg = (struct vg_dev *) filp->private_data;
  if (vg != NULL) {
    DBG(vg, "Write status data to the host\n");
    if (down_interruptible(&vg->status_wrlim) == 0) {
      struct usb_request *req;
      int rc;

      rc = 0;
      len = min_t(size_t, count, CONS_BUFSIZE);
      if ((rc == allocate_request(vg->bulk_status_in,
				   len,
				   &req)) == 0) {
	if ((rc = copy_from_user(req->buf, buf, len)) == 0) {
	  rc = enqueue_request(vg->bulk_status_in,
			       req,
			       status_write_complete);
	  atomic_inc(&vg->statuses_written);
	}
      }
      if (rc != 0) {
	len = rc;
      }
    } else {
      len = -ERESTARTSYS;
    }
  } else {
    MERROR("File private data is NULL\n");
    len = -EFAULT;
  }

  return len;
}

/* Opens the console for device */
static int cons_open (struct inode *inode, struct file *filp)
{
  struct vg_dev *vg;
  int rc;

  rc = 0;
  vg = the_vg;
  DBG(vg, "Open the console device\n");

  filp->private_data = vg;

  if (down_trylock(&vg->cmd_read_mutex) == 0) {
    vg->next_cmd_req = NULL;
    request_next_cmd(vg);
  }

  return rc;
}

/* Closes the console device */
static int cons_release (struct inode *inode, struct file *filp)
{
  int rc;
  struct vg_dev *vg;

  rc = 0;
  vg = (struct vg_dev *) filp->private_data;
  filp->private_data = NULL;
  if (vg != NULL) {
    DBG(vg, "Release the console device\n");
    if (vg->next_cmd_req != NULL) {
      free_request(vg->bulk_out, vg->next_cmd_req);
    }
  } else {
    MERROR("File private data is NULL\n");
  }

  return rc;
}


/* Returns a bit mask indicating the current non-blocking
 * operations that are available for the CS device */
static unsigned int cons_poll(struct file *filp,
			      struct poll_table_struct *wait)
{
  struct vg_dev *vg;
  int rc;

  vg = (struct vg_dev *)filp->private_data;

  rc = 0;
  if (down_trylock(&vg->cmd_read_mutex) == 0) {
    if (vg->next_cmd_req != NULL) {
      /* Indicate that the next command data is ready */
      rc |= (POLLIN | POLLRDNORM);
    }
    up(&vg->cmd_read_mutex);
  }
  if (atomic_read(&vg->statuses_written) < mod_data.maxwrites) {
    /* Indicate that the write limit isn't exceeded */
    rc |= (POLLOUT | POLLWRNORM);
  } else {
    /* Add the command wait-queue to the poll table */
    poll_wait(filp, &vg->cons_wait, wait);
  }

  return rc;
}

/* Writes data to the FIFO */
static ssize_t fifo_write (struct file *filp,
			   const char __user *buf,
			   size_t count,
			   loff_t *offp)
{
  ssize_t len;
  struct vg_dev *vg;

  vg = (struct vg_dev *) filp->private_data;
  if (vg != NULL) {
    DBG(vg, "Write data to the FIFO\n");
    len = 0; //TODO: implement write
  } else {
    MERROR("File private data is NULL\n");
    len = -EFAULT;
  }

  return len;
}

/* Opens the FIFO device */
static int fifo_open (struct inode *inode, struct file *filp)
{
  struct vg_dev *vg;
  int rc;

  rc = 0;
  vg = the_vg;
  DBG(vg, "Open the FIFO device\n");

  filp->private_data = vg;

  return rc;
}

/* Closes the FIFO device */
static int fifo_release (struct inode *inode, struct file *filp)
{
  int rc;
  struct vg_dev *vg;

  rc = 0;
  vg = (struct vg_dev *) filp->private_data;
  filp->private_data = NULL;
  if (vg != NULL) {
    DBG(vg, "Release the FIFO device\n");
    if (test_and_clear_bit(FIFO_ERROR, &vg->flags)) {
      DBG(vg, "Clear the FIFO error flag\n");
    }
  } else {
    MERROR("File private data is NULL\n");
  }

  return rc;
}

/* USB request with atomic refcounter */
struct usb_mapped_request {
  struct usb_request *req;
  atomic_t refcnt;
};

/* Handles the VMA open for a mapped FIFO request */
static void fifo_vma_open(struct vm_area_struct *vma)
{
  int rc;
  struct usb_mapped_request *mreq;
  
  mreq = (struct usb_mapped_request *) vma->vm_private_data;

  if (mreq != NULL) {
    rc = atomic_inc_return(&mreq->refcnt);
    MDBG("Add a new reference to the request (%d)\n", rc);
  } else {
    MERROR("VMA private data is NULL\n");
  }
}

/* Handles a finished FIFO request */
static void fifo_complete(struct usb_ep *ep, struct usb_request *req)
{
  ep_complete_common(ep, req);
  free_request(ep, req);
}

/* Handles the VMA close for a mapped FIFO request */
static void fifo_vma_close(struct vm_area_struct *vma)
{
  struct usb_mapped_request *mreq;
  struct vg_dev *vg;
  struct usb_request *req;
  int rc;
  
  mreq = (struct usb_mapped_request *) vma->vm_private_data;
  if (mreq != NULL) {
    vg = (struct vg_dev *) mreq->req->context;
    if (vg != NULL) {
      rc = atomic_dec_return(&mreq->refcnt);
      MDBG("Remove a reference to the request (%d)\n", rc);
      if (rc == 0) {
	req = mreq->req;
	kfree(mreq);
	DBG(vg, "Enqueue a mapped request\n");
	if (enqueue_request(vg->bulk_in, req, fifo_complete) != 0) {
	  ERROR(vg, "Unable to enqueue a mapped request\n");
	}
      }
    } else {
      MERROR("Request context is NULL\n");
    }
  } else {
    MERROR("VMA private data is NULL\n");
  }
}

/* Handles a page fault for a mapped FIFO request */
static int fifo_vma_fault(struct vm_area_struct *vma,
			  struct vm_fault *vmf)
{
  struct usb_mapped_request *mreq;
  int rc;

  mreq = (struct usb_mapped_request *) vma->vm_private_data;
  if (mreq != NULL) {
    MDBG("Handle page fault request at 0x%lx\n",
	 vmf->pgoff << PAGE_SHIFT);
    //vmf->page = NOPAGE_SIGBUS;
    vmf->page = virt_to_page(mreq->req->buf			\
			     + (vmf->pgoff << PAGE_SHIFT));
    if (vmf->page != NULL) {
      get_page(vmf->page);
      vmf->flags |= VM_FAULT_MINOR;
      rc = 0;
    } else {
      MERROR("Unable to map the missing page\n");
      vmf->flags |= VM_FAULT_NOPAGE;
      rc = -EFAULT;
    }
  } else {
    MERROR("VMA private data is NULL\n");
    rc = -EFAULT;
  }

  return rc;
}

/* Declare the set of FIFO VMA operations */
static struct vm_operations_struct fifo_vma_ops = {
  .open = fifo_vma_open,
  .close = fifo_vma_close,
  .fault = fifo_vma_fault,
};

/* Maps a next FIFO buffer for userspace access */
static int fifo_mmap (struct file *filp, struct vm_area_struct *vma)
{
  struct vg_dev *vg;
  struct usb_request *req;
  int rc;

  vg = (struct vg_dev *) filp->private_data;
  if (vg != NULL) {
    DBG(vg, "Map a FIFO buffer of %lu bytes\n",
	vma->vm_end - vma->vm_start);
    if ((rc = allocate_request(vg->bulk_in,
			       vma->vm_end - vma->vm_start,
			       &req)) != 0) {
      ERROR(vg, "Unable to allocate a request of size %lu b\n",
	    vma->vm_end - vma->vm_start);
    } else {
      req->context = vg;
    }
  } else {
    MERROR("File private data is NULL\n");
    rc = -EFAULT;
  }

  if (rc == 0) {
    struct usb_mapped_request *mreq;
    vma->vm_ops = &fifo_vma_ops;
    vma->vm_flags |= VM_RESERVED;
    mreq = kmalloc(sizeof *mreq, GFP_KERNEL);
    if (mreq != NULL) {
      mreq->req = req;
      atomic_set(&mreq->refcnt, 0);
      vma->vm_private_data = mreq;
      fifo_vma_open(vma);
    } else {
      rc = -ENOMEM;
    }
  }

  return rc;
}

/* Free the page reqeust resources */
static void free_page_request(struct usb_ep *ep,
			      struct usb_request *req)
{
  struct vg_dev *vg;

  vg = (struct vg_dev *) ep->driver_data;

  DBG(vg, "Free a request for page delivery\n");
  dma_unmap_page(&vg->gadget->dev, req->dma, req->length,
		 DMA_TO_DEVICE);
  usb_ep_free_request(ep, req);
}

/* Handles a finished FIFO request, set error status accordingly 
 * and sent the notification */
static void fifo_complete_notify(struct usb_ep *ep,
				 struct usb_request *req)
{
  struct vg_dev *vg;
  struct completion *req_compl;

  ep_complete_common(ep, req);

  vg = (struct vg_dev *) ep->driver_data;

  if (req->status != 0 || req->actual < req->length) {
    DBG(vg, "Set the FIFO error status\n");
    set_bit(FIFO_ERROR, &vg->flags);
  }

  if (req->actual > req->length) {
    MDBG("Fix the actual sent value: %d/%d -> %d\n",
	 req->actual,
	 req->length,
	 req->length);
    req->actual = req->length;
  }// TODO: fix the accumulation effect

  req_compl = (struct completion *) req->context;
  if (req_compl != NULL) {
    DBG(vg, "Sent request completion notification");
    complete(req_compl);
  } else {
    free_page_request(ep, req);
    DBG(vg, "Turn up the FIFO write limit semaphore\n");
    up(&vg->fifo_wrlim);
  }
}

/* Send a given memory page over the USB channel */
static ssize_t fifo_sendpage (struct file *filp,
			      struct page *page,
			      int offset,
			      size_t length,
			      loff_t *offp,
			      int more)
{
  ssize_t len;
  struct vg_dev *vg;
  struct usb_request *req;

  vg = (struct vg_dev *) filp->private_data;

  if (test_bit(FIFO_ERROR, &vg->flags) != 0) {
    ERROR(vg, "One or more FIFO requests failed\n");
    len = -EFAULT;
  }

  if (length == PAGE_SIZE) {
    more = 1;
  }

  if (vg != NULL) {
    DBG(vg, "Send a page over the USB channel\n");
    DBG(vg, "Allocate a request for page delivery\n");
    if ((req = usb_ep_alloc_request(vg->bulk_in, GFP_KERNEL)) != NULL) {
      req->buf = page_address(page);
      req->complete = fifo_complete_notify;
      req->length = length;
      req->actual = 0;
      req->zero = 0;
      len = 0;
    } else {
      ERROR(vg, "Unable to allocate a request for page delivery\n");
      len = -ENOMEM;
    }
  } else {
    MERROR("File private data is NULL\n");
    len = -EFAULT;
  }

  if (len == 0) {
    if ((req->dma = dma_map_page(&vg->gadget->dev,
				 page,
				 offset,
				 length,
				 DMA_TO_DEVICE)) != 0) {
      struct completion req_compl;
      if (!more) {
	init_completion(&req_compl);
	req->context = &req_compl;
      } else {
	req->context = NULL;
      }
      DBG(vg, "Turn down the FIFO write limit semaphore\n");
      if (down_interruptible(&vg->fifo_wrlim) == 0) {
	if (enqueue_request(vg->bulk_in,
			    req,
			    fifo_complete_notify) == 0) {
	  if (!more) {
	    DBG(vg, "No more data to send. "
		"Wait for the request to complete\n");
	    wait_for_completion(&req_compl);
	    DBG(vg, "Last request finished (%d/%d)\n",
		req->actual,
		length);
	    if (test_bit(FIFO_ERROR, &vg->flags) == 0) {
	      len = req->actual;
	    } else {
	      ERROR(vg, "One or more FIFO requests failed\n");
	      len = -EFAULT;
	    }
	    free_page_request(vg->bulk_in, req);
	    DBG(vg, "Turn up the FIFO write limit semaphore\n");
	    up(&vg->fifo_wrlim);
	    DBG(vg, "No more data to send -- flush the EP FIFO\n");
	    usb_ep_fifo_flush(vg->bulk_in);
	  } else {
	    len = length;
	  }
	} else {
	  ERROR(vg, "Unable to enqueue a USB request\n");
	  len = -EFAULT;
	}
      } else {
	ERROR(vg, "Interrupted. Send restart system signal\n");
	len = -ERESTARTSYS;
      }
    } else {
      ERROR(vg, "Unable to map a page for DMA\n");
      len = -EFAULT;
    }
  }
  
  return len;
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
		for (i = 0; i < num; i++) {
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
