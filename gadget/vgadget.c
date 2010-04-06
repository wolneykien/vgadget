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

#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/utsname.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

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
} mod_data = {					// Default values
	.vendor		= DRIVER_VENDOR_ID,
	.product	= DRIVER_PRODUCT_ID,
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

/* The character device parameters */
#define CONS_FNAME "usbconsS"
#define CONS_MAJOR 63
#define FIFO_FNAME "usbconsF"
#define FIFO_MAJOR 73

/* Define operations for the character devices */
static ssize_t cmd_read (struct file *, char __user *, size_t, loff_t *);
static ssize_t status_write (struct file *, const char __user *, size_t,
			     loff_t *);
static int cons_open (struct inode *, struct file *);
static int cons_release (struct inode *, struct file *);
static struct file_operations cons_fops = {
	.owner		= THIS_MODULE,
	.read           = cmd_read,
	.write          = status_write,
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

/* The gadget device object */
static struct vg_dev *the_vg;

/* Starts the main process */
static int __init vg_main_process_start(struct vg_dev *vg)
{
  int rc;

  DBG(vg, "Initialize synchronization objects\n");
  init_completion(&the_vg->main_event);
  init_completion(&the_vg->main_exit);
  DBG(vg, "Start the main process\n");
  set_bit(RUNNING, &vg->flags);
  rc = kernel_thread(main_process,
		     the_vg,
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
  set_bit(RUNNING, &vg->flags);
  complete(&vg->main_event);
  wait_for_completion(&vg->main_exit);

  return rc;
}

/* Initializes and registeres the module  */
static int __init vg_init(void)
{
	int rc;
	int cdevno, fdevno;

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
	  } else {
	    vg_free(the_vg);
	    MERROR("Unable to register driver\n");
	  }
	} else {
	  MERROR("Unable to allocate the gadget device object\n");
	}

	rc = vg_main_process_start(the_vg);

	if (rc == 0) {
	  MDBG("Register the console device\n");
	  cdevno = MKDEV(CONS_MAJOR, 0);
	  cdev_init(&the_vg->cons_dev, &cons_fops);
	  the_vg->cons_dev.owner = THIS_MODULE;
	  the_vg->cons_dev.ops = &cons_fops;
	  the_vg->cons_dev.kobj.name = CONS_FNAME;
	  if ((rc = cdev_add(&the_vg->cons_dev, cdevno, 1)) != 0) {
	    MERROR("Unable to register the console device\n");
	  }
	}

	if (rc == 0) {
	  MDBG("Register the FIFO device\n");
	  fdevno = MKDEV(FIFO_MAJOR, 0);
	  cdev_init(&the_vg->fifo_dev, &fifo_fops);
	  the_vg->fifo_dev.owner = THIS_MODULE;
	  the_vg->fifo_dev.ops = &fifo_fops;
	  the_vg->fifo_dev.kobj.name = FIFO_FNAME;
	  if ((rc = cdev_add(&the_vg->fifo_dev, fdevno, 1)) != 0) {
	    MERROR("Unable to register the FIFO device\n");
	  }
	}

	return rc;
}
/* Set up the module initialization handler */
module_init(vg_init);


/* Unregisteres the module, frees up the allocated resources */
static void __exit vg_cleanup(void)
{
	struct vg_dev	*vg = the_vg;

	/* Terminate the main process */
	vg_main_process_terminate(the_vg);

	MDBG("Unregister the console device\n");
	cdev_del(&vg->cons_dev);

	MDBG("Unregister the FIFO device\n");
	cdev_del(&vg->fifo_dev);

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
static int __init vg_alloc(struct vg_dev **vg)
{
        int rc;

	MDBG("Allocate the gadget device object\n");
	*vg = kmalloc(sizeof *vg, GFP_KERNEL);
	if (vg) {
	  memset(*vg, 0, sizeof *vg);
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
static int __init autoconfig_endpoint(struct vg_dev *vg,
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

  vg = (vg_dev *) ep->driver_data;

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
  }

  if (rc == 0) {
    MDBG("Allocate a buffer for the request of %d b\n", size);
    if (size <= DMA_POOL_BUF_SIZE) {
      (*req)->buf =
	dma_pool_alloc(vg->dma_pool, GFP_KERNEL, &(*req)->dma);
    } else {
      MWARN("Allocate a large coherent DMA buffer not from the pool\n");
      (*req)->buf =
	dma_alloc_coherent(vg->gadget->dev, size, &(*req)->dma, GFP_KERNEL);
    }
    if (*req->buf == NULL) {
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

  vg = (vg_dev *) ep->driver_data;

  if (req->buf != NULL) {
    MDBG("Free the buffer of a request for endpoint %s\n", ep->name);
    if (size <= DMA_POOL_BUF_SIZE) {
      dma_pool_free(vg->dma_pool, req->buf, req->dma);
    } else {
      dma_free_coherent(vg->gadget->dev, req->length, req->buf, req->dma);
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
static void set_request_length(struct urb_request *req,
			       int len)
{
  if (size <= DMA_POOL_BUF_SIZE) {
    req->length = len;
  }
}

/* Enqueues a URB request */
static int enqueue_request(struct usb_ep *ep,
			   struct urb_request *req,
			   (*complete)(struct usb_ep *ep,
				       struct usb_request *req))
{
  int rc;

  struct vg_dev *vg;

  vg = ep->driver_data;
  DBG(vg, "Enqueue a request of size %d b for endpoint %s\n",
      ep-name,
      req->length);
  req->complete = complete;
  req->context = vg;
  rc = usb_ep_queue(ep, req, GFP_ATOMIC);
  if (rc != 0 && rc != -ESHUTDOWN) {
    /* Can't do much more than wait for a reset */
    WARN(vg, "Error in submission: %s --> %d\n",
	 ep->name, rc);
  }

  return rc;
}

/* Handles a finished request of an endpoint */
static void ep_complete_common(struct usb_ep *ep, struct usb_request *req)
{
  struct vg_dev *vg = (struct vg_dev *) req->context;

  if (req->status || req->actual != req->length) {
    WARN(vg, "Request completed with error: %d %u/%u\n",
	 req->status, req->actual, req->length);
  }

  /* Request was cancelled */
  if (req->status == -ECONNRESET) {
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
static int __init vg_bind(struct usb_gadget *gadget)
{
	struct vg_dev *vg = the_vg;
	int rc;

	vg->gadget = gadget;
	set_gadget_data(gadget, vg);
	vg->ep0 = gadget->ep0;
	vg->ep0->driver_data = vg;

	MDBG("Allocate the DMA pool\n");
	vg->dma_pool =
	  dma_pool_create(DMA_POOL_NAME,
			  gadget->dev, 
			  DMA_POOL_BUF_SIZE,
			  PAGE_SIZE,
			  PAGE_SIZE);

	if ((rc = check_parameters(vg)) != 0) {
	  ERROR(vg, "Invalid parameter(s) passed\n");
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
    WARN(vg, "An other reconfiguration is in progress\n");
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
    WARN(vg, "An other interface reconfiguration is in progress\n");
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
						      *(res_req)->buf);
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
		if (wValue >= 0 && wValue <= NUMBER_OF_CONFIGS) {
		  if ((value = vg_request_reconf(vg, wValue)) == 0) {
		    value = DELAYED_STATUS;
		  }
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
		  if ((value =
		       vg_request_intf_reconf(vg,
					      wIndex,
					      wValue)) == 0) {
		    value = DELAYED_STATUS;
		  }
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
	DBG(vg, "Setup request number %d\n", vg->req_tag);
	vg->ep0_req->context = NULL;
	vg->ep0_req->length = 0;
	dump_msg(vg, "ep0-setup", (u8 *) ctrl, sizeof(*ctrl));

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
	//TODO: stop all transfer processes, may not sleep
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

  return rc;
}

/* Resets the interface setting and re-init endpoints */
static int do_set_interface(struct vg_dev *vg,
			    int index,
			    int altsetting)
{
  int rc = 0;

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

  return rc;
}

/* Change the operational configuration */
static int do_set_config(struct vg_dev *vg, int new_config)
{
  int	rc = 0;

  VDBG(vg, "Reset the configuration\n");

  /* Disable the single interface */
  DBG(vg, "Disable the interfaces\n");
  if (vg->config != 0) {
    vg->config = 0;
    rc |= do_set_interface(vg, 0, -1);
    rc |= do_set_interface(vg, 1, -1);
  }

  if (new_config > 0) {
    char *speed;
    DBG(vg, "Setup the configuration number %d\n", new_config);
    vg->config = new_config;
    switch (vg->gadget->speed) {
    case USB_SPEED_LOW:	   speed = "low";       break;
    case USB_SPEED_FULL:   speed = "full";      break;
    case USB_SPEED_HIGH:   speed = "high";      break;
    default:               speed = "?";	        break;
    }
    INFO(vg, "Set up the %s speed config number %d\n",
	 speed,
	 vg->config);
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
  while (test_bit(REGISTERED, &vg->flags)) {
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
			   vg->intf_config[vg->intf_index]) != 0) {
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

/* Read a command from the host */
static ssize_t cmd_read (struct file *filp,
			 char __user *buf,
			 size_t count,
			 loff_t *offp)
{
  ssize_t len;
  struct vg_dev *vg;

  vg = (struct vg_dev *) filp->private_data;
  DBG(vg, "Read command data from the host\n");
  len = 0; //TODO: implement read

  return len;
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
  DBG(vg, "Write status data to the host\n");
  len = 0; //TODO: implement write

  return len;
}

/* Opens the console for device */
static int cons_open (struct inode *inode, struct file *filp)
{
  struct vg_dev *vg;
  int rc;

  vg = container_of(inode->i_cdev, struct vg_dev, cdev);
  DBG(vg, "Open the console device\n");

  filp->private_data = vg;
}

/* Closes the console device */
static int cons_release (struct inode *inode, struct file *filp)
{
  struct vg_dev *vg;

  vg = (struct vg_dev *) filp->private_data;
  DBG(vg, "Release the console device\n");
  filp->private_data = NULL;
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
  DBG(vg, "Write data to the FIFO\n");
  len = 0; //TODO: implement write

  return len;
}

/* Opens the FIFO device */
static int fifo_open (struct inode *, struct file *)
{
  struct vg_dev *vg;
  int rc;

  vg = container_of(inode->i_cdev, struct vg_dev, fifo_dev);
  DBG(vg, "Open the FIFO device\n");

  filp->private_data = vg;
}

/* Closes the FIFO device */
static int fifo_release (struct inode *, struct file *)
{
  struct vg_dev *vg;

  vg = (struct vg_dev *) filp->private_data;
  DBG(vg, "Release the FIFO device\n");
  filp->private_data = NULL;
}

/* USB request with atomic refcounter */
struct usb_mapped_request {
  struct usb_request *req;
  atomic_t refcnt;
}

/* Handles the VMA open for a mapped FIFO request */
static int fifo_vma_open(struct vm_area_struct *vma)
{
  int rc;
  struct usb_mapped_request *mreq;
  
  mreq = (usb_mapped_request *) vma->vm_private_data;
  
  rc = atomic_inc_return(&mreq->refcnt);
  MDBG("Add a new reference to the request (%d)\n", rc);

  return rc;
}

/* Handles a finished FIFO request */
static void fifo_complete(struct usb_ep *ep, struct usb_request *req)
{
  ep_complete_common(ep, req);
  free_request(ep, req);
}

/* Handles the VMA close for a mapped FIFO request */
static int fifo_vma_close(struct vm_area_struct *vma)
{
  struct usb_mapped_request *mreq;
  struct vg_dev *vg;
  int rc;
  
  mreq = (usb_mapped_request *) vma->vm_private_data;
  vg = (vg_dev *) mreq->req->context;
  
  rc = atomic_dec_return(&mreq->refcnt);
  MDBG("Remove a reference to the request (%d)\n", rc);
  if (rc == 0) {
    DBG(vg, "Enqueue a mapped request\n");
    if (enquque_request(vg->bulk_in, mreq->req, fifo_complete) != 0) {
      ERROR(vg, "Unable to enqueue a mapped request\n");
    }
  }

  return rc;
}

/* Handles a page fault for a mapped FIFO request */
static int fifo_vma_fault(struct vm_area_struct *vma,
				   struct vm_fault *vmf)
{
  int rc;
  //vmf->page = NOPAGE_SIGBUS;
  vmf->page = virt_to_page(req->buf + vmf->pgoff);
  if (vmf->page != NULL) {
    get_page(vmf->page);
    vmf->flags |= VM_FAULT_MINOR;
    rc = 0;
  } else {
    vmf->flags |= VM_FAULT_NOPAGE;
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
  DBG(vg, "Map a FIFO buffer\n");
  if ((rc = allocate_request(vg->bulk_in,
			     vma->vm_end - vma->vm_start,
			     &req)) != 0) {
    ERROR(vg, "Unable to allocate a request of size %d\n",
	  vma->vm_end - vma->vm_start);
  } else {
    req->context = vg;
  }

  if (rc == 0) {
    struct usb_mapped_request *mreq;
    vma->vm_ops = &fifo_vma_ops;
    vma->vm_flags |= VM_RESERVED;
    mreq = kmalloc(sizeof struct usb_mapped_request, GFP_KERNEL);
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

/* Handles a finished FIFO request and send notification */
static void fifo_complete_notify(struct usb_ep *ep,
				 struct usb_request *req)
{
  struct completion *req_compl;

  ep_complete_common(ep, req);

  req_compl = (struct completion *) req->context;
  if (req_cmopl != NULL) {
    complete(req_compl);
  } else {
    MWARN("No nofinication handler. Free the request\n");
    free_request(ep, req);
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
  struct usb_request req;

  vg = (struct vg_dev *) filp->private_data;
  DBG(vg, "Send a page over the USB channel\n");

  DBG(vg, "Allocate a request for page delivery\n");
  if ((req = usb_ep_alloc_request(vg->bulk_in, GFP_KERNEL)) != NULL) {
    req->buf = page->virtual;
    req->complete = fifo_complete_notify;
    req->length = length;
    req->zero = 0;
    if ((req->dma = dma_map_page(vg->gadget->dev,
				 page,
				 offset,
				 length,
				 DMA_TO_DEVICE)) != 0) {
      struct completion *req_compl;
      if ((req_compl =
	   kmalloc(sizeof struct completion, GFP_KRNEL) != NULL)) {
	init_completion(req_compl);
	req->context = req_compl;
	if (enqueue_request(req) == 0) {
	  DBG(vg, "Wait for the request to complete\n");
	  wait_for_completion(&req_compl);
	  DBG(vg, "Return the actual amount sent (%d/%d)\n",
	      legnth,
	      req->actual);
	  len = req->actual;
	  DBG(vg, "Free a request for page delivery\n");
	  dma_unmap_page(vg->gadget->dev, req->dma, length,
			 DMA_TO_DEVICE);
	  usb_ep_free_request(vg->bulk_in, req);
	  kfree(req_compl);
	} else {
	  ERROR(vg, "Unable to enqueue a USB request\n");
	  len = -EFAULT;
	}
      } else {
	ERROR(vg, "Unable to allocate a completion object\n");
	len = -ENOMEM;
      }
    } else {
      ERROR(vg, "Unable to map a page for DMA\n");
      len = -EFAULT;
    }
  } else {
    ERROR(vg, "Unable to allocate a request for endpoint %s\n",
	  ep->name);
    len = -ENOMEM;
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
