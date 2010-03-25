/*
 * Versatile USB device driver
 *
 * Copyright (C) 2010 Paul Wolneykien
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.28 version of drivers/usb/usb-skeleton.c.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <asm/uaccess.h>
#include <linux/usb.h>

/* String constants */
#define DRIVER_DESC		"Versatile Device"
#define DRIVER_NAME		"vdevice"
#define DRIVER_VERSION		"25 March 2010"
static const char longname[] = DRIVER_DESC;
static const char shortname[] = DRIVER_NAME;

/* Claim module description */
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Paul Wolneykien");
MODULE_LICENSE("GPL");

/* Static variables for vendor and product IDs */
unsigned short vendor = 0xfff;
unsigned short product = 0xfff;

/* Table of devices that work with this driver */
static struct usb_device_id vdev_table[] = {
	{ USB_DEVICE(vendor, product) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, skel_table);

/* Parameterize vendor and product IDs */
module_param_named(vendor, vendor, ushort, S_IRUGO);
MODULE_PARM_DESC(vendor, "USB Vendor ID");

module_param_named(product, product, ushort, S_IRUGO);
MODULE_PARM_DESC(product, "USB Product ID");

/* General I/O timemout, jiffies */
static int timeout = 10000;
module_param_named(timeout, timeout, int, S_IRUGO);
MODULE_PARM_DESC(timeout, "I/O timeout, jiffies");

/* USB interface base minor number */
static int minor = 192;
module_param_named(minor, minor, int, S_IRUGO);
MODULE_PARM_DESC(minor, "USB interface base minor number");

/* Define limits */
#define MAX_TRANSFER		( PAGE_SIZE - 512 )
#define WRITES_IN_FLIGHT	8

/* Structure to hold all of our device specific stuff */
struct usb_vdev {
  /* general params */
  struct kref kref;
  /* USB params */
  struct usb_device *udev;
  struct usb_interface *interface;
  /* limiting the number of writes in progress */
  struct semaphore limit_sem;
  /* the buffer to receive data */
  unsigned char *bulk_in_buffer;
  size_t bulk_in_size;
  __u8 bulk_in_epaddr;
  /* the buffer to receive status data */
  unsigned char *bulk_status_in_buffer
  size_t bulk_status_in_size;
  __u8 bulk_status_in_epaddr;
  /* the buffer to send command data */
  __u8 bulk_out_epaddr;
};
#define to_vdev(d) container_of(d, struct usb_vdev, kref)

/* Device driver instance (prototype) */
static struct usb_driver vdev_driver;

/* Cleanup procedure */
static void vdev_delete(struct kref *kref)
{	
	struct usb_vdev *dev = to_vdev(kref);

	usb_put_dev(dev->udev);
	if (dev->bulk_in_buffer) {
	  kfree(dev->bulk_in_buffer);
	}
	if (dev->bulk_status_in_buffer) {
	  kfree(dev->bulk_status_in_buffer);
	}
	kfree(dev);
}

/* Accociates the device instance with an interface on the
 * device class file open. Device driver instance is also
 * initialized */
static int vdev_open(struct inode *inode, struct file *file)
{
	struct usb_vdev *dev;
	struct usb_interface *interface;
	int subminor;

	subminor = iminor(inode);

	interface = usb_find_interface(&vdev_driver, subminor);
	if (!interface) {
		err ("%s - error, can't find device for minor %d",
		     __FUNCTION__, subminor);
		return -ENODEV;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
	        return -ENODEV;
	}

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

	return 0;
}

/* De-associetes the device and driver instances */
static int vdev_release(struct inode *inode, struct file *file)
{
	struct usb_vdev *dev;

	dev = (struct usb_vdev *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* decrement the count on our device */
	kref_put(&dev->kref, vdev_delete);
	return 0;
}

/* Interface procedure for reading status data from a gadget */
static ssize_t status_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct usb_vdev *dev;
	int retval = 0;
	int bytes_read;

	dev = (struct usb_vdev *)file->private_data;
	
	/* do a blocking bulk read to get data from the device */
	retval = usb_bulk_msg(dev->udev,
			      usb_rcvbulkpipe(dev->udev, dev->bulk_status_in_epaddr),
			      dev->bulk_status_in_buffer,
			      min(dev->bulk_status_in_size, count),
			      &bytes_read, timeout);

	/* if the read was successful, copy the data to userspace */
	// TODO: copy via DMA
	if (!retval) {
		if (copy_to_user(buffer, dev->bulk_in_buffer, bytes_read))
			retval = -EFAULT;
		else
			retval = bytes_read;
	}

	return retval;
}

/* Handles a finished bulk write request */
static void vdev_write_bulk_callback(struct urb *urb, struct pt_regs *regs)
{
	struct usb_vdev *dev;

	dev = (struct usb_vdev *)urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status && 
	    !(urb->status == -ENOENT || 
	      urb->status == -ECONNRESET ||
	      urb->status == -ESHUTDOWN)) {
		dbg("%s - nonzero write bulk status received: %d",
		    __FUNCTION__, urb->status);
	}

	/* free up our allocated buffer */
	usb_buffer_free(urb->dev, urb->transfer_buffer_length, 
			urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
}

/* Interface procedure for writing command data to a gadget */
static ssize_t cmd_write(struct file *file, const char *user_buffer, size_t count, loff_t *ppos)
{
	struct usb_vdev *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = min(count, (size_t)MAX_TRANSFER);

	dev = (struct usb_vdev *)file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0) {
	  return 0;
	}

	/* limit the number of URBs in flight to stop a user from using up all RAM */
	if (down_interruptible(&dev->limit_sem)) {
		return -ERESTARTSYS;
	}

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
	        up(&dev->limit_sem);
		return -ENOMEM;
	}

	buf = usb_buffer_alloc(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
	        usb_free_urb(urb);
		up(&dev->limit_sem);
		return -ENOMEM;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
	        usb_buffer_free(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
		up(&dev->limit_sem);
		return = -EFAULT;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_epaddr),
			  buf, writesize, vdev_write_bulk_callback, dev);
	// TODO: transfer via DMA
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		err("%s - failed submitting write urb, error %d", __FUNCTION__, retval);
		usb_buffer_free(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
		up(&dev->limit_sem);
		return retval;
	}

	/* release our reference to this urb, the USB core will eventually free it entirely */
	usb_free_urb(urb);

	return writesize;
}

/* Command-status I/O and class device file operations */
static struct file_operations vdev_fops = {
	.owner =	THIS_MODULE,
	.read =		status_read,
	.write =	cmd_write,
	.open =		vdev_open,
	.release =	vdev_release,
};

/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver vdev_class = {
	.name =		"usbcons%d",
	.fops =		&vdev_fops,
	.minor_base =	minor,
};

#define fill_in_ep(ep_prefix) \
  dev->ep_prefix##_size = le16_to_cpu(endpoint->wMaxPacketSize);	\
  dev->ep_prefix##_epaddr = endpoint->bEndpointAddress;			\
  dev->ep_prefix##_buffer = kmalloc(le16_to_cpu(endpoint->wMaxPacketSize), GFP_KERNEL);

/* USB probbing procedure */
static int vdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_vdev *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		err("Out of memory");
	        return -ENOMEM;
	}
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	dev->bulk_in_epaddr = 0;
	dev->bulk_status_in_epaddr = 0;
	dev->bulk_out_epaddr = 0;
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
	  endpoint = &iface_desc->endpoint[i].desc;

	  if ( ((endpoint->bEndpointAddress				\
		 & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)		\
	       && ((endpoint->bmAttributes				\
		    & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)) {
	    /* we found a bulk in endpoint */
	    if (!dev->bulk_in_epaddr) {
	      fill_in_ep(bulk_in);
	    } else if (!dev->bulk_status_in_epaddr) {
	      fill_in_ep(bulk_status_in);
	    } else {
	      continue;
	    }
	  }

	  if ( ((endpoint->bEndpointAddress				\
		 & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)		\
	       && ((endpoint->bmAttributes				\
		    & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)) {
	    /* we found a bulk out endpoint */
	    if (!dev->bulk_out_epaddr) {
	      dev->bulk_out_epaddr = endpoint->bEndpointAddress;
	    }
	  }
	}

	if (!dev->bulk_in_epaddr \
	    || !dev->bulk_status_in_epaddr \
	    || !dev->bulk_out_epaddr) {
	  err("Could not find both bulk-in and bulk-out endpoints");
	  kref_put(&dev->kref, vdev_delete);
	  return -EINVAL;
	}

	if (!dev->bulk_in_buffer || !dev->bulk_status_in_buffer) {
	  err("Could not allocate bulk_in_buffer");
	  kref_put(&dev->kref, vdev_delete);
	  return -ENOMEM;
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &vdev_class);
	if (retval) {
		/* something prevented us from registering this driver */
		err("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		kref_put(&dev->kref, vdev_delete);
		return retval;
	}

	/* let the user know what node this device is now attached to */
	info("USB Versatile device is now attached to USBSkel-%d", interface->minor);
	return 0;
}

/* Handles a USB device disconnect event */
static void vdev_disconnect(struct usb_interface *interface)
{
	struct usb_vdev *dev;
	int minor = interface->minor;

	/* prevent vdev_open() from racing vdev_disconnect() */
	lock_kernel();

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor number */
	usb_deregister_dev(interface, &vdev_class);

	unlock_kernel();

	/* decrement the usage count */
	kref_put(&dev->kref, vdev_delete);

	info("USB Versatile device #%d is now disconnected", minor);
}

/* Device driver instance */
static struct usb_driver vdev_driver = {
	.name =		"vdevice",
	.probe =	vdev_probe,
	.disconnect =	vdev_disconnect,
	.id_table =	vdev_table,
};

/* Module initialization procedure */
static int __init usb_vdev_init(void)
{
	int result;

	/* register the driver with the USB subsystem */
	result = usb_register(&vdev_driver);
	if (result) {
	  err("usb_register failed. Error number %d", result);
	}

	return result;
}

/* Module exit procedure */
static void __exit usb_vdev_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&vdev_driver);
}

module_init (usb_vdev_init);
module_exit (usb_vdev_exit);
