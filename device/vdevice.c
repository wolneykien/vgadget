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
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/kthread.h>
#include <asm/bitops.h>
#include <linux/wait.h>
#include <linux/splice.h>
#include <linux/mm.h>
#include <asm/page.h>

#include "pipem.c"

/* Various macros */
#define info(format, arg...) \
  printk(KERN_INFO KBUILD_MODNAME ": " format "\n", ##arg)

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

/* Table of devices that work with this driver */
static struct usb_device_id vdev_table[] = {
	{ USB_DEVICE(0xfff, 0xfff) },
	{ }					/* Terminating entry */
};
//MODULE_DEVICE_TABLE (usb, vdev_table);

/* Parameterize vendor and product IDs */
module_param_named(vendor, vdev_table[0].idVendor, ushort, S_IRUGO);
MODULE_PARM_DESC(vendor, "USB Vendor ID");

module_param_named(product, vdev_table[0].idProduct, ushort, S_IRUGO);
MODULE_PARM_DESC(product, "USB Product ID");

/* General I/O timemout, jiffies */
static int timeout = 10000;
module_param_named(timeout, timeout, int, S_IRUGO);
MODULE_PARM_DESC(timeout, "I/O timeout, jiffies");

/* CS USB class driver info (prototype) */
static struct usb_class_driver vdev_class;

/* USB interface base minor number */
module_param_named(minor, vdev_class.minor_base, int, S_IRUGO);
MODULE_PARM_DESC(minor, "USB interface base minor number");

/* Define limits */
#define MAX_TRANSFER	PAGE_SIZE
#define READ_BUF_SIZE   PAGE_SIZE

/* The maximal number of write requests in the queue */
static int maxwrites = 4;
module_param_named(maxwrites, maxwrites, int, S_IRUGO);
MODULE_PARM_DESC(maxwrites,
		 "The maximal number of write requests "
		 "in the queue");

/* The maximal number of read requests in the queue */
static int maxreads = 4;
module_param_named(maxreads, maxreads, int, S_IRUGO);
MODULE_PARM_DESC(maxreads, "The number of read-ahead buffers");

/* Fake read parameter */
static int fakeread = 0;
module_param_named(fakeread, fakeread, int, S_IRUGO);
MODULE_PARM_DESC(fakeread, "Short-circuit the read loop "
		 "to measure maximum speed");

/* Common structure to hold working data of the devices */
struct usb_vdev_common {
  /* general params */
  struct kref kref;
  /* USB params */
  struct usb_device *udev;
  struct usb_interface *interface;
};

/* Structure to hold working data of the command-status device */
struct usb_vdev {
  /* general params */
  struct kref kref;
  /* USB params */
  struct usb_device *udev;
  struct usb_interface *interface;
  /* limiting the number of writes in progress */
  struct semaphore limit_sem;
  /* the buffer to receive status data */
  unsigned char *bulk_status_in_buffer;
  size_t bulk_status_in_size;
  __u8 bulk_status_in_epaddr;
  /* the buffer to send command data */
  __u8 bulk_out_epaddr;
  wait_queue_head_t cmd_wait;
  atomic_t cmds_sent;
};
#define to_vdev(d) container_of(d, struct usb_vdev, kref)

/* Define process flag bits */
#define RUNNING 1

/* The URB buffer information linked list entry structure */
struct urb_entry {
  struct urb *urb;
  struct urb_entry *next;
};

/* Structure to hold working data of the FIFO device */
struct usb_vfdev {
  /* general params */
  struct kref kref;
  /* USB params */
  struct usb_device *udev;
  struct usb_interface *interface;
  /* limiting the number of reads in progress */
  struct semaphore limit_sem;
  /* mutex for queue access */
  struct semaphore mutex;
  /* the URB queue */
  struct semaphore queue_sem;
  struct urb_entry *queue;
  /* the buffer to receive data */
  unsigned char *bulk_in_buffer;
  size_t bulk_in_size;
  __u8 bulk_in_epaddr;
  /* The read-ahead process data */
  int read_ahead_pid;
  unsigned long read_ahead_flags;
  struct semaphore read_ahead_running;
  struct completion read_ahead_exit;
  struct urb *current_urb;
  unsigned long current_offs;
  wait_queue_head_t fifo_wait;
};
#define to_vfdev(d) container_of(d, struct usb_vfdev, kref)

/* CS device driver instance (prototype) */
static struct usb_driver vdev_driver;

/* FIFO device driver instance (prototype) */
static struct usb_driver vfdev_driver;

/* CS device cleanup procedure */
static void vdev_delete(struct kref *kref)
{	
	struct usb_vdev *dev = to_vdev(kref);

	dbg("Delete the CS device\n");
	usb_put_dev(dev->udev);
	if (dev->bulk_status_in_buffer) {
	  kfree(dev->bulk_status_in_buffer);
	}
	dbg("Free the CS device structure\n");
	kfree(dev);
}

/* FIFO device cleanup procedure */
static void vfdev_delete(struct kref *kref)
{	
        struct usb_vfdev *dev = to_vfdev(kref);

	dbg("Delete the FIFO device\n");
	usb_put_dev(dev->udev);
	if (dev->bulk_in_buffer) {
	  kfree(dev->bulk_in_buffer);
	}
	dbg("Free the FIFO device structure\n");
	kfree(dev);
}

/* Accociates a device instance with a given interface on the
 * device class file open. A given device driver instance is
 * also initialized */
static int open_common(struct inode *inode,
		       struct file *file,
		       struct usb_driver *driver)
{
	struct usb_vdev_common *dev;
	struct usb_interface *interface;
	int subminor;

	subminor = iminor(inode);

	interface = usb_find_interface(driver, subminor);
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

/* Accociates the CS device on the device class file open */
static int vdev_open(struct inode *inode, struct file *file)
{
  return open_common(inode, file, &vdev_driver);
}

/* Accociates the FIFO device on the device class file open */
static int vfdev_open(struct inode *inode, struct file *file)
{
  return open_common(inode, file, &vfdev_driver);
}

/* De-associetes a device and driver instances */
static int release_common(struct inode *inode,
			  struct file *file,
			  void (*delete)(struct kref *kref))
{
	struct usb_vdev_common *dev;

	dev = (struct usb_vdev_common *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* decrement the count on our device */
	kref_put(&dev->kref, delete);
	return 0;
}

/* De-associetes the CS device and driver instances */
static int vdev_release(struct inode *inode, struct file *file)
{
  return release_common(inode, file, vdev_delete);
}

/* De-associetes the FIFO device and driver instances */
static int vfdev_release(struct inode *inode, struct file *file)
{
  return release_common(inode, file, vfdev_delete);
}

/* Interface procedure for reading data from a gadget */
static ssize_t read_common(struct file *file,
			   char *buffer,
			   size_t count,
			   loff_t *ppos,
			   unsigned char *bulk_in_buffer,
			   size_t bulk_in_size,
			   __u8 bulk_in_epaddr)
{
  struct usb_vdev_common *dev;
  int retval = 0;
  int bytes_read;

  dev = (struct usb_vdev_common *)file->private_data;
	
  /* do a blocking bulk read to get data from the device */
  retval = usb_bulk_msg(dev->udev,
			usb_rcvbulkpipe(dev->udev,
					bulk_in_epaddr),
			bulk_in_buffer,
			min(bulk_in_size, count),
			&bytes_read, timeout);

  /* if the read was successful, copy the data to userspace */
  // TODO: copy via DMA
  if (!retval) {
    if (copy_to_user(buffer, bulk_in_buffer, bytes_read)) {
      retval = -EFAULT;
    } else {
      retval = bytes_read;
    }
  }

  return retval;
}

/* Interface procedure for reading status data from a gadget */
static ssize_t status_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
  struct usb_vdev *dev;

  dev = (struct usb_vdev *)file->private_data;
  return read_common(file, buffer, count, ppos,
		     dev->bulk_status_in_buffer,
		     dev->bulk_status_in_size,
		     dev->bulk_status_in_epaddr);
}

/* Free a given URB */
static void free_urb(struct urb *urb)
{
  /* Free up the allocated buffer(s) */
  if (urb->transfer_buffer != NULL) {
    usb_buffer_free(urb->dev, urb->transfer_buffer_length, 
		    urb->transfer_buffer, urb->transfer_dma);
  }
  /* Release the reference to the URB */
  usb_free_urb(urb);
}

/* Handles a finished bulk write request */
static void vdev_write_bulk_callback(struct urb *urb)
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
	/* Send notifications */
	up(&dev->limit_sem);
	wake_up(&dev->cmd_wait);
	atomic_dec(&dev->cmds_sent);
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
	} else {
	  atomic_inc(&dev->cmds_sent);
	}
	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		return -ENOMEM;
	}

	buf = usb_buffer_alloc(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		return -ENOMEM;
	}
	// TODO: transfer via DMA
	if (copy_from_user(buf, user_buffer, writesize)) {
		return -EFAULT;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_epaddr),
			  buf, writesize, vdev_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval != 0) {
		err("%s - failed submitting write urb, error %d", __FUNCTION__, retval);
		if (urb != NULL) {
		  free_urb(urb);
		}
		up(&dev->limit_sem);
		return retval;
	}

	/* release our reference to this urb, the USB core will eventually free it entirely */
	usb_free_urb(urb);

	return writesize;
}

/* Add a given URB to the queue */
static int vfdev_urb_offer(struct usb_vfdev *dev, struct urb *urb)
{
  struct urb_entry *queue;
  struct urb_entry *entry;
  int rc;

  dbg("Turn down the FIFO device mutex");
  if (down_interruptible(&dev->mutex) == 0) {
    /* Make a new queue entry */
    dbg("Make a new read queue entry");
    entry = kmalloc(sizeof *entry, GFP_KERNEL);
    if (entry != NULL) {
      /* Offer the new entry to the queue */
      entry->urb = urb;
      entry->next = NULL;
      rc = 0;
    } else {
      err("Unable to allocate a read queue entry");
      rc = -ENOMEM;
    }

    /* Offer the entry to the queue */
    if (rc == 0) {
      if (dev->queue != NULL) {
	queue = dev->queue;
	while (queue->next != NULL) {
	  queue = queue->next;
	}
	queue->next = entry;
	dbg("Add new entry to the tail of the queue\n");
      } else {
	dbg("New entry is the queue head");
	dev->queue = entry;
      }
      /* Send notifications */
      dbg("Turn up the queue semaphore");
      up(&dev->queue_sem);
      dbg("Send wake-up notifications");
      wake_up(&dev->fifo_wait);
    }
    dbg("Turn up the FIFO device mutex");
    up(&dev->mutex);
    return rc;
  } else {
    dbg("Interrupted. Return the restart system value");
    return -ERESTARTSYS;
  }
}

/* Handles a finished read-ahead request */
static void vfdev_bulk_read_callback(struct urb *urb)
{
  struct usb_vfdev *dev;

  dbg("A FIFO bulk read URB finished");

  dev = (struct usb_vfdev *)urb->context;
  if (urb->status && 
      !(urb->status == -ENOENT || 
	urb->status == -ECONNRESET ||
	urb->status == -ESHUTDOWN)) {
    dbg("%s - nonzero write bulk status received: %d",
	__FUNCTION__, urb->status);
  } else {
    dbg("Offer the finished USB to the read queue");
    if (vfdev_urb_offer(dev, urb) != 0) {
      err("Unable to offer the urb. Free it up");
      free_urb(urb);
    }
  }
}

/* Enqueues a next read-request */
static int fifo_read_enqueue(struct usb_vfdev *dev)
{
  static struct urb *urb;
  void *buf;
  int rc;

  dbg("Turn down the read-limit semaphore");
  if ((rc = down_interruptible(&dev->limit_sem)) == 0) {
    /* Exit immediately if the read-ahead process has been terminated */
    if (! test_bit(RUNNING, &dev->read_ahead_flags)) {
      return 1;
    }
    /* Create an urb, and a buffer for it, and copy the data to the urb */
    urb = usb_alloc_urb(0, GFP_KERNEL);
    if (urb == NULL) {
      rc = -ENOMEM;
    }
    if (rc == 0) {
      buf = usb_buffer_alloc(dev->udev,
			     MAX_TRANSFER,
			     GFP_KERNEL,
			     &urb->transfer_dma);
      if (buf == NULL) {
	rc = -ENOMEM;
      }
    }
    if (rc == 0) {
      /* initialize the urb properly */
      usb_fill_bulk_urb(urb, dev->udev,
			usb_rcvbulkpipe(dev->udev, dev->bulk_in_epaddr),
			buf, MAX_TRANSFER, vfdev_bulk_read_callback,
			dev);
      urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

      if (fakeread) {
	/* Short-circuit the URB: put it directly into the read queue */
	urb->actual_length = urb->transfer_buffer_length;
	urb->status = 0;
	rc = vfdev_urb_offer(dev, urb);
      } else {
	/* Send the read request */
	if ((rc = usb_submit_urb(urb, GFP_KERNEL)) == 0) {
	  dbg("Submit a read-ahead URB: "
	      "addr = 0x%x, "
	      "size = %d b",
	      dev->bulk_in_epaddr,
	      urb->transfer_buffer_length);
	} else {
	  err("%s - failed submitting read urb, error %d",
	      __FUNCTION__,
	      rc);
	}
      }
    }

    if (rc != 0) {
      /* Cleanup on error */
      if (urb != NULL) {
	free_urb(urb);
      }
      up(&dev->limit_sem);
    }
    return rc;
  } else {
    dbg("Interrupted. Return the restart system value");
    return -ERESTARTSYS;
  }
}

/* Take one URB from the head of the queue if any */
static int vfdev_urb_try_take(struct usb_vfdev *dev,
			      struct urb **urb,
			      int down)
{
  struct urb_entry *next;
  int rc;

  dbg("Try to turn down the queue semaphore");
  if (down || down_trylock(&dev->queue_sem) == 0) {
    dbg("The queue semaphore has been turned down");
    dbg("Turn down the FIFO device mutex");
    if (down_interruptible(&dev->mutex) == 0) {
      if (dev->queue != NULL) {
	*urb = dev->queue->urb;
	next = dev->queue->next;
	kfree(dev->queue);
	dev->queue = next;
	dbg("Turn up the read-limit semaphore");
	/* Invite the read-ahead procedure to continue */
	up(&dev->limit_sem);
      } else {
	err("Error: semaphore is up but read queue is empty");
	*urb = NULL;
	rc = 1;
      }
      dbg("Turn up the FIFO device mutex");
      up(&dev->mutex);
      rc = 0;
    } else {
      *urb = NULL;
      dbg("Interrupted. Return the restart system value");
      rc = -ERESTARTSYS;
    }
  } else {
    dbg("Semaphore is down: read queue is empty");
    *urb = NULL;
    rc = 0;
  }

  return rc;
}

/* Takes one URB from the head of the queue */
static int vfdev_urb_take(struct usb_vfdev *dev,
			  struct urb **urb)
{
  int rc;
  int down;
  
  down = 0;
  /* Try to take an URB from the queue */
  dbg("Try to take an URB from the read queue");
  while ((rc = vfdev_urb_try_take(dev, urb, down)) == 0) {
    down = 0;
    if (*urb != NULL) {
      /* Exit with the taken URB */
      dbg("Successful. Return the taken URB");
      break;
    } else {
      /* Wait for the next available URB */
      dbg("Wait for the next available URB. Turn down the queue semaphore");
      if ((rc = down_interruptible(&dev->queue_sem)) == 0) {
	if (! test_bit(RUNNING, &dev->read_ahead_flags)) {
	  rc = -EINTR;
	  break;
	}
	down = 1;
      } else {	
	dbg("Interrupted. Return the restart system value");
	rc = -ERESTARTSYS;
	break;
      }
    }
  }
  return rc;
}

/* Leaves the current incomplete URB if any or a next one
 * from the queue */
static int vfdev_prepare_urb(struct usb_vfdev *dev)
{
  int rc;

  if (dev->current_urb) {
    dbg("Use the incomplete URB: %lu/%d",
	dev->current_offs,
	dev->current_urb->actual_length);
    rc = 0;
  } else {
    /* Take the next URB from the queue */
    rc = vfdev_urb_take(dev, &dev->current_urb);
    dev->current_offs = 0;
  }

  return rc;
}

/* Interface procedure for reading file data from a gadget */
static ssize_t fifo_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
  struct usb_vfdev *dev;
  size_t len;
  size_t rc;

  dev = (struct usb_vfdev *)file->private_data;

  dbg("Process a FIFO read request");

  rc = vfdev_prepare_urb(dev);

  if (rc == 0) {
    dbg("Copy taken URB to the userspace");
    len = min((unsigned long) count,
	      ((unsigned long) dev->current_urb->actual_length) \
	      - dev->current_offs);
    if ((rc = copy_to_user(buffer,
			   dev->current_urb->transfer_buffer \
			   + dev->current_offs,
			   len)) >= 0) {
      dev->current_offs += len;
      dbg("Return the actual bytes read: %d/%lu",
	  dev->current_urb->actual_length,
	  dev->current_offs);
      rc = len;
    }
    if (dev->current_offs == dev->current_urb->actual_length) {
      dbg("Free the URB");
      free_urb(dev->current_urb);
      dev->current_urb = NULL;
    }
  }

  /*if (rc == -ERESTARTSYS) {
    dbg("Read interrupted. Force return -EINTR code");
    rc = -EINTR;
  }*/

  return rc;
}

/* Releases the URB related to a given FIFO pipe buffer if any */
static void fifo_pipe_buf_release(struct pipe_inode_info *pipe,
				  struct pipe_buffer *buf)
{
  struct urb *urb;

  urb = (struct urb *)page_private(buf->page);
  dbg("Release a page");
  //  page_cache_release(buf->page);

  if (urb != NULL) {
    dbg("Free an URB all the data has been read from");
    free_urb(urb);
  }
}

/* Describes the FIFO pipe buffer operations */
static const struct pipe_buf_operations fifo_pipe_buf_ops = {
  .can_merge = 0,
  .map = generic_pipe_buf_map,
  .unmap = generic_pipe_buf_unmap,
  .confirm = generic_pipe_buf_confirm,
  .release = fifo_pipe_buf_release,
  .steal = generic_pipe_buf_steal,
  .get = generic_pipe_buf_get,
};

/* Releases the given page (currently do nothing) */
static void fifo_page_release(struct splice_pipe_desc *spd,
			      unsigned int i)
{
  dbg("Release a page #%d", i);
  page_cache_release(spd->pages[i]);
}

/* Splices the number of buffers to a given pipe */
static ssize_t fifo_splice_read(struct file *in,
				loff_t *ppos,
				struct pipe_inode_info *pipe,
				size_t count,
				unsigned int flags)
{
  struct usb_vfdev *dev;
  struct page *pages[PIPE_BUFFERS];
  struct partial_page partial[PIPE_BUFFERS];
  struct splice_pipe_desc spd = {
    .pages = pages,
    .nr_pages = 0,
    .partial = partial,
    .flags = flags,
    .ops = &fifo_pipe_buf_ops,
    .spd_release = fifo_page_release,
  };
  ssize_t ret;
  size_t mapped;
  int rc;

  dev = (struct usb_vfdev *)in->private_data;

  dbg("Process a FIFO splice-read request");

  /* Ignore the position -- splice the data as it comes */

  /* Splice the data to the pipe until the len bytes spliced */
  rc = 0;
  ret = 0;
  mapped = 0;
  while (rc == 0 && ret < count) {
    size_t spliced;

    /* Map up the pipe buffers */
    spd.nr_pages = 0;
    while (rc == 0 \
	   && mapped < count \
	   && spd.nr_pages < PIPE_BUFFERS) {
      struct page *page;
      unsigned long offs;
      unsigned long len;

      /* Get the URB and offset to transfer */
      if ((rc = vfdev_prepare_urb(dev)) == 0) {
	/* Add URB page to the pipe */
	page =
	  virt_to_page(dev->current_urb->transfer_buffer	\
		       + dev->current_offs);
	pages[spd.nr_pages] = page;
	/* Calculate the in-page offset of the buffer start */
	offs =
	  dev->current_urb->transfer_buffer		\
	  + dev->current_offs				\
	  - page_address(page);
	partial[spd.nr_pages].offset = offs;
	/* Calculate the data length on the page */
	len =
	  min((unsigned long) PAGE_SIZE - offs,
	      ((unsigned long) dev->current_urb->actual_length)	\
	      - dev->current_offs);
	partial[spd.nr_pages].len = len;
	/* Increment the pages counter */
	dbg("Add a page to the pipe");
	spd.nr_pages++;
	/* Shift the URB budder offset len bytes more */
	if ((dev->current_offs += len)			\
	    == dev->current_urb->actual_length) {
	  dbg("Whole URB buffer is mapped to the pipe");
	  /* Set the URB as a page private data to free it when the
	   * data is read */
	  set_page_private(page, (unsigned long) dev->current_urb);
	  dev->current_urb = NULL;
	} else {
	  dbg("%ld/%ld of the URB buffer is mapped to the pipe",
	      dev->current_offs,
	      (unsigned long) dev->current_urb->actual_length);
	  set_page_private(page, (unsigned long) NULL);
	}
	mapped += len;
      }
    }

    /* Splice the mapped buffers out */
    if (rc == 0) {
      if ((spliced = splice_to_pipe(pipe, &spd)) > 0) {
	dbg("%ld (more) bytes has been spliced", (unsigned long) spliced);
	ret += spliced;
      } else {
	if (spliced == 0 && (flags & SPLICE_F_NONBLOCK)) {
	  rc = -EAGAIN;
	  err("Signal to read again in the case of non-clocking reading");
	} else {
	  rc = spliced;
	  err("Unable to splice the mapped buffers to the pipe");
	}
      }
    }
  }

  /* In the case of an error return its code */
  if (rc != 0) {
    ret = rc;
  }

  return ret;
}

/* Returns a bit mask indicating the current non-blocking
 * operations that are available for the CS device */
static unsigned int vdev_poll(struct file *filp,
			      struct poll_table_struct *wait) 
{
  struct usb_vdev *dev;
  int rc;

  dev = (struct usb_vdev *)filp->private_data;

  rc = 0;
  if (atomic_read(&dev->cmds_sent) < maxwrites) {
    /* Indicate that the write limit isn't exceeded */
    rc |= (POLLOUT | POLLWRNORM);
  } else {
    /* Add the command wait-queue to the poll table */
    poll_wait(filp, &dev->cmd_wait, wait);
  }

  return rc;
}

/* Returns a bit mask indicating the current non-blocking
 * operations that are available for the FIFO device */
static unsigned int vfdev_poll(struct file *filp,
			       struct poll_table_struct *wait)
{
  struct usb_vfdev *dev;
  int rc;

  dev = (struct usb_vfdev *)filp->private_data;

  dbg("Poll the FIFO device");

  dbg("Turn down the FIFO device mutex");
  if (down_interruptible(&dev->mutex) == 0) { //TODO: use an R/W mutex
    rc = 0;
    if (dev->queue != NULL) {
      dbg("Report that some data is waiting in the read queue");
      /* Indicate that some data is ready for reading */
      rc |= (POLLIN | POLLRDNORM);
    } else {
      /* Add the FIFO wait-queue to the poll table */
      dbg("Add the FIFO wait-queue to the poll table");
      poll_wait(filp, &dev->fifo_wait, wait);
    }
    dbg("Turn down the FIFO device mutex");
    up(&dev->mutex);
    return rc;
  } else {
    dbg("Interrupted. Return the restart system value");
    return -ERESTARTSYS;
  }
}

/* Command-status I/O and class device file operations */
static struct file_operations vdev_fops = {
	.owner =	THIS_MODULE,
	.read =		status_read,
	.write =	cmd_write,
	.open =		vdev_open,
	.release =	vdev_release,
	.poll =         vdev_poll,
};

/* CS USB class driver info */
static struct usb_class_driver vdev_class = {
	.name =		"usbcons%d",
	.fops =		&vdev_fops,
	.minor_base =	192,
};

/* FIFO I/O and class device file operations */
static struct file_operations vfdev_fops = {
	.owner =	THIS_MODULE,
	.read =		fifo_read,
	.open =		vfdev_open,
	.release =	vfdev_release,
	.poll =         vfdev_poll,
	.splice_read =  fifo_splice_read,
};

/* FIFO USB class driver info */
static struct usb_class_driver vfdev_class = {
	.name =		"usbconsf%d",
	.fops =		&vfdev_fops,
	.minor_base =	192 + 0x10,
};

#define fill_in_ep(ep_prefix) \
  dev->ep_prefix##_size = le16_to_cpu(endpoint->wMaxPacketSize);	\
  dev->ep_prefix##_epaddr = endpoint->bEndpointAddress;			\
  dev->ep_prefix##_buffer = kmalloc(le16_to_cpu(endpoint->wMaxPacketSize), GFP_KERNEL);

/* USB probbing procedure for the CS device */
static int vdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_vdev *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	int retval = -ENODEV;

	dbg("CS driver is currently probbing for %d:%d\n",
	    id->idVendor,
	    id->idProduct);

	/* Check the interface number (0) */
	if (interface->cur_altsetting->desc.bInterfaceNumber != 0) {
	  return retval;
	}

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		err("Out of memory");
	        return -ENOMEM;
	}
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, maxwrites);
	atomic_set(&dev->cmds_sent, 0);
	init_waitqueue_head(&dev->cmd_wait);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
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
	    if (!dev->bulk_status_in_epaddr) {
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

	if (!dev->bulk_status_in_epaddr || !dev->bulk_out_epaddr) {
	  err("Could not find both bulk-in and bulk-out endpoints");
	  kref_put(&dev->kref, vdev_delete);
	  return -EINVAL;
	}

	if (!dev->bulk_status_in_buffer) {
	  err("Could not allocate status input buffer");
	  kref_put(&dev->kref, vdev_delete);
	  return -ENOMEM;
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &vdev_class);
	if (retval) {
		/* something prevented us from registering this driver */
		err("Not able to get a minor for the CS device.");
		usb_set_intfdata(interface, NULL);
		kref_put(&dev->kref, vdev_delete);
		return retval;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "USB Versatile device command-status intarface "
	         "is now attached to usbcons%d", interface->minor);
	return 0;
}

/* The read-ahead process procedure */
static int vfdev_read_ahead_loop(void *context)
{
  static struct usb_vfdev *dev;
  int rc;

  dev = (struct usb_vfdev *)context;
  if (dev == NULL) {
    err("Read ahead loop: context is NULL\n");
    return -EFAULT;
  }

  do {
    rc = fifo_read_enqueue(dev);
  } while (rc == 0);

  up(&dev->read_ahead_running);
  dbg("Read-ahead semaphore is up\n");

  dbg("Read ahead process finished (%d)", rc);
  complete_and_exit(&dev->read_ahead_exit, rc);

  return rc;
}

/* Starts the read-ahead loop */
static int vfdev_read_ahead_start(struct usb_vfdev *dev)
{
  int rc;

  dbg("Turn down the read-ahead semaphore\n");
  if (down_interruptible(&dev->read_ahead_running) == 0) {
    dbg("Set up the read-ahead thread");
    rc = kernel_thread(vfdev_read_ahead_loop,
		       dev,
		       (CLONE_VM | CLONE_FS | CLONE_FILES));
    if (rc >= 0) {
      dev->read_ahead_pid = rc;
      set_bit(RUNNING, &dev->read_ahead_flags);
      rc = 0;
    }

    if (rc == 0) {
      dbg("Read ahead thread pid: %d\n", dev->read_ahead_pid);
    }
  } else {
    rc = -ERESTARTSYS;
  }

  return rc;
}

/* Cleanups the read-ahead queue */
static int vfdev_read_ahead_cleanup(struct usb_vfdev *dev)
{
  struct urb *urb;
  int rc;

  rc = 0;
  dbg("Clean all of the URBS from the queue\n");
  do {
    rc |= vfdev_urb_try_take(dev, &urb, 0);
    if (urb != NULL) {
      free_urb(urb);
    }
  } while (urb == NULL);
  dbg("Done clean the URBs\n");

  return rc;
}

/* Interrupts (and terminates) the read-ahead loop */
static int vfdev_read_ahead_stop(struct usb_vfdev *dev)
{
  int rc;

  rc = 0;
  if (test_and_clear_bit(RUNNING, &dev->read_ahead_flags)) {
    vfdev_read_ahead_cleanup(dev);
  }
  dbg("Turn up the queue semaphore to interrupt possible waiting");
  up(&dev->queue_sem);
  dbg("Wait for the read-ahead process to finish\n");
  wait_for_completion(&dev->read_ahead_exit);

  return rc;
}

/* USB probbing procedure for the FIFO device */
static int vfdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_vfdev *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	int retval = -ENODEV;

	dbg("FIFO driver is currently probbing for %d:%d\n",
	    id->idVendor,
	    id->idProduct);

	/* Check the interface number (1) */
	if (interface->cur_altsetting->desc.bInterfaceNumber != 1) {
	  return retval;
	}

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		err("Out of memory");
	        return -ENOMEM;
	}
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, maxreads);
	sema_init(&dev->queue_sem, 0);
	sema_init(&dev->mutex, 1);
	sema_init(&dev->read_ahead_running, 1);
	init_completion(&dev->read_ahead_exit);
	init_waitqueue_head(&dev->fifo_wait);
	dev->queue = NULL;
	dev->current_urb = NULL;

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	dev->bulk_in_epaddr = 0;
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
	  endpoint = &iface_desc->endpoint[i].desc;

	  if ( ((endpoint->bEndpointAddress				\
		 & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)		\
	       && ((endpoint->bmAttributes				\
		    & USB_ENDPOINT_XFERTYPE_MASK) \
		   == USB_ENDPOINT_XFER_BULK)) {
	    /* we found a bulk in endpoint */
	    if (!dev->bulk_in_epaddr) {
	      fill_in_ep(bulk_in);
	    }
	  }
	}

	if (!dev->bulk_in_epaddr) {
	  err("Could not find both bulk-in and bulk-out endpoints");
	  kref_put(&dev->kref, vfdev_delete);
	  return -EINVAL;
	}

	if (!dev->bulk_in_buffer) {
	  err("Could not allocate file input buffer");
	  kref_put(&dev->kref, vfdev_delete);
	  return -ENOMEM;
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &vfdev_class);
	if (retval) {
		/* something prevented us from registering this driver */
		err("Not able to get a minor for the FIFO device.");
		usb_set_intfdata(interface, NULL);
		kref_put(&dev->kref, vfdev_delete);
		return retval;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "USB Versatile device FIFO intarface "
	         "is now attached to usbconsf%d", interface->minor);
	vfdev_read_ahead_start(dev);
	return retval;
}

/* Handles a USB device disconnect event */
static void disconnect_common(struct usb_interface *interface,
			      struct usb_class_driver *dev_class,
			      void (*delete)(struct kref *kref))
{
	struct usb_vdev_common *dev;
	int minor = interface->minor;

	/* prevent vdev_open() from racing vdev_disconnect() */
	lock_kernel();

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor number */
	usb_deregister_dev(interface, dev_class);

	unlock_kernel();

	/* decrement the usage count */
	kref_put(&dev->kref, delete);

	dev_info(&interface->dev,
		 "USB Versatile device #%d is now disconnected",
		 minor);
}

/* Handles a USB CS device disconnect event */
static void vdev_disconnect(struct usb_interface *interface)
{
  disconnect_common(interface, &vdev_class, vdev_delete);
}

/* Handles a USB FIFO device disconnect event */
static void vfdev_disconnect(struct usb_interface *interface)
{
  struct usb_vfdev *dev;

  dev = (struct usb_vfdev *)usb_get_intfdata(interface);
  vfdev_read_ahead_stop(dev);
  disconnect_common(interface, &vfdev_class, vfdev_delete);
}

/* CS device driver instance */
static struct usb_driver vdev_driver = {
	.name =		"vdevice",
	.probe =	vdev_probe,
	.disconnect =	vdev_disconnect,
	.id_table =	vdev_table,
};

/* FIFO device driver instance */
static struct usb_driver vfdev_driver = {
	.name =		"vdevice-fifo",
	.probe =	vfdev_probe,
	.disconnect =	vfdev_disconnect,
	.id_table =	vdev_table,
};

/* Module initialization procedure */
static int __init usb_vdev_init(void)
{
	int result;

	/* Adjust the minor base number */
	vfdev_class.minor_base = vdev_class.minor_base + 0x10;
	/* Register the CS driver with the USB subsystem */
	if ((result = usb_register(&vdev_driver)) == 0) {
	  info("CS device USB driver registered for "
	       "vendor 0x%x, product 0x%x",
	       vdev_driver.id_table[0].idVendor,
	       vdev_driver.id_table[0].idProduct);
	} else {
	  err("CS device registration failed. Error number %d", result);
	}
	if (result == 0) {
	  /* Register the FIFO driver with the USB subsystem */
	  if ((result = usb_register(&vfdev_driver)) == 0) {
	    info("FIFO device USB driver registered for "
		 "vendor 0x%x, product 0x%x",
		 vdev_driver.id_table[0].idVendor,
		 vdev_driver.id_table[0].idProduct);
	  } else {
	    err("FIFO device registration failed. Error number %d", result);
	  }
	}

	return result;
}

/* Module exit procedure */
static void __exit usb_vdev_exit(void)
{
	/* Unregister the CS driver with the USB subsystem */
	usb_deregister(&vdev_driver);
	/* Unregister the FIFO driver with the USB subsystem */
	usb_deregister(&vfdev_driver);
}

module_init (usb_vdev_init);
module_exit (usb_vdev_exit);
