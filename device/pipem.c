
#include <linux/pipe_fs_i.h>
#include <linux/pagemap.h>

/* Drop the inode semaphore and wait for a pipe event, atomically */
void pipe_wait(struct pipe_inode_info *pipe)
{
  DEFINE_WAIT(wait);

  /*
   * Pipes are system-local resources, so sleeping on them
   * is considered a noninteractive wait:
   */
  prepare_to_wait(&pipe->wait, &wait, TASK_INTERRUPTIBLE);
  pipe_unlock(pipe);
  schedule();
  finish_wait(&pipe->wait, &wait);
  pipe_lock(pipe);
}

/**
 * splice_to_pipe - fill passed data into a pipe
 * @pipe:	pipe to fill
 * @spd:	data to fill
 *
 * Description:
 *    @spd contains a map of pages and len/offset tuples, along with
 *    the struct pipe_buf_operations associated with these pages. This
 *    function will link that data to the pipe.
 *
 */
ssize_t splice_to_pipe(struct pipe_inode_info *pipe,
		       struct splice_pipe_desc *spd)
{
  unsigned int spd_pages = spd->nr_pages;
  int ret, do_wakeup, page_nr;

  ret = 0;
  do_wakeup = 0;
  page_nr = 0;

  dbg("Lock the pipe");
  pipe_lock(pipe);

  for (;;) {
    if (!pipe->readers) {
      dbg("Send SIGPIPE signal");
      send_sig(SIGPIPE, current, 0);
      if (!ret)
	ret = -EPIPE;
      break;
    }

    if (pipe->nrbufs < PIPE_BUFFERS) {
      int newbuf = (pipe->curbuf + pipe->nrbufs) & (PIPE_BUFFERS - 1);
      struct pipe_buffer *buf = pipe->bufs + newbuf;

      dbg("Form new pipe buffer");
      buf->page = spd->pages[page_nr];
      buf->offset = spd->partial[page_nr].offset;
      buf->len = spd->partial[page_nr].len;
      buf->private = spd->partial[page_nr].private;
      buf->ops = spd->ops;
      if (spd->flags & SPLICE_F_GIFT)
	buf->flags |= PIPE_BUF_FLAG_GIFT;

      pipe->nrbufs++;
      page_nr++;
      ret += buf->len;

      if (pipe->inode)
	do_wakeup = 1;

      if (!--spd->nr_pages)
	break;
      if (pipe->nrbufs < PIPE_BUFFERS)
	continue;

      break;
    }

    if (spd->flags & SPLICE_F_NONBLOCK) {
      if (!ret)
	ret = -EAGAIN;
      break;
    }

    if (signal_pending(current)) {
      if (!ret)
	ret = -ERESTARTSYS;
      break;
    }

    if (do_wakeup) {
      dbg("Send notification to the readers");
      smp_mb();
      if (waitqueue_active(&pipe->wait))
	wake_up_interruptible_sync(&pipe->wait);
      kill_fasync(&pipe->fasync_readers, SIGIO, POLL_IN);
      do_wakeup = 0;
    }

    dbg("Wait on the pipe");
    pipe->waiting_writers++;
    pipe_wait(pipe);
    pipe->waiting_writers--;
  }

  dbg("Unlock the pipe");
  pipe_unlock(pipe);

  if (do_wakeup) {
    dbg("Send notification to the readers");
    smp_mb();
    if (waitqueue_active(&pipe->wait))
      wake_up_interruptible(&pipe->wait);
    kill_fasync(&pipe->fasync_readers, SIGIO, POLL_IN);
  }

  while (page_nr < spd_pages)
    spd->spd_release(spd, page_nr++);

  return ret;
}

/**
 * generic_pipe_buf_get - get a reference to a &struct pipe_buffer
 * @pipe:	the pipe that the buffer belongs to
 * @buf:	the buffer to get a reference to
 *
 * Description:
 *	This function grabs an extra reference to @buf. It's used in
 *	in the tee() system call, when we duplicate the buffers in one
 *	pipe into another.
 */
void generic_pipe_buf_get(struct pipe_inode_info *pipe,
			  struct pipe_buffer *buf)
{
  dbg("Get a page from the cache");
  page_cache_get(buf->page);
}

/**
 * generic_pipe_buf_map - virtually map a pipe buffer
 * @pipe:	the pipe that the buffer belongs to
 * @buf:	the buffer that should be mapped
 * @atomic:	whether to use an atomic map
 *
 * Description:
 *	This function returns a kernel virtual address mapping for the
 *	pipe_buffer passed in @buf. If @atomic is set, an atomic map is provided
 *	and the caller has to be careful not to fault before calling
 *	the unmap function.
 *
 *	Note that this function occupies KM_USER0 if @atomic != 0.
 */
void *generic_pipe_buf_map(struct pipe_inode_info *pipe,
			   struct pipe_buffer *buf, int atomic)
{
  if (atomic) {
    buf->flags |= PIPE_BUF_FLAG_ATOMIC;
    dbg("Atomic map a page");
    return kmap_atomic(buf->page, KM_USER0);
  }

  dbg("Map a page");
  return kmap(buf->page);
}

/**
 * generic_pipe_buf_unmap - unmap a previously mapped pipe buffer
 * @pipe:	the pipe that the buffer belongs to
 * @buf:	the buffer that should be unmapped
 * @map_data:	the data that the mapping function returned
 *
 * Description:
 *	This function undoes the mapping that ->map() provided.
 */
void generic_pipe_buf_unmap(struct pipe_inode_info *pipe,
			    struct pipe_buffer *buf, void *map_data)
{
  if (buf->flags & PIPE_BUF_FLAG_ATOMIC) {
    buf->flags &= ~PIPE_BUF_FLAG_ATOMIC;
    dbg("Atomic unmap a page");
    kunmap_atomic(map_data, KM_USER0);
  } else {
    dbg("Un map a page");
    kunmap(buf->page);
  }
}

/**
 * generic_pipe_buf_steal - attempt to take ownership of a &pipe_buffer
 * @pipe:	the pipe that the buffer belongs to
 * @buf:	the buffer to attempt to steal
 *
 * Description:
 *	This function attempts to steal the &struct page attached to
 *	@buf. If successful, this function returns 0 and returns with
 *	the page locked. The caller may then reuse the page for whatever
 *	he wishes; the typical use is insertion into a different file
 *	page cache.
 */
int generic_pipe_buf_steal(struct pipe_inode_info *pipe,
			   struct pipe_buffer *buf)
{
  struct page *page = buf->page;

  /*
   * A reference of one is golden, that means that the owner of this
   * page is the only one holding a reference to it. lock the page
   * and return OK.
   */
  if (page_count(page) == 1) {
    dbg("Lock a page");
    lock_page(page);
    return 0;
  }

  return 1;
}

/**
 * generic_pipe_buf_confirm - verify contents of the pipe buffer
 * @info:	the pipe that the buffer belongs to
 * @buf:	the buffer to confirm
 *
 * Description:
 *	This function does nothing, because the generic pipe code uses
 *	pages that are always good when inserted into the pipe.
 */
int generic_pipe_buf_confirm(struct pipe_inode_info *info,
			     struct pipe_buffer *buf)
{
  return 0;
}
