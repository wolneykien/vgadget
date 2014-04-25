# USB communication line system

Here are two Linux kernel modules: `g_vgadget` and `vdevice`.
The first one is a USB gadget (slave) module, the second one
is the complementary USB device module (driver). Together they
let two computers be connected via USB bus as by a null-modem.
The communication line can be used in the two modes: *command* mode
and *data transfer* mode.

Arbitrary commands/replies are read and written via the `/dev/usbconsS`
device-file on the slave and `/dev/usbcons<n>` on the host.

Data-transfer is currently one-way: from the slave to the host. On the
slave arbitrary data is written to the `/dev/usbconsF` device-file
and is read on the host from the `/dev/usbconsf<n>`. Highest transfer
rates can be reached by the use of `splice_read` and `splice_write`
calls with the device files. That way, memory pages are transfered
directly via DMA.
