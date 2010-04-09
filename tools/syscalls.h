#include <asm/unistd.h>

#ifndef SPLICE_F_MOVE
#define SPLICE_F_MOVE 1
#endif

#ifndef __NR_splice
#error "__NR_splice is undefined"
#endif

ssize_t splice(int __fdin, __off64_t *__offin, int __fdout,
	       __off64_t *__offout, size_t __len,
	       unsigned int __flags)
{

return syscall(__NR_splice, __fdin, __offin, __fdout,
	       __offout, __len, __flags);
}
