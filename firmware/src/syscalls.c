/* No file I/O on this target. These stubs only satisfy newlib's reentrant wrappers, which
 * newlib 4.4 links in (and then garbage-collects) even when stdio is unused. */
#include <errno.h>

int _close(int fd) { (void)fd; errno = ENOSYS; return -1; }
int _lseek(int fd, int off, int whence) { (void)fd; (void)off; (void)whence; errno = ENOSYS; return -1; }
int _read(int fd, char *buf, int len) { (void)fd; (void)buf; (void)len; errno = ENOSYS; return -1; }
int _write(int fd, const char *buf, int len) { (void)fd; (void)buf; (void)len; errno = ENOSYS; return -1; }
