/*
 * Phoenix-RTOS
 *
 * uACPI kernel API implementation
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/io.h>
#include <sys/interrupt.h>

#include <phoenix/arch/ia32/ia32.h>

#include <uacpi/kernel_api.h>

#define COL_RED    "\033[1;31m"
#define COL_CYAN   "\033[1;36m"
#define COL_YELLOW "\033[1;33m"
#define COL_NORMAL "\033[0m"

#define LOG_TAG "uacpi: "

/* clang-format off */
#define log_debug(fmt, ...)     do { if (1) printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#define log_info(fmt, ...)      do { if (0) printf(COL_CYAN LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_warn(fmt, ...)      do { if (1) printf(COL_YELLOW LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...)     do { if (1) printf(COL_RED LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)

#define unimplemented()         do { log_warn("%s unimplemented!", __func__); exit(1); } while (0)
/* clang-format on */


uacpi_status uacpi_kernel_get_rsdp(uacpi_phys_addr *out_rsdp_address)
{
	int ret;
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_acpi,
		.acpi = { .var = acpi_rsdpAddr }
	};

	ret = platformctl(&pctl);
	if (ret < 0) {
		log_error("pctl_acpi failed: %d", ret);
		return UACPI_STATUS_INTERNAL_ERROR;
	}

	*out_rsdp_address = pctl.acpi.value;

	return UACPI_STATUS_OK;
}


uacpi_status uacpi_kernel_raw_memory_read(
		uacpi_phys_addr address, uacpi_u8 byte_width, uacpi_u64 *out_value) { unimplemented(); }


uacpi_status uacpi_kernel_raw_memory_write(
		uacpi_phys_addr address, uacpi_u8 byte_width, uacpi_u64 in_value) { unimplemented(); }


uacpi_status uacpi_kernel_raw_io_read(
		uacpi_io_addr address, uacpi_u8 byte_width, uacpi_u64 *out_value)
{
	switch (byte_width) {
		case 1:
			*out_value = inb((void *)address);
			break;
		case 2:
			*out_value = inw((void *)address);
			break;
		case 4:
			*out_value = inl((void *)address);
			break;
		default:
			return UACPI_STATUS_INVALID_ARGUMENT;
	}

	return UACPI_STATUS_OK;
}


uacpi_status uacpi_kernel_raw_io_write(
		uacpi_io_addr address, uacpi_u8 byte_width, uacpi_u64 in_value)
{
	switch (byte_width) {
		case 1:
			outb((void *)address, (unsigned char)in_value);
			break;
		case 2:
			outw((void *)address, (unsigned short)in_value);
			break;
		case 4:
			outl((void *)address, (unsigned int)in_value);
			break;
		default:
			return UACPI_STATUS_INVALID_ARGUMENT;
	}

	return UACPI_STATUS_OK;
}


uacpi_status uacpi_kernel_pci_read(
		uacpi_pci_address *address, uacpi_size offset,
		uacpi_u8 byte_width, uacpi_u64 *value)
{
	return uacpi_kernel_io_read(0, offset, byte_width, value);
}


uacpi_status uacpi_kernel_pci_write(
		uacpi_pci_address *address, uacpi_size offset,
		uacpi_u8 byte_width, uacpi_u64 value)
{
	return uacpi_kernel_io_write(0, offset, byte_width, value);
}


uacpi_status uacpi_kernel_io_map(
		uacpi_io_addr base, uacpi_size len, uacpi_handle *out_handle)
{
	*out_handle = (uacpi_handle)base;
	return UACPI_STATUS_OK;
}


void uacpi_kernel_io_unmap(uacpi_handle handle) { unimplemented(); }


uacpi_status uacpi_kernel_io_read(
		uacpi_handle handle, uacpi_size offset,
		uacpi_u8 byte_width, uacpi_u64 *value)
{
	return uacpi_kernel_raw_io_read(
			(uacpi_io_addr)handle + offset, byte_width, value);
}


uacpi_status uacpi_kernel_io_write(
		uacpi_handle handle, uacpi_size offset,
		uacpi_u8 byte_width, uacpi_u64 value)
{
	return uacpi_kernel_raw_io_write(
			(uacpi_io_addr)handle + offset, byte_width, value);
}


void *uacpi_kernel_map(uacpi_phys_addr addr, uacpi_size len)
{

	off_t offs;
	unsigned int memsz;
	void *mem;

	offs = addr % _PAGE_SIZE;
	memsz = (len + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);

	mem = mmap(NULL, memsz, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, addr - offs);
	if (mem == MAP_FAILED) {
		return UACPI_NULL;
	}

	return (void *)((char *)mem + offs);
}


void uacpi_kernel_unmap(void *addr, uacpi_size len)
{
	munmap(addr, len);
}


void *uacpi_kernel_alloc(uacpi_size size)
{
	return malloc(size);
}


void *uacpi_kernel_calloc(uacpi_size count, uacpi_size size)
{
	return calloc(count, size);
}


void uacpi_kernel_free(void *mem)
{
	free(mem);
}


void uacpi_kernel_log(uacpi_log_level level, const uacpi_char *msg)
{
	switch (level) {
		case UACPI_LOG_DEBUG:
		case UACPI_LOG_TRACE:
			log_debug("%s", msg);
			break;
		case UACPI_LOG_INFO:
			log_info("%s", msg);
			break;
		case UACPI_LOG_WARN:
			log_warn("%s", msg);
			break;
		case UACPI_LOG_ERROR:
			log_error("%s", msg);
			break;
	}
}


uacpi_u64 uacpi_kernel_get_nanoseconds_since_boot(void)
{
	struct timespec tp;
	int rv;

	rv = clock_gettime(CLOCK_MONOTONIC_RAW, &tp);
	if (rv < 0) {
		log_error("failed to get time");
		return UACPI_STATUS_INTERNAL_ERROR;
	}

	return SECS_TO_USECS_T(tp.tv_sec) + tp.tv_nsec;
}


void uacpi_kernel_stall(uacpi_u8 usec)
{
	struct timespec ts[2];
	int diff;

	clock_gettime(CLOCK_MONOTONIC_RAW, &ts[0]);
	for (;;) {
		clock_gettime(CLOCK_MONOTONIC_RAW, &ts[1]);
		diff = (SECS_TO_USECS_T(ts[1].tv_sec) + SECS_TO_USECS_T(ts[0].tv_sec)) + (ts[1].tv_nsec - ts[0].tv_nsec);
		if (diff >= usec) {
			break;
		}
	}
}


void uacpi_kernel_sleep(uacpi_u64 msec)
{
	usleep(msec * 1000);
}


uacpi_handle uacpi_kernel_create_mutex(void)
{
	int ret;
	handle_t mutex;

	ret = mutexCreate(&mutex);
	if (ret < 0) {
		return UACPI_NULL;
	}

	return (uacpi_handle)mutex;
}

void uacpi_kernel_free_mutex(uacpi_handle handle)
{
	resourceDestroy((handle_t)handle);
}


uacpi_handle uacpi_kernel_create_event(void)
{
	int ret;
	semaphore_t *sem = malloc(sizeof(semaphore_t));

	ret = semaphoreCreate(sem, 0);
	if (ret < 0) {
		return UACPI_NULL;
	}

	return (uacpi_handle)sem;
}


void uacpi_kernel_free_event(uacpi_handle handle)
{

	semaphoreDone((semaphore_t *)handle);
}


uacpi_thread_id uacpi_kernel_get_thread_id(void)
{
	return (uacpi_thread_id)gettid();
}


uacpi_status uacpi_kernel_acquire_mutex(uacpi_handle handle, uacpi_u16 timeout)
{
	handle_t mutex;
	if (handle == NULL) {
		return UACPI_STATUS_INVALID_ARGUMENT;
	}

	mutex = (handle_t)handle;
	mutexLock(mutex);

	/* TODO handle timeout? */

	return UACPI_STATUS_OK;
}


void uacpi_kernel_release_mutex(uacpi_handle handle)
{
	handle_t mutex;
	if (handle == NULL) {
		return;
	}

	mutex = (handle_t)handle;
	mutexUnlock(mutex);
}


uacpi_bool uacpi_kernel_wait_for_event(uacpi_handle handle, uacpi_u16 timeout)
{
	int err;

	if (timeout == 0xFFFF) {
		timeout = 0;
	}

	err = semaphoreDown((semaphore_t *)handle, timeout);
	if (err < 0) {
		return UACPI_FALSE;
	}

	return UACPI_TRUE;
}


void uacpi_kernel_signal_event(uacpi_handle handle)
{
	semaphoreUp((semaphore_t *)handle);
}


void uacpi_kernel_reset_event(uacpi_handle handle)
{
	semaphore_t *sem = (semaphore_t *)handle;
	semaphoreDone(sem);
	semaphoreCreate(sem, 0);
}


uacpi_status uacpi_kernel_handle_firmware_request(uacpi_firmware_request *req)
{
	switch (req->type) {
		case UACPI_FIRMWARE_REQUEST_TYPE_BREAKPOINT:
			log_info("Ignoring breakpoint");
			break;
		case UACPI_FIRMWARE_REQUEST_TYPE_FATAL:
			log_info("Fatal firmware error: type: %x code: %x arg: %llx", (int)req->fatal.type, req->fatal.code, req->fatal.arg);
			break;
	}

	return UACPI_STATUS_OK;
}


uacpi_status uacpi_kernel_install_interrupt_handler(
		uacpi_u32 irq, uacpi_interrupt_handler handler, uacpi_handle ctx,
		uacpi_handle *out_irq_handle)
{
	/* FIXME noop for now - passed handlers are doing unsafe reads leading to PFs during interrupts
	 * (possibly a bug in uACPI). Since for now we use ACPI for getting the IRQ
	 * routing, we don't care about ACPI events for now */
	return UACPI_STATUS_OK;

	handle_t cond, handle;
	int ret;

	ret = condCreate(&cond);
	if (ret < 0) {
		return UACPI_STATUS_OUT_OF_MEMORY;
	}

	ret = interrupt(irq, handler, ctx, cond, &handle);
	if (ret < 0) {
		log_error("failed to attach interrupt handler");
		return UACPI_STATUS_INTERNAL_ERROR;
	}

	*out_irq_handle = (void *)handle;

	return UACPI_STATUS_OK;
}


uacpi_status uacpi_kernel_uninstall_interrupt_handler(
		uacpi_interrupt_handler handler, uacpi_handle irq_handle)
{
	return UACPI_STATUS_OK;
}


uacpi_handle uacpi_kernel_create_spinlock(void)
{
	return uacpi_kernel_create_mutex();
}


void uacpi_kernel_free_spinlock(uacpi_handle handle)
{
	return uacpi_kernel_free_mutex(handle);
}


uacpi_cpu_flags uacpi_kernel_lock_spinlock(uacpi_handle handle)
{
	uacpi_kernel_acquire_mutex(handle, 0xFFFF);
	return 0;
}


void uacpi_kernel_unlock_spinlock(uacpi_handle handle, uacpi_cpu_flags flags)
{
	uacpi_kernel_release_mutex(handle);
}


uacpi_status uacpi_kernel_schedule_work(
		uacpi_work_type type, uacpi_work_handler _handler, uacpi_handle ctx)
{
	_handler(ctx);
	return UACPI_STATUS_OK;
}


uacpi_status uacpi_kernel_wait_for_work_completion(void)
{
	return UACPI_STATUS_OK;
}
