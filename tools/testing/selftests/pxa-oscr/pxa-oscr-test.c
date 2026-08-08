// SPDX-License-Identifier: GPL-2.0
#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/pxa-oscr.h>
#include <linux/reboot.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/reboot.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define DEVICE_PATH "/dev/pxa-oscr"
#define BENCH_SAMPLES 31
#define BENCH_ITERATIONS 50000

static bool init_process;
static unsigned int failures;

#define CHECK(condition, format, ...) do { \
	if (!(condition)) { \
		fprintf(stderr, "not ok - " format "\n", ##__VA_ARGS__); \
		failures++; \
	} else { \
		printf("ok - " format "\n", ##__VA_ARGS__); \
	} \
} while (0)

static uint64_t timespec_ns(const struct timespec *value)
{
	return (uint64_t)value->tv_sec * 1000000000ULL + value->tv_nsec;
}

static int raw_time(struct timespec *value)
{
	return clock_gettime(CLOCK_MONOTONIC_RAW, value);
}

static inline __attribute__((always_inline))
uint32_t oscr_read(const volatile uint32_t *counter)
{
	uint32_t value = *counter;

	asm volatile("" ::: "memory");
	return value;
}

/* Kept out of line so objdump can prove that the hot path is one MMIO load. */
__attribute__((noinline, used))
uint32_t pxa_oscr_hotpath_probe(const volatile uint32_t *counter)
{
	uint32_t value = *counter;

	asm volatile("" ::: "memory");
	return value;
}

static int double_compare(const void *left, const void *right)
{
	double a = *(const double *)left;
	double b = *(const double *)right;

	return (a > b) - (a < b);
}

static bool reserved_is_zero(const uint64_t *reserved, size_t count)
{
	size_t index;

	for (index = 0; index < count; index++)
		if (reserved[index])
			return false;
	return true;
}

static void benchmark(const volatile uint32_t *counter)
{
	double mapped[BENCH_SAMPLES];
	double monotonic[BENCH_SAMPLES];
	struct timespec before, after, now;
	volatile uint64_t sink = 0;
	unsigned int sample, iteration;

	for (sample = 0; sample < BENCH_SAMPLES; sample++) {
		raw_time(&before);
		for (iteration = 0; iteration < BENCH_ITERATIONS; iteration++)
			sink += oscr_read(counter);
		raw_time(&after);
		mapped[sample] = (double)(timespec_ns(&after) -
					 timespec_ns(&before)) / BENCH_ITERATIONS;
	}

	for (sample = 0; sample < BENCH_SAMPLES; sample++) {
		raw_time(&before);
		for (iteration = 0; iteration < BENCH_ITERATIONS; iteration++) {
			if (clock_gettime(CLOCK_MONOTONIC, &now)) {
				perror("clock_gettime(CLOCK_MONOTONIC)");
				failures++;
				return;
			}
			sink += now.tv_nsec;
		}
		raw_time(&after);
		monotonic[sample] = (double)(timespec_ns(&after) -
					    timespec_ns(&before)) /
					    BENCH_ITERATIONS;
	}

	qsort(mapped, BENCH_SAMPLES, sizeof(mapped[0]), double_compare);
	qsort(monotonic, BENCH_SAMPLES, sizeof(monotonic[0]), double_compare);
	printf("BENCH iterations=%u samples=%u sink=%" PRIu64 "\n",
	       BENCH_ITERATIONS, BENCH_SAMPLES, sink);
	printf("BENCH mapped_oscr median_ns=%.3f p95_ns=%.3f throughput_per_s=%.0f\n",
	       mapped[BENCH_SAMPLES / 2], mapped[(BENCH_SAMPLES * 95) / 100],
	       1000000000.0 / mapped[BENCH_SAMPLES / 2]);
	printf("BENCH clock_gettime_monotonic median_ns=%.3f p95_ns=%.3f throughput_per_s=%.0f\n",
	       monotonic[BENCH_SAMPLES / 2],
	       monotonic[(BENCH_SAMPLES * 95) / 100],
	       1000000000.0 / monotonic[BENCH_SAMPLES / 2]);
	printf("BENCH median_speedup=%.2fx\n",
	       monotonic[BENCH_SAMPLES / 2] / mapped[BENCH_SAMPLES / 2]);
}

static void finish(int status)
{
	printf("PXA_OSCR_TEST_RESULT=%s failures=%u\n",
	       status ? "FAIL" : "PASS", failures);
	fflush(NULL);

	if (init_process) {
		sync();
		reboot(LINUX_REBOOT_CMD_POWER_OFF);
	}

	exit(status);
}

int main(int argc, char **argv)
{
	const uint32_t required_flags =
		PXA_OSCR_INFO_F_MMAP_READ_ONLY |
		PXA_OSCR_INFO_F_COUNTER_WRAPS |
		PXA_OSCR_INFO_F_SNAPSHOT |
		PXA_OSCR_INFO_F_SNAPSHOT_MONOTONIC_RAW |
		PXA_OSCR_INFO_F_REGISTER_READS_SAFE;
	struct pxa_oscr_snapshot snapshot;
	struct pxa_oscr_info info;
	struct timespec raw_before, raw_after, delay = { .tv_nsec = 250000000 };
	volatile uint32_t *counter;
	struct stat device_stat;
	void *mapping, *bad_mapping;
	uint64_t elapsed_ns;
	uint32_t old, current, delta;
	double measured_rate, error;
	bool require_device = false;
	unsigned int index;
	int fd, writable_fd;

	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
	init_process = getpid() == 1;
	if (init_process) {
		require_device = true;
		if (mount("devtmpfs", "/dev", "devtmpfs", 0, NULL) &&
		    errno != EBUSY)
			perror("mount devtmpfs");
	}
	for (index = 1; index < (unsigned int)argc; index++)
		if (!strcmp(argv[index], "--require-device"))
			require_device = true;

	fd = open(DEVICE_PATH, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		if (errno == ENOENT && !require_device) {
			printf("PXA_OSCR_TEST_RESULT=SKIP device=%s\n", DEVICE_PATH);
			return 4;
		}
		perror("open " DEVICE_PATH);
		failures++;
		finish(1);
	}
	CHECK(fstat(fd, &device_stat) == 0, "device exists");
	CHECK(!(device_stat.st_mode & 0222), "device mode has no write bits");

	writable_fd = open(DEVICE_PATH, O_RDWR | O_CLOEXEC);
	CHECK(writable_fd < 0 && (errno == EPERM || errno == EACCES),
	      "writable open is denied");
	if (writable_fd >= 0)
		close(writable_fd);

	memset(&info, 0xa5, sizeof(info));
	CHECK(ioctl(fd, PXA_OSCR_GET_INFO, &info) == 0, "GET_INFO succeeds");
	CHECK(info.struct_size == PXA_OSCR_INFO_SIZE_V1,
	      "GET_INFO size is %u", info.struct_size);
	CHECK(info.version == PXA_OSCR_ABI_VERSION,
	      "GET_INFO version is %u", info.version);
	CHECK(info.rate_hz > 0 && info.rate_hz < 1000000000ULL,
	      "registered rate is %" PRIu64 " Hz", (uint64_t)info.rate_hz);
	CHECK(info.counter_bits == 32, "counter width is %u", info.counter_bits);
	CHECK(info.mmap_page_size == (uint32_t)sysconf(_SC_PAGESIZE),
	      "mmap size is one page (%u bytes)", info.mmap_page_size);
	CHECK(!(info.mmap_oscr_offset & 3) &&
	      info.mmap_oscr_offset + sizeof(uint32_t) <= info.mmap_page_size,
	      "OSCR mmap offset is coherent (0x%x)", info.mmap_oscr_offset);
	CHECK((info.flags & required_flags) == required_flags,
	      "GET_INFO flags are coherent (0x%x)", info.flags);
	if (!(info.flags & PXA_OSCR_INFO_F_SUSPEND_GENERATION))
		CHECK(!info.generation,
		      "unsupported suspend generation is zero");
	CHECK(!info.reserved0 && reserved_is_zero(info.reserved, 3),
	      "GET_INFO reserved fields are zero");

	bad_mapping = mmap(NULL, info.mmap_page_size, PROT_READ | PROT_WRITE,
			   MAP_PRIVATE, fd, 0);
	CHECK(bad_mapping == MAP_FAILED && (errno == EPERM || errno == EACCES),
	      "writable mmap is denied");
	if (bad_mapping != MAP_FAILED)
		munmap(bad_mapping, info.mmap_page_size);

	mapping = mmap(NULL, info.mmap_page_size, PROT_READ, MAP_SHARED, fd, 0);
	if (mapping == MAP_FAILED) {
		perror("read-only mmap");
		failures++;
		finish(1);
	}
	counter = (volatile uint32_t *)((char *)mapping +
					      info.mmap_oscr_offset);
	CHECK(mprotect(mapping, info.mmap_page_size,
		       PROT_READ | PROT_WRITE) < 0 &&
	      (errno == EACCES || errno == EPERM),
	      "mprotect write upgrade is denied");
	CHECK(mprotect(mapping, info.mmap_page_size,
		       PROT_READ | PROT_EXEC) < 0 &&
	      (errno == EACCES || errno == EPERM),
	      "mprotect execute upgrade is denied");

	old = oscr_read(counter);
	delta = 0;
	for (index = 0; index < 100000; index++) {
		current = oscr_read(counter);
		if ((uint32_t)(current - old) > 0x7fffffffU) {
			failures++;
			fprintf(stderr, "not ok - counter moved backwards modulo 32 bits\n");
			break;
		}
		delta |= current - old;
		old = current;
	}
	CHECK(index == 100000 && delta,
	      "OSCR increments monotonically modulo 32 bits");

	raw_time(&raw_before);
	old = oscr_read(counter);
	while (nanosleep(&delay, &delay) && errno == EINTR)
		;
	current = oscr_read(counter);
	raw_time(&raw_after);
	elapsed_ns = timespec_ns(&raw_after) - timespec_ns(&raw_before);
	measured_rate = (double)(uint32_t)(current - old) * 1000000000.0 /
			elapsed_ns;
	error = measured_rate > info.rate_hz ?
		(measured_rate - info.rate_hz) / info.rate_hz :
		(info.rate_hz - measured_rate) / info.rate_hz;
	CHECK(error < 0.20,
	      "measured rate %.0f Hz is plausible against registered %" PRIu64
	      " Hz (error %.2f%%)", measured_rate, (uint64_t)info.rate_hz,
	      error * 100.0);

	memset(&snapshot, 0xa5, sizeof(snapshot));
	raw_time(&raw_before);
	CHECK(ioctl(fd, PXA_OSCR_GET_SNAPSHOT, &snapshot) == 0,
	      "GET_SNAPSHOT succeeds");
	raw_time(&raw_after);
	CHECK(snapshot.struct_size == PXA_OSCR_SNAPSHOT_SIZE_V1 &&
	      snapshot.version == PXA_OSCR_ABI_VERSION,
	      "snapshot size/version are coherent");
	if (info.flags & PXA_OSCR_INFO_F_SUSPEND_GENERATION)
		CHECK(snapshot.generation_before == snapshot.generation_after,
		      "snapshot generation is stable (%u)",
		      snapshot.generation_before);
	else
		CHECK(!snapshot.generation_before && !snapshot.generation_after,
		      "unsupported snapshot generations are zero");
	CHECK(snapshot.monotonic_raw_ns >= timespec_ns(&raw_before) &&
	      snapshot.monotonic_raw_ns <= timespec_ns(&raw_after),
	      "snapshot raw timestamp is bracketed by userspace raw time");
	CHECK((uint32_t)(snapshot.oscr_after - snapshot.oscr_before) <
	      info.rate_hz / 10,
	      "snapshot OSCR window is bounded (%u ticks)",
	      (uint32_t)(snapshot.oscr_after - snapshot.oscr_before));
	CHECK(reserved_is_zero(snapshot.reserved, 4),
	      "snapshot reserved fields are zero");

	benchmark(counter);
	munmap(mapping, info.mmap_page_size);
	close(fd);
	finish(failures ? 1 : 0);
}
