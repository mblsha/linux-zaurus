// SPDX-License-Identifier: GPL-2.0-only
#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <setjmp.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <linux/arm_palm_domain.h>

#include "../kselftest.h"

#ifndef MAP_FIXED_NOREPLACE
#define MAP_FIXED_NOREPLACE 0x100000
#endif

#define DEVICE "/dev/arm_palm_domain"
#define ZIRE_START ((void *)0x70400000UL)
#define ZIRE_LENGTH (8UL * 1024 * 1024)
#define T3_START ((void *)0x63400000UL)
#define T3_LENGTH (11UL * 1024 * 1024)
#define SAMPLE_COUNT 500

static sigjmp_buf fault_jmp;
static volatile sig_atomic_t expect_fault;
static volatile sig_atomic_t signal_seen;

static void fault_handler(int signal)
{
	if (expect_fault)
		siglongjmp(fault_jmp, 1);
	_exit(128 + signal);
}

static void user_signal_handler(int signal)
{
	signal_seen = signal;
}

static int set_mode(int fd, uint32_t mode)
{
	struct arm_palm_domain_mode request = { .mode = mode };

	return ioctl(fd, ARM_PALM_DOMAIN_SET_MODE, &request);
}

static int write_faults(volatile unsigned char *address)
{
	if (sigsetjmp(fault_jmp, 1)) {
		expect_fault = 0;
		return 1;
	}

	expect_fault = 1;
	*address ^= 0x5a;
	expect_fault = 0;
	return 0;
}

static uint64_t monotonic_ns(void)
{
	struct timespec time;

	clock_gettime(CLOCK_MONOTONIC, &time);
	return (uint64_t)time.tv_sec * 1000000000ULL + time.tv_nsec;
}

static int compare_u64(const void *left, const void *right)
{
	uint64_t a = *(const uint64_t *)left;
	uint64_t b = *(const uint64_t *)right;

	return (a > b) - (a < b);
}

static int benchmark(int fd, void *normal, size_t length)
{
	uint64_t domain[SAMPLE_COUNT * 2];
	uint64_t protect[SAMPLE_COUNT * 2];
	struct arm_palm_domain_info before;
	struct arm_palm_domain_info after;
	int i;

	if (set_mode(fd, ARM_PALM_DOMAIN_MODE_CLIENT) ||
	    ioctl(fd, ARM_PALM_DOMAIN_GET_INFO, &before))
		return -1;
	for (i = 0; i < SAMPLE_COUNT; i++) {
		uint64_t start = monotonic_ns();

		if (set_mode(fd, ARM_PALM_DOMAIN_MODE_MANAGER))
			return -1;
		domain[i * 2] = monotonic_ns() - start;
		start = monotonic_ns();
		if (set_mode(fd, ARM_PALM_DOMAIN_MODE_CLIENT))
			return -1;
		domain[i * 2 + 1] = monotonic_ns() - start;

		start = monotonic_ns();
		if (mprotect(normal, length, PROT_READ | PROT_WRITE))
			return -1;
		protect[i * 2] = monotonic_ns() - start;
		start = monotonic_ns();
		if (mprotect(normal, length, PROT_READ))
			return -1;
		protect[i * 2 + 1] = monotonic_ns() - start;
	}
	if (ioctl(fd, ARM_PALM_DOMAIN_GET_INFO, &after))
		return -1;

	qsort(domain, SAMPLE_COUNT * 2, sizeof(domain[0]), compare_u64);
	qsort(protect, SAMPLE_COUNT * 2, sizeof(protect[0]), compare_u64);
	ksft_print_msg("domain ns median=%llu p95=%llu; mprotect ns median=%llu p95=%llu\n",
		       (unsigned long long)domain[SAMPLE_COUNT],
		       (unsigned long long)domain[SAMPLE_COUNT * 19 / 10],
		       (unsigned long long)protect[SAMPLE_COUNT],
		       (unsigned long long)protect[SAMPLE_COUNT * 19 / 10]);

	return after.transitions == before.transitions + SAMPLE_COUNT * 2 &&
	       after.pmd_mutations == before.pmd_mutations &&
	       after.tlb_flushes == before.tlb_flushes ? 0 : -1;
}

struct thread_request {
	int fd;
	int denied;
};

static void *thread_try_manager(void *opaque)
{
	struct thread_request *request = opaque;

	errno = 0;
	request->denied = set_mode(request->fd, ARM_PALM_DOMAIN_MODE_MANAGER) == -1 &&
		errno == EPERM;
	return NULL;
}

static int open_and_map(int *fd_out, void **storage_out, int cloexec)
{
	int flags = O_RDWR | (cloexec ? O_CLOEXEC : 0);
	void *storage;
	int fd;

	fd = open(DEVICE, flags);
	if (fd < 0)
		return -1;
	storage = mmap(ZIRE_START, ZIRE_LENGTH, PROT_READ,
		       MAP_SHARED | MAP_FIXED_NOREPLACE, fd, 0);
	if (storage == MAP_FAILED) {
		close(fd);
		return -1;
	}
	*fd_out = fd;
	*storage_out = storage;
	return 0;
}

static int exec_cleanup_child(int fd)
{
	struct arm_palm_domain_mode mode = {
		.mode = ARM_PALM_DOMAIN_MODE_MANAGER,
	};
	unsigned char vector;
	int fresh;
	int denied;

	errno = 0;
	denied = ioctl(fd, ARM_PALM_DOMAIN_SET_MODE, &mode) == -1 &&
		errno == EPERM;
	errno = 0;
	if (!denied || mincore(ZIRE_START, 4096, &vector) != -1 ||
	    errno != ENOMEM)
		return 1;
	close(fd);
	fresh = open(DEVICE, O_RDWR | O_CLOEXEC);
	if (fresh < 0)
		return 1;
	close(fresh);
	return 0;
}

static int test_exec_cleanup(void)
{
	char fd_text[32];
	void *storage;
	pid_t child;
	int status = -1;
	int fd;

	child = fork();
	if (!child) {
		if (open_and_map(&fd, &storage, 0) ||
		    set_mode(fd, ARM_PALM_DOMAIN_MODE_MANAGER))
			_exit(1);
		snprintf(fd_text, sizeof(fd_text), "%d", fd);
		execl("/proc/self/exe", "palm_domain", "--exec-cleanup-child",
		      fd_text, NULL);
		_exit(1);
	}
	if (child > 0)
		waitpid(child, &status, 0);
	return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

static int test_exit_cleanup(void)
{
	void *storage;
	pid_t child;
	int status = -1;
	int fresh;
	int fd;

	child = fork();
	if (!child) {
		if (open_and_map(&fd, &storage, 1) ||
		    set_mode(fd, ARM_PALM_DOMAIN_MODE_MANAGER))
			_exit(1);
		_exit(0);
	}
	if (child > 0)
		waitpid(child, &status, 0);
	if (!WIFEXITED(status) || WEXITSTATUS(status))
		return 0;
	fresh = open(DEVICE, O_RDWR | O_CLOEXEC);
	if (fresh < 0)
		return 0;
	close(fresh);
	return 1;
}

static int test_t3_mapping(void)
{
	void *storage;
	int fd;

	fd = open(DEVICE, O_RDWR | O_CLOEXEC);
	if (fd < 0)
		return 0;
	storage = mmap(T3_START, T3_LENGTH, PROT_READ,
		       MAP_SHARED | MAP_FIXED_NOREPLACE, fd, 0);
	if (storage == MAP_FAILED) {
		close(fd);
		return 0;
	}
	munmap(storage, T3_LENGTH);
	close(fd);
	return storage == T3_START;
}

int main(int argc, char **argv)
{
	struct sigaction fault_action = { .sa_handler = fault_handler };
	struct sigaction user_action = { .sa_handler = user_signal_handler };
	struct arm_palm_domain_info info;
	struct thread_request request;
	pthread_t thread;
	unsigned char byte = 0xa5;
	unsigned char readback = 0;
	void *normal;
	void *storage;
	pid_t child;
	int pipefd[2];
	int console;
	size_t offset;
	int status;
	int fd;

	if (argc == 3 && !strcmp(argv[1], "--exec-cleanup-child"))
		return exec_cleanup_child(atoi(argv[2]));
	if (getpid() == 1) {
		mkdir("/dev", 0755);
		mkdir("/proc", 0555);
		if (mount("devtmpfs", "/dev", "devtmpfs", 0, NULL) &&
		    errno != EBUSY) {
			ksft_print_header();
			ksft_exit_fail_msg("mount devtmpfs: %s\n", strerror(errno));
		}
		console = open("/dev/console", O_RDWR);
		if (console >= 0) {
			dup2(console, STDIN_FILENO);
			dup2(console, STDOUT_FILENO);
			dup2(console, STDERR_FILENO);
			if (console > STDERR_FILENO)
				close(console);
		}
		if (mount("proc", "/proc", "proc", 0, NULL) && errno != EBUSY) {
			ksft_print_header();
			ksft_exit_fail_msg("mount proc: %s\n", strerror(errno));
		}
	}

	fd = open(DEVICE, O_RDWR | O_CLOEXEC);
	if (fd < 0 && (errno == ENOENT || errno == ENODEV)) {
		ksft_print_header();
		ksft_exit_skip("ARM Palm domain ABI unavailable; use mprotect fallback\n");
	}
	if (fd < 0) {
		ksft_print_header();
		ksft_exit_fail_msg("open: %s\n", strerror(errno));
	}
	close(fd);

	ksft_print_header();
	ksft_set_plan(16);
	ksft_test_result(test_exec_cleanup(), "exec revokes owner mapping and capability\n");
	ksft_test_result(test_exit_cleanup(), "task exit releases mapping and device\n");
	ksft_test_result(test_t3_mapping(), "fixed T3 storage mapping\n");

	fd = open(DEVICE, O_RDWR | O_CLOEXEC);
	if (fd < 0)
		ksft_exit_fail_msg("second open: %s\n", strerror(errno));

	memset(&info, 0, sizeof(info));
	ksft_test_result(ioctl(fd, ARM_PALM_DOMAIN_GET_INFO, &info) == 0 &&
			 info.abi_version == ARM_PALM_DOMAIN_ABI_VERSION &&
			 info.domain == 4,
			 "ABI discovery\n");

	storage = mmap((void *)0x70000000UL, 1024 * 1024, PROT_READ,
		       MAP_SHARED | MAP_FIXED_NOREPLACE, fd, 0);
	ksft_test_result(storage == MAP_FAILED && errno == EINVAL,
			 "unsupported mapping geometry is rejected\n");

	storage = mmap(ZIRE_START, ZIRE_LENGTH, PROT_READ,
		       MAP_SHARED | MAP_FIXED_NOREPLACE, fd, 0);
	if (storage == MAP_FAILED)
		ksft_exit_fail_msg("storage mmap: %s\n", strerror(errno));
	ksft_test_result(storage == ZIRE_START, "fixed Zire storage mapping\n");

	sigemptyset(&fault_action.sa_mask);
	sigaction(SIGSEGV, &fault_action, NULL);
	sigaction(SIGBUS, &fault_action, NULL);
	errno = 0;
	status = mprotect(storage, ZIRE_LENGTH, PROT_READ | PROT_WRITE);
	ksft_test_result(status == -1 &&
			 (errno == EACCES || errno == EPERM),
			 "mprotect cannot make storage writable\n");
	ksft_test_result(write_faults(storage), "Client mode enforces read-only AP\n");

	ksft_test_result(set_mode(fd, ARM_PALM_DOMAIN_MODE_MANAGER) == 0 &&
			 (*(volatile unsigned char *)storage = 0x3c, true),
			 "Manager mode permits writes\n");

	sigemptyset(&user_action.sa_mask);
	sigaction(SIGUSR1, &user_action, NULL);
	raise(SIGUSR1);
	for (status = 0; status < 100; status++)
		sched_yield();
	*(volatile unsigned char *)storage = 0x5a;
	ksft_test_result(signal_seen == SIGUSR1 &&
			 *(volatile unsigned char *)storage == 0x5a,
			 "Manager survives signal and context switches\n");

	request = (struct thread_request) { .fd = fd };
	status = pthread_create(&thread, NULL, thread_try_manager, &request);
	if (!status)
		status = pthread_join(thread, NULL);
	ksft_test_result(!status && request.denied,
			 "non-owner thread cannot switch domain\n");

	child = fork();
	if (!child) {
		unsigned char vector;
		int denied;

		errno = 0;
		denied = set_mode(fd, ARM_PALM_DOMAIN_MODE_MANAGER) == -1 &&
			errno == EPERM;
		errno = 0;
		_exit(denied && mincore(storage, 4096, &vector) == -1 &&
		      errno == ENOMEM ? 0 : 1);
	}
	status = -1;
	if (child > 0)
		waitpid(child, &status, 0);
	ksft_test_result(WIFEXITED(status) && WEXITSTATUS(status) == 0,
			 "fork child has neither mapping nor Manager capability\n");

	ksft_test_result(set_mode(fd, ARM_PALM_DOMAIN_MODE_CLIENT) == 0 &&
			 write_faults(storage),
			 "Client mode re-protects storage\n");

	if (pipe(pipefd))
		ksft_exit_fail_msg("pipe: %s\n", strerror(errno));
	write(pipefd[1], &byte, 1);
	errno = 0;
	status = read(pipefd[0], storage, 1);
	ksft_test_result(status == -1 && errno == EFAULT,
			 "kernel copy_to_user respects Client protection\n");
	write(pipefd[1], &byte, 1);
	status = set_mode(fd, ARM_PALM_DOMAIN_MODE_MANAGER);
	status |= read(pipefd[0], storage, 1) != 1;
	status |= write(pipefd[1], storage, 1) != 1;
	status |= read(pipefd[0], &readback, 1) != 1;
	ksft_test_result(!status && readback == byte,
			 "kernel uaccess preserves Manager state securely\n");
	close(pipefd[0]);
	close(pipefd[1]);

	normal = mmap(NULL, ZIRE_LENGTH, PROT_READ | PROT_WRITE,
		      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (normal != MAP_FAILED) {
		for (offset = 0; offset < ZIRE_LENGTH; offset += 4096)
			*((unsigned char *)normal + offset) = 0;
		status = mprotect(normal, ZIRE_LENGTH, PROT_READ);
		if (!status)
			status = benchmark(fd, normal, ZIRE_LENGTH);
	} else {
		status = -1;
	}
	ksft_test_result(!status,
			 "hot transitions avoid page-table mutation and TLBI\n");

	set_mode(fd, ARM_PALM_DOMAIN_MODE_CLIENT);
	if (normal != MAP_FAILED)
		munmap(normal, ZIRE_LENGTH);
	munmap(storage, ZIRE_LENGTH);
	close(fd);
	ksft_exit_pass();
}
