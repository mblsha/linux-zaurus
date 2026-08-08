.. SPDX-License-Identifier: GPL-2.0

=======================================
Fast Palm storage protection on ARM/PXA
=======================================

``CONFIG_ARM_PALM_DOMAIN`` provides a narrow, optional ABI for RePalm Redux on
ARMv5 PXA systems using the short-descriptor MMU.  It replaces repeated
``mprotect()`` calls at Palm storage-protection boundaries with a DACR domain
change.

Scope and configuration
=======================

ARM domain 4 is reserved by this feature.  The in-tree kernel, user, I/O and
vectors domains use 0 through 3; the build also checks that the assignments do
not overlap.  The option requires ``CONFIG_CPU_USE_DOMAINS``, a CP15 MMU,
non-LPAE page tables, and a uniprocessor kernel.  These restrictions describe
the PXA255 target and avoid pretending that per-CPU revocation is implemented.

ABI
===

The root-only ``/dev/arm_palm_domain`` misc device accepts a single opener.
``ARM_PALM_DOMAIN_GET_INFO`` reports ABI version 1, domain 4, current mode,
mapping identity, transition count, PMD mutation count and TLB-flush count.
Absence of the device, ``ENODEV`` or ``ENOTTY`` is the portable signal for
Redux to retain its ``mprotect()`` fallback.

The owning task maps exactly one of these layouts with ``PROT_READ``,
``MAP_SHARED`` and the exact address:

* Zire 31: ``0x70400000``, 8 MiB;
* Tungsten T3: ``0x63400000``, 11 MiB.

The driver owns zeroed vmalloc-backed storage.  It rejects writable mappings
and explicit executable mappings where the architecture distinguishes them,
then clears write/execute VMA state and later eligibility.  ARMv5 lacks
execute-never permission and applies ``READ_IMPLIES_EXEC``, so the capability
and single-owner boundary remains essential there.  The VMA rejects splitting,
``mprotect()`` and ``mremap()``.  ``VM_DONTCOPY`` keeps the mapping out of
forked children.  The fixed range and 1 MiB granularity ensure every retagged
hardware PMD describes only Redux storage.

``ARM_PALM_DOMAIN_SET_MODE`` accepts absolute CLIENT or MANAGER states.  Redux
owns the Palm-compatible eight-bit nesting counter and issues an ioctl only on
effective 0-to-1 and 1-to-0 transitions.  Initial and teardown states are
CLIENT.  Only the task that opened and mapped the device may switch; other
threads and forked children have domain 4 set to NOACCESS.

Entry, context-switch and uaccess rules
=======================================

The ARM syscall return path calls ``uaccess_enable``.  Its historical fixed
DACR write would erase domain 4.  User exceptions likewise use fixed
``uaccess_enable``/``uaccess_disable`` values, while nested privileged
exceptions save and restore the exact DACR in ``svc_pt_regs.dacr``.  With this
feature, the assembly helpers read-modify-write only the normal user/kernel
domain bits and preserve optional domains.

A mode ioctl updates both the live DACR and ``thread_info.cpu_domain``.
Consequently interrupts, signals and context switches preserve the selected
state.  ``copy_thread()`` clears domain 4 in every child, and exec/task teardown
revokes the mapping and returns the owner to CLIENT.  C uaccess already saves,
enables and restores only the normal user domain; the assembly changes make
the same preservation rule universal.  Kernel copies to read-only storage
therefore fail in CLIENT and work in MANAGER without broadening any other
mapping.

Performance and mutation boundary
=================================

Mapping setup and teardown take the mmap write lock, change one hardware PMD
domain field per MiB, and flush the affected TLB range.  They are not hot-path
operations.  CLIENT/MANAGER transitions take no VMA lock, walk no page table,
perform no cache maintenance and issue no TLBI.  The information counters let
the selftest prove that PMD and TLB counts remain unchanged across a measured
transition loop.

Run ``tools/testing/selftests/arm/palm_domain`` on a supported kernel.  It
checks registration validation, Client write faults, Manager writes,
re-protection, uaccess, signal/context-switch survival, thread and fork
isolation, and transition latency versus ``mprotect()``.  Unsupported kernels
report a kselftest skip so the userspace fallback remains testable.

Security boundary
=================

The ABI requires ``CAP_SYS_RAWIO``, permits one active opener, owns the mapped
pages, and accepts only the two Palm layouts.  Userspace cannot retag an
arbitrary VMA or grant Manager access to a file mapping.  Manager mode affects
only domain-4 PMDs in the owning mm.  A/B update, physical devices, and the
Linux root block device remain outside this ABI.
