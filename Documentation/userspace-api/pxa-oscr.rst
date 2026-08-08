.. SPDX-License-Identifier: GPL-2.0

==================================
PXA OS timer counter userspace API
==================================

The PXA OS timer counter (OSCR) is the free-running 32-bit counter used by the
PXA clocksource and clock-event driver.  Systems that enable
``CONFIG_PXA_OSCR_UAPI`` provide ``/dev/pxa-oscr`` so a tracing client can use
one read-only MMIO load per event instead of a system call.

The interface is intentionally owned by the timer driver.  It does not expose
a physical address and has no relationship to process ARM domains, DACR
settings, storage drivers, ``/dev/mem``, or debugfs.

Device access policy
====================

The misc device is created with mode 0444.  Unprivileged clients may open it
read-only because the purpose of the interface is low-overhead application
tracing.  Opens requesting write access fail with ``EPERM``.

Exposing a fast timer can help construct timing side channels, but PXA Linux
already makes clocks available to unprivileged processes.  Administrators that
do not want direct timer access should disable ``CONFIG_PXA_OSCR_UAPI`` or
apply a more restrictive device policy before applications start.

``PXA_OSCR_GET_INFO``
=====================

This ioctl fills ``struct pxa_oscr_info`` from
``<linux/pxa-oscr.h>``.  Version 1 has a fixed size of 64 bytes.  Clients must
check both ``struct_size`` and ``version`` and must ignore fields added beyond
the size they understand.  All reserved fields are zero.

``rate_hz`` is the rate registered by the timer driver from the clock
framework; it is not an ABI constant.  ``counter_bits`` is 32.
``mmap_page_size`` is the only accepted mapping length and
``mmap_oscr_offset`` locates OSCR within that page.  The physical address of
the page is deliberately not returned.

The ``flags`` field describes available operations and counter behaviour.
In version 1 the mapping is read-only, the counter wraps, snapshots use
``CLOCK_MONOTONIC_RAW``, and reads of the documented timer registers are
side-effect free.  Builds with power-management support also advertise the
suspend generation.

Mapping and the hot path
========================

Map exactly ``mmap_page_size`` bytes at file offset zero with ``PROT_READ``.
Writable mappings are rejected.  ARMv5 Linux applies ``READ_IMPLIES_EXEC``
to every readable mapping because the architecture has no execute-never page
permission.  The driver therefore cannot distinguish an explicit
``PROT_EXEC`` request from the execute flag added to a plain ``PROT_READ``
request.  It accepts that initial flag and clears both ``VM_EXEC`` and
``VM_MAYEXEC`` before installing the mapping.  ARMv5 hardware does not have
an execute-never page permission, so this is a Linux VMA policy rather than a
hardware execution guarantee.  The VMA cannot later be
upgraded with ``mprotect(PROT_WRITE)`` or ``mprotect(PROT_EXEC)``.  It uses
device/noncached protection and the ``VM_IO``, ``VM_PFNMAP``,
``VM_DONTEXPAND``, and ``VM_DONTDUMP`` properties.

The page contains OS timer match registers OSMR0--OSMR3, OSCR, the status
register OSSR, watchdog-enable register OWER, interrupt-enable register OIER,
and reserved locations.  On PXA, reads of the documented registers do not
acknowledge status, arm matches, enable the watchdog, or change interrupts;
those effects require writes.  Page-granular MMIO mapping cannot isolate OSCR
from the other registers, so enforcing read-only PTEs and forbidding later
write upgrades is a security requirement of this ABI.  Applications should
read only OSCR at ``mmap_oscr_offset``.

After setup, the timestamp hot path can be a volatile 32-bit load followed by
a compiler barrier.  It needs no ioctl, system call, allocation, lock,
conversion, or I/O operation.  Device memory must not be cached in an ordinary
variable across events.

Wrap and extension
==================

OSCR increments modulo 2^32.  Its wrap period is ``2^32 / rate_hz`` seconds;
at 3,686,400 Hz this is about 1165.08 seconds.  Given consecutive samples
``old`` and ``new``, the modulo delta is::

    (__u32)(new - old)

Add these unsigned deltas to a wider accumulator.  A client must sample at
least once per wrap to distinguish zero wraps from one or more wraps.  A
shorter maximum interval, such as half a wrap, gives useful scheduling margin.

``PXA_OSCR_GET_SNAPSHOT``
=========================

This ioctl fills a fixed-size ``struct pxa_oscr_snapshot``.  The kernel reads
``oscr_before``, obtains ``monotonic_raw_ns`` with ``ktime_get_raw_ns()``, then
reads ``oscr_after``.  The raw timestamp therefore lies within a counter
window bounded by the two OSCR samples; that window can cross 32-bit wrap and
must be interpreted with unsigned arithmetic.

``generation_before`` and ``generation_after`` bracket the same operation.
They normally match.  When ``PXA_OSCR_INFO_F_SUSPEND_GENERATION`` is set, the
generation increments after every PXA timer resume callback.  The timer driver
saves OSCR during suspend and restores the saved value on resume, so OSCR does
not represent elapsed suspend time and hardware continuity during suspend is
not promised.  A client must not extend or align samples across a generation
change.  Take a new snapshot after resume and start a new correlation segment.
The mapping deliberately contains no kernel-maintained generation word, so a
client that must detect suspend while collecting direct samples needs an
occasional snapshot or another lifecycle notification.  Kernels built without
power-management support omit the flag and cannot suspend through this timer
callback.

The generation value is 32-bit and may itself wrap after 2^32 resumes.  A
system reset starts a new process/device lifetime and is not represented as a
generation transition visible to the old process.

Future extension: live metadata
===============================

An optional, cacheable, read-only metadata page is deliberately deferred.  A
future, separately reviewed ABI could provide a sequence counter with ARMv5
ordering rules, OSCR rate/width/mask, a base OSCR value correlated with a
``CLOCK_MONOTONIC_RAW`` base timestamp, conversion multiplier and shift,
suspend/discontinuity generation, clocksource-validity state, and feature
flags.  A client needing a current Linux timestamp could then read a stable
metadata sequence, read OSCR, and calculate approximately::

    base_ns + (((oscr - base_oscr) & mask) * multiplier >> shift)

Such a page could detect discontinuities without an ioctl, make long captures
and multi-process correlations more robust, permit immediate syscall-free
timestamp reconstruction, and provide groundwork for wider PXA vDSO clock
support.  Raw tracing would still read only OSCR per event and inspect live
generation at capture or chunk boundaries.  Reading the metadata for every
event would make that hot path slower by adding sequence checks, barriers,
cacheable loads, 64-bit arithmetic, and possible retries.

This extension is not part of version 1.  Correctly publishing potentially
torn 64-bit fields on ARMv5, defining memory ordering, updating state across
suspend and clocksource transitions, and managing another mapping are a
substantial synchronization and permanent-UAPI commitment.  The zeroed
reserved fields in the versioned ioctls leave room to negotiate future
features, but version 1 assigns no live-metadata capability, mapping offset,
or structure layout.  Exposing any of them requires a deliberate scope
increase, implementation, and separate review; applications must not infer an
unfinished page from the current reserved fields.

The companion Zaurus integration repository records downstream design and
QEMU evidence in ``docs/components/pxa2xx/oscr-userspace-tracing.md``.
