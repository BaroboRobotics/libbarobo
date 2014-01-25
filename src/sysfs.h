#ifndef SYSFS_H
#define SYSFS_H

/* The Linux kernel exposes system devices via sysfs. This filesystem is what
 * systems such as udev or mdev query to access information about devices.
 * udev in fact has an API, libudev, which allows you to enumerate through
 * devices programmatically, and activity on the mdev list indicates that they
 * might support the same API someday. However, crawling through sysfs with
 * shell commands isn't any harder, and we can even make a little embedded
 * domain-specific language using C preprocessor macros to help us out.
 *
 * sysfs has some docs here: kernel.org/doc/Documentation/sysfs-rules.txt
 *
 * As a simple example, the following would select all tty devices which are
 * children of usb devices manufactured by Barobo.
 *
 *   FROM("/sys/devices") SELECT SUBSYSTEM("usb")
 *                        AND SYSATTR("manufacturer", "Barobo, Inc.")
 *                        SELECT SUBSYSTEM("tty")
 *
 * To be more clear, that sequence of macros would generate a single string
 * literal which would contain a command, suitable for running with popen(3)
 * or similar, whose standard output would have a null-delimited sequence of
 * paths to devices in /sys/devices which satisfy the filter predicates.
 *
 * Alternatively, you could replace SUBSYSTEM with SUBSYSTEMF and SYSATTR with
 * SYSATTRF, which would generate a string literal suitable for use as a format
 * string for sprintf(3) or similar. This would allow you to offer the user a
 * choice of manufacturer (i.e., SYSATTR("manufacturer", "%s")), or allow you
 * to override "/sys" with "$SYSFS_PATH" as suggested in the kernel docs.
 *
 *
 * A Parsing Expression Grammar for this EDSL is given here:
 *
 * Selection <- FromRoot (Select Filter)* 'FIRST'?
 *              if the FIRST atom is present, this selection will only contain
 *              the first matching device
 *
 * FromRoot <- 'FROM' '(' Root ')'
 * Root <- a C string literal
 *         should always be "/sys/devices" or "$SYSFS_PATH/devices"
 *
 * Select <- 'SELECT' / 'SELECTUP' / 'AND'
 *           a Select keyword chooses which device(s) to test with the
 *           subsequent filter
 *             SELECT:   select current device and devices in subdirectories
 *             SELECTUP: select devices in superdirectories
 *             AND:      select current device
 *
 * Filter <- SubsystemFilter / SysAttrFilter
 *
 * SubsystemFilter <- 'SUBSYSTEM' '(' Subsystem ')'
 * SysAttrFilter <- 'SYSATTR' '(' SysAttrKey ',' SysAttrValue ')'
 *
 *   Note that SUBSYSTEM and SYSATTR also come in SUBSYSTEMF and SYSATTRF
 *   variants for use with sprintf(3) and family (literal '%'s in the
 *   generated string literal are escaped with another '%').
 *
 * Subsystem <- a C string literal
 *              examples: "usb", "pci", "tty", etc.
 *              for a complete list, try:
 *                find /sys/devices -name subsystem -printf '%l\n' | rev | cut -f1 -d/ | rev | sort | uniq
 *
 * SysAttrKey <- a C string literal
 *               attribute files in the device directories
 *               examples: "product", "manufacturer"
 *
 * SysAttrValue <- a C string literal
 *                 value to match with the data in an attribute file
 */

#define FROM(x)       " find " x " -maxdepth 0 -print0 "
#define SELECT        " | xargs -0 -I}{ find '}{' "
#define AND           SELECT " -maxdepth 1 "
#define SUBSYSTEM(x)  " -type l -name subsystem -lname \\*/" x " -printf '%h\\0' "
#define SUBSYSTEMF(x) " -type l -name subsystem -lname \\*/" x " -printf '%%h\\0' "
#define SYSATTR(x,y)  " -type f -name " x " -execdir grep -q " y " '{}' \\; -printf '%h\\0' "
#define SYSATTRF(x,y) " -type f -name " x " -execdir grep -q " y " '{}' \\; -printf '%%h\\0' "
#define FIRST         " -quit "

#define SELECTUP      " | xargs -0 -I}{ sh -c 'x=\"}{\"; while [ \"/\" != \"$x\" ]; do dirname -z \"$x\"; x=$(dirname -z \"$x\"); done' " AND

#endif
