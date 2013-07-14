/* 
 * inspired by : usb-relay - a tiny control program for a CH341A based relay board.
 * Copyright (C) 2010  Henning Rohlfs GPL2 license
 * This version is by Edwin van den Oetelaar (2013/03/11)
 * used for Relay control on Diana2-alpha audio mp3/ogg/aac streamers
 * and Diana-7 streamer (h264/aac) video streamer as sold by www.Schaapsound.nl
 *
 * stand alone C program using libusb-1.0 on linux
 * comes with simple Makefile, make sure that libusb-1.0-dev is installed
 * pkg-config --cflags libusb-1.0 
 */

#include <assert.h>
#include <libusb.h>
#include <malloc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/inotify.h>
#include <errno.h>
#include <sys/stat.h>

/* Control IO via existence of files in Temp directory 
 * External programs can easily monitor this using inotify scripts
 * every physical IO device has its own directory eg. XXX in this example
 * /tmp/XXX/D_IN_01 .. D_IN_99
 * /tmp/XXX/D_OUT_01 .. D_OUT_99
 * create a file by script or other means and the IO pin will change
 * if an input in changes, a file will be created/removed to reflect status
 */

#define EVENT_SIZE  ( sizeof (struct inotify_event) )
#define EVENT_BUF_LEN     ( 1024 * ( EVENT_SIZE + 16 ) )

static const unsigned char cmd_part1[] = {0xa1, 0x6a, 0x1f, 0x00, 0x10};
static const unsigned char cmd_part2[] = {0x3f, 0x00, 0x00, 0x00, 0x00};
static const int USB_VENDOR_ID = 0x1a86;
static const int USB_PROUCT_ID = 0x5512;
static const int FIRST_PIN = 1;
static const int LAST_PIN = 8;

typedef struct
{
    uint32_t active_relays; // bit mask requested 
    uint32_t outputbits; // bit mask set

    libusb_context *usb_context; // pointer to usb context
    libusb_device_handle *device_handle; // pointer to the usb device handle 

    /* flag when output needs to be sent, but is not yet done (retry later ?) */
    int output_pending; // cleared by write success 
    int verbose; // verbose output to console
    int use_syslog; // use syslog for logging instead of console
    int run_as_daemon; // run as daemon, use /tmp/ID/D_OUT_99 inotify for control
    char *event_dir; // where to listen and send events
} ios_handle_t;

int
send_relay_cmd(libusb_device_handle *dev, uint8_t cmd, uint8_t verbose)
{
    uint8_t buf[32] = {0}; // buf is large enough
    int n = sizeof (cmd_part1);
    int m = sizeof (cmd_part2);

    /* fill buf with complete message */
    memcpy(buf, cmd_part1, n);
    buf[n] = cmd;
    memcpy(buf + n + 1, cmd_part2, m);

    /* send message to usb endpoint */
    static const int endpointid = 2; // for some reason
    int numbytes = n + m + 1;
    int actual_length = 0;

    /* do usb action, rv !=0 on error */
    int rv = libusb_bulk_transfer(dev, endpointid, buf, numbytes, &actual_length, 100);

    if (verbose) {
        fprintf(stderr, "\n");
        for (int i = 0; i < numbytes; i++)
            fprintf(stderr, "pos=%02d val=%02x\n", i, buf[i]);
    }

    if (rv != 0)
        fprintf(stderr, "libusb_bulk_transfer() failed\n");

    // return 0 on successful write
    return (numbytes != actual_length);
}

/* Actual communication with the device and saving the status */
int
USB_write_IO(ios_handle_t *handle)
{
    assert(handle);
    libusb_device_handle *dev = handle->device_handle;
    assert(dev);
    uint8_t active_relays = (uint8_t) handle->active_relays;
    uint8_t verbose = handle->verbose;

    /* Send the command frame */
    if (send_relay_cmd(dev, 0x00, verbose)) goto error;
    /* send stuff for every bit in the mask */
    for (uint8_t mask = 128; mask > 0; mask >>= 1) {
        if (active_relays & mask) {
            /* Send "relay on" */
            if (send_relay_cmd(dev, 0x20, verbose)) goto error;
            if (send_relay_cmd(dev, 0x28, verbose)) goto error;
            if (send_relay_cmd(dev, 0x20, verbose)) goto error;
        } else {
            /* Send "relay off" */
            if (send_relay_cmd(dev, 0x00, verbose)) goto error;
            if (send_relay_cmd(dev, 0x08, verbose)) goto error;
            if (send_relay_cmd(dev, 0x00, verbose)) goto error;
        }
    }
    /* End the command frame */
    if (send_relay_cmd(dev, 0x00, verbose)) goto error;
    if (send_relay_cmd(dev, 0x01, verbose)) goto error;

    /* Remember the status */
    handle->output_pending = 0;
    handle->outputbits = active_relays;
    return 0; // success
error:
    if (handle->device_handle != NULL)
        libusb_close(handle->device_handle);
    handle->device_handle = NULL;
    handle->output_pending = 1;
    return -1; // problems
}

int
USB_open_device(ios_handle_t *handle)
{
    assert(handle->device_handle == NULL);

    libusb_device **devs = {0}; // to retrieve a list of devices
    libusb_device_handle *udh = NULL;
    libusb_context *ctx = NULL;

    int r = libusb_init(&ctx); // initialize the library for the session we just declared
    handle->usb_context = ctx;

    if (r < 0) {
        fprintf(stderr, "Init Error %d\n", r); // there was an error
        return -1;
    }

    libusb_set_debug(ctx, 3);

    ssize_t cnt = libusb_get_device_list(ctx, &devs); // get the list of devices
    if (cnt < 0) {
        fprintf(stderr, "Get Device Error\n"); // there was an error
        return -1;
    }

    if (handle->verbose)
        fprintf(stderr, "[%ld] Devices in list.\n", cnt);

    udh = libusb_open_device_with_vid_pid(ctx, USB_VENDOR_ID, USB_PROUCT_ID);
    if (!udh) {
        fprintf(stderr, "Cannot open device: libusb %p\n", udh);
        return -1;
    } else {
        if (handle->verbose)
            fprintf(stderr, "Device is open\n");

        handle->device_handle = udh; // copy for later use
    }

    libusb_free_device_list(devs, 1); // free the list, unref the devices in it

    if (libusb_kernel_driver_active(udh, 0) == 1) { // find out if kernel driver is attached
        if (handle->verbose)
            fprintf(stderr, "Kernel Driver Active\n");

        if (libusb_detach_kernel_driver(udh, 0) == 0) {// detach it
            if (handle->verbose)
                fprintf(stderr, "Kernel Driver Detached!\n");
            else
                fprintf(stderr, "Kernel Driver Detach failed!\n");
        }
    }

    r = libusb_claim_interface(udh, 0); // claim interface 0 

    if (r < 0) {
        fprintf(stderr, "Cannot Claim Interface : %d\n", r);
        handle->device_handle = NULL;
        handle->output_pending = 1;
        return -1;
    }

    if (handle->verbose)
        fprintf(stderr, "Claimed Interface\n");

    handle->output_pending = 1;

    return 0; // success
}

void
USB_close_device(ios_handle_t *h)
{
    assert(h);
    assert(h->usb_context);

    if (h->device_handle)
        libusb_close(h->device_handle);

    libusb_exit(h->usb_context);
}

int
run_once(ios_handle_t *h, int argc, char *argv[])
{
    /* just set some outputs on or off and quit */
    for (int i = optind; i < argc; i++) {
        int relay = atoi(argv[i]);
        if (relay < FIRST_PIN || relay > LAST_PIN) {
            fprintf(stderr, "error: only give valid relay numbers (1-8) as parameter\n");
            fprintf(stderr, "you can use -v as first option to enable verbose output debugging\n");
            fprintf(stderr, "example: ./%s -v 1 5 7 will switch 1 5 and 7 on the rest will be off\n", argv[0]);
            return 2;
        }
        h->active_relays |= (1 << (relay - 1));
    }
    if (h->verbose)
        fprintf(stderr, "writing byte %d to usb\n", h->active_relays);

    if (0 == USB_open_device(h)) {
        USB_write_IO(h);
        USB_close_device(h);
    } else {
        fprintf(stderr, "Error : device not open\n");
        return 3;
    }
    return 0;
}

int
run_as_daemon(ios_handle_t *h)
{
    if (h->verbose)
        fprintf(stderr, "Keep Running, daemon not forking, eventpath=%s pid=%d\n",
                h->event_dir, getpid());

    /* connect to USB IO board */
    while (1) {
        if (0 == USB_open_device(h)) {
            break;
        } else {
            fprintf(stderr, "IO board not found, try again in 1 sec\n");
            sleep(1);
        }
    }

    /* start the Inotify stuff */
    int fd = 0;
    int length = 0;
    char buffer[EVENT_BUF_LEN] = {0};
    int wd = 0;
    int i = 0;
    unsigned long int eventcounter = 0;

    fd = inotify_init();

    if (fd < 0)
        perror("inotify_init");

    wd = inotify_add_watch(fd, h->event_dir, IN_ALL_EVENTS);
    // IN_OPEN | IN_CLOSE | IN_CREATE | IN_DELETE | IN_DELETE_SELF);

    if (wd < 0)
        perror("inotify_add_watch");

    /* set initial outputs based on stat() of files already present 
     h->active_relays |= (1 << (relay - 1));
     */
    char b[4096] = {0};
    struct stat sb;
    unsigned relaybits = 0;
    for (i = FIRST_PIN; i != LAST_PIN; i++) {
        int len = snprintf(b, sizeof (b), "%s/D_OUT_%d", h->event_dir, i);
        if (h->verbose)
            fprintf(stderr, "stat( %s ) len=%d\n", b, len);
        if (stat(b, &sb) == 0) {
            relaybits |= (1 << (i - 1));
        }
    }

    h->active_relays = relaybits;
    USB_write_IO(h);
    /* read to determine the event change happens on “/tmp” directory. 
     * Actually this read blocks until the change event occurs*/

    while (1) {
        length = read(fd, buffer, EVENT_BUF_LEN);

        /*checking for error*/
        if (length < 0) {
            perror("read");
        }

        int i = 0;

        /*actually read return the list of change events happens. 
         * Here, read the change event one by one and process it accordingly.*/
        while (i < length) {
            struct inotify_event *event = (struct inotify_event *) &buffer[i];

            if (event->len) {

                if (event->mask & IN_CREATE) {
                    if (event->mask & IN_ISDIR) {
                        if (h->verbose)
                            fprintf(stderr, "New directory %s created.\n", event->name);
                    } else {
                        if (h->verbose)
                            fprintf(stderr, "New file %s created.\n", event->name);
                        /* check pattern */
                        int pin = 0;
                        if (sscanf(event->name, "D_OUT_%d", &pin)) {
                            h->active_relays |= 1 << (pin - 1);
                            if (h->verbose)
                                fprintf(stderr, "set pin=%d HIGH\n", pin);
                            eventcounter++;
                        }
                    }
                } else if (event->mask & IN_DELETE) {
                    if (event->mask & IN_ISDIR) {
                        if (h->verbose)
                            fprintf(stderr, "Directory %s deleted.\n", event->name);
                    } else {
                        if (h->verbose)
                            fprintf(stderr, "File %s deleted.\n", event->name);
                        /* check pattern */
                        int pin = 0;
                        if (sscanf(event->name, "D_OUT_%d", &pin)) {
                            h->active_relays &= ~(1 << (pin - 1));

                            if (h->verbose)
                                fprintf(stderr, "set pin=%d LOW\n", pin);
                            eventcounter++;
                        }
                    }
                }
            }
            i += EVENT_SIZE + event->len;
        }
        /* send the pins states to the IO board */
        USB_write_IO(h);

    }
    /*removing the “/tmp” directory from the watch list.*/
    inotify_rm_watch(fd, wd);

    /*closing the INOTIFY instance*/
    close(fd);


    USB_close_device(h);


    return 0;
}

int
main(int argc, char *argv[])
{

    ios_handle_t *h = calloc(1, sizeof (ios_handle_t));
    int rc = 0; // return value to shell
    /* 
     * use old style getopt() to be compatible
     * opterr, optopt, optind, optarg are from <unistd.h>
     */

    opterr = 0;
    int c;

    while ((c = getopt(argc, argv, "dhi:sv")) != -1)
        switch (c) {
        case 'v':
            h->verbose = 1;
            break;
        case 's':
            h->use_syslog = 1;
            break;
        case 'd':
            h->run_as_daemon = 1;
            break;
        case 'i':
            h->event_dir = strdup(optarg);
            break;
        case 'h':
            fprintf(stderr, "Help Text here");
            abort();
            break;
        default:
            abort();
        }

    if (h->verbose)
        fprintf(stderr, "verbose = %s\n", h->verbose ? "Yes" : "No");

    if (h->run_as_daemon) {
        /* we keep running until the end of time (or signal) */
        if (h->event_dir == 0) {
            fprintf(stderr, "using /tmp as default event directory\n");
            h->event_dir = strdup("/tmp");
        }
        rc = run_as_daemon(h);
    } else {
        rc = run_once(h, argc, argv);
    }

    free(h);
    return rc;
}