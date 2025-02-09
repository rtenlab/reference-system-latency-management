#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>

#define DEVICE_PATH "/dev/rt_be_mutex"

// ioctl commands
#define IOCTL_LOCK   _IOW('r', 1, int)
#define IOCTL_UNLOCK _IOW('r', 2, int)

// Task types
#define TASK_TYPE_RT 1
#define TASK_TYPE_BE 2

int main(int argc, char *argv[]) {
    int fd, task_type;

    if (argc != 2) {
        printf("Usage: %s <task_type>\n", argv[0]);
        printf("Task type: 1 (RT), 2 (BE)\n");
        return -1;
    }

    task_type = atoi(argv[1]);
    if (task_type != TASK_TYPE_RT && task_type != TASK_TYPE_BE) {
        printf("Invalid task type. Use 1 for RT and 2 for BE.\n");
        return -1;
    }

    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open the device");
        return -1;
    }

    printf("Requesting lock as %s task...\n", task_type == TASK_TYPE_RT ? "RT" : "BE");
    if (ioctl(fd, IOCTL_LOCK, &task_type) < 0) {
        perror("Failed to acquire lock");
        close(fd);
        return -1;
    }
    printf("Lock acquired!\n");

    sleep(5); // Simulate critical section

    printf("Releasing lock...\n");
    if (ioctl(fd, IOCTL_UNLOCK, NULL) < 0) {
        perror("Failed to release lock");
        close(fd);
        return -1;
    }
    printf("Lock released!\n");

    close(fd);
    return 0;
}