#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
// #include <sys/boardctl.h>     // Needed for boardctl()
// #include <nuttx/board.h>      // Required for BOARDIOC_INIT

int main(int argc, FAR char *argv[])
{
    int fd;
    float data[4]; // X, Y, Z, and magnitude
    ssize_t nread;

    // Uncomment if sensor needs board initialization
    fd = open("/dev/mag0", O_RDONLY);
    if (fd < 0)
    {
        perror("Failed to open /dev/mag0");
        return 1;
    }

    for (int i = 0; i < 100; i++)
    {
        nread = read(fd, data, sizeof(data));
        // printf("Bytes read: %zd\n", nread); 
        if (nread != sizeof(data))
        {
            perror("Failed to read magnetometer data");
            close(fd);
            return 1;
        }

        printf("X: %.6f nT, Y: %.6f nT, Z: %.6f nT, Mag: %.6f nT\n", data[0], data[1], data[2], data[3]);
        
        // usleep(50); // 500ms delay
    }

    close(fd);
    return 0;
}