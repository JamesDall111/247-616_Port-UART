#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>

const char *portTTY = "/dev/ttyUSB0"; // ton port série

int init_serial() {
    int fd = open(portTTY, O_RDWR | O_NOCTTY);
    if(fd == -1) {
        perror("Erreur ouverture port série");
        exit(1);
    }

    struct termios settings;
    tcgetattr(fd, &settings);

    cfsetispeed(&settings, B115200);
    cfsetospeed(&settings, B115200);

    settings.c_cflag &= ~PARENB;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cflag |= CREAD | CLOCAL;

    settings.c_iflag &= ~(IXON | IXOFF | IXANY);

    settings.c_lflag &= ~(ECHO | ECHOE | ISIG);
    settings.c_lflag |= ICANON; // mode canonique
    settings.c_oflag &= ~OPOST;

    // Lire 1 caractère, délai infini
    settings.c_cc[VMIN] = 2;
    settings.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &settings);

    return fd;
}

int main() {
    int fd = init_serial();
    pid_t pid = fork();

    if(pid < 0) {
        perror("Erreur fork");
        exit(1);
    }

    if(pid == 0) { // Fils : écrit sur le port série
        char input;
        printf("Je suis le processus Fils, j'écrit sur le port série...\n");
        while(1) {
            input = getchar();
            if(input == 'q') break;
            write(fd, &input, 1);
        }
        printf("Fin du Fils\n");
    } else { // Père : lit le port série
        char buf[256];
        int n;
        printf("Je suis le processus Père, j'écrit sur la console...\n");
        while(1) {
            n = read(fd, buf, 256);
            if(n > 0) {
                buf[n] = '\0';
                printf("processus Père: nombres d'octets recus : %d --> %s\n", n, buf);
                if(strchr(buf, '!')) break;
            }
        }
        wait(NULL);
        printf("Fin du Père\n");
    }

    close(fd);
    return 0;
}
