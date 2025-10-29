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

    // Lire 0 caractère pour test (VMIN=0, lecture non bloquante)
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &settings);
    return fd;
}

int main() {
    int fd = init_serial();
    pid_t pid_read, pid_write;

    pid_read = fork();
    if(pid_read < 0) { perror("fork read"); exit(1); }
    if(pid_read == 0) { // enfant lecture
        char buf[256];
        int n;
        printf("Enfant Lecture actif...\n");
        while(1) {
            n = read(fd, buf, 256);
            if(n > 0) {
                buf[n] = '\0';
                printf("Lecture: %s\n", buf);
                if(strchr(buf, '!')) break;
            }
        }
        printf("Fin Enfant Lecture\n");
        exit(0);
    }

    pid_write = fork();
    if(pid_write < 0) { perror("fork write"); exit(1); }
    if(pid_write == 0) { // enfant écriture
        char input;
        printf("Enfant Écriture actif...\n");
        while(1) {
            input = getchar();
            if(input == 'q') break;
            write(fd, &input, 1);
        }
        printf("Fin Enfant Écriture\n");
        exit(0);
    }

    // Processus principal fait autre chose
    int n = 1;
    while(n < 10) {
        printf("Processus principal fait quelques trucs... (%d)\n", n);
        n++;
        sleep(3);
    }

    wait(NULL); // attendre enfant lecture
    wait(NULL); // attendre enfant écriture
    close(fd);
    printf("Fin du processus Principal\n");
    return 0;
}
