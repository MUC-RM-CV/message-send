#ifndef MUC_CLEMENTINE_MESSAGESEND_H
#define MUC_CLEMENTINE_MESSAGESEND_H

#include <iostream>
#include <cmath>
#include <string>

#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

class MessageSend {
private:
    char missedData[8] = {'a', '0', '0', '0', '0', '0', '0', '0'};
    char data[8];
    bool opened = false;
    int fd = -1;
public:
    MessageSend() { init(); }
    void init() {
        std::string path = "/dev/ttyTHS2";
        int fd = -1; 
        int err;

        fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);

        if (fd < 0) {
            std::cerr << "Cannot open serial port!\n";
            opened = false;
            return;
        }

        termios opt;
        tcgetattr(fd, &opt);

        cfsetispeed(&opt, B9600);
        cfsetospeed(&opt, B9600);

        opt.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
        opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        opt.c_oflag &= ~(OPOST);
        opt.c_cflag &= ~(CSIZE | PARENB);
        opt.c_cflag |= CS8;

        opt.c_cc[VMIN] = 0xFF;
        opt.c_cc[VTIME] = 150;

        if (tcsetattr(fd, TCSANOW, &opt) < 0) {
            opened = false;
            return;
        }
        opened = true;
    }

    bool isOpened() { return opened; }

    bool sendMessage(bool fire, int yawAngle, int pitchAngle) {
        data[0] = 'a' - '0';
        data[1] = yawAngle < 0 ? 0 : 1;
        data[2] = abs(yawAngle) / 10;
        data[3] = abs(yawAngle) % 10;
        data[4] = pitchAngle < 0 ? 0 : 1;
        data[5] = abs(pitchAngle) / 10;
        data[6] = abs(pitchAngle) % 10;
        data[7] = (abs(pitchAngle) + abs(yawAngle)) % 10;
        for (auto & val : data) val += '0';

        if (write(fd, data, sizeof(data)) == -1) return false;
        else return true;
    }
    auto getData() { return data; }
};


#endif //MUC_CLEMENTINE_MESSAGESEND_H
