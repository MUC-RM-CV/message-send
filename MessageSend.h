#ifndef MUC_CLEMENTINE_MESSAGESEND_H
#define MUC_CLEMENTINE_MESSAGESEND_H


class MessageSend {
private:
    char missedData[8] = {'a', '0', '0', '0', '0', '0', '0', '0'};
    char data[8];
public:
    MessageSend();
    void init() {
        std::string path = "/dev/ttySH02";
    }
    void sendMessage(bool fire, int yawAngle, int pitchAngle) {
        data[0] = 'a' - '0';
        data[1] = yawAngle < 0 ? 0 : 1;
        data[2] = abs(yawAngle) / 10;
        data[3] = abs(yawAngle) % 10;
        data[4] = pitchAngle < 0 ? 0 : 1;
        data[5] = abs(pitchAngle) / 10;
        data[6] = abs(pitchAngle) % 10;
        data[7] = (abs(pitchAngle) + abs(yawAngle)) % 10;
        for (auto & val : data) data += '0';
    }
    auto getData() { return data; }
};


#endif //MUC_CLEMENTINE_MESSAGESEND_H
