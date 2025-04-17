#ifndef COMMUNICATION_SYSTEM_H
#define COMMUNICATION_SYSTEM_H

#include "subsystem.h"
#include "environment_state.h"
#include <openssl/evp.h>
#include <atomic>
#include <thread>
#include <string>
#include "shared_data.h"
#include "resource_wrappers.h"

class CommunicationSystem : public Subsystem {
protected:
    std::atomic<bool> sender{false};
    std::atomic<bool> receiver{false};
public:
    explicit CommunicationSystem(std::string name, int runtime, SystemData& system, int order, EnvironmentState& envState, SharedGroundCommand& groundCommand);
    ~CommunicationSystem();
    bool midInit() override;
    void midSuspend() override;
    void restart() override;
    void midHalt() override;
    void midResume() override;
    void liveLoop() override;
    
    private:
    EnvironmentState& envState;
    SharedGroundCommand& groundCommand;
    std::mutex connectionMutex;
    int connectionSocket;
    
    unsigned char encryptionKey[32];
    CipherContext encryptCtx;
    CipherContext decryptCtx;
    
    ThreadGuard senderThread;
    ThreadGuard receiverThread;

    void setupConnection();
    void readEncryptionKeys(const std::string& keyFile);
    void senderLoop();
    void receiverLoop();
    bool reconnect();
    std::vector<uint8_t> encryptMessage(const std::vector<uint8_t>& message);
    std::vector<uint8_t> decryptMessage(const unsigned char* ciphertext, int length);

    bool checkConnection();
    bool sendData(const std::vector<uint8_t>& data, int socket);
    void restartConnection();
    std::vector<uint8_t> preparePayload();
    void processReceivedData(const uint8_t* data, size_t length);
    std::vector<uint8_t> compressData(const std::vector<uint8_t>& input);
    std::vector<uint8_t> decompressData(const std::vector<uint8_t>& input);
    std::vector<uint8_t> decryptMessage(const uint8_t* ciphertext, size_t length);
};

#endif