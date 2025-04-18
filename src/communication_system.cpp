#include "communication_system.h"
#include <fstream>
#include <chrono>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <openssl/aes.h>
#include <openssl/err.h>
#include <filesystem>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <openssl/rand.h>
#include <zlib.h>

CommunicationSystem::CommunicationSystem(std::string name, int runtime, SystemData& system, int order,
                    EnvironmentState& envState, SharedGroundCommand& groundCommand)
    : Subsystem(name, runtime, system, order), envState(envState), 
    groundCommand(groundCommand), connectionSocket(-1) {}

CommunicationSystem::~CommunicationSystem() { halt(); }

bool CommunicationSystem::midInit() {
    sender.store(true);
    receiver.store(true);
    try {
        readEncryptionKeys("/home/eren/GitHub/ControlSystem/encryption_keys.bin");
        setupConnection();
        return connectionSocket >= 0;
    } catch (const std::exception& e) {
        std::cerr << "Init failed: " << e.what() << std::endl;
        return false;
    }
}

void CommunicationSystem::liveLoop() {
    if (!senderThread.joinable()) {
    senderThread.move(std::thread(&CommunicationSystem::senderLoop, this));
    }
    if (!receiverThread.joinable()) {
    receiverThread.move(std::thread(&CommunicationSystem::receiverLoop, this));
    }
}

bool CommunicationSystem::isLive() const { return sender.load() && receiver.load(); }

void CommunicationSystem::midHalt() {
    sender.store(false);
    receiver.store(false);

    {
        std::lock_guard<std::mutex> lock(connectionMutex);
        if (connectionSocket >= 0) {
            shutdown(connectionSocket, SHUT_RDWR);
            close(connectionSocket);
            connectionSocket = -1;
        }
    }

    auto safeJoin = [](std::thread& t) {
        if (t.joinable()) {
            t.join();
        }
    };

    senderThread.join();
    receiverThread.join();

    if (encryptCtx) EVP_CIPHER_CTX_free(encryptCtx);
    if (decryptCtx) EVP_CIPHER_CTX_free(decryptCtx);
    EVP_cleanup();
}

void CommunicationSystem::setupConnection() {
    std::lock_guard<std::mutex> lock(connectionMutex);
    if (connectionSocket >= 0) return;

    int sock = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (sock < 0) throw std::runtime_error("Socket creation failed");

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8080);
    inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr);

    int result = connect(sock, (sockaddr*)&serverAddr, sizeof(serverAddr));
    if (result < 0 && errno != EINPROGRESS) {
        close(sock);
        throw std::runtime_error("Connect error: " + std::string(strerror(errno)));
    }

    fd_set set;
    FD_ZERO(&set);
    FD_SET(sock, &set);
    timeval timeout{.tv_sec = 5, .tv_usec = 0};

    if (select(sock + 1, nullptr, &set, nullptr, &timeout) <= 0) {
        close(sock);
        throw std::runtime_error("Connection timeout");
    }

    int error = 0;
    socklen_t len = sizeof(error);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
    if (error != 0) {
        close(sock);
        throw std::runtime_error("Connection failed: " + std::string(strerror(error)));
    }

    // Configure keepalive and timeouts
    int enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));
    timeval tv{.tv_sec = 5, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    connectionSocket = sock;
}

void CommunicationSystem::senderLoop() {
    constexpr auto SEND_INTERVAL = std::chrono::milliseconds(500);

    while (initialized.load()) {
        while (sender.load()) {
            try {
                int currentSocket;
                {
                    std::lock_guard<std::mutex> lock(connectionMutex);
                    currentSocket = connectionSocket;
                }

                if (currentSocket < 0) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }

                auto start = std::chrono::steady_clock::now();

                std::vector<uint8_t> payload = preparePayload();
                std::vector<uint8_t> compressed = compressData(payload);
                std::vector<uint8_t> encrypted = encryptMessage(compressed);

                if (!sendData(encrypted, currentSocket)) { // Pass currentSocket
                } else {
                    updateHeartbeat();
                }

                std::this_thread::sleep_until(start + SEND_INTERVAL);
            }
            catch (const std::exception& e) {
                std::cerr << "Sender error: " << e.what() << std::endl;
            }
        }
    }
}

bool CommunicationSystem::sendData(const std::vector<uint8_t>& data, int socket) {
    size_t totalSent = 0;
    const uint8_t* buffer = data.data();
    size_t remaining = data.size();

    while (remaining > 0) {
        fd_set writeSet;
        FD_ZERO(&writeSet);
        FD_SET(socket, &writeSet); // Use passed socket

        timeval timeout{.tv_sec = 5, .tv_usec = 0};
        int ready = select(socket + 1, nullptr, &writeSet, nullptr, &timeout);

        if (ready <= 0) return false;

        ssize_t sent = send(socket, buffer + totalSent, remaining, MSG_NOSIGNAL);
        if (sent <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            return false;
        }

        totalSent += sent;
        remaining -= sent;
    }
    return true;
}

void CommunicationSystem::receiverLoop() {
    constexpr size_t BUFFER_SIZE = 4096;
    std::vector<uint8_t> readBuffer(BUFFER_SIZE);
    std::vector<uint8_t> receiveBuffer;
    while (initialized.load()) {
        while (receiver.load()) {
            try {
                if (connectionSocket < 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }

                ssize_t bytesReceived = recv(connectionSocket, readBuffer.data(), readBuffer.size(), 0);
                if (bytesReceived > 0) {
                    receiveBuffer.insert(receiveBuffer.end(), readBuffer.begin(), readBuffer.begin() + bytesReceived);

                    while (true) {
                        if (receiveBuffer.size() < sizeof(uint32_t)) {
                            break;
                        }

                        uint32_t messageLength;
                        memcpy(&messageLength, receiveBuffer.data(), sizeof(messageLength));
                        messageLength = ntohl(messageLength);

                        if (receiveBuffer.size() < sizeof(messageLength) + messageLength) {
                            break;
                        }

                        const uint8_t* messageData = receiveBuffer.data() + sizeof(messageLength);
                        size_t dataLength = messageLength;

                        processReceivedData(messageData, dataLength);

                        receiveBuffer.erase(receiveBuffer.begin(), receiveBuffer.begin() + sizeof(messageLength) + messageLength);
                    }
                    updateHeartbeat();
                } else if (bytesReceived == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        continue;
                    }
                }

            } catch (const std::exception& e) {
                std::cerr << "Receiver error: " << e.what() << std::endl;
            }
        }
    }
}

void CommunicationSystem::restartConnection() {
    std::cout << "Attempting to reconnect..." << std::endl;
    {
        std::lock_guard<std::mutex> lock(connectionMutex);
        if (connectionSocket >= 0) {
            close(connectionSocket);
            connectionSocket = -1;
        }
    }

    for (int attempt = 0; attempt < 3; ++attempt) {
        try {
            if (midInit()) {liveLoop(); updateHeartbeat();};
        } catch (...) {
            // Log error if needed
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cerr << "Failed to reconnect after 3 attempts" << std::endl;
}

bool CommunicationSystem::checkConnection() {
    std::lock_guard<std::mutex> lock(connectionMutex);
    return connectionSocket >= 0;
}

std::vector<uint8_t> CommunicationSystem::preparePayload() {
    // Serialize environment and system data
    std::vector<uint8_t> envData = envState.serialize();
    std::vector<uint8_t> systemData = system.serialize();

    // 2. Validate sizes before packing
    const size_t envSize = envData.size();
    
    if (envSize < 49) { // Match Python expected minimum
        std::cerr << "Environment data too small: " << envSize << std::endl;
    }

    // Create combined payload with length headers
    std::vector<uint8_t> payload;
    payload.reserve(envData.size() + systemData.size() + 2*sizeof(uint32_t));

    // Add environment data with length prefix
    uint32_t envLen = htonl(static_cast<uint32_t>(envData.size()));
    payload.insert(payload.end(), 
                reinterpret_cast<uint8_t*>(&envLen),
                reinterpret_cast<uint8_t*>(&envLen) + sizeof(envLen));
    payload.insert(payload.end(), envData.begin(), envData.end());

    // Add system data with length prefix
    uint32_t sysLen = htonl(static_cast<uint32_t>(systemData.size()));
    payload.insert(payload.end(),
                reinterpret_cast<uint8_t*>(&sysLen),
                reinterpret_cast<uint8_t*>(&sysLen) + sizeof(sysLen));
    payload.insert(payload.end(), systemData.begin(), systemData.end());

    return payload;
}

void CommunicationSystem::processReceivedData(const uint8_t* data, size_t length) {
    try {
        std::vector<uint8_t> decrypted = decryptMessage(data, length);
        std::vector<uint8_t> decompressed = decompressData(decrypted);

        // Extract the single integer
        uint32_t netInt;
        memcpy(&netInt, decompressed.data(), sizeof(netInt));
        int32_t value = static_cast<int32_t>(ntohl(netInt));

        // Process the single integer command
        groundCommand.write(value);
    } catch (const std::exception& e) {
        std::cerr << "Failed to process received data: " << e.what() << std::endl;
        throw;
    }
}

std::vector<uint8_t> CommunicationSystem::encryptMessage(const std::vector<uint8_t>& message) {
    std::vector<uint8_t> iv(12);
    std::vector<uint8_t> ciphertext(message.size() + EVP_MAX_BLOCK_LENGTH);
    std::vector<uint8_t> tag(16);
    int ciphertext_len = 0, final_len = 0;

    // Generate random IV
    if (RAND_bytes(iv.data(), iv.size()) != 1) {
        throw std::runtime_error("Failed to generate IV");
    }

    // Initialize encryption
    if (EVP_EncryptInit_ex(encryptCtx, nullptr, nullptr, encryptionKey, iv.data()) != 1) {
        throw std::runtime_error("Encryption initialization failed");
    }

    // Encrypt plaintext
    if (EVP_EncryptUpdate(encryptCtx, ciphertext.data(), &ciphertext_len,
                        message.data(), message.size()) != 1) {
        throw std::runtime_error("Encryption failed");
    }

    // Finalize encryption
    if (EVP_EncryptFinal_ex(encryptCtx, ciphertext.data() + ciphertext_len, &final_len) != 1) {
        throw std::runtime_error("Encryption finalization failed");
    }
    ciphertext_len += final_len;

    // Get authentication tag
    if (EVP_CIPHER_CTX_ctrl(encryptCtx, EVP_CTRL_GCM_GET_TAG, 16, tag.data()) != 1) {
        throw std::runtime_error("Failed to get authentication tag");
    }

    std::vector<uint8_t> result(iv.size() + ciphertext_len + tag.size());

    // Copy components using memcpy for exact size control
    uint8_t* ptr = result.data();
    memcpy(ptr, iv.data(), iv.size());
    ptr += iv.size();
    memcpy(ptr, ciphertext.data(), ciphertext_len);
    ptr += ciphertext_len;
    memcpy(ptr, tag.data(), tag.size());

    return result;
}

std::vector<uint8_t> CommunicationSystem::decryptMessage(const uint8_t* ciphertext, size_t length) {
    // Verify minimum length (12 IV + 16 tag + min 1 byte data)
    if (length < 29) {
        throw std::runtime_error("Message too short");
    }

    // Extract components
    const uint8_t* iv = ciphertext;  // First 12 bytes
    const uint8_t* encrypted = ciphertext + 12;  // After IV
    size_t encrypted_len = length - 12 - 16;  // Remaining minus tag
    const uint8_t* tag = ciphertext + length - 16;  // Last 16 bytes

    // Reset context
    EVP_CIPHER_CTX_reset(decryptCtx);

    // Initialize with cipher, key and IV
    if (1 != EVP_DecryptInit_ex(decryptCtx, EVP_aes_256_gcm(), 
                            nullptr, encryptionKey, iv)) {
        throw std::runtime_error("Decryption init failed");
    }

    // Decrypt
    std::vector<uint8_t> plaintext(encrypted_len + 16);
    int len;
    if (1 != EVP_DecryptUpdate(decryptCtx, plaintext.data(), &len, 
                            encrypted, encrypted_len)) {
        throw std::runtime_error("Decryption failed");
    }
    int plaintext_len = len;

    // Set expected tag
    if (1 != EVP_CIPHER_CTX_ctrl(decryptCtx, EVP_CTRL_GCM_SET_TAG, 16, (void*)tag)) {
        throw std::runtime_error("Failed to set tag");
    }

    // Final verification
    if (1 != EVP_DecryptFinal_ex(decryptCtx, plaintext.data() + plaintext_len, &len)) {
        char errbuf[256];
        ERR_error_string_n(ERR_get_error(), errbuf, sizeof(errbuf));
        std::cerr << "OpenSSL Error: " << errbuf << std::endl;
        throw std::runtime_error("Tag verification failed - Check IV/Key alignment");
    }
    plaintext_len += len;

    plaintext.resize(plaintext_len);
    return plaintext;
}

void CommunicationSystem::readEncryptionKeys(const std::string& keyFile) {
    namespace fs = std::filesystem;

    try {
        if (!fs::exists(keyFile)) {
            throw std::runtime_error("Key file not found: " + keyFile);
        }

        if (fs::file_size(keyFile) != 32) {
            throw std::runtime_error("Invalid 256-bit key size");
        }

        std::ifstream file(keyFile, std::ios::binary);
        if (!file.read(reinterpret_cast<char*>(encryptionKey), sizeof(encryptionKey))) {
            throw std::runtime_error("Failed to read key file");
        }

        // Initialize encryption context
        encryptCtx.reset();
        if (!encryptCtx) {
            throw std::runtime_error("Failed to create encryption context");
        }

        if (EVP_EncryptInit_ex(encryptCtx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) != 1) {
            char err[256];
            ERR_error_string_n(ERR_get_error(), err, sizeof(err));
            throw std::runtime_error(std::string("Encryption init failed: ") + err);
        }

        // Initialize decryption context
        decryptCtx.reset();
        if (!decryptCtx) {
            throw std::runtime_error("Failed to create decryption context");
        }

        if (EVP_DecryptInit_ex(decryptCtx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr) != 1) {
            char err[256];
            ERR_error_string_n(ERR_get_error(), err, sizeof(err));
            throw std::runtime_error(std::string("Decryption init failed: ") + err);
        }

        if (EVP_CIPHER_CTX_ctrl(encryptCtx, EVP_CTRL_GCM_SET_IVLEN, 12, nullptr) != 1 ||
            EVP_CIPHER_CTX_ctrl(decryptCtx, EVP_CTRL_GCM_SET_IVLEN, 12, nullptr) != 1) {
            throw std::runtime_error("Failed to set IV length");
        }
    }
    catch (const std::exception& e) {
        encryptCtx.cleanup();
        decryptCtx.cleanup();
        throw;
    }
}

std::vector<uint8_t> CommunicationSystem::compressData(const std::vector<uint8_t>& input) {
    z_stream zs;
    memset(&zs, 0, sizeof(zs));

    // Use faster compression with Z_BEST_SPEED
    if (deflateInit2(&zs, Z_BEST_SPEED, Z_DEFLATED, 15 + 16, 8, Z_DEFAULT_STRATEGY) != Z_OK) {
        throw std::runtime_error("deflateInit failed");
    }

    zs.next_in = const_cast<Bytef*>(input.data());
    zs.avail_in = input.size();

    std::vector<uint8_t> output(deflateBound(&zs, zs.avail_in));
    zs.next_out = output.data();
    zs.avail_out = output.size();

    int ret = deflate(&zs, Z_FINISH);
    if (ret != Z_STREAM_END) {
        deflateEnd(&zs);
        throw std::runtime_error("deflate failed: " + std::to_string(ret));
    }

    output.resize(zs.total_out);
    deflateEnd(&zs);
    return output;
}

std::vector<uint8_t> CommunicationSystem::decompressData(const std::vector<uint8_t>& input) {
    z_stream zs;
    memset(&zs, 0, sizeof(zs));

    // Use raw deflate format (matches Python's wbits=-MAX_WBITS)
    if (inflateInit2(&zs, -MAX_WBITS) != Z_OK) {
        throw std::runtime_error("inflateInit failed");
    }

    zs.next_in = const_cast<Bytef*>(input.data());
    zs.avail_in = input.size();

    std::vector<uint8_t> output(4096);
    int ret;

    do {
        if (zs.total_out >= output.size()) {
            output.resize(output.size() * 2);
        }
        zs.next_out = output.data() + zs.total_out;
        zs.avail_out = output.size() - zs.total_out;

        ret = inflate(&zs, Z_NO_FLUSH);
        if (ret != Z_OK && ret != Z_STREAM_END) {
            inflateEnd(&zs);
            throw std::runtime_error("inflate failed: " + std::to_string(ret));
        }
    } while (ret != Z_STREAM_END);

    output.resize(zs.total_out);
    inflateEnd(&zs);
    return output;
}

void CommunicationSystem::midSuspend() {
    system.addPacket(0, 0, 1, 2, order);
    sender.store(false);
}

void CommunicationSystem::midResume() {
    sender.store(true);
    system.addPacket(0, 0, 1, 3, order);
}

void CommunicationSystem::restart() {
    system.addPacket(1, 9, 0, 2, order);
    restarting.store(true);
    restartConnection();
}