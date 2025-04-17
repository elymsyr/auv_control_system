#include <stdexcept>
#include <openssl/evp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <openssl/err.h>

class CipherContext
{
public:
    void reset(bool allocateNew = true) {
        cleanup();
        if (allocateNew) {
            ctx = EVP_CIPHER_CTX_new();
            if (!ctx) throw std::runtime_error("Failed to recreate EVP_CIPHER_CTX");
        }
    }

    ~CipherContext()
    {
        if (ctx)
        {
            EVP_CIPHER_CTX_free(ctx);
        }
    }

    CipherContext() : ctx(EVP_CIPHER_CTX_new()) {
        if (!ctx) {
            last_error_ = "EVP_CIPHER_CTX_new failed: ";
            last_error_ += ERR_error_string(ERR_get_error(), nullptr);
            throw std::runtime_error(last_error_);
        }
    }

    CipherContext(CipherContext&& other) noexcept : ctx(other.ctx) {
        other.ctx = nullptr;
    }

    CipherContext &operator=(CipherContext &&other) noexcept
    {
        if (this != &other)
        {
            if (ctx)
                EVP_CIPHER_CTX_free(ctx);
            ctx = other.ctx;
            other.ctx = nullptr;
        }
        return *this;
    }

    operator EVP_CIPHER_CTX *() { return ctx; }
    bool valid() const noexcept { return ctx != nullptr; }

    void cleanup() {
        if (ctx) {
            EVP_CIPHER_CTX_free(ctx);
            ctx = nullptr;
        }
    }

private:
    std::string last_error_;
    EVP_CIPHER_CTX *ctx;
    CipherContext(const CipherContext &) = delete;
    CipherContext &operator=(const CipherContext &) = delete;
};

// 2. Socket Handle Wrapper
class SocketHandle
{
public:
    explicit SocketHandle(int fd = -1) : fd_(fd) {}

    ~SocketHandle()
    {
        if (fd_ >= 0)
        {
            ::shutdown(fd_, SHUT_RDWR);
            ::close(fd_);
        }
    }

    // Move semantics
    SocketHandle(SocketHandle &&other) noexcept : fd_(other.fd_)
    {
        other.fd_ = -1;
    }

    SocketHandle &operator=(int other) noexcept
    {
        reset();
        fd_ = other;
        return *this;
    }

    SocketHandle& operator=(const SocketHandle& other) noexcept {
        if (this != &other) {
            reset();  // Proper cleanup before takeover
            std::lock_guard<std::mutex> lock(mtx_);
            fd_ = other.fd_;
        }
        return *this;
    }

    explicit operator bool() const noexcept { return fd_ >= 0; }

    bool operator<(int other) const noexcept
    {
        return fd_ < other;
    }

    int get() const { std::lock_guard<std::mutex> lk(mtx_); return fd_; }
    int release()
    {
        int temp = fd_;
        fd_ = -1;
        return temp;
    }

    void reset()
    {
        if (fd_ >= 0)
        {
            ::shutdown(fd_, SHUT_RDWR);
            ::close(fd_);
            fd_ = -1;
        }
    }
    bool valid() const
    {
        return fd_ >= 0;
    }

private:
    int fd_;
    SocketHandle(const SocketHandle &) = delete;
protected:
    mutable std::mutex mtx_;
};

// 3. Thread Guard
class ThreadGuard
{
public:
    explicit ThreadGuard(std::thread &&t) : t_(std::move(t)) {}
    
    ThreadGuard() : t_() {}

    ~ThreadGuard() {
        if(t_.joinable()) {
            try { t_.join(); } 
            catch(...) { /* Log exception */ }
        }
    }

    ThreadGuard(ThreadGuard &&) = default;
    ThreadGuard &operator=(ThreadGuard &&) = default;
    std::thread::native_handle_type get() { return t_.native_handle(); }

    // Safe accessors
    bool joinable() const noexcept { return t_.joinable(); }
    std::thread& thread() noexcept { return t_; }
    bool join() {
        if (t_.joinable()) {
            t_.join();
            return true;
        }
        return false;
    }
    bool move(std::thread&& t) {
        if (t_.joinable()) {
            return false;
        }
        t_ = std::move(t);
        return true;
    }
private:
    std::thread t_;
};
