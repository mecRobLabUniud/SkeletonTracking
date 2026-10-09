/*
░█▀▄░█▀█░▀█▀░█▀█░░░▀█▀░█▀▄░█▀█░█▀█░█▀▀░█▄█░▀█▀░▀█▀░▀█▀░█▀▀░█▀▄
░█░█░█▀█░░█░░█▀█░░░░█░░█▀▄░█▀█░█░█░▀▀█░█░█░░█░░░█░░░█░░█▀▀░█▀▄
░▀▀░░▀░▀░░▀░░▀░▀░░░░▀░░▀░▀░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀▀░░▀░░░▀░░▀▀▀░▀░▀
*/

#pragma once

#include <zmq.hpp>
#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <regex>

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────
namespace params {
    constexpr int H = 480;
    constexpr int W = 848;
    constexpr int C = 3;
}

// ─────────────────────────────────────────────────────────────────────────────
// Shared Memory Manager
// ─────────────────────────────────────────────────────────────────────────────
class SharedMemoryManager {
public:
    SharedMemoryManager(const std::string& name, size_t size, bool create)
        : name_(name), size_(size), create_(create) {
        open();
    }

    ~SharedMemoryManager() {
        try {
            shutdown();
        } catch (...) {
            // swallow, mirroring Python's __del__ best-effort cleanup
        }
    }

    // Non-copyable, movable
    SharedMemoryManager(const SharedMemoryManager&) = delete;
    SharedMemoryManager& operator=(const SharedMemoryManager&) = delete;

    SharedMemoryManager(SharedMemoryManager&& other) noexcept { *this = std::move(other); }
    SharedMemoryManager& operator=(SharedMemoryManager&& other) noexcept {
        if (this != &other) {
            shutdown();
            name_ = std::move(other.name_);
            size_ = other.size_;
            create_ = other.create_;
            fd_ = other.fd_;
            ptr_ = other.ptr_;
            other.fd_ = -1;
            other.ptr_ = nullptr;
        }
        return *this;
    }

    // ── Shutdown block ───────────────────────────────────────────────────────
    void close() {
        if (ptr_ != nullptr) {
            munmap(ptr_, size_);
            ptr_ = nullptr;
        }
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void unlink() {
        if (!create_) return;
        shm_unlink(("/" + name_).c_str()); // ignore errors, mirrors Python's broad except
    }

    void shutdown() {
        close();
        unlink();
    }

private:
    std::string name_;
    size_t size_;
    bool create_;
    int fd_ = -1;
    void* ptr_ = nullptr;

    void open() {
        if (create_) {
            create_or_replace();
        } else {
            attach();
        }
    }

    void create_or_replace() {
        std::string posix_name = "/" + name_;
        fd_ = shm_open(posix_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
        if (fd_ == -1 && errno == EEXIST) {
            // Stale segment from a previous crashed run - unlink and retry,
            // mirroring the Python FileExistsError fallback.
            shm_unlink(posix_name.c_str());
            fd_ = shm_open(posix_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
        }
        if (fd_ == -1) {
            throw std::runtime_error("Failed to create shared memory: " + name_);
        }
        if (ftruncate(fd_, static_cast<off_t>(size_)) == -1) {
            throw std::runtime_error("Failed to size shared memory: " + name_);
        }
        map();
    }

    void attach() {
        std::string posix_name = "/" + name_;
        bool attached = false;
        while (!attached) {
            fd_ = shm_open(posix_name.c_str(), O_RDWR, 0666);
            if (fd_ != -1) {
                attached = true;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        map();
    }

    void map() {
        ptr_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            ptr_ = nullptr;
            throw std::runtime_error("mmap failed for shared memory: " + name_);
        }
    }
};


// ─────────────────────────────────────────────────────────────────────────────
// Data transmitter
// ─────────────────────────────────────────────────────────────────────────────
class DataTransmitter {
public:
    enum class Mode { Sender, Receiver };

    DataTransmitter(Mode mode, int device_id, const std::string& topic, int port = 6000)
        : mode_(mode),
          device_id_(device_id),
          port_(port + device_id),
          topic_(topic),
          nbytes_(static_cast<size_t>(params::H) * params::W * params::C) {
        if (mode_ == Mode::Sender) {
            setup_zmq_sender();
            setup_shm_sender();
        } else {
            setup_zmq_receiver();
            setup_shm_receiver();
        }
    }

    DataTransmitter(DataTransmitter&&) noexcept = default;
    DataTransmitter& operator=(DataTransmitter&&) noexcept = default;

    ~DataTransmitter() {
        try {
            shutdown();
        } 
        catch (...) {
        }
    }

    DataTransmitter(const DataTransmitter&) = delete;
    DataTransmitter& operator=(const DataTransmitter&) = delete;

    // ── Send block (sender mode only) ───────────────────────────────────────
    void send_data(const std::vector<nlohmann::json>& arrays) {
        std::string msg = topic_ + "_" + std::to_string(device_id_);
        for (const auto& elem : arrays) {
            msg += "; " + elem.dump();
        }
        socket_->send(zmq::buffer(msg), zmq::send_flags::none);
    }

    void send_rula_score(const std::array<int, 2>& score) {
        require(Mode::Sender);

        json score_json = json::array();
        score_json.push_back({score[0], score[1]});

        std::string msg = topic_ + "_" + std::to_string(device_id_) + "; " +
                           score_json.dump();

        socket_->send(zmq::buffer(msg), zmq::send_flags::none);
    }

    // ── Receive block (receiver mode only) ──────────────────────────────────
    std::string receive_packed_msg() {
        require(Mode::Receiver);
        zmq::message_t zmsg;
        auto result = socket_->recv(zmsg, zmq::recv_flags::none);
        (void)result;
        return std::string(static_cast<char*>(zmsg.data()), zmsg.size());
    }

    std::vector<nlohmann::json> receive_data() {
        std::string packed = receive_packed_msg();

        std::vector<std::string> parts;
        size_t start = 0, pos;
        while ((pos = packed.find("; ", start)) != std::string::npos) {
            parts.push_back(packed.substr(start, pos - start));
            start = pos + 2;
        }
        parts.push_back(packed.substr(start));

        // Replace bare NaN tokens with valid JSON null (word-boundary safe)
        static const std::regex nan_re(R"(\bNaN\b)");

        std::vector<nlohmann::json> result;
        result.reserve(parts.size() - 1);
        for (size_t i = 1; i < parts.size(); ++i) {
            std::string sanitized = std::regex_replace(parts[i], nan_re, "null");
            result.push_back(nlohmann::json::parse(sanitized));
        }
        return result;
    }

    // ── Shutdown ─────────────────────────────────────────────────────────────
    void shutdown() {
        if (socket_) {
            socket_->close();
            socket_.reset();
        }
        if (shm_) {
            shm_->shutdown();
            shm_.reset();
        }
    }

private:
    Mode mode_;
    int device_id_;
    int port_;
    std::string topic_;
    size_t nbytes_;

    zmq::context_t ctx_{1};
    std::unique_ptr<zmq::socket_t> socket_;
    std::unique_ptr<SharedMemoryManager> shm_;

    void require(Mode expected) {
        if (mode_ != expected) {
            throw std::runtime_error("DataTransmitter method called in wrong mode");
        }
    }

    // ── ZeroMQ setup ─────────────────────────────────────────────────────────
    void setup_zmq_sender() {
        try {
            socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pub);
            int linger = 0;
            socket_->set(zmq::sockopt::linger, linger);
            socket_->set(zmq::sockopt::sndhwm, 1);
            socket_->bind("tcp://*:" + std::to_string(port_));
        } catch (const std::exception& e) {
            // mirrors Python's broad "except Exception: pass"
            std::cerr << "ZMQ sender setup failed: " << e.what() << std::endl;
            socket_.reset();
        }
    }

    void setup_zmq_receiver() {
        socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::sub);
        socket_->set(zmq::sockopt::conflate, 1);
        std::string filter = topic_ + "_" + std::to_string(device_id_);
        socket_->set(zmq::sockopt::subscribe, filter);
        socket_->connect("tcp://localhost:" + std::to_string(port_));
    }

    // ── Shared memory setup ──────────────────────────────────────────────────
    void setup_shm_sender() {
        shm_ = std::make_unique<SharedMemoryManager>(
            "shared_image" + std::to_string(device_id_), nbytes_, /*create=*/true);
    }

    void setup_shm_receiver() {
        shm_ = std::make_unique<SharedMemoryManager>(
            "shared_image" + std::to_string(device_id_), nbytes_, /*create=*/false);
    }

};
