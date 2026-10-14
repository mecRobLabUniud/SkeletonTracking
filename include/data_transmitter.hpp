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

// ── Parameters ──────────────────────────────────────────────────────────────
namespace params {
    constexpr int H = 480;
    constexpr int W = 848;
    constexpr int C = 3;
}

// ─────────────────────────────────────────────────────────────────────────────
// POSIX shared-memory segment (create or attach) with best-effort cleanup
// ─────────────────────────────────────────────────────────────────────────────
class SharedMemoryManager {
public:
    // ─────────────────────────────────────────────────────────────────────────────
    // Open or create the named segment with the given size
    // ─────────────────────────────────────────────────────────────────────────────
    SharedMemoryManager(const std::string& name, size_t size, bool create)
        : name_(name), size_(size), create_(create) {
        open();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Release the mapping and unlink the segment if owned
    // ─────────────────────────────────────────────────────────────────────────────
    ~SharedMemoryManager() {
        try {
            shutdown();
        } catch (...) {
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Copy constructor, deleted to keep a single owner of the mapping
    // ─────────────────────────────────────────────────────────────────────────────
    SharedMemoryManager(const SharedMemoryManager&) = delete;

    // ─────────────────────────────────────────────────────────────────────────────
    // Copy assignment, deleted to keep a single owner of the mapping
    // ─────────────────────────────────────────────────────────────────────────────
    SharedMemoryManager& operator=(const SharedMemoryManager&) = delete;

    // ─────────────────────────────────────────────────────────────────────────────
    // Take ownership of another segment, invalidating the source
    // ─────────────────────────────────────────────────────────────────────────────
    SharedMemoryManager(SharedMemoryManager&& other) noexcept { *this = std::move(other); }

    // ─────────────────────────────────────────────────────────────────────────────
    // Transfer ownership of another segment into this manager
    // ─────────────────────────────────────────────────────────────────────────────
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

    // ─────────────────────────────────────────────────────────────────────────────
    // Unmap the segment and close its file descriptor
    // ─────────────────────────────────────────────────────────────────────────────
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

    // ─────────────────────────────────────────────────────────────────────────────
    // Remove the segment from the system if this manager created it
    // ─────────────────────────────────────────────────────────────────────────────
    void unlink() {
        if (!create_) return;
        shm_unlink(("/" + name_).c_str());
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Close the mapping and unlink the segment
    // ─────────────────────────────────────────────────────────────────────────────
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

    // ─────────────────────────────────────────────────────────────────────────────
    // Create the segment or attach to an existing one
    // ─────────────────────────────────────────────────────────────────────────────
    void open() {
        if (create_) {
            create_or_replace();
        } else {
            attach();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Open the segment, replacing a stale one left by a previous run
    // ─────────────────────────────────────────────────────────────────────────────
    void create_or_replace() {
        std::string posix_name = "/" + name_;
        fd_ = shm_open(posix_name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
        if (fd_ == -1 && errno == EEXIST) {
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

    // ─────────────────────────────────────────────────────────────────────────────
    // Wait for the producer and attach to its segment
    // ─────────────────────────────────────────────────────────────────────────────
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

    // ─────────────────────────────────────────────────────────────────────────────
    // Map the segment into this process address space
    // ─────────────────────────────────────────────────────────────────────────────
    void map() {
        ptr_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED) {
            ptr_ = nullptr;
            throw std::runtime_error("mmap failed for shared memory: " + name_);
        }
    }
};


// ─────────────────────────────────────────────────────────────────────────────
// ZeroMQ socket paired with a shared-memory image buffer, used to stream
// frames and serialized data between the C++ and Python processes
// ─────────────────────────────────────────────────────────────────────────────
class DataTransmitter {
public:
    enum class Mode { Sender, Receiver };

    // ─────────────────────────────────────────────────────────────────────────────
    // Set up the ZeroMQ socket and shared memory for the given mode
    // ─────────────────────────────────────────────────────────────────────────────
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

    // ─────────────────────────────────────────────────────────────────────────────
    // Move constructor
    // ─────────────────────────────────────────────────────────────────────────────
    DataTransmitter(DataTransmitter&&) noexcept = default;

    // ─────────────────────────────────────────────────────────────────────────────
    // Move assignment
    // ─────────────────────────────────────────────────────────────────────────────
    DataTransmitter& operator=(DataTransmitter&&) noexcept = default;

    // ─────────────────────────────────────────────────────────────────────────────
    // Shut down the socket and shared memory
    // ─────────────────────────────────────────────────────────────────────────────
    ~DataTransmitter() {
        try {
            shutdown();
        } 
        catch (...) {
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Copy constructor, deleted because the socket and mapping are unique
    // ─────────────────────────────────────────────────────────────────────────────
    DataTransmitter(const DataTransmitter&) = delete;

    // ─────────────────────────────────────────────────────────────────────────────
    // Copy assignment, deleted because the socket and mapping are unique
    // ─────────────────────────────────────────────────────────────────────────────
    DataTransmitter& operator=(const DataTransmitter&) = delete;

    // ─────────────────────────────────────────────────────────────────────────────
    // Publish a topic-tagged list of JSON arrays on the socket (sender mode)
    // ─────────────────────────────────────────────────────────────────────────────
    void send_data(const std::vector<nlohmann::json>& arrays) {
        std::string msg = topic_ + "_" + std::to_string(device_id_);
        for (const auto& elem : arrays) {
            msg += "; " + elem.dump();
        }
        socket_->send(zmq::buffer(msg), zmq::send_flags::none);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Publish a two-element RULA score on the socket (sender mode)
    // ─────────────────────────────────────────────────────────────────────────────
    void send_rula_score(const std::array<int, 2>& score) {
        require(Mode::Sender);

        json score_json = json::array();
        score_json.push_back({score[0], score[1]});

        std::string msg = topic_ + "_" + std::to_string(device_id_) + "; " +
                           score_json.dump();

        socket_->send(zmq::buffer(msg), zmq::send_flags::none);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Block for and return the next raw message (receiver mode)
    // ─────────────────────────────────────────────────────────────────────────────
    std::string receive_packed_msg() {
        require(Mode::Receiver);
        zmq::message_t zmsg;
        auto result = socket_->recv(zmsg, zmq::recv_flags::none);
        (void)result;
        return std::string(static_cast<char*>(zmsg.data()), zmsg.size());
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Parse the next message into a list of JSON values (receiver mode)
    // ─────────────────────────────────────────────────────────────────────────────
    std::vector<nlohmann::json> receive_data() {
        std::string packed = receive_packed_msg();

        std::vector<std::string> parts;
        size_t start = 0, pos;
        while ((pos = packed.find("; ", start)) != std::string::npos) {
            parts.push_back(packed.substr(start, pos - start));
            start = pos + 2;
        }
        parts.push_back(packed.substr(start));

        static const std::regex nan_re(R"(\bNaN\b)");

        std::vector<nlohmann::json> result;
        result.reserve(parts.size() - 1);
        for (size_t i = 1; i < parts.size(); ++i) {
            std::string sanitized = std::regex_replace(parts[i], nan_re, "null");
            result.push_back(nlohmann::json::parse(sanitized));
        }
        return result;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Close the socket and release the shared memory
    // ─────────────────────────────────────────────────────────────────────────────
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

    // ─────────────────────────────────────────────────────────────────────────────
    // Throw unless the transmitter is in the expected mode
    // ─────────────────────────────────────────────────────────────────────────────
    void require(Mode expected) {
        if (mode_ != expected) {
            throw std::runtime_error("DataTransmitter method called in wrong mode");
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Bind a publisher socket on the transmitter port
    // ─────────────────────────────────────────────────────────────────────────────
    void setup_zmq_sender() {
        try {
            socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::pub);
            int linger = 0;
            socket_->set(zmq::sockopt::linger, linger);
            socket_->set(zmq::sockopt::sndhwm, 1);
            socket_->bind("tcp://*:" + std::to_string(port_));
        } catch (const std::exception& e) {
            std::cerr << "ZMQ sender setup failed: " << e.what() << std::endl;
            socket_.reset();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Connect a subscriber socket to the transmitter topic
    // ─────────────────────────────────────────────────────────────────────────────
    void setup_zmq_receiver() {
        socket_ = std::make_unique<zmq::socket_t>(ctx_, zmq::socket_type::sub);
        socket_->set(zmq::sockopt::conflate, 1);
        std::string filter = topic_ + "_" + std::to_string(device_id_);
        socket_->set(zmq::sockopt::subscribe, filter);
        socket_->connect("tcp://localhost:" + std::to_string(port_));
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Create the shared frame buffer owned by this transmitter
    // ─────────────────────────────────────────────────────────────────────────────
    void setup_shm_sender() {
        shm_ = std::make_unique<SharedMemoryManager>(
            "shared_image" + std::to_string(device_id_), nbytes_, true);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Attach to the shared frame buffer owned by another process
    // ─────────────────────────────────────────────────────────────────────────────
    void setup_shm_receiver() {
        shm_ = std::make_unique<SharedMemoryManager>(
            "shared_image" + std::to_string(device_id_), nbytes_, false);
    }

};
