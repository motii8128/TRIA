#include "ydlidar_tmini_driver/lidar.hpp"

namespace ydlidar_tmini {

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kReadTimeoutMs = 500;
constexpr int kStartTimeoutMs = 3000;      // スキャン開始応答 (モーター立ち上がり込み)
constexpr int kFirstDataTimeoutMs = 5000;  // 開始後、最初のデータが来るまでの上限
constexpr int kMaxSyncBytes = 8192;        // 応答ヘッダ (A5 5A) 探索で読み捨てる最大バイト数

std::string errnoMessage(const char* what) {
    return std::string(what) + ": " + std::strerror(errno);
}

}  // namespace

Lidar::Lidar(const std::string& path) {
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw IoError(errnoMessage(("open " + path).c_str()));

    try {
        termios tio{};
        if (::tcgetattr(fd_, &tio) != 0) throw IoError(errnoMessage("tcgetattr"));
        ::cfmakeraw(&tio);                    // 8N1, 生データ
        tio.c_cflag |= (CLOCAL | CREAD);      // モデム制御線を無視, 受信有効
        tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CRTSCTS);
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (::cfsetispeed(&tio, B230400) != 0 || ::cfsetospeed(&tio, B230400) != 0) {
            throw IoError(errnoMessage("cfsetspeed"));
        }
        if (::tcsetattr(fd_, TCSANOW, &tio) != 0) throw IoError(errnoMessage("tcsetattr"));

        stopScan();
    } catch (...) {
        ::close(fd_);
        fd_ = -1;
        throw;
    }
}

Lidar::~Lidar() {
    if (fd_ >= 0) {
        if (scanning_) {
            // ベストエフォートでモーターとレーザーを止める
            const auto& c = cmd::kStopScan;
            const ssize_t ignored = ::write(fd_, c.data(), c.size());
            (void)ignored;
        }
        ::close(fd_);
    }
}

// ---- 低レベル I/O -----------------------------------------------------------

void Lidar::writeAll(const std::uint8_t* data, std::size_t n) {
    while (n > 0) {
        const ssize_t w = ::write(fd_, data, n);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN) {
                pollfd pf{fd_, POLLOUT, 0};
                ::poll(&pf, 1, 1000);
                continue;
            }
            throw IoError(errnoMessage("write"));
        }
        data += w;
        n -= static_cast<std::size_t>(w);
    }
    ::tcdrain(fd_);
}

std::size_t Lidar::readSome(std::uint8_t* buf, std::size_t max, int timeout_ms) {
    pollfd pf{fd_, POLLIN, 0};
    const int r = ::poll(&pf, 1, timeout_ms);
    if (r < 0) {
        if (errno == EINTR) return 0;
        throw IoError(errnoMessage("poll"));
    }
    if (r == 0) throw TimeoutError();
    if (pf.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        if (!(pf.revents & POLLIN)) throw IoError("serial port error or device disconnected");
    }
    const ssize_t n = ::read(fd_, buf, max);
    if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) return 0;
        throw IoError(errnoMessage("read"));
    }
    return static_cast<std::size_t>(n);
}

void Lidar::readExact(std::uint8_t* buf, std::size_t n, int timeout_ms) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    std::size_t got = 0;
    while (got < n) {
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        if (remaining <= 0) throw TimeoutError();
        got += readSome(buf + got, n - got, static_cast<int>(remaining));
    }
}

void Lidar::flushInput() { ::tcflush(fd_, TCIFLUSH); }

ResponseHeader Lidar::readResponseHeader(int timeout_ms) {
    std::uint8_t prev = 0;
    std::uint8_t b = 0;
    for (int i = 0; i < kMaxSyncBytes; ++i) {
        readExact(&b, 1, timeout_ms);
        if (prev == 0xA5 && b == 0x5A) {
            std::uint8_t raw[kResponseHeaderLen] = {0xA5, 0x5A, 0, 0, 0, 0, 0};
            readExact(raw + 2, kResponseHeaderLen - 2, timeout_ms);
            return ResponseHeader::parse(raw);
        }
        prev = b;
    }
    throw ProtocolError("response header (A5 5A) not found");
}

std::vector<std::uint8_t> Lidar::request(const std::array<std::uint8_t, 2>& command,
                                         std::uint8_t expect_type, std::uint32_t expect_len) {
    if (scanning_) {
        throw StateError("commands other than stop are not allowed while scanning");
    }
    flushInput();
    writeCommand(command);

    const ResponseHeader h = readResponseHeader(1000);
    if (h.type_code != expect_type || h.length != expect_len) {
        throw ProtocolError("unexpected response: type=" + std::to_string(h.type_code) +
                            " len=" + std::to_string(h.length) +
                            " (expected type=" + std::to_string(expect_type) +
                            " len=" + std::to_string(expect_len) + ")");
    }
    std::vector<std::uint8_t> payload(h.length);
    readExact(payload.data(), payload.size(), 1000);
    return payload;
}

// ---- 単発コマンド -----------------------------------------------------------

DeviceInfo Lidar::deviceInfo() {
    return DeviceInfo::parse(
        request(cmd::kDeviceInfo, type_code::kDeviceInfo, DeviceInfo::kPayloadLen));
}

Health Lidar::health() {
    return Health::parse(request(cmd::kHealth, type_code::kHealth, Health::kPayloadLen));
}

float Lidar::scanFrequency() {
    return parseFrequency(request(cmd::kGetFreq, type_code::kFrequency, 4));
}

float Lidar::adjustScanFrequency(FreqStep step) {
    const std::array<std::uint8_t, 2>* c = nullptr;
    switch (step) {
        case FreqStep::FineUp: c = &cmd::kFreqUp0_1Hz; break;
        case FreqStep::FineDown: c = &cmd::kFreqDown0_1Hz; break;
        case FreqStep::CoarseUp: c = &cmd::kFreqUp1Hz; break;
        case FreqStep::CoarseDown: c = &cmd::kFreqDown1Hz; break;
    }
    return parseFrequency(request(*c, type_code::kFrequency, 4));
}

// ---- スキャン ---------------------------------------------------------------

void Lidar::startScan() {
    if (scanning_) return;
    flushInput();
    parser_.clear();
    assembler_.reset();

    writeCommand(cmd::kStartScan);
    const ResponseHeader h = readResponseHeader(kStartTimeoutMs);
    if (h.type_code != type_code::kScan || h.mode != kModeContinuous) {
        throw ProtocolError("unexpected scan response: type=" + std::to_string(h.type_code) +
                            " mode=" + std::to_string(h.mode));
    }
    scanning_ = true;
}

void Lidar::stopScan() {
    writeCommand(cmd::kStopScan);
    scanning_ = false;
    // 停止コマンドは無応答。飛んでくるデータが止まるのを待ってから捨てる。
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    flushInput();
    parser_.clear();
    assembler_.reset();
}

Scan Lidar::readScan() {
    if (!scanning_) throw StateError("not scanning; call startScan() first");

    std::uint8_t buf[4096];
    const auto deadline = Clock::now() + std::chrono::milliseconds(kFirstDataTimeoutMs);
    bool got_data = false;
    for (;;) {
        while (auto pkt = parser_.next()) {
            if (auto scan = assembler_.push(*pkt)) return std::move(*scan);
        }
        try {
            const std::size_t n = readSome(buf, sizeof buf, kReadTimeoutMs);
            got_data = true;
            parser_.push(buf, n);
        } catch (const TimeoutError&) {
            if (!got_data && Clock::now() < deadline) continue;  // モーター立ち上がり待ち
            throw;
        }
    }
}

}  // namespace ydlidar_tmini