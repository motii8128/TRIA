// シリアルポート (POSIX termios) 経由で T-mini Plus / Pro を操作するドライバ。
// Linux / macOS 向け。Windows は protocol.hpp を使って別途ポート I/O を書くこと。
#ifndef YDLIDAR_LIDAR_HPP_
#define YDLIDAR_LIDAR_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#include "error.hpp"
#include "protocol.hpp"

namespace ydlidar_tmini {

/// 回転周波数の調整ステップ。
enum class FreqStep {
    FineUp,      // +0.1 Hz
    FineDown,    // -0.1 Hz
    CoarseUp,    // +1 Hz
    CoarseDown,  // -1 Hz
};

class Lidar {
public:
    /// ポートを開く (例: "/dev/ttyUSB0")。
    ///
    /// 前回のプロセスがスキャン中のまま終了していても大丈夫なように、
    /// 開いた直後に停止コマンドを送って受信バッファを空にする。
    explicit Lidar(const std::string& path);
    ~Lidar();

    Lidar(const Lidar&) = delete;
    Lidar& operator=(const Lidar&) = delete;

    bool isScanning() const { return scanning_; }

    /// 通信品質の目安: 捨てたパケット候補の数。
    std::uint64_t checksumErrors() const { return parser_.checksumErrors(); }

    // ---- 単発コマンド (スキャン停止中のみ) --------------------------------
    DeviceInfo deviceInfo();
    Health health();
    /// 現在の設定回転周波数 [Hz] (実測値ではない)。
    float scanFrequency();
    /// 回転周波数を 1 ステップ増減し、変更後の設定値 [Hz] を返す。
    float adjustScanFrequency(FreqStep step);

    // ---- スキャン -----------------------------------------------------------
    /// スキャン開始 (モーター始動 + レーザー点灯)。
    void startScan();
    /// スキャン停止。未スキャン時に呼んでも安全。
    void stopScan();
    /// 1 周分の点群が揃うまでブロックして返す。
    ///
    /// スキャン開始直後の最初の周は途中から始まるので捨てられ、2 周目から返る。
    /// モーター立ち上がり (約 0.5〜1 秒) の間は最大 5 秒待つ。
    /// データが途切れたら TimeoutError。呼び直せば続きから読める。
    Scan readScan();

private:
    int fd_ = -1;
    PacketParser parser_;
    ScanAssembler assembler_;
    bool scanning_ = false;

    void writeAll(const std::uint8_t* data, std::size_t n);
    void writeCommand(const std::array<std::uint8_t, 2>& c) { writeAll(c.data(), c.size()); }
    /// 少なくとも 1 バイト読む。timeout_ms 以内にデータが来なければ TimeoutError。
    std::size_t readSome(std::uint8_t* buf, std::size_t max, int timeout_ms);
    void readExact(std::uint8_t* buf, std::size_t n, int timeout_ms);
    void flushInput();
    ResponseHeader readResponseHeader(int timeout_ms);
    std::vector<std::uint8_t> request(const std::array<std::uint8_t, 2>& command,
                                      std::uint8_t expect_type, std::uint32_t expect_len);
};

}  // namespace ydlidar_tmini

#endif