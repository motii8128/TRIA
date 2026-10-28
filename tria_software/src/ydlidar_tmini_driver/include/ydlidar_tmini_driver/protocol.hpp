// YDLIDAR T-mini Plus / T-mini Pro のバイト列プロトコル (I/O 非依存・ヘッダのみ)。
//
// 仕様は "YDLIDAR T-mini Plus Development Manual" に基づく。
// T-mini Pro も同じパケット形式で読めることを実機で確認済み
// (ただし flag ビットの意味は Pro では未確認 → Scan::validPoints() を使い、
//  cleanPoints() は T-mini Plus 専用と考えること)。
#ifndef YDLIDAR_PROTOCOL_HPP_
#define YDLIDAR_PROTOCOL_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "error.hpp"

namespace ydlidar_tmini {

/// UART ボーレート (8N1, フロー制御なし)。
inline constexpr std::uint32_t kBaudRate = 230400;

namespace cmd {
inline constexpr std::array<std::uint8_t, 2> kStartScan{0xA5, 0x60};
inline constexpr std::array<std::uint8_t, 2> kStopScan{0xA5, 0x65};
inline constexpr std::array<std::uint8_t, 2> kDeviceInfo{0xA5, 0x90};
inline constexpr std::array<std::uint8_t, 2> kHealth{0xA5, 0x92};
inline constexpr std::array<std::uint8_t, 2> kFreqUp0_1Hz{0xA5, 0x09};
inline constexpr std::array<std::uint8_t, 2> kFreqDown0_1Hz{0xA5, 0x0A};
inline constexpr std::array<std::uint8_t, 2> kFreqUp1Hz{0xA5, 0x0B};
inline constexpr std::array<std::uint8_t, 2> kFreqDown1Hz{0xA5, 0x0C};
inline constexpr std::array<std::uint8_t, 2> kGetFreq{0xA5, 0x0D};
inline constexpr std::array<std::uint8_t, 2> kRestart{0xA5, 0x40};
}  // namespace cmd

namespace type_code {
inline constexpr std::uint8_t kScan = 0x81;
inline constexpr std::uint8_t kDeviceInfo = 0x04;
inline constexpr std::uint8_t kHealth = 0x06;
inline constexpr std::uint8_t kFrequency = 0x04;
}  // namespace type_code

/// 応答ヘッダ長 (開始サイン 2 + 長さ/モード 4 + タイプ 1)。
inline constexpr std::size_t kResponseHeaderLen = 7;
inline constexpr std::uint8_t kModeSingle = 0;
inline constexpr std::uint8_t kModeContinuous = 1;

/// 応答ヘッダ `A5 5A <len:30bit|mode:2bit LE> <type>`。
struct ResponseHeader {
    std::uint32_t length = 0;
    std::uint8_t mode = 0;
    std::uint8_t type_code = 0;

    /// `b` は kResponseHeaderLen (7) バイト。
    static ResponseHeader parse(const std::uint8_t* b) {
        if (b[0] != 0xA5 || b[1] != 0x5A) {
            throw ProtocolError("invalid response start sign");
        }
        const std::uint32_t raw = std::uint32_t(b[2]) | (std::uint32_t(b[3]) << 8) |
                                  (std::uint32_t(b[4]) << 16) | (std::uint32_t(b[5]) << 24);
        ResponseHeader h;
        h.length = raw & 0x3FFFFFFFu;
        h.mode = static_cast<std::uint8_t>(raw >> 30);
        h.type_code = b[6];
        return h;
    }
};

// ---------------------------------------------------------------------------
// 点・サンプル
// ---------------------------------------------------------------------------

/// パケット内の 1 サンプル (3 バイト)。
///  byte0: intensity[7:0]
///  byte1: distance[5:0] << 2 | flag[1:0]
///  byte2: distance[13:6]
struct Sample {
    std::uint8_t intensity = 0;
    std::uint16_t distance_mm = 0;  // 0 は無効 (遮蔽・未検出)
    std::uint8_t flag = 0;          // Plus: 2 = 鏡面反射, 3 = 環境光の干渉

    static Sample fromBytes(std::uint8_t b0, std::uint8_t b1, std::uint8_t b2) {
        Sample s;
        s.intensity = b0;
        s.distance_mm = static_cast<std::uint16_t>((std::uint16_t(b2) << 6) | (b1 >> 2));
        s.flag = b1 & 0x03;
        return s;
    }
    bool operator==(const Sample& o) const {
        return intensity == o.intensity && distance_mm == o.distance_mm && flag == o.flag;
    }
};

/// 角度付きの 1 点。
struct Point {
    float angle_deg;  // センサ出力の角度 0..360 (時計回りに増加)
    std::uint16_t distance_mm;
    std::uint8_t intensity;
    std::uint8_t flag;

    bool isValid() const { return distance_mm != 0; }
    bool isInterference() const { return flag == 2 || flag == 3; }

    /// デカルト座標 [m]。センサの角度は時計回りに増えるので、
    /// 反時計回りを正とする右手系 (ROS REP-103 風) になるよう符号を反転している。
    std::pair<float, float> toXYMeters() const {
        const float r = static_cast<float>(distance_mm) / 1000.0f;
        const float th = -angle_deg * 3.14159265358979f / 180.0f;
        return {r * std::cos(th), r * std::sin(th)};
    }
};

/// 1 周分の点群。
struct Scan {
    std::vector<Point> points;
    std::optional<float> scan_freq_hz;  // 開始パケットの CT から得た回転周波数

    /// 距離が得られている点 (distance > 0) だけ。
    std::vector<Point> validPoints() const {
        std::vector<Point> out;
        for (const auto& p : points) {
            if (p.isValid()) out.push_back(p);
        }
        return out;
    }
    /// さらに干渉フラグ (2, 3) の点も除く。T-mini Plus のマニュアルに基づく。
    /// T-mini Pro では ほぼ全点が flag=2 になるので使わないこと。
    std::vector<Point> cleanPoints() const {
        std::vector<Point> out;
        for (const auto& p : points) {
            if (p.isValid() && !p.isInterference()) out.push_back(p);
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// スキャンパケット
// ---------------------------------------------------------------------------

namespace detail {
inline constexpr std::size_t kHeaderLen = 10;  // PH(2) CT(1) LSN(1) FSA(2) LSA(2) CS(2)
inline constexpr std::size_t kSampleLen = 3;
}  // namespace detail

/// 角度フィールド (下位 1bit はチェックビット = 1 固定) を度に変換する。
inline float rawAngleToDeg(std::uint16_t raw) {
    return static_cast<float>(raw >> 1) / 64.0f;
}

struct ScanPacket {
    std::uint8_t ct = 0;
    float first_angle_deg = 0;
    float last_angle_deg = 0;
    std::vector<Sample> samples;

    /// CT bit0 = 1 は 1 周の先頭 (開始パケット, LSN = 1)。
    bool isStart() const { return (ct & 1) == 1; }

    /// 開始パケットの CT[7:1] は回転周波数 ×10 を運ぶ。
    std::optional<float> scanFreqHz() const {
        if (!isStart()) return std::nullopt;
        return static_cast<float>(ct >> 1) / 10.0f;
    }

    /// FSA〜LSA を線形補間して各サンプルへ角度を割り当てる。
    std::vector<Point> points() const {
        const std::size_t n = samples.size();
        float diff = last_angle_deg - first_angle_deg;
        if (diff < 0.0f) diff += 360.0f;  // 0° をまたぐ場合
        const float step = n > 1 ? diff / static_cast<float>(n - 1) : 0.0f;
        std::vector<Point> out;
        out.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            float a = first_angle_deg + step * static_cast<float>(i);
            if (a >= 360.0f) a -= 360.0f;
            out.push_back(Point{a, samples[i].distance_mm, samples[i].intensity, samples[i].flag});
        }
        return out;
    }
};

namespace detail {

/// `raw` はヘッダ 10 バイト + サンプル LSN*3 バイトちょうど。
/// チェックビットまたはチェックサムが不正なら nullopt。
inline std::optional<ScanPacket> decodePacket(const std::uint8_t* raw) {
    const std::uint8_t ct = raw[2];
    const std::size_t lsn = raw[3];
    const std::uint16_t fsa = std::uint16_t(raw[4] | (raw[5] << 8));
    const std::uint16_t lsa = std::uint16_t(raw[6] | (raw[7] << 8));
    const std::uint16_t cs = std::uint16_t(raw[8] | (raw[9] << 8));

    if ((fsa & 1) == 0 || (lsa & 1) == 0) return std::nullopt;  // チェックビットは 1 固定

    // CS = PH ^ (LSN<<8 | CT) ^ FSA ^ LSA ^ Σ( 0x00<<8|Si[1] , Si[3]<<8|Si[2] )
    std::uint16_t calc = static_cast<std::uint16_t>(0x55AA ^ ((lsn << 8) | ct) ^ fsa ^ lsa);
    ScanPacket pkt;
    pkt.samples.reserve(lsn);
    for (std::size_t i = 0; i < lsn; ++i) {
        const std::uint8_t* s = raw + kHeaderLen + i * kSampleLen;
        calc ^= s[0];
        calc ^= static_cast<std::uint16_t>((s[2] << 8) | s[1]);
        pkt.samples.push_back(Sample::fromBytes(s[0], s[1], s[2]));
    }
    if (calc != cs) return std::nullopt;

    pkt.ct = ct;
    pkt.first_angle_deg = rawAngleToDeg(fsa);
    pkt.last_angle_deg = rawAngleToDeg(lsa);
    return pkt;
}

}  // namespace detail

/// ストリーミングパーサ。任意の区切りでバイトを push し、next() で取り出す。
///
/// ヘッダ (AA 55) を探し、長さとチェックサムを検証する。壊れていれば 1 バイト進めて
/// 再同期するので、バイト欠落や途中接続でも復帰できる。
class PacketParser {
public:
    void push(const std::uint8_t* data, std::size_t n) { buf_.insert(buf_.end(), data, data + n); }
    void push(const std::vector<std::uint8_t>& v) { push(v.data(), v.size()); }

    /// 完全なパケットが 1 つ取れれば返す。データ不足なら nullopt (続きを push すること)。
    std::optional<ScanPacket> next() {
        using namespace detail;
        for (;;) {
            std::size_t pos = buf_.size();
            for (std::size_t i = 0; i + 1 < buf_.size(); ++i) {
                if (buf_[i] == 0xAA && buf_[i + 1] == 0x55) {
                    pos = i;
                    break;
                }
            }
            if (pos == buf_.size()) {
                // 末尾が 0xAA なら、次のバイト 0x55 と組になるかもしれないので残す
                const std::size_t keep = (!buf_.empty() && buf_.back() == 0xAA) ? 1 : 0;
                buf_.erase(buf_.begin(), buf_.end() - static_cast<std::ptrdiff_t>(keep));
                return std::nullopt;
            }
            if (pos > 0) buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(pos));

            if (buf_.size() < kHeaderLen) return std::nullopt;
            const std::size_t lsn = buf_[3];
            if (lsn == 0) {
                ++checksum_errors_;
                buf_.erase(buf_.begin());
                continue;
            }
            const std::size_t total = kHeaderLen + kSampleLen * lsn;
            if (buf_.size() < total) return std::nullopt;

            if (auto pkt = decodePacket(buf_.data())) {
                buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(total));
                return pkt;
            }
            ++checksum_errors_;
            buf_.erase(buf_.begin());  // 偽ヘッダかもしれないので 1 バイトだけ進める
        }
    }

    void clear() { buf_.clear(); }

    /// チェックサム/チェックビット不一致で捨てた候補の数 (通信品質の目安)。
    std::uint64_t checksumErrors() const { return checksum_errors_; }

private:
    std::vector<std::uint8_t> buf_;
    std::uint64_t checksum_errors_ = 0;
};

/// パケット列から 1 周分の Scan を組み立てる。
///
/// 開始パケット (CT bit0 = 1) が来たら、それまでに溜めた点を 1 周分として返す。
/// 最初の開始パケットより前のデータ (途中の周) は捨てる。
class ScanAssembler {
public:
    std::optional<Scan> push(const ScanPacket& pkt) {
        std::optional<Scan> finished;
        if (pkt.isStart()) {
            if (synced_) {
                Scan s;
                s.points = std::move(points_);
                s.scan_freq_hz = freq_;
                finished = std::move(s);
                points_.clear();
            }
            synced_ = true;
            freq_ = pkt.scanFreqHz();
        } else if (!synced_) {
            return std::nullopt;
        }
        const auto pts = pkt.points();
        points_.insert(points_.end(), pts.begin(), pts.end());
        return finished;
    }

    void reset() {
        synced_ = false;
        points_.clear();
        freq_.reset();
    }

private:
    bool synced_ = false;
    std::vector<Point> points_;
    std::optional<float> freq_;
};

// ---------------------------------------------------------------------------
// 単発応答 (A5 90 / A5 92 / 周波数)
// ---------------------------------------------------------------------------

/// `A5 90` の応答内容 (20 バイト)。
struct DeviceInfo {
    static constexpr std::uint32_t kPayloadLen = 20;

    std::uint8_t model = 0;  // T-mini Plus の実測値は 151 (Pro は未確認)
    std::uint8_t firmware_major = 0;
    std::uint8_t firmware_minor = 0;
    std::uint8_t hardware = 0;
    std::array<std::uint8_t, 16> serial{};

    static DeviceInfo parse(const std::vector<std::uint8_t>& p) {
        if (p.size() < kPayloadLen) throw ProtocolError("device info payload too short");
        DeviceInfo d;
        d.model = p[0];
        d.firmware_major = p[1];  // 2 バイト: 下位が major, 上位が minor
        d.firmware_minor = p[2];
        d.hardware = p[3];
        for (std::size_t i = 0; i < 16; ++i) d.serial[i] = p[4 + i];
        return d;
    }

    std::string toString() const {
        static const char* hex = "0123456789ABCDEF";
        std::string s = "model=" + std::to_string(model) + " firmware=" +
                        std::to_string(firmware_major) + "." + std::to_string(firmware_minor) +
                        " hardware=" + std::to_string(hardware) + " serial=";
        for (auto b : serial) {
            s += hex[b >> 4];
            s += hex[b & 0xF];
        }
        return s;
    }
};

/// `A5 92` の応答内容 (3 バイト)。
struct Health {
    static constexpr std::uint32_t kPayloadLen = 3;

    std::uint8_t status = 0;  // bit0..5 のいずれかが 1 なら異常
    std::uint16_t error_code = 0;

    static Health parse(const std::vector<std::uint8_t>& p) {
        if (p.size() < kPayloadLen) throw ProtocolError("health payload too short");
        Health h;
        h.status = p[0];
        h.error_code = static_cast<std::uint16_t>(p[1] | (p[2] << 8));
        return h;
    }
    /// 下位 6bit (bit6,7 は予約) がすべて 0 なら正常。
    bool isOk() const { return (status & 0x3F) == 0; }
};

/// 周波数コマンドの応答内容 (4 バイト, 1/100 Hz) を Hz に変換。
inline float parseFrequency(const std::vector<std::uint8_t>& p) {
    if (p.size() < 4) throw ProtocolError("frequency payload too short");
    const std::uint32_t v = std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) |
                            (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
    return static_cast<float>(v) / 100.0f;
}

}  // namespace ydlidar_tmini

#endif