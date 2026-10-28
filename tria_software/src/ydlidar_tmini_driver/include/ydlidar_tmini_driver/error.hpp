#ifndef YDLIDAR_ERROR_HPP_
#define YDLIDAR_ERROR_HPP_
#include <stdexcept>

namespace ydlidar_tmini 
{
    /// 本ライブラリが投げる例外の基底クラス。
    class Error : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

    /// シリアルポート / OS レベルの I/O エラー。
    class IoError : public Error {
    public:
        using Error::Error;
    };

    /// 読み取りタイムアウト (データが来ない)。
    class TimeoutError : public Error {
    public:
        TimeoutError() : Error("timed out waiting for data from the lidar") {}
    };

    /// 想定外の応答・フォーマット不正。
    class ProtocolError : public Error {
    public:
        using Error::Error;
    };

    /// 現在の状態では実行できない操作 (スキャン中にコマンドを送る等)。
    class StateError : public Error {
    public:
        using Error::Error;
    };

}  // namespace ydlidar_tmini

#endif