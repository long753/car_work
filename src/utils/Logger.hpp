#pragma once
#include <filesystem>
#include <fstream>
#include <chrono>
#include <ctime>
#include <string_view>
#include <ranges>
#include <mutex>
#include <utility>
#include <type_traits>
#include "SingletonHolder.hpp"



struct separator
 {
     constexpr separator() = default;
     constexpr separator(const separator&) = default;
     constexpr separator(separator&&) noexcept = default;
     constexpr explicit separator(std::string_view s) noexcept
     : str(s){

     }
     constexpr separator& operator=(const separator&) = default;
     constexpr separator& operator=(separator&&) noexcept = default;
     constexpr separator& operator=(std::string_view s) noexcept
     {
         str = s;
         return *this;
     }
     static const separator none;
     static const separator space;
     static const separator tab;
     static const separator newline;
     static const separator comma;
     std::string_view str;
 };

struct level{
    constexpr level(const level&) = default;
    constexpr level(level&&) noexcept = default;
    constexpr explicit level(std::string_view s) noexcept
    : str(s){
    }
    constexpr level& operator=(const level&) = default;
    constexpr level& operator=(level&&) noexcept = default;
    constexpr level& operator=(std::string_view s) noexcept
    {
        str = s;
        return *this;
    }
    static const level debug;
    static const level trace;
    static const level info;
    static const level warn;
    static const level error;
    std::string_view str;
};

template <typename stream_t>
class LogStream{
    public:
        template<typename T>
        LogStream& operator<<(const T& arg) { // 改为接受const引用
        if constexpr (std::is_same_v<separator, std::remove_cvref_t<T>>) {
            m_sep = arg;
        } else {
            if constexpr (!std::is_same_v<level, std::remove_cvref_t<T>>) {
                stream_put(m_ofs, m_sep.str);
            }
            stream_put(m_ofs, arg);
        }
    return *this;
    }

        template <typename _stream_t = stream_t>
        LogStream(std::mutex& mtx, _stream_t&& ofs,  level lv): m_trace_lock(mtx),m_ofs(ofs)
        {
            *this << lv;
        }

        template <typename _stream_t = stream_t, typename... Args>
        LogStream(std::mutex& mtx, _stream_t&& ofs,  level lv, Args&&... buff): m_trace_lock(mtx),m_ofs(ofs){
            ((*this << lv) << ... << std::forward<Args>(buff));
        }

        template <typename _stream_t = stream_t, typename... Args>
        LogStream(std::unique_lock<std::mutex>&& lock,_stream_t&& ofs, level lv,Args&&... buff):m_trace_lock(std::move(lock)),m_ofs(ofs){
            ((*this << lv) << ... << std::forward<Args>(buff));
        }

        LogStream(LogStream&&) = delete;
        LogStream(const LogStream&) = delete;
        LogStream& operator=(LogStream&&) = delete;
        LogStream& operator=(const LogStream&) = delete;

        ~LogStream() { m_ofs << std::endl; }
private:
    template <typename Stream, typename T>
    static Stream& stream_put(Stream& s, T&& v){
    if constexpr (std::same_as<std::filesystem::path, std::remove_cvref_t<T>>) {
    s << std::forward<T>(v);
    }else if constexpr (std::same_as< level, std::remove_cvref_t<T>>) {
        constexpr int buff_len = 128;
        char buff[buff_len] = { 0 };
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
        sprintf(buff,"[%s][%s]",ss.str().c_str(),v.str.data());
        s << buff;
    } else if constexpr (std::constructible_from<std::string, T>) {
        s << std::string(std::forward<T>(v));
    }
    else if constexpr(std::integral<T> || std::floating_point<T>){
        s << std::forward<T>(v);
    }
    else if constexpr (std::ranges::input_range<T>) {
    s << "[";
    std::string_view comma_space {};
    for (const auto& elem : std::forward<T>(v)) {
        s << comma_space;
        stream_put(s, elem);
        comma_space = ", ";
    }
    s << "]";
}
    return s;
}
separator m_sep = separator::space;
std::unique_lock<std::mutex> m_trace_lock;
stream_t m_ofs; 
};
template <typename stream_t, typename... Args>
LogStream(std::mutex&, stream_t&, Args&&...) -> LogStream<stream_t&>;

template <typename stream_t, typename... Args>
LogStream(std::mutex&, stream_t&&, Args&&...) -> LogStream<stream_t>;

template <typename stream_t, typename... Args>
LogStream(std::unique_lock<std::mutex>&&, stream_t&&, Args&&...) -> LogStream<stream_t>;


class Logger:public SingletonHolder<Logger>{
public:
    virtual ~Logger() override { flush(false); }
    template <typename T>
    auto operator<<(T&& arg){
        if (!m_ofs || !m_ofs.is_open()) {
            m_ofs = std::ofstream(m_log_path, std::ios::out | std::ios::app);
        }
        if constexpr (std::same_as<level, std::remove_cvref_t<T>>) {
            return LogStream(m_trace_mutex, m_ofs, arg);
        }
        else {
            return LogStream(m_trace_mutex, m_ofs, level::trace, arg);
        }
    }

    template <typename... Args>
    inline void log(level lv, Args&&... args)
    {
        ((*this << lv) << ... << std::forward<Args>(args));
    }
    template <typename... Args>
    inline void log(std::unique_lock<std::mutex>&& lock, level lv, Args&&... args){
     if (!m_ofs || !m_ofs.is_open()) {
         m_ofs = std::ofstream(m_log_path, std::ios::out | std::ios::app);
     }
     (LogStream(std::move(lock),m_ofs,lv)<< ... << std::forward<Args>(args));
    }
    void flush(bool rorate_log_file = true){
    std::unique_lock<std::mutex> m_trace_lock(m_trace_mutex);
    if (m_ofs.is_open()) {
        m_ofs.close();
    }
    if (rorate_log_file) {
    rotate();
    }
    }
    void rotate() const{
    constexpr uintmax_t MaxLogSize = 4ULL * 1024 * 1024;
    try {
        if (std::filesystem::exists(m_log_path)
            && std::filesystem::is_regular_file(m_log_path)) {
            const uintmax_t log_size = std::filesystem::file_size(m_log_path);
            if (log_size >= MaxLogSize) {
                std::filesystem::rename(m_log_path, m_log_bak_path);
            }
        }
    }
    catch (std::filesystem::filesystem_error& e) {
        std::cerr << e.what() << std::endl;
    }
    catch (...) {
    }
}
private:
    friend class SingletonHolder<Logger>;
    Logger(){  
    rotate();
    }
    std::filesystem::path m_directory;
    std::filesystem::path m_log_path = "../log/carlog.txt";
    std::filesystem::path m_log_bak_path = "../log/carlog_bak.txt";
    std::mutex m_trace_mutex;
    std::ofstream m_ofs;
};

inline constexpr  separator  separator::none;
inline constexpr  separator  separator::space(" ");
inline constexpr  separator  separator::tab("\t");
inline constexpr  separator  separator::newline("\n");
inline constexpr  separator  separator::comma(",");

inline constexpr  level  level::debug("DBG");
inline constexpr  level  level::trace("TRC");
inline constexpr  level  level::info("INF");
inline constexpr  level  level::warn("WRN");
inline constexpr  level  level::error("ERR");

#define Log Logger::get_instance()
#define LogDebug Log << level::debug
#define LogTrace Log << level::trace
#define LogInfo Log << level::info
#define LogWarn Log << level::warn
#define LogError Log << level::error
