#include "infer_base/common_utils/log.hpp"

#include <iostream>
#include <cstring>

namespace easy_deploy {

GlobalLogger::GlobalLogger() : logger_(nullptr)
{}

GlobalLogger::~GlobalLogger()
{
  if (logger_ != nullptr)
  {
    delete logger_;
  }
}

ILogger *GlobalLogger::GetLogger() noexcept
{
  return logger_;
}

void GlobalLogger::SetLogger(ILogger *logger) noexcept
{
  logger_ = logger;
}

class SimpleLogger : public ILogger {
protected:
  // Optional: thread safety for singleton use cases
  std::mutex mtx_;

  // ANSI escape colors
  const char *get_color(const char *level) const noexcept
  {
    if (strcmp(level, "DEBUG") == 0)
      return "\033[36m"; // Cyan
    else if (strcmp(level, "INFO") == 0)
      return "\033[32m"; // Green
    else if (strcmp(level, "WARN") == 0)
      return "\033[33m"; // Yellow
    else if (strcmp(level, "ERROR") == 0)
      return "\033[31m"; // Red
    else
      return "\033[0m"; // Default
  }

  // Get formatted current time
  void get_time_str(char *buf, size_t buflen) const noexcept
  {
    // Current system time
    auto now     = std::chrono::system_clock::now();
    auto now_sec = std::chrono::system_clock::to_time_t(now);

    // Milliseconds
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::tm tm_info;
#if defined(_WIN32)
    localtime_s(&tm_info, &now_sec);
#else
    localtime_r(&now_sec, &tm_info);
#endif
    std::strftime(buf, buflen, "%Y-%m-%d %H:%M:%S", &tm_info);
    size_t len = strlen(buf);
    snprintf(buf + len, buflen - len, ".%03ld", (long)ms.count());
  }

  void vlog(const char *level, const char *fmt, va_list args) noexcept
  {
    std::lock_guard<std::mutex> lock(mtx_);
    char                        timebuf[32];
    get_time_str(timebuf, sizeof(timebuf));
    const char *color = get_color(level);

    // Print [time][LEVEL] (with color)
    printf("%s[%s][%s]%s ", color, timebuf, level, "\033[0m");
    vprintf(fmt, args);
    printf("\n");
  }

  void do_log_debug(const char *fmt, ...) noexcept override
  {
    va_list args;
    va_start(args, fmt);
    vlog("DEBUG", fmt, args);
    va_end(args);
  }
  void do_log_info(const char *fmt, ...) noexcept override
  {
    va_list args;
    va_start(args, fmt);
    vlog("INFO", fmt, args);
    va_end(args);
  }
  void do_log_warn(const char *fmt, ...) noexcept override
  {
    va_list args;
    va_start(args, fmt);
    vlog("WARN", fmt, args);
    va_end(args);
  }
  void do_log_error(const char *fmt, ...) noexcept override
  {
    va_list args;
    va_start(args, fmt);
    vlog("ERROR", fmt, args);
    va_end(args);
  }
};

REGISTER_EasyDeploy_LOGGER(SimpleLogger);

} // namespace easy_deploy
