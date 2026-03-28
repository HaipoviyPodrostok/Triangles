#pragma once

#ifdef ENABLE_LOGGING
#include <spdlog/spdlog.h>

#define LOG_INFO(...) spdlog::info(__VA_ARGS__)
#define LOG_WARN(...) spdlog::warn(__VA_ARGS__)
#define LOG_ERR(...)  spdlog::error(__VA_ARGS__)

#else
#define LOG_INFO(...) (void)0
#define LOG_WARN(...) (void)0
#define LOG_ERR(...)  (void)0

#endif
