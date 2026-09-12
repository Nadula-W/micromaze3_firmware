#pragma once

#include <Arduino.h>
#include "Config.h"

#if MM3_DEV_WIFI

#include <Stream.h>
#include <WiFi.h>
#include <WebServer.h>

namespace MM3 {

// DEVELOPMENT ONLY.
// A tiny browser terminal that exposes the existing Debug console over the
// ESP32-S3's own Wi-Fi access point. It also mirrors all terminal output to USB
// Serial, and accepts input from either USB Serial or the browser.
class WebTerminal : public Stream {
public:
  WebTerminal();

  bool begin(const char *ssid, const char *password);
  void end();
  bool active() const { return _active; }
  IPAddress ip() const { return WiFi.softAPIP(); }

  int available() override;
  int read() override;
  int peek() override;
  void flush() override;

  size_t write(uint8_t b) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  using Print::write;

private:
  static constexpr size_t LOG_MAX = 24000;
  static constexpr size_t LOG_TRIM = 6000;

  WebServer _server;
  QueueHandle_t _inputQueue = nullptr;
  SemaphoreHandle_t _logMutex = nullptr;
  TaskHandle_t _serverTask = nullptr;
  String _log;
  bool _active = false;

  static void serverTaskThunk(void *arg);
  void serverTaskLoop();
  void appendLog(const uint8_t *buffer, size_t size);
  void queueCommand(const String &cmd);
  String snapshotLog();
  void setupRoutes();
};

} // namespace MM3

#endif // MM3_DEV_WIFI
