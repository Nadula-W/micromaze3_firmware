#include "Config.h"

#if MM3_DEV_WIFI
#include "WebTerminal.h"

namespace MM3 {

static const char PAGE_HTML[] PROGMEM = R"MM3HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MM3 Robot Terminal</title>
<style>
  body{font-family:system-ui,Arial,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
  h2{margin:0 0 8px}
  .note{font-size:13px;color:#bbb;margin-bottom:12px}
  #out{box-sizing:border-box;width:100%;height:62vh;overflow:auto;background:#000;color:#d8ffd8;
       border:1px solid #444;border-radius:8px;padding:10px;white-space:pre-wrap;font-family:monospace}
  .row{display:flex;gap:8px;margin-top:10px}
  #cmd{flex:1;background:#1d1d1d;color:#fff;border:1px solid #555;border-radius:7px;padding:11px;font-family:monospace}
  button{background:#2b2b2b;color:#fff;border:1px solid #666;border-radius:7px;padding:10px 14px}
</style>
</head>
<body>
<h2>MicroMaze 3 - DEV Terminal</h2>
<div class="note">Development Wi-Fi only. Set DIP 100 and press Key1 to enter Debug mode. Key2 remains the emergency stop.</div>
<pre id="out">Connecting...</pre>
<div class="row">
  <input id="cmd" autocomplete="off" placeholder="status / cell / turn 90 / ...">
  <button onclick="sendCmd(false)">Send</button>
  <button onclick="sendCmd(true)">ENTER</button>
</div>
<script>
const out=document.getElementById('out');
const cmd=document.getElementById('cmd');
let busy=false;
async function refresh(){
  if(busy) return;
  try{
    const r=await fetch('/log?t='+Date.now(),{cache:'no-store'});
    const t=await r.text();
    const nearBottom=(out.scrollTop+out.clientHeight+80>=out.scrollHeight);
    out.textContent=t;
    if(nearBottom) out.scrollTop=out.scrollHeight;
  }catch(e){}
}
async function sendCmd(enterOnly){
  const v=enterOnly ? '' : cmd.value;
  if(!enterOnly && !v.trim()) return;
  busy=true;
  try{
    await fetch('/cmd',{
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'cmd='+encodeURIComponent(v)
    });
    if(!enterOnly) cmd.value='';
  }catch(e){}
  busy=false;
  setTimeout(refresh,80);
}
cmd.addEventListener('keydown',e=>{
  if(e.key==='Enter'){e.preventDefault();sendCmd(false);}
});
setInterval(refresh,300);
refresh();
</script>
</body>
</html>
)MM3HTML";

WebTerminal::WebTerminal() : _server(80) {}

bool WebTerminal::begin(const char *ssid, const char *password) {
  if (_active) return true;

  _inputQueue = xQueueCreate(1024, sizeof(char));
  _logMutex = xSemaphoreCreateMutex();
  if (!_inputQueue || !_logMutex) return false;

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(50);

  if (!WiFi.softAP(ssid, password)) return false;

  setupRoutes();
  _server.begin();

  BaseType_t ok = xTaskCreatePinnedToCore(
      serverTaskThunk, "mm3-web", 6144, this, 1, &_serverTask, 0);

  if (ok != pdPASS) {
    _server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  _active = true;
  println();
  println("=== MM3 DEVELOPMENT Wi-Fi TERMINAL ===");
  print("SSID: "); println(ssid);
  print("Password: "); println(password);
  print("Browser: http://"); println(WiFi.softAPIP());
  println("Set DIP 100, press Key1, wait 5 s, then use the browser terminal.");
  println("WARNING: this Wi-Fi build is NOT competition compliant.");
  return true;
}

void WebTerminal::end() {
  if (_serverTask) {
    vTaskDelete(_serverTask);
    _serverTask = nullptr;
  }
  _server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  _active = false;
}

void WebTerminal::setupRoutes() {
  _server.on("/", HTTP_GET, [this]() {
    _server.send_P(200, "text/html", PAGE_HTML);
  });

  _server.on("/log", HTTP_GET, [this]() {
    _server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    _server.send(200, "text/plain; charset=utf-8", snapshotLog());
  });

  _server.on("/cmd", HTTP_POST, [this]() {
    if (!_server.hasArg("cmd")) {
      _server.send(400, "text/plain", "missing cmd");
      return;
    }
    queueCommand(_server.arg("cmd"));
    _server.send(200, "text/plain", "OK");
  });

  _server.onNotFound([this]() {
    _server.send(404, "text/plain", "Not found");
  });
}

void WebTerminal::serverTaskThunk(void *arg) {
  static_cast<WebTerminal *>(arg)->serverTaskLoop();
}

void WebTerminal::serverTaskLoop() {
  for (;;) {
    _server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void WebTerminal::queueCommand(const String &cmd) {
  if (!_inputQueue) return;
  for (size_t i = 0; i < cmd.length(); ++i) {
    char c = cmd[i];
    xQueueSend(_inputQueue, &c, pdMS_TO_TICKS(20));
  }
  char nl = '\n';
  xQueueSend(_inputQueue, &nl, pdMS_TO_TICKS(20));
}

String WebTerminal::snapshotLog() {
  if (!_logMutex) return String();
  String copy;
  if (xSemaphoreTake(_logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    copy = _log;
    xSemaphoreGive(_logMutex);
  }
  return copy;
}

void WebTerminal::appendLog(const uint8_t *buffer, size_t size) {
  if (!_logMutex || !buffer || !size) return;
  if (xSemaphoreTake(_logMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

  _log.reserve(LOG_MAX + 256);
  for (size_t i = 0; i < size; ++i) _log += (char)buffer[i];

  if (_log.length() > LOG_MAX) {
    size_t cut = LOG_TRIM;
    int nl = _log.indexOf('\n', cut);
    if (nl >= 0) cut = (size_t)nl + 1;
    _log.remove(0, cut);
  }
  xSemaphoreGive(_logMutex);
}

int WebTerminal::available() {
  int n = Serial.available();
  if (_inputQueue) n += (int)uxQueueMessagesWaiting(_inputQueue);
  return n;
}

int WebTerminal::read() {
  if (Serial.available()) return Serial.read();
  if (!_inputQueue) return -1;
  char c = 0;
  if (xQueueReceive(_inputQueue, &c, 0) == pdTRUE) return (uint8_t)c;
  return -1;
}

int WebTerminal::peek() {
  if (Serial.available()) return Serial.peek();
  if (!_inputQueue) return -1;
  char c = 0;
  if (xQueuePeek(_inputQueue, &c, 0) == pdTRUE) return (uint8_t)c;
  return -1;
}

void WebTerminal::flush() {
  Serial.flush();
}

size_t WebTerminal::write(uint8_t b) {
  return write(&b, 1);
}

size_t WebTerminal::write(const uint8_t *buffer, size_t size) {
  if (!buffer || !size) return 0;
  Serial.write(buffer, size);
  appendLog(buffer, size);
  return size;
}

} // namespace MM3

#endif // MM3_DEV_WIFI
