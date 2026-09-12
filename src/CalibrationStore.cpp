#include "CalibrationStore.h"

namespace MM3 {

bool CalibrationStore::load(CalibrationData &out) {
  Preferences prefs;
  if (!prefs.begin("mm3cal", true)) return false;
  size_t n = prefs.getBytesLength("data");
  if (n != sizeof(CalibrationData)) {
    prefs.end();
    return false;
  }
  CalibrationData tmp;
  size_t got = prefs.getBytes("data", &tmp, sizeof(tmp));
  prefs.end();
  if (got != sizeof(tmp) || tmp.magic != CAL_MAGIC || tmp.version != CAL_VERSION) {
    return false;
  }
  out = tmp;
  return true;
}

bool CalibrationStore::save(const CalibrationData &data) {
  Preferences prefs;
  if (!prefs.begin("mm3cal", false)) return false;
  size_t n = prefs.putBytes("data", &data, sizeof(data));
  prefs.end();
  return n == sizeof(data);
}

void CalibrationStore::clear() {
  Preferences prefs;
  if (!prefs.begin("mm3cal", false)) return;
  prefs.clear();
  prefs.end();
}

} // namespace MM3
