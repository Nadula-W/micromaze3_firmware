#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "RobotTypes.h"

namespace MM3 {

class CalibrationStore {
public:
  bool load(CalibrationData &out);
  bool save(const CalibrationData &data);
  void clear();
};

} // namespace MM3
