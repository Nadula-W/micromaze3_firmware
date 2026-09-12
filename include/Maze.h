#pragma once
#include <Arduino.h>
#include "RobotTypes.h"
#include "Motion.h"
#include "Hardware.h"

namespace MM3 {

class MazeMap {
public:
  void reset();
  void load(const StoredMaze &stored);
  void exportTo(StoredMaze &stored) const;

  bool inBounds(int x, int y) const;
  bool isGoal(int x, int y) const;
  bool isVisited(int x, int y) const;
  void setVisited(int x, int y, bool v = true);

  void setWall(int x, int y, Heading dir, bool present);
  bool wallKnown(int x, int y, Heading dir) const;
  bool hasWall(int x, int y, Heading dir) const;
  bool canMove(int x, int y, Heading dir, bool unknownAsWall) const;

  void senseCurrentCell(const Pose &pose, MotionController &motion);
  bool chooseNext(const Pose &pose, bool targetStart, Heading &outDir) const;
  bool buildShortestPath(uint8_t *outPath, uint16_t &outLen, uint8_t &goalX, uint8_t &goalY) const;

  void print(Print &out) const;

private:
  uint8_t _walls[MAZE_N * MAZE_N] = {0};
  uint8_t _known[MAZE_N * MAZE_N] = {0};
  uint8_t _visited[MAZE_N * MAZE_N] = {0};

  uint16_t idx(int x, int y) const { return (uint16_t)y * MAZE_N + x; }
  bool neighbor(int x, int y, Heading d, int &nx, int &ny) const;
  void computeDistances(bool targetStart, bool unknownAsWall, uint16_t dist[MAZE_N * MAZE_N]) const;
};

class MazeStorage {
public:
  explicit MazeStorage(ExternalEEPROM &eeprom) : _eeprom(eeprom) {}
  bool save(const StoredMaze &maze, Print &out);
  bool load(StoredMaze &maze, Print &out);
  bool clear(Print &out);

private:
  ExternalEEPROM &_eeprom;
  static uint32_t crc32(const uint8_t *data, size_t len);
};

class MazeNavigator {
public:
  MazeNavigator(MazeMap &map, MazeStorage &storage, MotionController &motion)
      : _map(map), _storage(storage), _motion(motion) {}

  // Small home test: follow a forced/simple route until the first dead end,
  // then retrace the recorded cell moves back to the exact start.
  bool deadEndReturnTest(int pwm, Print &out);

  // Generic home-maze DFS test: explores every reachable branch, backtracks from
  // dead ends/junctions, and finishes back at the exact starting cell.
  bool homeDfsTest(int pwm, Print &out);

  // Search run: map while going start -> goal -> start, then save map + shortest path.
  bool explorationRun(int pwm, Print &out);

  // Execute the stored optimal path. Optional autonomous return avoids the reset penalty.
  bool storedRun(int outboundPwm, bool returnHome, int returnPwm, Print &out);

  bool dumpStored(Print &out);
  bool clearStored(Print &out) { return _storage.clear(out); }

private:
  MazeMap &_map;
  MazeStorage &_storage;
  MotionController &_motion;

  bool stepPose(Pose &pose, Heading next, int pwm);
  bool executeAbsolutePath(const uint8_t *path, uint16_t len, Pose &pose, int pwm, Print &out);
};

} // namespace MM3
