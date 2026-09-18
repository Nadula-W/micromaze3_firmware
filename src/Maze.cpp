#include "Maze.h"
#include <string.h>
#include <stddef.h>

namespace MM3 {

// ---------------- MazeMap ----------------

void MazeMap::reset() {
  memset(_walls, 0, sizeof(_walls));
  memset(_known, 0, sizeof(_known));
  memset(_visited, 0, sizeof(_visited));

  // Outer boundary is known wall.
  for (int x = 0; x < MAZE_N; ++x) {
    setWall(x, 0, Heading::South, true);
    setWall(x, MAZE_N - 1, Heading::North, true);
  }
  for (int y = 0; y < MAZE_N; ++y) {
    setWall(0, y, Heading::West, true);
    setWall(MAZE_N - 1, y, Heading::East, true);
  }

  // Coordinate convention: regardless of which physical maze corner is used,
  // place the mouse in local cell (0,0) facing the single open exit as North.
  // The start cell is bounded on the other three sides by the competition rule.
  setWall(0, 0, Heading::West, true);
  setWall(0, 0, Heading::South, true);
  setWall(0, 0, Heading::East, true);
  setWall(0, 0, Heading::North, false);
}

void MazeMap::load(const StoredMaze &stored) {
  memcpy(_walls, stored.walls, sizeof(_walls));
  memcpy(_known, stored.known, sizeof(_known));
  memset(_visited, 0, sizeof(_visited));
}

void MazeMap::exportTo(StoredMaze &stored) const {
  memcpy(stored.walls, _walls, sizeof(_walls));
  memcpy(stored.known, _known, sizeof(_known));
}

bool MazeMap::inBounds(int x, int y) const {
  return x >= 0 && y >= 0 && x < MAZE_N && y < MAZE_N;
}

bool MazeMap::isGoal(int x, int y) const {
  return x == MAZE_GOAL_X && y == MAZE_GOAL_Y;
}

bool MazeMap::isVisited(int x, int y) const {
  return inBounds(x, y) ? _visited[idx(x, y)] != 0 : false;
}

void MazeMap::setVisited(int x, int y, bool v) {
  if (inBounds(x, y)) _visited[idx(x, y)] = v ? 1 : 0;
}

bool MazeMap::neighbor(int x, int y, Heading d, int &nx, int &ny) const {
  nx = x; ny = y;
  switch (d) {
    case Heading::North: ++ny; break;
    case Heading::East: ++nx; break;
    case Heading::South: --ny; break;
    case Heading::West: --nx; break;
  }
  return inBounds(nx, ny);
}

void MazeMap::setWall(int x, int y, Heading dir, bool present) {
  if (!inBounds(x, y)) return;
  uint8_t bit = headingWallBit(dir);
  uint16_t i = idx(x, y);
  _known[i] |= bit;
  if (present) _walls[i] |= bit;
  else _walls[i] &= ~bit;

  int nx, ny;
  if (neighbor(x, y, dir, nx, ny)) {
    Heading od = opposite(dir);
    uint8_t ob = headingWallBit(od);
    uint16_t ni = idx(nx, ny);
    _known[ni] |= ob;
    if (present) _walls[ni] |= ob;
    else _walls[ni] &= ~ob;
  }
}

bool MazeMap::wallKnown(int x, int y, Heading dir) const {
  if (!inBounds(x, y)) return true;
  return (_known[idx(x, y)] & headingWallBit(dir)) != 0;
}

bool MazeMap::hasWall(int x, int y, Heading dir) const {
  if (!inBounds(x, y)) return true;
  return (_walls[idx(x, y)] & headingWallBit(dir)) != 0;
}

bool MazeMap::canMove(int x, int y, Heading dir, bool unknownAsWall) const {
  int nx, ny;
  if (!neighbor(x, y, dir, nx, ny)) return false;
  if (wallKnown(x, y, dir)) return !hasWall(x, y, dir);
  return !unknownAsWall;
}

void MazeMap::senseCurrentCell(const Pose &pose, MotionController &motion) {
  bool firstVisit = !isVisited(pose.x, pose.y);
  setVisited(pose.x, pose.y, true);
  Heading left = turnLeft(pose.heading);
  Heading right = turnRight(pose.heading);
  setWall(pose.x, pose.y, left, motion.wallLeft());
  setWall(pose.x, pose.y, pose.heading, motion.wallFront());
  setWall(pose.x, pose.y, right, motion.wallRight());

  // The edge we arrived through is open. Only on the very first observation of
  // the start cell is the back direction the known third enclosing wall.
  if (firstVisit && pose.x == 0 && pose.y == 0 && pose.heading == Heading::North)
    setWall(0, 0, Heading::South, true);
  else
    setWall(pose.x, pose.y, opposite(pose.heading), false);
}

void MazeMap::computeDistances(bool targetStart, bool unknownAsWall,
                               uint16_t dist[MAZE_N * MAZE_N]) const {
  constexpr uint16_t INF = 0x3FFF;
  for (uint16_t i = 0; i < MAZE_N * MAZE_N; ++i) dist[i] = INF;

  uint16_t q[MAZE_N * MAZE_N];
  uint16_t head = 0, tail = 0;
  if (targetStart) {
    dist[idx(0, 0)] = 0;
    q[tail++] = idx(0, 0);
  } else {
    dist[idx(MAZE_GOAL_X, MAZE_GOAL_Y)] = 0;
    q[tail++] = idx(MAZE_GOAL_X, MAZE_GOAL_Y);
  }

  while (head < tail) {
    uint16_t ci = q[head++];
    int x = ci % MAZE_N;
    int y = ci / MAZE_N;
    uint16_t nd = dist[ci] + 1;
    for (uint8_t d = 0; d < 4; ++d) {
      Heading h = static_cast<Heading>(d);
      if (!canMove(x, y, h, unknownAsWall)) continue;
      int nx, ny;
      if (!neighbor(x, y, h, nx, ny)) continue;
      uint16_t ni = idx(nx, ny);
      if (nd < dist[ni]) {
        dist[ni] = nd;
        q[tail++] = ni;
      }
    }
  }
}

bool MazeMap::chooseNext(const Pose &pose, bool targetStart, Heading &outDir) const {
  uint16_t dist[MAZE_N * MAZE_N];
  // During exploration, unknown edges are treated as potentially open.
  computeDistances(targetStart, false, dist);

  struct Candidate { Heading d; int score; };
  Candidate best{pose.heading, 1000000};
  Heading preference[4] = {
    pose.heading,
    turnLeft(pose.heading),
    turnRight(pose.heading),
    opposite(pose.heading)
  };

  for (uint8_t p = 0; p < 4; ++p) {
    Heading d = preference[p];
    if (!canMove(pose.x, pose.y, d, false)) continue;
    int nx, ny;
    if (!neighbor(pose.x, pose.y, d, nx, ny)) continue;
    uint16_t di = dist[idx(nx, ny)];
    if (di >= 0x3FFF) continue;

    // Strongly prefer unvisited cells at equal flood distance; then prefer straight,
    // left, right, back to reduce unnecessary turns.
    int score = (int)di * 100 + (isVisited(nx, ny) ? 20 : 0) + p;
    if (score < best.score) best = {d, score};
  }

  if (best.score >= 1000000) return false;
  outDir = best.d;
  return true;
}

bool MazeMap::buildShortestPath(uint8_t *outPath, uint16_t &outLen,
                                uint8_t &goalX, uint8_t &goalY) const {
  outLen = 0;
  uint16_t dist[MAZE_N * MAZE_N];
  // Fast path must use only edges that were actually confirmed open.
  computeDistances(false, true, dist);
  int x = 0, y = 0;
  Heading heading = Heading::North;
  if (dist[idx(x, y)] >= 0x3FFF) return false;

  for (uint16_t steps = 0; steps < MAX_PATH && !isGoal(x, y); ++steps) {
    Heading pref[4] = {heading, turnLeft(heading), turnRight(heading), opposite(heading)};
    Heading chosen = heading;
    uint16_t bestDist = 0x3FFF;
    bool found = false;
    for (uint8_t p = 0; p < 4; ++p) {
      Heading d = pref[p];
      if (!canMove(x, y, d, true)) continue;
      int nx, ny;
      if (!neighbor(x, y, d, nx, ny)) continue;
      uint16_t dd = dist[idx(nx, ny)];
      if (dd < bestDist) {
        bestDist = dd;
        chosen = d;
        found = true;
      }
    }
    if (!found) return false;
    outPath[outLen++] = static_cast<uint8_t>(chosen);
    int nx, ny;
    neighbor(x, y, chosen, nx, ny);
    x = nx; y = ny; heading = chosen;
  }

  if (!isGoal(x, y)) return false;
  goalX = (uint8_t)x;
  goalY = (uint8_t)y;
  return true;
}

void MazeMap::print(Print &out) const {
  out.println("Known maze cells (x,y : walls N/E/S/W; ? = unknown):");
  for (int y = MAZE_N - 1; y >= 0; --y) {
    for (int x = 0; x < MAZE_N; ++x) {
      uint16_t i = idx(x, y);
      if (_known[i] == 0) continue;
      out.print("("); out.print(x); out.print(","); out.print(y); out.print(") ");
      const char names[4] = {'N','E','S','W'};
      for (uint8_t d = 0; d < 4; ++d) {
        uint8_t b = 1u << d;
        if (!(_known[i] & b)) out.print('?');
        else out.print((_walls[i] & b) ? names[d] : '.');
      }
      out.print("  ");
    }
    out.println();
  }
}

// ---------------- MazeStorage ----------------

namespace {
struct __attribute__((packed)) MazeBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t pathLen;
  uint8_t goalX;
  uint8_t goalY;
  uint8_t reserved[2];
  uint8_t walls[MAZE_N * MAZE_N];
  uint8_t known[MAZE_N * MAZE_N];
  uint8_t path[MAX_PATH];
  uint32_t crc;
};
}

uint32_t MazeStorage::crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1u)));
  }
  return ~crc;
}

bool MazeStorage::save(const StoredMaze &maze, Print &out) {
  if (!_eeprom.present()) {
    out.println("Maze save failed: external EEPROM missing.");
    return false;
  }
  MazeBlob blob{};
  blob.magic = MAZE_MAGIC;
  blob.version = MAZE_VERSION;
  blob.pathLen = (maze.pathLen < MAX_PATH) ? maze.pathLen : MAX_PATH;
  blob.goalX = maze.goalX;
  blob.goalY = maze.goalY;
  memcpy(blob.walls, maze.walls, sizeof(blob.walls));
  memcpy(blob.known, maze.known, sizeof(blob.known));
  memcpy(blob.path, maze.path, blob.pathLen);
  blob.crc = crc32((const uint8_t *)&blob, offsetof(MazeBlob, crc));

  bool ok = _eeprom.writeBytes(0, (const uint8_t *)&blob, sizeof(blob));
  out.print("Maze/path EEPROM save: "); out.println(ok ? "PASS" : "FAIL");
  return ok;
}

bool MazeStorage::load(StoredMaze &maze, Print &out) {
  if (!_eeprom.present()) return false;
  MazeBlob blob{};
  if (!_eeprom.readBytes(0, (uint8_t *)&blob, sizeof(blob))) return false;
  if (blob.magic != MAZE_MAGIC || blob.version != MAZE_VERSION || blob.pathLen > MAX_PATH) {
    out.println("No valid stored maze/path found.");
    return false;
  }
  uint32_t c = crc32((const uint8_t *)&blob, offsetof(MazeBlob, crc));
  if (c != blob.crc) {
    out.println("Stored maze CRC mismatch.");
    return false;
  }
  memcpy(maze.walls, blob.walls, sizeof(maze.walls));
  memcpy(maze.known, blob.known, sizeof(maze.known));
  maze.pathLen = blob.pathLen;
  memcpy(maze.path, blob.path, blob.pathLen);
  maze.goalX = blob.goalX;
  maze.goalY = blob.goalY;
  return true;
}

bool MazeStorage::clear(Print &out) {
  uint32_t zero = 0;
  bool ok = _eeprom.writeBytes(0, (uint8_t *)&zero, sizeof(zero));
  out.print("Clear stored maze: "); out.println(ok ? "PASS" : "FAIL");
  return ok;
}

// ---------------- MazeNavigator ----------------

bool MazeNavigator::stepPose(Pose &pose, Heading next, int pwm) {
  if (!_motion.turnToHeading(pose.heading, next)) return false;

  // v30 stability fix: DFS must use EXACTLY the same one-cell motion controller
  // as the proven `cell` diagnostic.  The anchored-segment controller used by
  // v28/v29 had different steering state and could fight the wall PID, producing
  // severe left-right wobble even though `cell` itself was smooth.
  //
  // driveCell() resets encoder sync locally for every 192 mm move, continuously
  // applies the saved wall PID, and keeps the front collision/reference logic.
  _motion.invalidateMazeSegmentAnchor();
  if (!_motion.driveCell(pwm, true)) return false;
  switch (next) {
    case Heading::North: ++pose.y; break;
    case Heading::East: ++pose.x; break;
    case Heading::South: --pose.y; break;
    case Heading::West: --pose.x; break;
  }
  pose.heading = next;
  return _map.inBounds(pose.x, pose.y);
}

bool MazeNavigator::deadEndReturnTest(int pwm, Print &out) {
  out.println("\n=== SAMPLE DEAD-END / RETURN TEST ===");
  out.println("Purpose: follow the simple home maze until a dead end, then retrace every cell back to start.");
  out.print("Navigation turns use calibrated encoder pivots: 90deg = "); out.print(TURN_LEFT_90_TICKS); out.print(" left / "); out.print(TURN_RIGHT_90_TICKS); out.println(" right average wheel ticks.");
  out.println("Keep Key2 ready. The test stops after 24 outbound cells for safety.");

  Pose pose;
  pose.x = 0;
  pose.y = 0;
  pose.heading = Heading::North;

  Heading route[24];
  uint8_t depth = 0;

  for (uint8_t step = 0; step < 24; ++step) {
    delay(100); // allow ToF readings to settle after the previous movement
    bool l = _motion.wallLeft();
    bool f = _motion.wallFront();
    bool r = _motion.wallRight();

    out.print("cell "); out.print(depth);
    out.print("  L="); out.print(l ? "W" : "OPEN");
    out.print(" F="); out.print(f ? "W" : "OPEN");
    out.print(" R="); out.println(r ? "W" : "OPEN");

    // Back is the cell we arrived from. If all three forward-facing choices are
    // walls, this is a true dead end for this route.
    if (l && f && r) {
      out.print("DEAD END detected after "); out.print(depth); out.println(" cell moves.");
      out.println("Retracing stored route back to start...");

      while (depth > 0) {
        Heading outbound = route[depth - 1];
        Heading back = opposite(outbound);
        if (!_motion.turnToHeading(pose.heading, back)) {
          out.println("RETURN FAIL: turn failed/killed.");
          return false;
        }
        if (!_motion.driveCell(pwm, true)) {
          out.println("RETURN FAIL: cell movement failed/killed.");
          return false;
        }
        pose.heading = back;
        --depth;
        out.print("returned, remaining cells = "); out.println(depth);
        delay(80);
      }

      out.println("SAMPLE TEST PASS: dead end identified and robot returned to its starting position.");
      return true;
    }

    // This diagnostic is intentionally simple: on the user's cardboard route
    // choose straight if possible, otherwise left, otherwise right. The reverse
    // edge is not selected until the dead-end return phase.
    Heading next;
    if (!f) next = pose.heading;
    else if (!l) next = turnLeft(pose.heading);
    else if (!r) next = turnRight(pose.heading);
    else {
      out.println("SAMPLE TEST FAIL: no legal direction.");
      return false;
    }

    route[depth++] = next;
    if (!_motion.turnToHeading(pose.heading, next)) {
      out.println("OUTBOUND FAIL: turn failed/killed.");
      return false;
    }
    if (!_motion.driveCell(pwm, true)) {
      out.println("OUTBOUND FAIL: cell movement failed/killed.");
      return false;
    }
    pose.heading = next;
  }

  out.println("SAMPLE TEST STOPPED: no dead end found within 24 cells.");
  return false;
}

bool MazeNavigator::homeDfsTest(int pwm, Print &out) {
  _motion.invalidateMazeSegmentAnchor();
  out.println("\n=== GENERIC HOME MAZE DFS TEST ===");
  out.println("Explores unknown branches, backtracks at dead ends, then returns to START.");
  out.println("Use a closed maze. Start at (0,0), facing North (+y), with the maze extending right (+x).");
  out.print("Maze size: "); out.print(MAZE_N); out.print('x'); out.println(MAZE_N);
  out.print("Navigation: cell="); out.print(CELL_MM, 0); out.print(" mm, 90deg pivot="); out.print(TURN_LEFT_90_TICKS); out.print("L/"); out.print(TURN_RIGHT_90_TICKS); out.println("R encoder ticks. Key2 = emergency stop.");

  // Use the same coordinates and bounds as stepPose() / MazeMap.
  constexpr int N = 10;
constexpr int START_X = 0;
constexpr int START_Y = 0;
constexpr uint16_t MAX_ACTIONS = 300;

  bool visited[N][N] = {};
  int8_t parentBack[N][N];
  for (int y = 0; y < N; ++y)
    for (int x = 0; x < N; ++x)
      parentBack[y][x] = -1;

  auto inBoundsLocal = [](int x, int y) { return x >= 0 && x < N && y >= 0 && y < N; };
  auto neighborLocal = [](int x, int y, Heading d, int &nx, int &ny) {
    nx = x; ny = y;
    switch (d) {
      case Heading::North: ++ny; break;
      case Heading::East:  ++nx; break;
      case Heading::South: --ny; break;
      case Heading::West:  --nx; break;
    }
  };

  Pose pose;
  pose.x = START_X;
  pose.y = START_Y;
  pose.heading = Heading::North;
  visited[START_Y][START_X] = true;

  uint16_t uniqueCells = 1;
  uint16_t backtracks = 0;
  // v31 diagnostic/stability fix: use EXACTLY the PWM requested by dfs_test.
  // dfs_test passes SLOW_PWM (90), which is the same PWM used by the proven
  // `cell` command.  Do not silently force this up to 105; speed changes the
  // response of the wall/encoder steering controller and can create wobble.
  const int navPwm = pwm;
  out.print("DFS drive PWM="); out.print(navPwm); out.println(" (same as cell)");

  for (uint16_t action = 0; action < MAX_ACTIONS; ++action) {
    delay(100); // VL53L0X continuous period is 50 ms; allow a fresh post-turn frame.
    const bool wallL = _motion.wallLeft();
    const bool wallF = _motion.wallFront();
    const bool wallR = _motion.wallRight();

    out.print("cell("); out.print(pose.x - START_X); out.print(','); out.print(pose.y - START_Y);
    out.print(") heading=");
    const char hc[4] = {'N','E','S','W'};
    out.print(hc[(uint8_t)pose.heading & 3]);
    out.print(" L="); out.print(wallL ? "W" : "OPEN");
    out.print(" F="); out.print(wallF ? "W" : "OPEN");
    out.print(" R="); out.println(wallR ? "W" : "OPEN");

    Heading dirs[3] = {pose.heading, turnLeft(pose.heading), turnRight(pose.heading)};
    bool blocked[3] = {wallF, wallL, wallR};
    Heading chosen = pose.heading;
    bool foundNew = false;

    // DFS preference: straight, left, right.
    // IMPORTANT: the robot starts facing North in this diagnostic, so the
    // physical edge behind the starting pose is South.  Relative left/front/right
    // is NOT enough to protect that edge after the robot comes back to START with
    // a different heading.  Without this absolute-direction guard the old code
    // could return to START, see the original rear/outside corridor as a "new"
    // branch, and drive past the start indefinitely.
    constexpr Heading START_REAR_DIR = Heading::South;
    for (uint8_t i = 0; i < 3; ++i) {
      if (blocked[i]) continue;

      // Never explore through the original rear edge of the test start cell.
      // That edge is only the launch boundary for dfs_test, not part of the
      // unknown maze to explore.
      if (pose.x == START_X && pose.y == START_Y && dirs[i] == START_REAR_DIR) {
        continue;
      }

      int nx, ny;
      neighborLocal(pose.x, pose.y, dirs[i], nx, ny);
      if (!inBoundsLocal(nx, ny)) continue;
      if (!visited[ny][nx]) {
        chosen = dirs[i];
        foundNew = true;
        break;
      }
    }

    if (foundNew) {
      int nx, ny;
      neighborLocal(pose.x, pose.y, chosen, nx, ny);
      out.print("EXPLORE -> "); out.println(hc[(uint8_t)chosen & 3]);
      if (!stepPose(pose, chosen, navPwm)) {
        out.println("DFS FAIL: movement/turn failed or Key2 killed the run.");
        return false;
      }
      visited[ny][nx] = true;
      parentBack[ny][nx] = (int8_t)opposite(chosen);
      ++uniqueCells;
      continue;
    }

    // No unvisited front/left/right branch remains in this cell.
    if (pose.x == START_X && pose.y == START_Y) {
      out.print("DFS TEST PASS: explored "); out.print(uniqueCells);
      out.print(" unique cells, performed "); out.print(backtracks);
      out.println(" backtracks, and returned to START.");
      return true;
    }

    int8_t pb = parentBack[pose.y][pose.x];
    if (pb < 0 || pb > 3) {
      out.println("DFS FAIL: parent/backtrack information missing.");
      return false;
    }
    Heading back = static_cast<Heading>(pb);
    out.print("NO NEW BRANCH -> BACKTRACK "); out.println(hc[(uint8_t)back & 3]);

    // This edge is known-open because it is exactly the edge used to enter this
    // cell. Backtracking is done more conservatively than exploration: slower
    // pivot/drive, a settling pause, and no front collision guard during the
    // one-cell return. This avoids false aborts when a slightly imperfect 180°
    // pivot lets a front ToF briefly see the side/dead-end wall.
    delay(80);
    const int backPwm = navPwm;
    out.print("BACKTRACK DRIVE: same-as-cell + wall-centering PWM="); out.println(backPwm);
    if (!stepPose(pose, back, backPwm)) {
      out.println("DFS FAIL: BACKTRACK DRIVE/TURN failed or Key2 killed the run.");
      return false;
    }
    ++backtracks;
  }

  out.println("DFS TEST STOPPED: safety action limit reached before returning to START.");
  return false;
}

bool MazeNavigator::explorationRun(int pwm, Print &out) {
  _motion.invalidateMazeSegmentAnchor();
  out.print("Exploration: START=(0,0), heading=N, GOAL=(");
  out.print(MAZE_GOAL_X); out.print(','); out.print(MAZE_GOAL_Y);
  out.println("). Reach goal, retrace outbound moves to START, then save route.");
  StoredMaze prior;
  if (_storage.load(prior, out)) {
    out.println("Exploration: continuing with previously discovered map.");
    _map.load(prior);
  } else {
    out.println("Exploration: starting a new map.");
    _map.reset();
  }

  Pose pose;
  pose.x = 0;
  pose.y = 0;
  pose.heading = Heading::North;
  bool returning = false;
  constexpr uint16_t MAX_EXPLORE_MOVES = 700;
  uint8_t outboundMoves[MAX_EXPLORE_MOVES];
  uint16_t remainingMoves = 0;

  const char headingNames[4] = {'N', 'E', 'S', 'W'};
  auto printPose = [&]() {
    out.print("cell("); out.print(pose.x); out.print(','); out.print(pose.y);
    out.print(") heading="); out.print(headingNames[(uint8_t)pose.heading & 3]);
    out.print(" target=("); out.print(returning ? 0 : MAZE_GOAL_X);
    out.print(','); out.print(returning ? 0 : MAZE_GOAL_Y); out.print(')');
  };
  auto dumpStopMap = [&]() {
    out.print("EXPLORATION STOP: "); printPose(); out.println();
    out.println("Map uses recorded coordinates; compare these with the physical maze.");
    _map.print(out);
  };

  // Allow the full outbound budget, every reverse move, and the final check.
  for (uint16_t step = 0; step <= 2 * MAX_EXPLORE_MOVES; ++step) {
    delay(100); // VL53L0X continuous period is 50 ms; allow fresh post-movement readings
    _map.senseCurrentCell(pose, _motion);

    if (!returning && _map.isGoal(pose.x, pose.y)) {
      out.print("Goal reached at ("); out.print(pose.x); out.print(','); out.print(pose.y);
      out.print("). Retracing "); out.print(remainingMoves); out.println(" outbound moves in reverse.");
      returning = true;
    }

    // Log the wall decisions already recorded by senseCurrentCell(), without
    // taking extra sensor readings that could disagree with this decision.
    out.print("EXPLORE step="); out.print(step); out.print(' '); printPose();
    const Heading observedDirs[3] = {
      turnLeft(pose.heading), pose.heading, turnRight(pose.heading)
    };
    const char *relativeNames[3] = {" L=", " F=", " R="};
    for (uint8_t i = 0; i < 3; ++i) {
      out.print(relativeNames[i]);
      out.print(_map.hasWall(pose.x, pose.y, observedDirs[i]) ? "W" : "OPEN");
    }
    out.println();

    // Do not finish at an intermediate visit to START inside an outbound loop.
    if (returning && remainingMoves == 0) {
      if (pose.x != 0 || pose.y != 0) {
        out.println("Return failed: move history ended away from START.");
        dumpStopMap();
        return false;
      }
      StoredMaze stored;
      _map.exportTo(stored);
      if (!_map.buildShortestPath(stored.path, stored.pathLen, stored.goalX, stored.goalY)) {
        out.println("Exploration completed, but no fully-known shortest path could be built.");
        dumpStopMap();
        return false;
      }
      out.print("Shortest confirmed path length: "); out.print(stored.pathLen); out.println(" cells");
      dumpStopMap();
      return _storage.save(stored, out);
    }

    Heading next;
    if (returning) {
      next = opposite(static_cast<Heading>(outboundMoves[remainingMoves - 1]));
    } else if (remainingMoves >= MAX_EXPLORE_MOVES) {
      out.println("Exploration stopped: outbound move history limit reached before goal.");
      dumpStopMap();
      return false;
    } else if (!_map.chooseNext(pose, false, next)) {
      out.println("Exploration failed: no reachable next cell.");
      out.println("Planner found no neighbouring cell with a route to the target in the current map.");
      dumpStopMap();
      return false;
    }
    out.print(returning ? "RETRACE -> " : "CHOOSE -> ");
    out.println(headingNames[(uint8_t)next & 3]);
    _map.setWall(pose.x, pose.y, next, false);
    if (!stepPose(pose, next, pwm)) {
      out.println("Exploration motion failed or was killed.");
      out.println("Motion may be incomplete; recorded pose may differ from physical position.");
      dumpStopMap();
      return false;
    }
    // Record/pop only completed cell movements; retain all outbound detours.
    if (returning) --remainingMoves;
    else outboundMoves[remainingMoves++] = static_cast<uint8_t>(next);
  }

  out.println("Exploration stopped: step safety limit reached.");
  dumpStopMap();
  return false;
}

bool MazeNavigator::executeAbsolutePath(const uint8_t *path, uint16_t len,
                                        Pose &pose, int pwm, Print &out) {
  for (uint16_t i = 0; i < len; ++i) {
    Heading next = static_cast<Heading>(path[i] & 0x03);
    if (!stepPose(pose, next, pwm)) {
      out.print("Path failed at step "); out.println(i);
      return false;
    }
  }
  return true;
}

bool MazeNavigator::storedRun(int outboundPwm, bool returnHome, int returnPwm, Print &out) {
  _motion.invalidateMazeSegmentAnchor();
  StoredMaze stored;
  if (!_storage.load(stored, out)) return false;
  if (stored.pathLen == 0) {
    out.println("Stored path is empty.");
    return false;
  }

  _map.load(stored);
  Pose pose;
  pose.x = 0;
  pose.y = 0;
  pose.heading = Heading::North;
  out.print("Running stored path, "); out.print(stored.pathLen); out.println(" cell moves.");
  if (!executeAbsolutePath(stored.path, stored.pathLen, pose, outboundPwm, out)) return false;

  if (returnHome) {
    out.println("Destination reached. Autonomous return to start.");
    uint8_t reversePath[MAX_PATH];
    for (uint16_t i = 0; i < stored.pathLen; ++i) {
      Heading f = static_cast<Heading>(stored.path[stored.pathLen - 1 - i] & 0x03);
      reversePath[i] = static_cast<uint8_t>(opposite(f));
    }
    if (!executeAbsolutePath(reversePath, stored.pathLen, pose, returnPwm, out)) return false;
    if (pose.x != 0 || pose.y != 0) {
      out.println("Return path ended away from logical start: FAIL");
      return false;
    }
  }
  return true;
}

bool MazeNavigator::dumpStored(Print &out) {
  StoredMaze stored;
  if (!_storage.load(stored, out)) return false;
  _map.load(stored);
  out.print("Stored goal: ("); out.print(stored.goalX); out.print(','); out.print(stored.goalY); out.println(")");
  out.print("Stored path len: "); out.println(stored.pathLen);
  out.print("Absolute path: ");
  const char hc[4] = {'N','E','S','W'};
  for (uint16_t i = 0; i < stored.pathLen; ++i) out.print(hc[stored.path[i] & 0x03]);
  out.println();
  _map.print(out);
  return true;
}

} // namespace MM3
