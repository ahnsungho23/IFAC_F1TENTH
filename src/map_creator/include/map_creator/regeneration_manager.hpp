// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#ifndef MAP_CREATOR__REGENERATION_MANAGER_HPP_
#define MAP_CREATOR__REGENERATION_MANAGER_HPP_

#include <atomic>
#include <string>
#include <thread>

namespace map_creator
{

// Runs the offline regeneration driver as a detached-thread subprocess so the
// ROS executor never blocks. Single-flight: start() while running() is a no-op.
class RegenerationManager
{
public:
  ~RegenerationManager();

  bool start(const std::string & command);
  bool running() const {return running_.load();}
  bool finished() const {return finished_.load();}
  int exitCode() const {return exit_code_.load();}
  void reset();

private:
  void join();

  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{false};
  std::atomic<int> exit_code_{-1};
};

}  // namespace map_creator

#endif  // MAP_CREATOR__REGENERATION_MANAGER_HPP_
