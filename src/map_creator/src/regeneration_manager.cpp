// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include "map_creator/regeneration_manager.hpp"

#include <sys/wait.h>

#include <cstdlib>

namespace map_creator
{

RegenerationManager::~RegenerationManager()
{
  join();
}

void RegenerationManager::join()
{
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool RegenerationManager::start(const std::string & command)
{
  if (running_.load()) {
    return false;
  }
  join();
  running_.store(true);
  finished_.store(false);
  exit_code_.store(-1);
  worker_ = std::thread(
    [this, command]() {
      const int raw = std::system(command.c_str());
      int code = -1;
      if (raw != -1 && WIFEXITED(raw)) {
        code = WEXITSTATUS(raw);
      }
      exit_code_.store(code);
      finished_.store(true);
      running_.store(false);
    });
  return true;
}

void RegenerationManager::reset()
{
  join();
  running_.store(false);
  finished_.store(false);
  exit_code_.store(-1);
}

}  // namespace map_creator
