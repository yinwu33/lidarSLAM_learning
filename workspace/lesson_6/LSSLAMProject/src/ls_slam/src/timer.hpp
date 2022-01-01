#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <ctime>
#include <iostream>

#include <unistd.h>


class Timer {
public:
  Timer(bool do_print = false) : do_print_(do_print) {}

  void Start() {
    time_start_ = std::chrono::system_clock::now();

    if (do_print_) {
      PrintTime(time_start_);
    }
  }

  double Stop() {
    time_end_ = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = time_end_ - time_start_;

    if (do_print_) {
      PrintTime(time_end_);
    }
    
    return elapsed_seconds.count();
  }

private:
  void PrintTime(const std::chrono::system_clock::time_point& time) {
    std::time_t now = std::chrono::system_clock::to_time_t(time);
    std::cout << std::ctime(&now) << std::endl;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> time_start_, time_end_;

  bool do_print_;
};

#endif // TIMER_HPP