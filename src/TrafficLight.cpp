#include "TrafficLight.h"
#include <iostream>
#include <random>
#include <ratio>

/* Implementation of class "MessageQueue" */

template <typename T> T MessageQueue<T>::receive() {
  // FP.5a : The method receive should use std::unique_lock<std::mutex>
  // and_condition.wait() to wait for and receive new messages and pull them
  // from the queue using move semantics. The received object should then be
  // returned by the receive function.

  // perform queue modification under the lock
  std::unique_lock<std::mutex> lck(_mutex);
  _cond.wait(lck, [this] {
    return !_queue.empty();
  }); // pass unique lock to condition variable

  // remove last element from queue
  T msg = std::move(_queue.back());
  _queue.pop_back();

  return msg; // not copied due to RVO
}

template <typename T> void MessageQueue<T>::send(T &&msg) {
  // FP.4a : The method send should use the mechanisms
  // std::lock_guard<std::mutex> as well as _condition.notify_one() to add a new
  // message to the queue and afterwards send a notification.

  // perform queue modification under the lock
  std::lock_guard<std::mutex> lck(_mutex);

  // add message to queue
  _queue.push_back(std::move(msg));

  // notify client after pushing new messasge into queue
  _cond.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight() { _currentPhase = TrafficLightPhase::red; }

void TrafficLight::waitForGreen() {
  // FP.5b : add the implementation of the method waitForGreen, in which an
  // infinite while-loop runs and repeatedly calls the receive function on the
  // message queue. Once it receives TrafficLightPhase::green, the method
  // returns.

  // return once message queue receives green TrafflicLightPhases
  while (true) {
    if (_messages.receive() == TrafficLightPhase::green) {
      return;
    }
  }
}

TrafficLightPhase TrafficLight::getCurrentPhase() { return _currentPhase; }

void TrafficLight::simulate() {
  // FP.2b : Finally, the private method „cycleThroughPhases“ should be started
  // in a thread when the public method „simulate“ is called. To do this, use
  // the thread queue in the base class.

  // start thread from member function cycleThroughPhases
  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases() {
  // FP.2a : Implement the function with an infinite loop that measures the time
  // between two loop cycles and toggles the current phase of the traffic light
  // between red and green and sends an update method to the message queue using
  // move semantics. The cycle duration should be a random value between 4 and 6
  // seconds. Also, the while-loop should use std::this_thread::sleep_for to
  // wait 1ms between two cycles.

  // random number with mt19937
  std::random_device device;
  std::mt19937 generator(device());

  // random value between 4 and 6 seconds
  std::uniform_real_distribution<double> distribution(4.0, 6.0);
  double cycleDurationMin = distribution(generator);

  // initialize start time
  std::chrono::high_resolution_clock::time_point startTime =
      std::chrono::high_resolution_clock::now();

  while (true) {
    // wait 1ms between two cycles
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // calculate cycle duration
    std::chrono::high_resolution_clock::time_point currTime =
        std::chrono::high_resolution_clock::now();
    double cycleDuration =
        std::chrono::duration<double, std::ratio<1, 1>>(currTime - startTime)
            .count();

    // only update if cycle duration has elapsed
    if (cycleDuration < cycleDurationMin) {
      continue;
    }
    // toggle current traffic light phase
    switch (_currentPhase) {
    case TrafficLightPhase::red:
      _currentPhase = TrafficLightPhase::green;
      break;
    case TrafficLightPhase::green:
      _currentPhase = TrafficLightPhase::red;
    default:
      _currentPhase = TrafficLightPhase::red;
      break;
    }

    // send update method to message queue
    TrafficLightPhase msg = _currentPhase;
    _messages.send(std::move(msg));

    // update start time for next iteration
    startTime = std::chrono::high_resolution_clock::now();
  }
}