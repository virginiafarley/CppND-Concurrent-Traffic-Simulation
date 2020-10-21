#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include "TrafficObject.h"
#include <condition_variable>
#include <deque>
#include <mutex>

// forward declarations to avoid include cycle
class Vehicle;

// FP.3 Define a class „MessageQueue“ which has the public methods send and
// receive. Send should take an rvalue reference of type TrafficLightPhase
// whereas receive should return this type. Also, the class should define an
// std::dequeue called _queue, which stores objects of type TrafficLightPhase.
// Also, there should be an std::condition_variable as well as an std::mutex as
// private members.

template <class T> class MessageQueue {
public:
  void send(T &&msg);
  T receive();

private:
  std::mutex _mutex;             // protect shared resource _queue
  std::condition_variable _cond; // signaling mechanism
  std::deque<T> _queue;
};

// FP.1 : Define a class „TrafficLight“ which is a child class of TrafficObject.
// The class shall have the public methods „void waitForGreen()“ and „void
// simulate()“ as well as „TrafficLightPhase getCurrentPhase()“, where
// TrafficLightPhase is an enum that can be either „red“ or „green“. Also, add
// the private method „void cycleThroughPhases()“. Furthermore, there shall be
// the private member _currentPhase which can take „red“ or „green“ as its
// value.

enum struct TrafficLightPhase { red, green }; // scoped enumeration

class TrafficLight : public TrafficObject {
public:
  // constructor / desctructor
  TrafficLight();

  // getters / setters

  // typical behaviour methods
  void waitForGreen();
  void simulate();
  TrafficLightPhase getCurrentPhase();

private:
  // typical behaviour methods
  void cycleThroughPhases(); // virtual function executed in a thread

  TrafficLightPhase _currentPhase; // phase of traffic light

  // FP.4b : create a private member of type MessageQueue for messages of type
  // TrafficLightPhase and use it within the infinite loop to push each new
  // TrafficLightPhase into it by calling send in conjunction with move
  // semantics.

  std::condition_variable _condition;
  std::mutex _mutex;
  MessageQueue<TrafficLightPhase> _messages; // queue of TrafficLightPhases
};

#endif