#include "event_queue.h"

// constructor and destructor
EventQueue::EventQueue() : queue(), mutex(), condition() { printf("EventQueue constructor\n"); }

// add a message to the queue
void EventQueue::enqueue(Event& e) {
  // lock the mutex
  std::lock_guard<std::mutex> lock(mutex);

  // push the element to the queue
  queue.push(e);

  // unlock the thread
  condition.notify_one();
}

// get the front message from the queue
const Event EventQueue::dequeue() {
  // lock the mutex
  std::unique_lock<std::mutex> lock(mutex);

  if (!queue.empty())
  {
    // remove the front message
    Event e = queue.front();
    queue.pop();
    return e;
  }
  else
    // return an empty message
    return Event();
}

// do nothing if the queue is empty
bool EventQueue::isEmpty() const {
  return queue.empty();
}
