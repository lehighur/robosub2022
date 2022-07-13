#include <queue>
#include <mutex>
#include <condition_variable>
#include <cstdint>

enum EventType: uint8_t {
  MESSAGE,
  CONTROL,
  CAMERA,
  BATTERY,
  OVERRIDE,
  CLEAR,
};

struct Event {
  EventType kind;
  // union
  uint8_t content[8];
};

class EventQueue {
  public:
    // constructor and destructor
    EventQueue();

    // add a message to the queue
    void enqueue(Event& e);

    // get the front message from the queue
    const Event dequeue();

    // do nothing if the queue is empty
    bool isEmpty() const;

  private:
    std::queue<Event> queue;				// the actual queue
    mutable std::mutex mutex;			// the mutex (basically telling which thread is allowed to access the queue)
    std::condition_variable condition;	// block the calling thread until notified to resume
};
