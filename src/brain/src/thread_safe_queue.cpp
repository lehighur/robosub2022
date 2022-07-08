#include <queue>
#include <mutex>
#include <condition_variable>
using namespace std;
struct Content {
  uint8_t bytes[8];
};

struct Event {
  string kind;
  Content *content;
};

template<typename T>
class ThreadSafeQueue
{
  public:
    // constructor and destructor
    ThreadSafeQueue() : queue(), mutex(), condition() {};
    ~ThreadSafeQueue() {};

    // add a message to the queue
    void enqueue(T& t)
    {
      // lock the mutex
      std::lock_guard<std::mutex> lock(mutex);

      // push the element to the queue
      queue.push(t);

      // unlock the thread
      condition.notify_one();
    }

    // get the front message from the queue
    const T dequeue()
    {
      // lock the mutex
      std::unique_lock<std::mutex> lock(mutex);

      if (!queue.empty())
      {
        // remove the front message
        T message = queue.front();
        queue.pop();
        return message;
      }
      else
        // return an empty message
        return T();
    }

    // do nothing if the queue is empty
    bool isEmpty() const
    {
      return queue.empty();
    }

  private:
    std::queue<T> queue;				// the actual queue
    mutable std::mutex mutex;			// the mutex (basically telling which thread is allowed to access the queue)
    std::condition_variable condition;	// block the calling thread until notified to resume
};
