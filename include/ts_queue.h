#ifndef TS_QUEUE
#define TS_QUEUE

#include <queue>
#include <mutex>
#include <condition_variable>

template<class T>
class TSQueue {
  public:
    // constructor and destructor
    TSQueue(): q(), mtx(), cv() { printf("EventQueue constructor\n"); };

    void enqueue(T e) {
      std::lock_guard<std::mutex> lck(mtx);
      q.push(e);
      cv.notify_one();
    };

    T dequeue() {
      std::unique_lock<std::mutex> lck(mtx);
      cv.wait(lck, [this]{return !q.empty();});
      T e = q.front();
      q.pop();
      return e;
    };

    int size() {
      return q.size();
    };

    T front() {
      return q.front();
    };

    T back() {
      return q.back();
    };

    void pop() {
      q.pop();
    };

    bool empty() {
      return q.empty();
    };

  private:
    std::queue<T> q;
    mutable std::mutex mtx;			// the mutex (basically telling which thread is allowed to access the queue)
    std::condition_variable cv;	// block the calling thread until notified to resume
};

#endif
