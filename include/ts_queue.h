#ifndef TS_QUEUE
#define TS_QUEUE

#include <queue>
#include <mutex>
#include <condition_variable>

template<class T>
class TSQueue {

  // need to double check logic
  public:
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

    // probably don't ever want to use this
    int pop() {
      std::unique_lock<std::mutex> lck(mtx);
      if (q.empty()) {
        return -1;
      }
      q.pop();
      return 0;
    };

    int size() {
      std::unique_lock<std::mutex> lck(mtx);
      return q.size();
    };

    T front() {
      std::unique_lock<std::mutex> lck(mtx);
      cv.wait(lck, [this]{return !q.empty();});
      return q.front();
    };

    T back() {
      std::unique_lock<std::mutex> lck(mtx);
      cv.wait(lck, [this]{return !q.empty();});
      return q.back();
    };


    bool empty() {
      std::unique_lock<std::mutex> lck(mtx);
      return q.empty();
    };

  private:
    std::queue<T> q;
    mutable std::mutex mtx;
    std::condition_variable cv;
};

#endif
