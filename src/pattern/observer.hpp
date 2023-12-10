#pragma once
// #include "common/caic_assert.h"

#ifndef RELEASE
#include <cassert>
#define ASSERT(f) assert(f)  // PRQA S 6005
#else
#define ASSERT(f) ((void)0)
#endif

namespace cm::pattern {

template <typename T>
class Observer {
 public:
  virtual ~Observer() = default;

  virtual void callback(std::shared_ptr<T> subject, const std::string& topic) {
    callback(subject.get(), topic);
  }

  virtual void callback(T* subject, const std::string& topic) { callback(*subject, topic); }

  virtual void callback(std::shared_ptr<const T> subject, const std::string& topic) {
    callback(*subject, topic);
  }

  virtual void callback(const T* const subject, const std::string& topic) {
    callback(*subject, topic);
  }

  virtual void callback(const T&, const std::string&) {
    ASSERT(false && "Expect user defined callback()");
  }
};

template <typename T>
class Subject {
 public:
  virtual ~Subject() = default;

  void addDownstream(Observer<T>* observer) { observers_.emplace_back(observer); }

  template <typename C>
  void notifyAll(C&& subject, const std::string& topic) {
    for (Observer<T>* const observer : observers_) {
      observer->callback(std::forward<C>(subject), topic);
    }
  }

  const std::vector<Observer<T>*>& getDownstreams() const { return observers_; }

 protected:
  std::vector<Observer<T>*> observers_;
};

}  // namespace cm::pattern
