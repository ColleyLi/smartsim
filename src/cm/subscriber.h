#pragma once

#include <memory>

namespace cm {

struct SubscriberImpl;

class Subscriber {
 public:
  inline Subscriber(Subscriber&&);
  inline ~Subscriber();

 protected:
  friend class NodeHandle;
  inline Subscriber(std::unique_ptr<SubscriberImpl>&&);

 private:
  std::unique_ptr<SubscriberImpl> impl_;
};

}  // namespace cm