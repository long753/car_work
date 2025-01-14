#ifndef SINGLETION_HPP
#define SINGLETION_HPP
#include <concepts>
template <typename T>
class SingletonHolder // 单例

{
public:
static T& get_instance()
{
  static T unique_instance;
  return unique_instance;
}
virtual ~SingletonHolder() = default;
 SingletonHolder(const SingletonHolder&) = delete;
 SingletonHolder(SingletonHolder&&) = delete;

 SingletonHolder& operator=(const SingletonHolder&) = delete;
 SingletonHolder& operator=(SingletonHolder&&) = delete;

 SingletonHolder() = default;
};
template <typename T>
concept Singleton = std::is_base_of_v<SingletonHolder<T>, T>;

#endif