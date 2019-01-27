#ifndef BLACK_BOARD_H
#define BLACK_BOARD_H

#include <chrono>
#include <thread>
#include <vector>

#include <ros/ros.h>

namespace shop{
namespace decision{

class Blackboard: public std::enable_shared_from_this<Blackboard>
{
public:
    typedef std::shared_ptr<Blackboard> Ptr;
    Blackboard()
    {};
    virtual ~Blackboard() = default;
protected:

};

} // namespace decision
} // namespace shop
#endif