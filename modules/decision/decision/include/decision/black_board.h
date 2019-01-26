#ifndef BLACK_BOARD_H
#define BLACK_BOARD_H

namespace shop{
namespace decision
{
class Blackboard
{
  public:
    typedef std::shared_ptr<Blackboard> Ptr;
    Blackboard()
    {};
    virtual ~Blackboard() = default;

    bool test1(){
        return true;
    }
    bool test2()
    {
        return true;
    }
    bool test3()
    {
        return true;
    }
    bool test4()
    {
        return true;
    }    
    bool test5()
    {
        return true;
    }    
    bool test6()
    {
        return true;
    }    
};

} // namespace decision
} // namespace shop
#endif