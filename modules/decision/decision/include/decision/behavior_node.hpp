#ifndef BEHAVIOR_NODE_H
#define BEHAVIOR_NODE_H

#include <chrono>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include <common/rrts.h>
#include <blackboard/black_board.hpp>
#include <decision/private_board.hpp>

namespace shop
{
namespace decision
{

//节点的类型
enum class BehaviorType
{
    PARALLEL,     //平行
    SELECTOR,     //选择
    SEQUENCE,     //序列
    ACTION,       //动作
    PRECONDITION, //初始
    CYCLE,        //循环
    SUCCESSDO     //子节点成功执行
};

//节点运行状态
enum class BehaviorState
{
    RUNNING, //运行
    SUCCESS, //成功
    FAILURE, //失败
    IDLE,    //空闲
};

// AbortType：
// - Self：是否可以中断自身的循环运行，只能中断同级（拥有相同的父节点（装饰节点）的Action节点。
// - LOW_PRIORITY：是否可以中断比自己优先级低的任务的循环运行（从左向右，优先级降低），
// - 当一个高优先级装饰节点A下的Condiontal子节点发生变化时，他只能中断一个低优先级的Action节点（跟A节点同级，但在A节点右侧的Action节点）。
// - Both：都进行中断，子节点Action既中断与自己父类同级的子节点Action（self），又中断与自己同级的子节点Action（lower）。
enum class AbortType
{
    NONE,
    SELF,
    LOW_PRIORITY,
    BOTH
};

//行为节点，核心

class BehaviorNode : public std::enable_shared_from_this<BehaviorNode>
{
  public:
    typedef std::shared_ptr<BehaviorNode> Ptr;

    BehaviorNode(std::string name, BehaviorType behavior_type,
                 const PrivateBoard::Ptr &blackboard_ptr) : name_(name), behavior_type_(behavior_type),
                                                            blackboard_ptr_(blackboard_ptr),
                                                            behavior_state_(BehaviorState::IDLE),
                                                            parent_node_ptr_(nullptr)
    {
    }
    virtual ~BehaviorNode() = default;

    //运行函数,封装统一接口
    BehaviorState Run()
    {
        if (behavior_state_ != BehaviorState::RUNNING)
        {
            OnInitialize();
        }
        behavior_state_ = Update();
        //结束调用函数OnTerminate
        if (behavior_state_ != BehaviorState::RUNNING)
        {
            OnTerminate(behavior_state_);
        }
        return behavior_state_;
    }

    // @ breif 重新挂载节点
    virtual void Reset()
    {
        if (behavior_state_ != BehaviorState::RUNNING)
        {
            behavior_state_ = BehaviorState::IDLE;
            OnTerminate(behavior_state_);
        }
    }
    BehaviorType GetBehaviorType()
    {
        return behavior_type_;
    }
    BehaviorState GetBehaviorState()
    {
        return behavior_state_;
    }
    std::string GetName()
    {
        return name_;
    }
    void SetParent(BehaviorNode::Ptr parent_node_ptr)
    {
        parent_node_ptr_ = parent_node_ptr;
    }

  protected:
    std::string name_;
    //类型
    BehaviorType behavior_type_;
    //黑板给入
    PrivateBoard::Ptr blackboard_ptr_;
    //状态
    BehaviorState behavior_state_;
    //父节点指针
    BehaviorNode::Ptr parent_node_ptr_;

    //逻辑
    virtual BehaviorState Update() = 0;
    //获取数据和资源,初始化
    virtual void OnInitialize() = 0;
    //释放资源
    virtual void OnTerminate(BehaviorState state) = 0;
};

//行为节点，最后的执行机制
// @brief 为虚类,被继承
class ActionNode : public BehaviorNode
{
  public:
    ActionNode(std::string name, const PrivateBoard::Ptr &blackboard_ptr) : BehaviorNode::BehaviorNode(name, BehaviorType::ACTION, blackboard_ptr)
    {
    }
    ~ActionNode() = default;

  protected:
    virtual void OnInitialize() = 0;
    virtual BehaviorState Update() = 0;
    virtual void OnTerminate(BehaviorState state) = 0;
};

// @breif 装饰节点
// 装饰器是只有一个子节点的行为，
//            A
//            |
//            |
//            B
//装饰即是在子节点的原有逻辑上增添细节(如重复执行子节点，改变子节点返回状态等)
class DecoratorNode : public BehaviorNode
{
  public:
    DecoratorNode(std::string name, BehaviorType behavior_type, const PrivateBoard::Ptr &blackboard_ptr,
                  const BehaviorNode::Ptr &child_node_ptr = nullptr) : BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
                                                                       child_node_ptr_(child_node_ptr) {}

    virtual ~DecoratorNode() = default;

    //注意，设在子节点时一定要调用这个函数，把此节点的指针给到父节点
    void SetChild(const BehaviorNode::Ptr &child_node_ptr)
    {
        child_node_ptr_ = child_node_ptr;
        child_node_ptr->SetParent(shared_from_this());
    }

  protected:
    BehaviorNode::Ptr child_node_ptr_;

    virtual void OnInitialize() = 0;
    virtual BehaviorState Update() = 0;
    virtual void OnTerminate(BehaviorState state) = 0;
};

class SuccessDoNode : public DecoratorNode
{
  public:
    SuccessDoNode(std::string name, const PrivateBoard::Ptr &blackboard_ptr,
                  const BehaviorNode::Ptr &child_node_ptr = nullptr,
                  std::function<bool()> precondition_function = std::function<bool()>())
        : DecoratorNode(name, BehaviorType::SUCCESSDO, blackboard_ptr, child_node_ptr),
          precondition_function_(precondition_function)
    {
    }
    virtual ~SuccessDoNode() = default;

  protected:
    std::function<bool()> precondition_function_;
    virtual void OnInitialize()
    {
        ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
    }

    virtual BehaviorState Update()
    {
        while (true)
        {
            BehaviorState state = child_node_ptr_->Run();
            if (state != BehaviorState::SUCCESS)
            {
                return state;
            }
            else
            {
                if (precondition_function_)
                {
                    precondition_function_();
                    return BehaviorState::SUCCESS;
                }
                else
                {
                    return BehaviorState::FAILURE;
                }
            }
        }
    }

    virtual void OnTerminate(BehaviorState state)
    {

        switch (state)
        {
        case BehaviorState::IDLE:
            ROS_INFO("%s %s is IDLE", name_.c_str(), __FUNCTION__);
            child_node_ptr_->Reset();

            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s is SUCCESS", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s is FAILURE", name_.c_str(), __FUNCTION__);
            child_node_ptr_->Reset();
            break;
        default:
            ROS_ERROR("%s %s ERROR", name_.c_str(), __FUNCTION__);
            return;
        }
    }
};

class CycleNode : public DecoratorNode
{
  public:
    CycleNode(uint8_t cycle_num, std::string name, const PrivateBoard::Ptr &blackboard_ptr,
              const BehaviorNode::Ptr &child_node_ptr = nullptr)
        : DecoratorNode(name, BehaviorType::CYCLE, blackboard_ptr, child_node_ptr),
          cycle_num_(cycle_num), count_(0)
    {
    }
    virtual ~CycleNode() = default;

  protected:
    uint8_t cycle_num_;
    uint8_t count_;
    virtual void OnInitialize()
    {
        count_ = 0;
        ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
    }
    virtual BehaviorState Update()
    {
        while (true)
        {
            BehaviorState state = child_node_ptr_->Run();
            if (state != BehaviorState::SUCCESS)
            {
                return state;
            }
            else
            {
                count_++;
                if (count_ == cycle_num_)
                {
                    return BehaviorState::SUCCESS;
                }
                else
                {
                    child_node_ptr_->Reset();
                    return BehaviorState::RUNNING;
                }
            }
        }
    }
    virtual void OnTerminate(BehaviorState state)
    {

        switch (state)
        {
        case BehaviorState::IDLE:
            ROS_INFO("%s %s is IDLE", name_.c_str(), __FUNCTION__);
            child_node_ptr_->Reset();

            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s is SUCCESS", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s is FAILURE", name_.c_str(), __FUNCTION__);
            child_node_ptr_->Reset();
            break;
        default:
            ROS_ERROR("%s %s ERROR", name_.c_str(), __FUNCTION__);
            return;
        }
    }

  private:
};

// @brief 构造器
// Example:
//   auto jud1_ptr = std::make_shared<shop::decision::PreconditionNode>
//                       ("jud1_tests", blackboard_ptr,
//                           action1_ptr,
//                           [&]()
//                           {
//                              return false;
//                           },
//                           shop::decision::AbortType::LOW_PRIORITY
//                       );
//
//  @param name: 描述节点名字
//         blackboard_ptr: 黑板类指针
//         child_node_ptr: 子节点指针
//         precondition_function:逻辑
//         abort_type: 节点属性
class PreconditionNode : public DecoratorNode
{
  public:
    PreconditionNode(std::string name, const PrivateBoard::Ptr &blackboard_ptr,
                     const BehaviorNode::Ptr &child_node_ptr = nullptr,
                     std::function<bool()> precondition_function = std::function<bool()>(),
                     AbortType abort_type = AbortType::NONE)
        : DecoratorNode::DecoratorNode(name, BehaviorType::PRECONDITION, blackboard_ptr, child_node_ptr),
          precondition_function_(precondition_function), abort_type_(abort_type)
    {
    }
    virtual ~PreconditionNode() = default;
    AbortType GetAbortType()
    {
        return abort_type_;
    }

  protected:
    //new 保护措施
    //传一个判断函数进来,可以理解为函数指针
    std::function<bool()> precondition_function_;
    AbortType abort_type_;

    virtual void OnInitialize()
    {
        ROS_INFO("%s", name_.c_str());
    }
    // @berif true return ture ,false printf&&return false
    virtual bool Precondition()
    {
        if (precondition_function_)
        {
            return precondition_function_();
        }
        else
        {
            return false;
        }
    }
    virtual BehaviorState Update()
    {
        if (child_node_ptr_ == nullptr)
        {
            return BehaviorState::SUCCESS;
        }
        //Reevaluarion
        if (Reevaluation())
        {
            BehaviorState state = child_node_ptr_->Run();
            return state;
        }
        return BehaviorState::FAILURE;
    }
    virtual void OnTerminate(BehaviorState state)
    {

        switch (state)
        {
        case BehaviorState::IDLE:
            ROS_INFO("%s %s is IDLE", name_.c_str(), __FUNCTION__);
            child_node_ptr_->Reset();
            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s is SUCCESS", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s is FAILURE", name_.c_str(), __FUNCTION__);
            child_node_ptr_->Reset();
            break;
        default:
            ROS_ERROR("%s %s ERROR", name_.c_str(), __FUNCTION__);
            return;
        }
    }
    virtual bool Reevaluation();
};

//复合节点，及一个节点可以有多个子节点
class CompositeNode : public BehaviorNode
{
  public:
    CompositeNode(std::string name, BehaviorType behavior_type, const PrivateBoard::Ptr &blackboard_ptr) : BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
                                                                                                           children_node_index_(0)
    {
    }
    virtual ~CompositeNode() = default;
    // @breif 加入一个子节点
    // @param child_node_ptr：节点指针
    virtual void AddChildren(const BehaviorNode::Ptr &child_node_ptr)
    {
        children_node_ptr_.push_back(child_node_ptr);
        (child_node_ptr)->SetParent(shared_from_this());
    }
    // @breif 加入list节点
    // @param child_node_ptr_list：节点list
    virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list)
    {
        for (auto child_node_ptr = child_node_ptr_list.begin();
             child_node_ptr != child_node_ptr_list.end(); child_node_ptr++)
        {
            children_node_ptr_.push_back(*child_node_ptr);
            (*child_node_ptr)->SetParent(shared_from_this());
        }
    }
    std::vector<BehaviorNode::Ptr> GetChildren()
    {
        return children_node_ptr_;
    }
    unsigned int GetChildrenIndex()
    {
        return children_node_index_;
    }
    unsigned int GetChildrenNum()
    {
        return children_node_ptr_.size();
    }

  protected:
    //子节点指针向量容器
    std::vector<BehaviorNode::Ptr> children_node_ptr_;
    unsigned int children_node_index_;

    virtual BehaviorState Update() = 0;
    virtual void OnInitialize() = 0;
    virtual void OnTerminate(BehaviorState state) = 0;
};

// 选择节点
// 选择子节点的某一个执行
class SelectorNode : public CompositeNode
{
  public:
    SelectorNode(std::string name, const PrivateBoard::Ptr &blackboard_ptr) : CompositeNode::CompositeNode(name, BehaviorType::SELECTOR, blackboard_ptr)
    {
    }
    virtual ~SelectorNode() = default;
    // @breif 加入单个子节点
    // @param child_node_ptr：节点指针
    virtual void AddChildren(const BehaviorNode::Ptr &child_node_ptr)
    {
        CompositeNode::AddChildren(child_node_ptr);
        //實現對優先級的判斷
        children_node_reevaluation_.push_back(child_node_ptr->GetBehaviorType() == BehaviorType::PRECONDITION && (std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() == AbortType::LOW_PRIORITY || std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() == AbortType::BOTH));
    }
    // @breif 加入多个子节点
    // @param child_node_ptr_list：节点list
    virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list)
    {
        CompositeNode::AddChildren(child_node_ptr_list);
        //遍历
        for (auto child_node_ptr = child_node_ptr_list.begin(); child_node_ptr != child_node_ptr_list.end(); child_node_ptr++)
        {
            //實現對優先級的判斷
            children_node_reevaluation_.push_back((*child_node_ptr)->GetBehaviorType() == BehaviorType::PRECONDITION && (std::dynamic_pointer_cast<PreconditionNode>(*child_node_ptr)->GetAbortType() == AbortType::LOW_PRIORITY || std::dynamic_pointer_cast<PreconditionNode>(*child_node_ptr)->GetAbortType() == AbortType::BOTH));
        }
    }
    void SetChildrenIndex(unsigned int children_node_index)
    {
        children_node_index_ = children_node_index;
    }

  protected:
    std::vector<bool> children_node_reevaluation_;

    virtual void OnInitialize()
    {
        children_node_index_ = 0;
        ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
    }
    virtual BehaviorState Update()
    {
        if (children_node_ptr_.size() == 0)
        {
            return BehaviorState::SUCCESS;
        }
        //Reevaluation
        //init後children_node_index_爲0
        for (unsigned int index = 0; index < children_node_index_; index++)
        {
            ROS_INFO("Reevaluation");
            //對LOW_PRIORITY和BOTH的Precondition進行判斷
            if (children_node_reevaluation_.at(index))
            {
                BehaviorState state = children_node_ptr_.at(index)->Run();
                if (index == children_node_index_)
                {
                    ROS_INFO("%s abort goes on!", name_.c_str());
                    if (state != BehaviorState::FAILURE)
                    {
                        return state;
                    }
                    
                    ++children_node_index_;
                    break;
                }
            }
        }
        while (true)
        {
            BehaviorState state = children_node_ptr_.at(children_node_index_)->Run();

            if (state != BehaviorState::FAILURE)
            {
                return state;
            }
            if (++children_node_index_ == children_node_ptr_.size())
            {
                children_node_index_ = 0;
                return BehaviorState::FAILURE;
            }
        }
    }
    virtual void OnTerminate(BehaviorState state)
    {
        switch (state)
        {
        case BehaviorState::IDLE:
            ROS_INFO("%s %s is IDLE", name_.c_str(), __FUNCTION__);
            children_node_ptr_.at(children_node_index_)->Reset();
            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s is SUCCESS", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s is FAILURE", name_.c_str(), __FUNCTION__);
            break;
        default:
            ROS_ERROR("%s %s ERROR", name_.c_str(), __FUNCTION__);
            return;
        }
    }
};

// 序列执行节点
// 将其所有子节点依次执行，也就是说当前一个返回“完成”状态后，再运行先一个子节点
class SequenceNode : public CompositeNode
{
  public:
    SequenceNode(std::string name, const PrivateBoard::Ptr &blackboard_ptr)
        : CompositeNode::CompositeNode(name, BehaviorType::SEQUENCE, blackboard_ptr)
    {
    }
    virtual ~SequenceNode() = default;

  protected:
    virtual void OnInitialize()
    {
        children_node_index_ = 0;
        ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
    }

    virtual BehaviorState Update()
    {
        if (children_node_ptr_.size() == 0)
        {
            return BehaviorState::SUCCESS;
        }
        while (true)
        {
            //依次执行
            BehaviorState state = children_node_ptr_.at(children_node_index_)->Run();

            if (state != BehaviorState::SUCCESS)
            {
                return state;
            }
            if (++children_node_index_ == children_node_ptr_.size())
            {
                children_node_index_ = 0;
                return BehaviorState::SUCCESS;
            }
        }
    }
    virtual void OnTerminate(BehaviorState state)
    {
        switch (state)
        {
        case BehaviorState::IDLE:
            ROS_INFO("%s %s is IDLE", name_.c_str(), __FUNCTION__);
            if (children_node_ptr_.size() != 0)
            {
                children_node_ptr_.at(children_node_index_)->Reset();
            }
            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s is SUCCESS", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s is FAILURE", name_.c_str(), __FUNCTION__);
            break;
        default:
            ROS_ERROR("%s %s ERROR", name_.c_str(), __FUNCTION__);
            return;
        }
    }
};

// 平行运行节点
// 将其所有子节点都运行一遍
class ParallelNode : public CompositeNode
{
  public:
    ParallelNode(std::string name, const PrivateBoard::Ptr &blackboard_ptr, unsigned int threshold)
        : CompositeNode::CompositeNode(name, BehaviorType::PARALLEL, blackboard_ptr),
          threshold_(threshold), success_count_(0),
          failure_count_(0)
    {
    }
    virtual ~ParallelNode() = default;

  protected:
    //成员变量
    std::vector<bool> children_node_done_;
    unsigned int success_count_;
    unsigned int failure_count_;
    unsigned int threshold_;
    //function
    virtual void OnInitialize()
    {
        failure_count_ = 0;
        success_count_ = 0;
        //复位
        children_node_done_.clear();
        children_node_done_.resize(children_node_ptr_.size(), false);
        ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
    }
    virtual BehaviorState Update()
    {
        if (children_node_ptr_.size() == 0)
        {
            return BehaviorState::SUCCESS;
        }

        for (unsigned int index = 0; index != children_node_ptr_.size(); index++)
        {
            if (children_node_done_.at(index) == false)
            {
                BehaviorState state = children_node_ptr_.at(index)->Run();

                if (state == BehaviorState::SUCCESS)
                {
                    //当成功节点大于预设的节点数时
                    //平行节点ruturn成功
                    children_node_done_.at(index) = true;
                    if (++success_count_ >= threshold_)
                    {
                        return BehaviorState::SUCCESS;
                    }
                }
                else if (state == BehaviorState::FAILURE)
                {
                    children_node_done_.at(index) = true;
                    //只有当失败节点数量大于
                    if (++failure_count_ >= children_node_ptr_.size() - threshold_)
                    //0 
                    {
                        return BehaviorState::FAILURE;
                    }
                }
            }
        }
        return BehaviorState::RUNNING;
    }
    virtual void OnTerminate(BehaviorState state)
    {
        switch (state)
        {
        case BehaviorState::IDLE:
            ROS_INFO("%s %s is IDLE", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s is SUCCESS", name_.c_str(), __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s is FAILURE", name_.c_str(), __FUNCTION__);
            ;
            break;
        default:
            ROS_ERROR("%s %s ERROR", name_.c_str(), __FUNCTION__);
            return;
        }
        for (unsigned int index = 0; index != children_node_ptr_.size(); index++)
        {
            children_node_ptr_.at(index)->Reset();
        }
    }
};

//初始化节点的评估函数，初始化的时候判断是否能够启动
bool PreconditionNode::Reevaluation()
{

    // Back Reevaluation
    //检查父节点的存在，父节点的类型是否为选择节点，本身的是否为
    if (parent_node_ptr_ != nullptr && parent_node_ptr_->GetBehaviorType() == BehaviorType::SELECTOR && (abort_type_ == AbortType::LOW_PRIORITY || abort_type_ == AbortType::BOTH))
    {
        auto parent_selector_node_ptr = std::dynamic_pointer_cast<SelectorNode>(parent_node_ptr_);

        auto parent_children = parent_selector_node_ptr->GetChildren();
        //在父节点的子节点中找到自己
        auto iter_in_parent = std::find(parent_children.begin(), parent_children.end(), shared_from_this());
        if (iter_in_parent == parent_children.end())
        {
            ROS_ERROR_STREAM("Can't find current node in parent!");
            return false;
        }
        //迭代与自己同级的,在自己之前的子节点
        unsigned int index_in_parent = iter_in_parent - parent_children.begin();
        if (index_in_parent < parent_selector_node_ptr->GetChildrenIndex())
        {
            //@breif自己描述的一个判断fuc,Precondition
            if (Precondition())
            {
                //Abort Measures
                ROS_INFO("Abort Measures");
                //把在自己运行之前的节点reset,转为执行自己
                parent_children.at(parent_selector_node_ptr->GetChildrenIndex())->Reset();
                parent_selector_node_ptr->SetChildrenIndex(index_in_parent);
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    // Self Reevaluation本身的评估
    if (abort_type_ == AbortType::SELF || abort_type_ == AbortType::BOTH || child_node_ptr_->GetBehaviorState() != BehaviorState::RUNNING)
    {
        if (!Precondition())
        {
            return false;
        }
    }
    return true;
}

} // namespace decision
} // namespace shop

#endif