#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <chrono>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include <black_borad.h>

namespace shop
{
namespace decision
{

enum class BehaviorType
{
    PARALLEL,
    SELECTOR,
    SEQUENCE,
    ACTION,
    PRECONDITION,
};
//节点运行状态
enum class BehaviorState
{
    RUNNING, //运行
    SUCCESS, //成功
    FAILURE, //失败
    IDLE,    //空闲
};
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

    BehaviorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr) : name_(name),
                                                                                                        behavior_type_(behavior_type),
                                                                                                        blackboard_ptr_(blackboard_ptr),
                                                                                                        behavior_state_(BehaviorState::IDLE),
                                                                                                        parent_node_ptr_(nullptr)
    {
    }
    virtual ~BehaviorNode() = default;
    //运行函数
    BehaviorState Run()
    {
        if (behavior_state_ != BehaviorState::RUNNING)
        {
            OnInitialize();
        }
        behavior_state_ = Update();
        if (behavior_state_ != BehaviorState::RUNNING)
        {
            OnInitialize();
        }
        return behavior_state_;
    }
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
    //TODO 对应黑板给入
    Blackboard::Ptr blackboard_ptr_;
    //状态
    BehaviorState behavior_state_;
    //父节点指针
    BehaviorNode::Ptr parent_node_ptr_;

    virtual void OnInitialize() = 0;
    virtual void OnTerminate(BehaviorState state) = 0;
    virtual BehaviorState Update() = 0;
};

//行为节点，最后的执行机制
class ActionNode : public BehaviorNode
{
  public:
    ActionNode(std::string name, const Blackboard::Ptr &blackboard_ptr) : BehaviorNode::BehaviorNode(name, BehaviorType::ACTION, blackboard_ptr)
    {
    }
    ~ActionNode() = default;

  protected:
    virtual void OnInitialize() = 0;
    virtual BehaviorState Update() = 0;
    virtual void OnTerminate(BehaviorState state) = 0;
};

//装饰器是只有一个子节点的行为，顾名思义，
//装饰即是在子节点的原有逻辑上增添细节(如重复执行子节点，改变子节点返回状态等)
class DecoratorNode : public BehaviorNode
{
  public:
    DecoratorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr,
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
    virtual void OnInitialize() = 0;
    virtual BehaviorState Update() = 0;
    virtual void OnTerminate(BehaviorState state) = 0;
    BehaviorNode::Ptr child_node_ptr_;
};

//初始化节点
class PreconditionNode : public DecoratorNode
{
  public:
    PreconditionNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
                     const BehaviorNode::Ptr &child_node_ptr = nullptr,
                     std::function<bool()> precondition_function = std::function<bool()>(),
                     AbortType abort_type = AbortType::NONE) : DecoratorNode::DecoratorNode(name, BehaviorType::PRECONDITION, blackboard_ptr, child_node_ptr),
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
    //TODO 等待理解
    std::function<bool()> precondition_function_;
    AbortType abort_type_;

    virtual void OnInitialize()
    {
        ROS_INFO("%s", &name_);
    }
    virtual bool Precondition()
    {
        if (precondition_function_)
        {
            return precondition_function_();
        }
        else
        {
            ROS_ERROR_STREAM("There is no chosen precondition function, then return false by default!");
            return false;
        }
    }
    virtual BehaviorNode Update()
    {
        if (child_node_ptr_ == nullptr)
        {
            retrun BehaviorState::SUCCESS;
        }
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
            ROS_INFO("%s %s is IDLE", name_, __FUNCTION__);
            child_node_ptr_->Reset();
            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s is SUCCESS", name_, __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s is FAILURE", name_, __FUNCTION__);
            child_node_ptr_->Reset();
            break;
        default:
            ROS_ERROR_STREAM("%s %s ERROR", name_, __FUNCTION__);
            return;
        }
    }
    virtual bool Reevaluation();
};

//复合节点，及一个节点可以有多个子节点
class CompositeNode : public BehaviorNode
{
  public:
    CompositeNode(std::string name, BehaviorType behavior_type, const blackboard::Ptr &blackboard_ptr) : BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
                                                                                                         children_node_index_(0) {}
    virtual ~CompositeNode() = default;

    virtual void AddChildren(const BehaviorNode::Ptr &child_node_ptr)
    {
        child_node_ptr_.push_back(*child_node_ptr);
        (*child_node_ptr)->SetParent(shared_from_this());
    }
    std::vector<BehaviorNode::Ptr> GetChildren()
    {
        return child_node_ptr_;
    }

    unsigned int GetChildrenIndex()
    {
        return children_node_index_;
    }
    unsigned int GetChildrenNum()
    {
        return child_node_ptr_.size();
    }

  protected:
    virtual BehaviorState Update() = 0;
    virtual void OnInitialize() = 0;
    virtual void OnTerminate(BehaviorState state) = 0;

    //子节点指针向量
    std::vector<BehaviorNode::Ptr> child_node_ptr_;
    unsigned int children_node_index_;
};

//选择节点
class SelectorNode : public CompositeNode
{
  public:
    SelectorNode(std::string name, const Blackboard::Ptr &blackboard_ptr) : CompositeNode::CompositeNode(name, BehaviorType::SELECTOR, blackboard_ptr) {}
    virtual ~SelectorNode() = default;

    virtual void AddChildren(const BehaviorNode::Ptr &child_node_ptr)
    {
        CompositeNode::AddChildren(child_node_ptr);

        children_node_reevaluation_.push_back(child_node_ptr->GetBehaviorTpye() == BehaviorType::PRECONITION && (std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() == AbortType::LOW_PRIORITY || std::dynamic_pointer_cast<PreconditionNode>(child_node_ptr)->GetAbortType() == AbortType::BOTH));
    }
    virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list)
    {
        CompositeNode::AddChildren(child_node_ptr_list);

        for (auto child_node_ptr = child_node_ptr_list.begin(); child_node_ptr != child_node_ptr_list.end(); child_node_ptr++)
        {
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
        ROS_INFO("%s %s", name_, __FUNCTION__);
    }
    virtual BehaviorState Update()
    {
        if (child_node_ptr_.size() == 0)
        {
            return BehaviorState::SUCCESS;
        }

        //Reevaluation 从新估计
        for (unsigned int index = 0; index < children_node_index_; index++)
        {
            ROS_INFO("Reevaluation");
            if (children_node_reevaluation_.at(index))
            {
                BehaviorState state = children_node_ptr_.at(index)->Run();
                if (index == children_node_index_)
                {
                    ROS_INFO("%s abort goes on!", name_);
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
            if (++children_node_index_ == children_node_ptr.size())
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
            ROS_INFO("%s %s is IDLE", name_, __FUNCTION__);
            child_node_ptr_->Reset();
            break;
        case BehaviorState::SUCCESS:
            ROS_INFO("%s %s is SUCCESS", name_, __FUNCTION__);
            break;
        case BehaviorState::FAILURE:
            ROS_INFO("%s %s is FAILURE", name_, __FUNCTION__);
            child_node_ptr_->Reset();
            break;
        default:
            ROS_ERROR_STREAM("%s %s ERROR", name_, __FUNCTION__);
            return;
        }
    }
};

} // namespace decision

} // namespace shop

#endif