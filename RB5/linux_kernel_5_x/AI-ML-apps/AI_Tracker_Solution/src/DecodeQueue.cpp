#include "DecodeQueue.h"

int DecodeQueue::Dequeue(shared_ptr<DetectionItem> &item, unsigned int timeOutMs)
{
    std::unique_lock<std::mutex> lock(mutex_);
    auto realTime = std::chrono::milliseconds(timeOutMs);

    while (queue_.empty() && !is_stoped_)
    {
        empty_cond_.wait_for(lock, realTime);
    }

    if (is_stoped_)
    {
        // stop
        return 1;
    }

    if (queue_.empty())
    {
        // empty
        return 2;
    }
    else
    {
        item = queue_.front();
        queue_.pop_front();
    }

    full_cond_.notify_one();

    return 0;
}

int DecodeQueue::Enqueue(const shared_ptr<DetectionItem> &item, bool isWait)
{
    std::unique_lock<std::mutex> lock(mutex_);

    while (queue_.size() >= max_size_ && isWait && !is_stoped_)
    {
        full_cond_.wait(lock);
    }

    if (is_stoped_)
    {
        return 1;
    }

    if (queue_.size() >= max_size_)
    {
        return 3;
    }
    queue_.push_back(item);
    empty_cond_.notify_one();
    return 0;
}

void DecodeQueue::Unlock()
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        is_stoped_ = true;
    }

    full_cond_.notify_all();
    empty_cond_.notify_all();
}

std::list<shared_ptr<DetectionItem>> DecodeQueue::GetRemainItems()
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (!is_stoped_)
    {
        return std::list<shared_ptr<DetectionItem>>();
    }

    return queue_;
}

int DecodeQueue::IsEmpty()
{
    return queue_.empty();
}