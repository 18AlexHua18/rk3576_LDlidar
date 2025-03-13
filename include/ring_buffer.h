#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>

template <typename T>
class RingBuffer {
public:
    RingBuffer(size_t size)
    : buffer_(size), head_(0), tail_(0), count_(0), exit_flag_(false) {}
    
    bool push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (count_ == buffer_.size()) {
            return false;  // 缓冲区满
        }
        
        buffer_[tail_] = item;
        tail_ = (tail_ + 1) % buffer_.size();
        ++count_;
        
        lock.unlock();
        cond_.notify_one();
        return true;
    }
    
    bool pop(T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        while (count_ == 0) {
            if (exit_flag_) {
                return false;  // 退出标志设置，不再等待
            }
            cond_.wait(lock);
        }
        
        item = buffer_[head_];
        head_ = (head_ + 1) % buffer_.size();
        --count_;
        
        return true;
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }
    
    void setExit(bool flag) {
        exit_flag_ = flag;
        cond_.notify_all();  // 唤醒所有等待线程
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_ == 0;
    }
    
    bool full() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_ == buffer_.size();
    }

    // 返回缓冲区总容量
    size_t capacity() const {
        return buffer_.size();
    }

private:
    std::vector<T> buffer_;
    size_t head_;
    size_t tail_;
    size_t count_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    std::atomic<bool> exit_flag_;
};