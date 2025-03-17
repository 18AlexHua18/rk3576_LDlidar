#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>

template <typename T>
class RingBuffer {
public:
    RingBuffer(size_t capacity) : 
        buffer_(capacity), 
        head_(0), 
        tail_(0), 
        count_(0), 
        capacity_(capacity),
        exit_(false) {}
    
    bool push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (count_ == capacity_) {
            // 缓冲区满
            return false;
        }
        
        buffer_[tail_] = item;
        tail_ = (tail_ + 1) % capacity_;
        ++count_;
        
        // 通知等待的消费者有新数据可用
        cond_.notify_one();
        return true;
    }
    
    bool pop(T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        while (count_ == 0) {
            // 如果设置了退出标志且缓冲区为空，直接返回失败
            if (exit_) {
                return false;
            }
            
            // 等待生产者添加数据
            cond_.wait(lock);
            
            // 再次检查退出标志
            if (exit_ && count_ == 0) {
                return false;
            }
        }
        
        item = buffer_[head_];
        head_ = (head_ + 1) % capacity_;
        --count_;
        return true;
    }
    
    void setExit(bool exit) {
        exit_ = exit;
        // 通知所有等待的线程
        cond_.notify_all();
    }
    
    size_t size() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return count_;
    }
    
    bool empty() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return count_ == 0;
    }
    
    bool full() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return count_ == capacity_;
    }
    
private:
    std::vector<T> buffer_;
    size_t head_;
    size_t tail_;
    size_t count_;
    size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    std::atomic<bool> exit_;
};