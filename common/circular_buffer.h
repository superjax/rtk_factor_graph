#include <cstddef>
#include <cstdint>
#include <iterator>

namespace mc {

template <typename T, int N = 100>
class CircularBuffer
{
 public:
    static constexpr int SIZE = N;

    struct iterator;
    struct const_iterator;

    CircularBuffer() = default;

    ~CircularBuffer() { clear(); }
    const_iterator begin() const { return head_; }
    const_iterator end() const { return tail_; }
    iterator begin() { return head_; }
    iterator end() { return tail_; }
    T& front() { return *head_; }
    const T& front() const { return *head_; }
    T& back() { return *(tail_ - 1); }
    const T& back() const { return *(tail_ - 1); }

    T& operator[](int i) { return *(head_ + i); }
    const T& operator[](int i) const { return *(head_ + i); }
    T& at(int i) { return *(head_ + i); }
    const T& at(int i) const { return *(head_ + i); }

    bool empty() const { return head_ == tail_; }
    bool full() const { return tail_ + 1 == head_; }

    size_t max_size() const { return SIZE - 1; }
    size_t capacity() const { return SIZE - 1; }
    size_t size() const { return tail_ - head_; }

    void clear()
    {
        while (!empty())
        {
            head_->~T();
            head_++;
        }
    }
    void pop_front()
    {
        head_->~T();  // call destructor manually without freeing memory
        head_++;
    }
    void pop_back()
    {
        tail_->~T();  // call destructor manually without freeing memory
        tail_--;
    }

    template <typename... Args>
    void emplace_back(Args&&... args)
    {
        new (&*tail_) T(args...);
        ++tail_;
    }

    void push_back(const T& obj)
    {
        *tail_ = obj;
        ++tail_;
    }

    template <typename... Args>
    void emplace_front(Args&&... args)
    {
        --head_;
        new (&*head_) T(args...);
    }

    void push_front(const T& obj)
    {
        --head_;
        *head_ = obj;
    }

    T* data() { return buf_; }
    const T* data() const { return buf_; }

 private:
    uint8_t memory_[SIZE * sizeof(T)];
    T* buf_ = reinterpret_cast<T*>(memory_);
    iterator end_ = {buf_ + SIZE, this};
    iterator head_ = {buf_, this};
    iterator tail_ = {buf_, this};

 public:
    struct const_iterator
    {
        typedef ptrdiff_t difference_type;
        typedef T value_type;
        typedef const T& reference_type;
        typedef const T* pointer;
        typedef std::random_access_iterator_tag iterator_category;

        const_iterator(T* _val, const CircularBuffer* _buf)
            : val(_val), buf(const_cast<CircularBuffer*>(_buf))
        {
        }
        const_iterator(const const_iterator& other) : val(other.val), buf(other.buf) {}
        ~const_iterator() = default;

        const_iterator& operator=(const const_iterator& other)
        {
            val = other.val;
            buf = other.buf;
            return *this;
        }

        const_iterator operator++(int)
        {
            const_iterator tmp = *this;
            operator++();
            return tmp;
        }
        const_iterator& operator++()
        {
            if (++val == buf->end_.val)
            {
                val = buf->data();
            }
            return *this;
        }

        const_iterator operator--(int)
        {
            const_iterator tmp = *this;
            operator--();
            return tmp;
        }
        const_iterator& operator--()
        {
            if (val-- == buf->data())
            {
                val = buf->end_.val - 1;
            }
            return *this;
        }

        const_iterator& operator+=(size_t i)
        {
            const size_t dest = (static_cast<size_t>(val - buf->buf_) + i) % SIZE;
            val = buf->buf_ + dest;
            return *this;
        }

        const_iterator operator+(size_t d) const
        {
            const size_t dest = (static_cast<size_t>(val - buf->data()) + d) % SIZE;
            return const_iterator(buf->data() + dest, buf);
        }

        const_iterator& operator-=(size_t i)
        {
            i = i % SIZE;
            const size_t dest = (static_cast<int>(val - buf->data()) + SIZE - i) % SIZE;
            val = buf->data() + dest;
            return *this;
        }
        const_iterator operator-(size_t i) const
        {
            i = i % SIZE;
            const size_t dest = (static_cast<int>(val - buf->data()) + SIZE - i) % SIZE;
            return const_iterator(buf->data() + dest, buf);
        }

        friend int operator-(const const_iterator& l, const const_iterator& r)
        {
            int lidx = static_cast<int>(l.idx());
            int ridx = static_cast<int>(r.idx());
            return lidx - ridx;
        }

        size_t idx() const
        {
            if (val >= buf->head_.val)
            {
                return val - buf->head_.val;
            }
            else
            {
                return SIZE - (buf->head_.val - val);
            }
        }

        const T& operator*() const { return *val; }

        const T* operator->() const { return val; }

        friend bool operator==(const const_iterator& l, const const_iterator& r)
        {
            return l.val == r.val;
        }
        friend bool operator!=(const const_iterator& l, const const_iterator& r)
        {
            return l.val != r.val;
        }
        friend bool operator<(const const_iterator& l, const const_iterator& r)
        {
            return l.idx() < r.idx();
        }
        friend bool operator>(const const_iterator& l, const const_iterator& r)
        {
            return l.idx() > r.idx();
        }
        friend bool operator<=(const const_iterator& l, const const_iterator& r)
        {
            return l.idx() <= r.idx();
        }
        friend bool operator>=(const const_iterator& l, const const_iterator& r)
        {
            return l.idx() >= r.idx();
        }
        friend void swap(const_iterator& lhs, const_iterator& rhs)
        {
            std::swap(*(lhs->val), *(rhs.val));
        }

     protected:
        T* val;  // these are non-const so that we can inherit with the non-const iterator
        CircularBuffer* buf;
    };

    struct iterator : public const_iterator
    {
        T& operator*() const { return *(this->val); }
        T* operator->() const { return this->val; }

        iterator(T* _val, CircularBuffer* _buf) : const_iterator(_val, _buf) {}
        iterator(const const_iterator& other) : const_iterator(other.val, other.buf) {}

        iterator operator+(size_t d) const
        {
            const size_t dest = (static_cast<size_t>(this->val - this->buf->data()) + d) % SIZE;
            return iterator(this->buf->data() + dest, this->buf);
        }

        iterator operator-(size_t i) const
        {
            i = i % SIZE;
            const size_t dest = (static_cast<int>(this->val - this->buf->data()) + SIZE - i) % SIZE;
            return iterator(this->buf->data() + dest, this->buf);
        }
    };
};

}  // namespace mc
