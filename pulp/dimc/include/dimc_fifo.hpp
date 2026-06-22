#ifndef __DIMC_FIFO_HPP__
#define __DIMC_FIFO_HPP__

#include <cstddef>
#include <vector>

// Simple bounded FIFO used between streamers and macros.
// Depth is configurable at construction (runtime).

template <typename T>
class Dimc_Fifo {
    public:
        Dimc_Fifo() : depth(0), head(0), tail(0), count(0) {}

        explicit Dimc_Fifo(std::size_t depth) { set_depth(depth); }

        void set_depth(std::size_t d) {
            this->depth = d;
            this->data.assign(d, T());
            this->head  = 0;
            this->tail  = 0;
            this->count = 0;
        }

        std::size_t get_depth() const { return depth; }

        bool push(const T &v) {
            if (count >= depth) return false;
            data[tail] = v;
            tail = (tail + 1) % depth;
            count++;
            return true;
        }

        bool pop(T &v) {
            if (count == 0) return false;
            v = data[head];
            head = (head + 1) % depth;
            count--;
            return true;
        }

        bool empty() const { return count == 0; }
        bool full()  const { return count >= depth; }
        std::size_t size() const { return count; }

        void reset() { head = tail = count = 0; }

    private:
        std::size_t depth;
        std::vector<T> data;
        std::size_t head;
        std::size_t tail;
        std::size_t count;
};

#endif
