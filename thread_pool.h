#include<bits/stdc++.h>
#include<thread>
#include<mutex>
using namespace std;

struct Fixed {
    constexpr Fixed(int v): v(v << 16) {}
    constexpr Fixed(float f): v(f * (1 << 16)) {}
    constexpr Fixed(double f): v(f * (1 << 16)) {}
    constexpr Fixed(): v(0) {}

    static constexpr Fixed from_raw(int32_t x) {
        Fixed ret;
        ret.v = x;
        return ret;
    } 

    int32_t v;

    bool operator<(const Fixed & other) const {
        return v < other.v;
    }
    bool operator>(const Fixed & other) const {
        return v > other.v;
    }

    bool operator<=(const Fixed & other) const {
        return v <= other.v;
    }
    bool operator>=(const Fixed & other) const {
        return v >= other.v;
    }
    bool operator==(const Fixed & other) const {
        return v == other.v;
    }

    bool operator!=(const Fixed & other) const {
        return v != other.v;
    }

    auto operator<=>(const Fixed&) const = default;
};

Fixed operator+(Fixed a, Fixed b) {
    return Fixed::from_raw(a.v + b.v);
}

Fixed operator-(Fixed a, Fixed b) {
    return Fixed::from_raw(a.v - b.v);
}

Fixed operator*(Fixed a, Fixed b) {
    return Fixed::from_raw(((int64_t) a.v * b.v) >> 16);
}

Fixed operator/(Fixed a, Fixed b) {
    return Fixed::from_raw(((int64_t) a.v << 16) / b.v);
}

Fixed &operator+=(Fixed &a, Fixed b) {
    return a = a + b;
}

Fixed &operator-=(Fixed &a, Fixed b) {
    return a = a - b;
}

Fixed &operator*=(Fixed &a, Fixed b) {
    return a = a * b;
}

Fixed &operator/=(Fixed &a, Fixed b) {
    return a = a / b;
}

Fixed operator-(Fixed x) {
    return Fixed::from_raw(-x.v);
}



ostream &operator<<(ostream &out, Fixed x) {
    return out << x.v / (double) (1 << 16);
}


class ThreadPool {
public:

ThreadPool(unsigned int init_num) {
    //cnt - количество потоков больше или равно 1 и меньше или равно
    // максимальному количеству потоков на машине
    cnt = min(init_num, std::thread::hardware_concurrency());
    cnt = max(cnt, (unsigned int)1);
    for (int i = 0; i < cnt; i++) {
        // создаем потоки в которых будет исполняться функцияя класса execute
        all_threads.emplace_back(&ThreadPool::execute, this);
    }
}

void execute() {
    // Пока ThreadPool не хотят удалить выполняется
    while (!is_end) {
        // Блокируем mutex_1 чтобы забрать одну из задач из дека, 
        // если они там есть, если нет то ждем пока что задача все же не
        // появится или не вызовется деструктор
        unique_lock<mutex> get_task_lock(mutex_1);
        cond_1.wait(get_task_lock, [this]() -> bool {
            return is_end || deque_for_tasks.size() != 0;
        });

        // если задача есть то будем ее исполнять
        if (deque_for_tasks.size() != 0) {
            // заберем номер выполненной задачи и саму задачу и освобождаем мьютекс
            unsigned long long num = deque_for_tasks.back().second;
            auto task = move(deque_for_tasks.back().first);
            deque_for_tasks.pop_back();
            get_task_lock.unlock();

            // Выполняем задачу
            task.get();
            
            // забираем второй мьютекс чтобы положить в set завершенных номеров задач
            // номер только что выполненной задачи данным потоком
            unique_lock<mutex> put_in_set(mutex_2);
            complited_tasks.push_back(num);
            // уведомляем о том что задачу положили -> wait
            cond_2.notify_all();
        }
    }

}

template <typename Func, typename ... Type_arguments>
void put(const Func func, Type_arguments ... args) {

    // Захватываем первй мьютекс, чтобы положить ноаую задачу в дек
    unique_lock<mutex> put_task(mutex_1);

    deque_for_tasks.emplace_back(async(std::launch::deferred, func, args ...), ++num);

    // Уведомляем один поток, что задача появилась
    cond_1.notify_one();
}



void wait() {

    
    unique_lock<mutex> deque_bloock(mutex_1);

    cond_2.wait(deque_bloock, [this]() -> bool {
        // Захватываем мьютекс чтобы нельзя было изменить complited_tasks
        // При этом продолжим крутиться так как завершены не все задачи
        unique_lock<mutex> end_insert(mutex_2);
        return deque_for_tasks.size() == 0 || complited_tasks.size() != num;});
    
}

~ThreadPool() {
    // Ждем завершения всех задач
    wait();
    // флаг делаем true чтобы в execute потоки могли выйти из ожидания
    is_end = true;
    // будим всех
    cond_1.notify_all();
    for (int i = 0; i < cnt; i++) {
        // соединяем в один
        all_threads[i].join();
    }
}


private:
unsigned int cnt;
mutex mutex_1, mutex_2;
condition_variable cond_1, cond_2;

vector<thread> all_threads;
deque<pair<future<void>, unsigned long long>> deque_for_tasks;

vector<unsigned long long> complited_tasks;
atomic<bool> is_end = false;
atomic<unsigned long long> num = 0;

};