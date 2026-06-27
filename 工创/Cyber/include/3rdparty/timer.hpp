#ifndef _TIMER_H_
#define _TIMER_H_

#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>
#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#ifdef WIN32
int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

class Timer
{
public:
        Timer() : _expired(true), _try_to_expire(false)
        {
        }

        Timer(const Timer &timer)
        {
                _expired = timer._expired.load();
                _try_to_expire = timer._try_to_expire.load();
        }

        ~Timer()
        {
                stop();
        }

        void start(int interval, std::function<void()> task)
        {
                // is started, do not start again
                if (_expired == false)
                        return;

                // start async timer, launch thread and wait in that thread
                _expired = false;
                std::thread([this, interval, task]() {
                        while (!_try_to_expire)
                        {
                                int64_t pre = get_cur_stamp();
                                task();
                                int64_t cur = get_cur_stamp();
                                int64_t gap = (cur - pre) / 1000;
                                // sleep every interval and do the task again and again until times up
                                int rel_interval = interval - gap;
                                if (rel_interval > 0)
                                std::this_thread::sleep_for(std::chrono::milliseconds(rel_interval));
                                // task();
                        }

                        {
                                // timer be stopped, update the condition variable expired and wake main thread
                                std::lock_guard<std::mutex> locker(_mutex);
                                _expired = true;
                                _expired_cond.notify_one();
                        }
                }).detach();
        }

        void startOnce(int delay, std::function<void()> task)
        {
                std::thread([delay, task]() {
                        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
                        task();
                }).detach();
        }

        void stop()
        {
                // do not stop again
                if (_expired)
                        return;

                if (_try_to_expire)
                        return;

                // wait until timer
                _try_to_expire = true; // change this bool value to make timer while loop stop
                {
                        std::unique_lock<std::mutex> locker(_mutex);
                        _expired_cond.wait(locker, [this] { return _expired == true; });

                        // reset the timer
                        if (_expired == true)
                                _try_to_expire = false;
                }
        }

        int64_t get_cur_stamp()
        {
                struct timeval tv;
                gettimeofday(&tv, NULL);
                // return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
                return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
        }


private:
        std::atomic<bool> _expired;       // timer stopped status
        std::atomic<bool> _try_to_expire; // timer is in stop process
        std::mutex _mutex;
        std::condition_variable _expired_cond;
};

#endif // !_TIMER_H_
