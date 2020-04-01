#include <Ticker.h>
#include <stdint.h>
#include <functional>

void tickerFlagHandle(bool * flag);

typedef std::function<void(void)> tscallback_t;

struct TickerSchedulerItem
{
    Ticker t;
    bool flag = false;
    tscallback_t cb;
    bool is_used = false;
};

class TickerScheduler
{
private:
    uint size;
    TickerSchedulerItem *items = NULL;

    void handleTicker(tscallback_t, bool * flag);

public:
    TickerScheduler(uint size);
    ~TickerScheduler();
    
    boolean add(uint i, uint32_t period, tscallback_t, boolean shouldFireNow = false);

    boolean remove(uint i);

    void update();
};
