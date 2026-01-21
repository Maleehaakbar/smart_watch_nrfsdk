#include <zephyr/kernel.h>
#ifdef CONFIG_ENABLE_TEST
#include "test.h"
#endif
#define SLEEP_TIME_MS 100U

int main(void)
{
       
        
        while(1)
        {
                printk("helloworld\n");
                k_msleep(SLEEP_TIME_MS);
                
                #ifdef CONFIG_ENABLE_TEST
                int sum = CONFIG_NUM_A + CONFIG_NUM_B;
                printk("sum is %d", sum);
                k_msleep(SLEEP_TIME_MS);
                #endif
        }
        
        return 0;

}
