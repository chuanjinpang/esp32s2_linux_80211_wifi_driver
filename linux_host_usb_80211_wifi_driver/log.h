#ifndef _DEBUG_LOG_H_
#define _DEBUG_LOG_H_

//#define DEBUG_MODE 1
#ifdef DEBUG_MODE

#define LOGW(args...) do {printk(args);} while(0)
#define LOGE(args...) do {printk(args);} while(0)

#else

#define LOGW(args...)   do {} while(0)
#define LOGE(args...)  do {printk(args);} while(0)

#endif


#endif

