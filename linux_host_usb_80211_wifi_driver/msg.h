
#ifndef __MSG_MGR__H_
#define __MSG_MGR__H_

#include <linux/kernel.h>


#define MGR_INIT_MAGIC_NUM 0x55AA9527

typedef struct
{
    spinlock_t lock;
    struct list_head msg_tb;
    atomic_t atm_cnt;
    int init_magic;

}msg_queue_t;

#define MSG_ITEM_DATA_SIZE 2048

typedef struct
{
int id;
uint8_t data[MSG_ITEM_DATA_SIZE];
size_t len;
struct list_head list;

} msg_item_t;

int msg_queue_init(msg_queue_t * mgr);
int msg_queue_uninit(msg_queue_t * mgr);
int  msg_queue_get_msg(msg_queue_t * mgr,int *id,uint8_t * msg, size_t len);
int msg_queue_put_msg(msg_queue_t * mgr,int id,uint8_t * msg, size_t len);




#endif

