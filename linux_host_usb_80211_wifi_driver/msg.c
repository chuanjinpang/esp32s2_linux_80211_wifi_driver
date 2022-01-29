#include <linux/kernel.h>

#include <linux/spinlock.h>
#include <linux/slab.h>

#include "log.h"

#include "msg.h"


#define  ops_sync_lock(lock,flag) spin_lock_irqsave(lock, flag)
#define  ops_sync_unlock(lock,flag) spin_unlock_irqrestore(lock, flag)



static void  msg_queue_del_all_msg(msg_queue_t * mgr)
{
    msg_item_t * node, *n;
    unsigned long flag;
    int i = 0;
    ops_sync_lock(&mgr->lock, flag);
 
    list_for_each_entry_safe(node, n, &mgr->msg_tb, list)
    {
        list_del(&(node->list));
        atomic_dec(&mgr->atm_cnt);     
        kfree(node);
    }    
    ops_sync_unlock(&mgr->lock, flag);

}



int  msg_queue_put_msg(msg_queue_t * mgr,int id,uint8_t * msg, size_t len)
{
  
    msg_item_t * node;
    unsigned long flag;

    node = kmalloc(sizeof(msg_item_t), GFP_ATOMIC );//call form irq
    if(!node)
    {
        LOGE("%d kmalloc failure\n");
        return -1;
    }
    memset(node, 0, sizeof(msg_item_t));
    INIT_LIST_HEAD(&(node->list));
	memcpy(node->data,msg,len);
	node->len=len;
	node->id=id;
    ops_sync_lock(&mgr->lock, flag);
    list_add(&node->list, &mgr->msg_tb);
    ops_sync_unlock(&mgr->lock, flag);
    atomic_inc(&mgr->atm_cnt);

    return 0;
}



int  msg_queue_get_msg(msg_queue_t * mgr,int *id,uint8_t * msg, size_t len)
{
    msg_item_t * node, *n;
	int ret=-1;
    unsigned long flag;
    ops_sync_lock(&mgr->lock, flag);
    list_for_each_entry_safe(node, n, &mgr->msg_tb, list)
    {
    
            list_del(&(node->list));
            atomic_dec(&mgr->atm_cnt);
   			memcpy(msg,node->data,node->len);
			if(id)
				*id=node->id;
			ret=node->len;
            kfree(node);
			break;
    }
    ops_sync_unlock(&mgr->lock, flag);

return ret;
}


int msg_queue_init(msg_queue_t * mgr)
{
    int ret = 0;
    int i = 0;
   
    if(MGR_INIT_MAGIC_NUM  != mgr->init_magic)
    {

        INIT_LIST_HEAD(&mgr->msg_tb);  
        spin_lock_init(&mgr->lock);
        mgr->init_magic = MGR_INIT_MAGIC_NUM;
        atomic_set(&mgr->atm_cnt, 0);
       
    }
    return ret;
}



int msg_queue_uninit(msg_queue_t * mgr)
{
  
    if(MGR_INIT_MAGIC_NUM  == mgr->init_magic)
    {
        msg_queue_del_all_msg(mgr);
        LOGW("%s list atm:%d\n", __FUNCTION__, atomic_read(&mgr->atm_cnt));       
        mgr->init_magic = 0;
    }
    return 0;
}

