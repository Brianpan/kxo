/* kxo: A Tic-Tac-Toe Game Engine implemented as Linux kernel module */

#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "game.h"
#include "mcts.h"
#include "negamax.h"

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("National Cheng Kung University, Taiwan");
MODULE_DESCRIPTION("In-kernel Tic-Tac-Toe game engine");

/* Macro DECLARE_TASKLET_OLD exists for compatibility.
 * See https://lwn.net/Articles/830964/
 */
#ifndef DECLARE_TASKLET_OLD
#define DECLARE_TASKLET_OLD(arg1, arg2) DECLARE_TASKLET(arg1, arg2, 0L)
#endif

#define DEV_NAME "kxo"

#define IOCTL_READ_SIZE 0
#define IOCTL_READ_LIST 1 

#define NR_KMLDRV 1

static int delay = 100; /* time (in ms) to generate an event */

/* Declare kernel module attribute for sysfs */

struct kxo_attr {
    char display;
    char resume;
    char end;
    int game_no;
    rwlock_t lock;
};

static struct kxo_attr attr_obj;

/* We use an additional "faster" circular buffer to quickly store data from
 * interrupt context, before adding them to the kfifo.
 */
static struct circ_buf fast_buf;

#define MAX_GAMES (PAGE_SIZE - sizeof(u64))/sizeof(u64)

// define each game moves as u64 since 4*4 board
// 4 bits for each cell * 16 = 64 bits 
static u64 game_moves;
static int cur_move;

/* Clear all data from the circular buffer fast_buf */
static void fast_buf_clear(void)
{
    fast_buf.head = fast_buf.tail = 0;
}

static void fast_buf_put(void)
{
    struct circ_buf *ring = &fast_buf;
    unsigned long head = ring->head;

    unsigned long tail = READ_ONCE(ring->tail);

    if (unlikely(!CIRC_SPACE(head, tail , PAGE_SIZE))) {
        pr_warn_ratelimited("%s: fast buffer full, will drop some data\n", __func__);
    }
    memcpy(&ring->buf[head], &game_moves, sizeof(u64));
    smp_wmb();

    ring->head = (ring->head + sizeof(u64)) & (PAGE_SIZE - 1);
}

static u64 fast_buf_get(void)
{
    struct circ_buf *ring = &fast_buf;
    unsigned long head = READ_ONCE(ring->head), tail = ring->tail;

    // nothing to read
    if (unlikely(!CIRC_CNT(head, tail, PAGE_SIZE)))
        return 0;
    
    smp_rmb();
    u64 move;
    memcpy(&move, (void *) &ring->buf[tail], sizeof(u64));

    smp_mb();

    ring->tail = (ring->tail + sizeof(u64)) & (PAGE_SIZE - 1);
    return move;
}

static ssize_t kxo_state_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf)
{
    read_lock(&attr_obj.lock);
    int ret = snprintf(buf, 6, "%c %c %c\n", attr_obj.display, attr_obj.resume,
                       attr_obj.end);
    // int game_no = attr_obj.game_no;
    // memcpy(buf + ret, &game_no, sizeof(int));
    // ret += sizeof(int);
    
    // // read the game moves
    // for (int i = 0; i < attr_obj.game_no; i++) {
    //     u64 game_move = fast_buf_get();
    //     if (game_move == 0)
    //         continue;
    //     memcpy(buf + ret, &game_move, sizeof(u64));
    //     ret += sizeof(u64);
    //     if (ret >= PAGE_SIZE)
    //         break;
    // }
    read_unlock(&attr_obj.lock);
    
    return ret;
}

static ssize_t kxo_state_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count)
{
    write_lock(&attr_obj.lock);
    sscanf(buf, "%c %c %c", &(attr_obj.display), &(attr_obj.resume),
           &(attr_obj.end));
    pr_info("kxo: display: %c, resume: %c, end: %c\n", attr_obj.display,
            attr_obj.resume, attr_obj.end);
    write_unlock(&attr_obj.lock);
    return count;
}

static DEVICE_ATTR_RW(kxo_state);

/* Data produced by the simulated device */

/* Timer to simulate a periodic IRQ */
static struct timer_list timer;

/* Character device stuff */
static int major;
static struct class *kxo_class;
static struct cdev kxo_cdev;

// change to sizeof(int)
static char draw_buffer[sizeof(int)];

/* Data are stored into a kfifo buffer before passing them to the userspace */
static DECLARE_KFIFO_PTR(rx_fifo, unsigned char);

/* NOTE: the usage of kfifo is safe (no need for extra locking), until there is
 * only one concurrent reader and one concurrent writer. Writes are serialized
 * from the interrupt context, readers are serialized using this mutex.
 */
static DEFINE_MUTEX(read_lock);

/* Wait queue to implement blocking I/O from userspace */
static DECLARE_WAIT_QUEUE_HEAD(rx_wait);

/* Insert the whole chess board into the kfifo buffer */
static void produce_board(void)
{
    unsigned int len = kfifo_in(&rx_fifo, draw_buffer, sizeof(draw_buffer));
    if (unlikely(len < sizeof(draw_buffer)))
        pr_warn_ratelimited("%s: %zu bytes dropped\n", __func__,
                            sizeof(draw_buffer) - len);

    pr_debug("kxo: %s: in %u/%u bytes\n", __func__, len, kfifo_len(&rx_fifo));
}

/* Mutex to serialize kfifo writers within the workqueue handler */
static DEFINE_MUTEX(producer_lock);

/* Mutex to serialize fast_buf consumers: we can use a mutex because consumers
 * run in workqueue handler (kernel thread context).
 */
static DEFINE_MUTEX(consumer_lock);

static char table[N_GRIDS];

/* Draw the board into draw_buffer */
static int draw_board(char *table)
{
    int hash = table_to_hash(table);
    memcpy(draw_buffer, &hash, sizeof(int));
    smp_wmb();
    return 0;
}

/* Workqueue handler: executed by a kernel thread */
static void drawboard_work_func(struct work_struct *w)
{
    int cpu;

    /* This code runs from a kernel thread, so softirqs and hard-irqs must
     * be enabled.
     */
    WARN_ON_ONCE(in_softirq());
    WARN_ON_ONCE(in_interrupt());

    /* Pretend to simulate access to per-CPU data, disabling preemption
     * during the pr_info().
     */
    cpu = get_cpu();
    pr_info("kxo: [CPU#%d] %s\n", cpu, __func__);
    put_cpu();

    read_lock(&attr_obj.lock);
    if (attr_obj.display == '0') {
        read_unlock(&attr_obj.lock);
        return;
    }
    read_unlock(&attr_obj.lock);

    mutex_lock(&producer_lock);
    draw_board(table);
    mutex_unlock(&producer_lock);

    /* Store data to the kfifo buffer */
    mutex_lock(&consumer_lock);
    produce_board();
    mutex_unlock(&consumer_lock);

    wake_up_interruptible(&rx_wait);
}

static char turn;
static int finish;

static void write_move(int move)
{
    // pr_info("kxo: %s: move: %d, cur_move: %d\n", __func__, move, cur_move);
    // 4 bits for each cell
    game_moves |= (((u64) move & 0x0F) << (cur_move << 2));
    cur_move += 1;
}

static void pack_move_number(void)
{
    if (cur_move > 15)
        return;
    // pr_info("kxo: %s: cur_move: %d, %lld\n", __func__, cur_move, game_moves);
    // total moves pack to left most 4 bits
    game_moves |= (((u64) cur_move & 0x0F) << 60);
}

static void ai_one_work_func(struct work_struct *w)
{
    ktime_t tv_start, tv_end;
    s64 nsecs;

    int cpu;

    WARN_ON_ONCE(in_softirq());
    WARN_ON_ONCE(in_interrupt());

    cpu = get_cpu();
    pr_info("kxo: [CPU#%d] start doing %s\n", cpu, __func__);
    tv_start = ktime_get();
    mutex_lock(&producer_lock);
    int move;
    WRITE_ONCE(move, mcts(table, 'O'));

    smp_mb();

    if (move != -1)
        WRITE_ONCE(table[move], 'O');
    
    write_move(move);
    WRITE_ONCE(turn, 'X');
    WRITE_ONCE(finish, 1);
    smp_wmb();
    mutex_unlock(&producer_lock);
    tv_end = ktime_get();

    nsecs = (s64) ktime_to_ns(ktime_sub(tv_end, tv_start));
    pr_info("kxo: [CPU#%d] %s completed in %llu usec\n", cpu, __func__,
            (unsigned long long) nsecs >> 10);
    put_cpu();
}

static void ai_two_work_func(struct work_struct *w)
{
    ktime_t tv_start, tv_end;
    s64 nsecs;

    int cpu;

    WARN_ON_ONCE(in_softirq());
    WARN_ON_ONCE(in_interrupt());

    cpu = get_cpu();
    pr_info("kxo: [CPU#%d] start doing %s\n", cpu, __func__);
    tv_start = ktime_get();
    mutex_lock(&producer_lock);
    int move;
    WRITE_ONCE(move, negamax_predict(table, 'X').move);

    smp_mb();

    if (move != -1)
        WRITE_ONCE(table[move], 'X');

    write_move(move);
    WRITE_ONCE(turn, 'O');
    WRITE_ONCE(finish, 1);
    smp_wmb();
    mutex_unlock(&producer_lock);
    tv_end = ktime_get();

    nsecs = (s64) ktime_to_ns(ktime_sub(tv_end, tv_start));
    pr_info("kxo: [CPU#%d] %s completed in %llu usec\n", cpu, __func__,
            (unsigned long long) nsecs >> 10);
    put_cpu();
}

/* Workqueue for asynchronous bottom-half processing */
static struct workqueue_struct *kxo_workqueue;

/* Work item: holds a pointer to the function that is going to be executed
 * asynchronously.
 */
static DECLARE_WORK(drawboard_work, drawboard_work_func);
static DECLARE_WORK(ai_one_work, ai_one_work_func);
static DECLARE_WORK(ai_two_work, ai_two_work_func);

/* Tasklet handler.
 *
 * NOTE: different tasklets can run concurrently on different processors, but
 * two of the same type of tasklet cannot run simultaneously. Moreover, a
 * tasklet always runs on the same CPU that schedules it.
 */
static void game_tasklet_func(unsigned long __data)
{
    ktime_t tv_start, tv_end;
    s64 nsecs;

    WARN_ON_ONCE(!in_interrupt());
    WARN_ON_ONCE(!in_softirq());

    tv_start = ktime_get();

    READ_ONCE(finish);
    READ_ONCE(turn);
    smp_rmb();

    if (finish && turn == 'O') {
        WRITE_ONCE(finish, 0);
        smp_wmb();
        queue_work(kxo_workqueue, &ai_one_work);
    } else if (finish && turn == 'X') {
        WRITE_ONCE(finish, 0);
        smp_wmb();
        queue_work(kxo_workqueue, &ai_two_work);
    }
    queue_work(kxo_workqueue, &drawboard_work);
    tv_end = ktime_get();

    nsecs = (s64) ktime_to_ns(ktime_sub(tv_end, tv_start));

    pr_info("kxo: [CPU#%d] %s in_softirq: %llu usec\n", smp_processor_id(),
            __func__, (unsigned long long) nsecs >> 10);
}

/* Tasklet for asynchronous bottom-half processing in softirq context */
static DECLARE_TASKLET_OLD(game_tasklet, game_tasklet_func);

static void ai_game(void)
{
    WARN_ON_ONCE(!irqs_disabled());

    pr_info("kxo: [CPU#%d] doing AI game\n", smp_processor_id());
    pr_info("kxo: [CPU#%d] scheduling tasklet\n", smp_processor_id());
    tasklet_schedule(&game_tasklet);
}

static void timer_handler(struct timer_list *__timer)
{
    ktime_t tv_start, tv_end;
    s64 nsecs;

    pr_info("kxo: [CPU#%d] enter %s\n", smp_processor_id(), __func__);
    /* We are using a kernel timer to simulate a hard-irq, so we must expect
     * to be in softirq context here.
     */
    WARN_ON_ONCE(!in_softirq());

    /* Disable interrupts for this CPU to simulate real interrupt context */
    local_irq_disable();

    tv_start = ktime_get();

    char win = check_win(table);

    if (win == ' ') {
        ai_game();
        mod_timer(&timer, jiffies + msecs_to_jiffies(delay));
    } else {
        mutex_lock(&producer_lock);
        // number of moves in the left 4 most bits
        pack_move_number();
        fast_buf_put();
        smp_wmb();
        game_moves = 0;
        cur_move = 0;
        mutex_unlock(&producer_lock);

        // reset the game moves
        write_lock(&attr_obj.lock);
        if (attr_obj.game_no < MAX_GAMES)
            attr_obj.game_no++;
        write_unlock(&attr_obj.lock);

        read_lock(&attr_obj.lock);
        if (attr_obj.display == '1') {
            int cpu = get_cpu();
            pr_info("kxo: [CPU#%d] Drawing final board\n", cpu);
            put_cpu();

            mutex_lock(&producer_lock);
            draw_board(table);
            mutex_unlock(&producer_lock);

            /* Store data to the kfifo buffer */
            mutex_lock(&consumer_lock);
            produce_board();
            mutex_unlock(&consumer_lock);

            wake_up_interruptible(&rx_wait);
        }

        if (attr_obj.end == '0') {
            memset(table, ' ',
                   N_GRIDS); /* Reset the table so the game restart */
            mod_timer(&timer, jiffies + msecs_to_jiffies(delay));
        }

        read_unlock(&attr_obj.lock);

        pr_info("kxo: %c win!!!\n", win);
    }
    tv_end = ktime_get();

    nsecs = (s64) ktime_to_ns(ktime_sub(tv_end, tv_start));

    pr_info("kxo: [CPU#%d] %s in_irq: %llu usec\n", smp_processor_id(),
            __func__, (unsigned long long) nsecs >> 10);

    local_irq_enable();
}

static ssize_t kxo_read(struct file *file,
                        char __user *buf,
                        size_t count,
                        loff_t *ppos)
{
    unsigned int read;
    int ret;

    pr_debug("kxo: %s(%p, %zd, %lld)\n", __func__, buf, count, *ppos);

    if (unlikely(!access_ok(buf, count)))
        return -EFAULT;

    if (mutex_lock_interruptible(&read_lock))
        return -ERESTARTSYS;

    do {
        ret = kfifo_to_user(&rx_fifo, buf, count, &read);
        if (unlikely(ret < 0))
            break;
        if (read)
            break;
        if (file->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            break;
        }
        ret = wait_event_interruptible(rx_wait, kfifo_len(&rx_fifo));
    } while (ret == 0);
    pr_debug("kxo: %s: out %u/%u bytes\n", __func__, read, kfifo_len(&rx_fifo));

    mutex_unlock(&read_lock);

    return ret ? ret : read;
}

static atomic_t open_cnt;

static int kxo_open(struct inode *inode, struct file *filp)
{
    pr_debug("kxo: %s\n", __func__);
    if (atomic_inc_return(&open_cnt) == 1)
        mod_timer(&timer, jiffies + msecs_to_jiffies(delay));
    pr_info("openm current cnt: %d\n", atomic_read(&open_cnt));

    return 0;
}

static int kxo_release(struct inode *inode, struct file *filp)
{
    pr_debug("kxo: %s\n", __func__);
    if (atomic_dec_and_test(&open_cnt)) {
        del_timer_sync(&timer);
        flush_workqueue(kxo_workqueue);
        fast_buf_clear();
    }
    pr_info("release, current cnt: %d\n", atomic_read(&open_cnt));
    attr_obj.end = 48;

    return 0;
}

// ioctl interface
static long kxo_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret;
    switch (cmd & 1) {
    case IOCTL_READ_SIZE:
        int game_no;
        write_lock(&attr_obj.lock);
        game_no = attr_obj.game_no;
        attr_obj.game_no = 0;
        write_unlock(&attr_obj.lock);
        return game_no;
        break;
    case IOCTL_READ_LIST:
        u64 game_move = fast_buf_get();
        ret = copy_to_user((void*) arg, &game_move, sizeof(u64));
        break;
    default:
        ret = -ENOTTY;
    }
    return ret;
}

static const struct file_operations kxo_fops = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    .owner = THIS_MODULE,
#endif
    .read = kxo_read,
    .llseek = no_llseek,
    .open = kxo_open,
    .release = kxo_release,
    .unlocked_ioctl = kxo_ioctl,
};

static int __init kxo_init(void)
{
    dev_t dev_id;
    int ret;

    if (kfifo_alloc(&rx_fifo, PAGE_SIZE, GFP_KERNEL) < 0)
        return -ENOMEM;

    /* Register major/minor numbers */
    ret = alloc_chrdev_region(&dev_id, 0, NR_KMLDRV, DEV_NAME);
    if (ret)
        goto error_alloc;
    major = MAJOR(dev_id);

    /* Add the character device to the system */
    cdev_init(&kxo_cdev, &kxo_fops);
    ret = cdev_add(&kxo_cdev, dev_id, NR_KMLDRV);
    if (ret) {
        kobject_put(&kxo_cdev.kobj);
        goto error_region;
    }

    /* Create a class structure */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    kxo_class = class_create(THIS_MODULE, DEV_NAME);
#else
    kxo_class = class_create(DEV_NAME);
#endif
    if (IS_ERR(kxo_class)) {
        printk(KERN_ERR "error creating kxo class\n");
        ret = PTR_ERR(kxo_class);
        goto error_cdev;
    }

    /* Register the device with sysfs */
    struct device *kxo_dev =
        device_create(kxo_class, NULL, MKDEV(major, 0), NULL, DEV_NAME);

    ret = device_create_file(kxo_dev, &dev_attr_kxo_state);
    if (ret < 0) {
        printk(KERN_ERR "failed to create sysfs file kxo_state\n");
        goto error_device;
    }

    /* Allocate fast circular buffer */
    fast_buf.buf = vmalloc(PAGE_SIZE);
    if (!fast_buf.buf) {
        ret = -ENOMEM;
        goto error_vmalloc;
    }

    /* Create the workqueue */
    kxo_workqueue = alloc_workqueue("kxod", WQ_UNBOUND, WQ_MAX_ACTIVE);
    if (!kxo_workqueue) {
        ret = -ENOMEM;
        goto error_workqueue;
    }

    negamax_init();
    mcts_init();
    memset(table, ' ', N_GRIDS);
    turn = 'O';
    finish = 1;
    game_moves = 0;
    cur_move = 0;

    attr_obj.display = '1';
    attr_obj.resume = '1';
    attr_obj.end = '0';
    rwlock_init(&attr_obj.lock);
    /* Setup the timer */
    timer_setup(&timer, timer_handler, 0);
    atomic_set(&open_cnt, 0);

    pr_info("kxo: registered new kxo device: %d,%d\n", major, 0);
out:
    return ret;
error_workqueue:
    vfree(fast_buf.buf);
error_vmalloc:
    device_destroy(kxo_class, dev_id);
error_device:
    class_destroy(kxo_class);
error_cdev:
    cdev_del(&kxo_cdev);
error_region:
    unregister_chrdev_region(dev_id, NR_KMLDRV);
error_alloc:
    kfifo_free(&rx_fifo);
    goto out;
}

static void __exit kxo_exit(void)
{
    dev_t dev_id = MKDEV(major, 0);

    del_timer_sync(&timer);
    tasklet_kill(&game_tasklet);
    flush_workqueue(kxo_workqueue);
    destroy_workqueue(kxo_workqueue);
    vfree(fast_buf.buf);
    device_destroy(kxo_class, dev_id);
    class_destroy(kxo_class);
    cdev_del(&kxo_cdev);
    unregister_chrdev_region(dev_id, NR_KMLDRV);

    kfifo_free(&rx_fifo);
    pr_info("kxo: unloaded\n");
}

module_init(kxo_init);
module_exit(kxo_exit);
