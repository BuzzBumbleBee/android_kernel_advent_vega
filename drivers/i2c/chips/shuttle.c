/*======================================================================
    A kernel module: usb_cam_switch.c                                              
    This module uses proc fs to let user turn on or of usb camera                         
    <antliao@gmail.com>. All Rights Reserved.                       
======================================================================*/

#include <linux/kernel.h>                    
#include <linux/module.h>                    
#include <linux/types.h>                     
#include <linux/fs.h>                     
#include <linux/errno.h>                     
#include <linux/mm.h>                     
#include <linux/sched.h>                     
#include <linux/init.h>                     
#include <linux/cdev.h>                     
#include <asm/io.h>                     
#include <asm/system.h>                     
#include <asm/uaccess.h> 
#include <linux/proc_fs.h>
#include <linux/gpio.h>

#include <linux/timer.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <linux/interrupt.h>

#define MOD_NAME	"[Shuttle-GPIO]"
#define SHUTTLE_DEBUG	0

#ifdef SHUTTLE_DEBUG
#define DEBUG( fmt, args... ) { \
        printk( MOD_NAME "%s():%d - ", __FUNCTION__, __LINE__); printk( fmt, ## args); \
	}
#else
#define DEBUG( fmt, args...) { \
        do {} while( 0 ); }
#endif

#define MAX_LEN		16
#define MAX_GPIO_PIN	223
#define PROC_ENT1	"gpio"
#define PROC_ENT2	"s3"
#define PROC_ENT3	"bat"
#define PROC_ENT4	"ec"
#define PROC_ENT5	"ota"
#define PROC_ENT6	"cf"
#define PROC_ENT7	"rild"
#define PROC_ENT8   "ite"
#define PROC_ENT9   "cal"

#define GPIO_GET	0
#define GPIO_SET_INPUT	1
#define GPIO_SET	2
#define GPIO_SET_OUTPUT	3
#define GPIO_SET_ALL	4

static struct proc_dir_entry *proc_entry1 ;
static struct proc_dir_entry *proc_entry2 ;
static struct proc_dir_entry *proc_entry3 ;
static struct proc_dir_entry *proc_entry4 ;
static struct proc_dir_entry *proc_entry5 ;
static struct proc_dir_entry *proc_entry6 ;
static struct proc_dir_entry *proc_entry7 ;
static struct proc_dir_entry *proc_entry8 ;
static struct proc_dir_entry *proc_entry9 ;
static int gpio = -1;
static int ota = 0 ;
static int cf = 1 ;   // default value is 1 to set 50HZ
static int rild = 4 ; // default value is 4 to stop rild
static int ite_tp = -1 ; // if 0, ite TP is not exist, if 1, exist
static int egal_tp = -1 ; // if 0, egal TP is not exist, if 1, exist
static int ite_i2c_nostop = 0 ; // if 0, not use no stop in i2c transfer

static int cal_on = 0;
extern void sendCalibrationCmd(void);
static struct workqueue_struct *tp_cal_wq;
static struct work_struct  work;

#define KEY_MU 139  // KEY_MENU

#define KEY_VU 115 // KEY_VOLUME_UP
#define KEY_VD 114 // KEY_VOLUME_DOWN
#define KEY_PW 116 // KEY_POWER
#define KEY_BK 158 // KEY_BACK
#define KEY_F4 62 // KEY_F4
#define SHUTTLE_D1 HZ/10*4  // 0.4 second
#define SHUTTLE_D2 HZ/10*2  // 0.2 second

#define SHUTTLE_D_TICK HZ/100*2  // 20ms
#define SHUTTLE_D_BK HZ/100*5  // 50ms
#define SHUTTLE_D_MU HZ  // 1000ms

//#define SHUTTLE_D3 HZ  // 1 second

static struct input_dev *button_dev ;
static struct timer_list shuttle_timer1 ;  // for volume key
static struct timer_list shuttle_timer2 ;  // for press power key 2 second
static int gpio_num[4] ;
static int menuShow = 0;
static int timerRunning = 0;
static int range = 0;

extern int nvec_battery_stop(void) ;
extern int nvec_battery_start(void) ;
extern int kernel_restart(char *cmd) ;

/*
static irqreturn_t menu_key(int irq, void *dummy)
{

	return IRQ_HANDLED ;
}
*/

// for TP start
int get_tp_status(int number)
{
	if(number == 0)
	{
		return(egal_tp) ;
	} else if(number == 1) 
	{
		return(ite_tp) ;
	} else {
		return(-1) ;
	}
}
void set_tp_status(int number, int status)
{
	if(number == 0)
	{
		egal_tp = status ;
	}
	if(number == 1)
	{
		ite_tp = status ;
	}
}
void set_ite_i2c_nostop(int toggle)
{
	ite_i2c_nostop = toggle ;
}
int get_ite_i2c_nostop(void)
{
	return(ite_i2c_nostop) ;
}
EXPORT_SYMBOL(get_tp_status);
EXPORT_SYMBOL(set_tp_status);
EXPORT_SYMBOL(set_ite_i2c_nostop);
EXPORT_SYMBOL(get_ite_i2c_nostop);
// for TP end

static void emit_keycode(int key_code)
{
	input_report_key(button_dev, key_code, 1) ;
	input_sync(button_dev) ;
	input_report_key(button_dev, key_code, 0) ;
	input_sync(button_dev) ;
}

/* 
 *  Read by user
 */
ssize_t
ucs_proc_read(char *page, char **start, off_t off, int count,
  int*eof, void *data)
{
	int len ;
	int i;
	int retval;

	// don't allow the user to read using offset
	if (off > 0)
	{
		*eof = 1;
		return 0;
	}

	// get one GPIO pin
	if( 0 <= gpio && gpio <= MAX_GPIO_PIN ) {

		retval = gpio_get_value( gpio );
		len = sprintf(page, "%d-%d", gpio, retval);
		gpio = -1;

		return len;
	}

	// get all GPIO pin
	len = sprintf(page, "  ");
	for( i=0; i<20;) {
		len += sprintf(page+len, "%d", (i++)%10);
		if( i % 5 == 0 )
			len += sprintf(page+len, " ");
	}
	len += sprintf(page+len, "\n0:");

	for( i=1; i<MAX_GPIO_PIN+1; ++i) {

		retval = gpio_get_value(i-1);
		len += sprintf(page+len, "%d", retval);
		if( i % 20 == 0 )
			len += sprintf(page+len, "\n%d:", ((int)(i/10))%10);
		else if( i % 5 == 0 )
			len += sprintf(page+len, " ");
	}

	return len;
}

/* 
 *  Write by the user
 */
ssize_t
ucs_proc_write(struct file *filp, const char __user *buff,
  unsigned long len, void *data)
{
	char k_buf[MAX_LEN] = "\0";
	int count = min(MAX_LEN, (int)len) ;
	int offset;

	int action;
	int val;

	int retval ;

	if(copy_from_user( k_buf, buff, count))
	{
		printk(KERN_ERR MOD_NAME "Some Data Copy failed!\n");
		goto ERROR;
	} else {

		// Protocol:
		//	1) get + NUMBER; 2) seti + NUMBER; 3) set + NUMBER + {h/l}
		//	4) seto + NUMBER + {h/l}; 5) seta + {h/l}
		DEBUG("====== [%s] ======\n", k_buf);

		if( strncmp( k_buf, "get", 3) == 0 ) {
			offset = 3;
			action = GPIO_GET;
			goto GPIO_NUM;
		}
		else if( strncmp( k_buf, "set", 3) == 0 )
			offset = 3;
		else
			goto ERROR;

		if( offset < count && '0'<=k_buf[offset] && k_buf[offset]<='9' )
			action = GPIO_SET;
		else {

			if( 'i' == k_buf[offset] )
				action = GPIO_SET_INPUT;
			else if( 'o' == k_buf[offset] )
				action = GPIO_SET_OUTPUT;
			else if( 'a' == k_buf[offset] )
				action = GPIO_SET_ALL;
			else
				goto ERROR;

			++offset;
		}

GPIO_NUM:
		gpio = 0;
		for(; offset < count && '0'<=k_buf[offset] && k_buf[offset]<='9'; ++offset)
			gpio = gpio*10 + k_buf[offset]-'0';

		if( gpio < 0 || gpio > MAX_GPIO_PIN )
			goto ERROR;

		val = 0;
		if( action > GPIO_SET_INPUT ) {
			if( offset < count ) {
				switch( k_buf[offset] ) {
					case 'h':
						val = 1;
						break;
					case 'l':
						val = 0;
						break;
					default:
						goto ERROR;
				}
			}
			else
				goto ERROR;
		}

		switch( action ) {
			// ex: get20 ==> get GPIO Pin 20
			case GPIO_GET:
				DEBUG("gpio_get_value(%d)\n",gpio);
				break;

			// ex: seti20 ==> set GPIO Pin 20 to be input
			case GPIO_SET_INPUT:
				DEBUG("gpio_direction_input(%d)\n",gpio);
				gpio_direction_input(gpio);
				break;

			// ex: set20h ==> set GPIO Pin 20 to be high
			case GPIO_SET:
				DEBUG("gpio_set_value(%d,%d)\n",gpio,val);
				gpio_set_value( gpio, val);
				break;

			// ex: seto20h ==> set GPIO Pin 20 to be output and high
			case GPIO_SET_OUTPUT:
				DEBUG("gpio_direction_output(%d,%d)\n",gpio,val);
				gpio_direction_output( gpio, val);
				break;

			// ex: setah ==> set all GPIO Pin to be high
			case GPIO_SET_ALL:
				DEBUG("(all)gpio_set_value(%d)\n",val);
				for( gpio=0; gpio<=MAX_GPIO_PIN; ++gpio)
					gpio_set_value( gpio, val);
				break;
			default:
				goto ERROR;
		}
		retval = count;
		goto END;
	}

ERROR:
	printk(KERN_ERR MOD_NAME "ERROR for GPIO access!\n");
	retval = - EFAULT;

END:
	return retval;
}

/* 
 *  Write by the user to enter S3 mode
 */
ssize_t
s3_proc_write(struct file *filp, const char __user *buff,
  unsigned long len, void *data)
{
	emit_keycode(KEY_F4) ;
	return 1 ;
}

/* 
 *  Read by user to return EC version
 */
char shuttle_ec_v[3] ;
ssize_t
ec_proc_read(char *page, char **start, off_t off, int count,
  int*eof, void *data)
{
	int len ;

	// don't allow the user to read using offset
	if (off > 0)
	{
		*eof = 1;
		return 0;
	}

	len = sprintf(page, "%x.%x.%x", shuttle_ec_v[0], shuttle_ec_v[1],
	  shuttle_ec_v[2]);
	return len;
}
void shuttle_set_ec_version(int order, char v)
{
	shuttle_ec_v[order] = v ;
}
EXPORT_SYMBOL(shuttle_set_ec_version);

/* 
 *  Read by demond, if return char is start, then ....
 */
ssize_t
ota_proc_read(char *page, char **start, off_t off, int count,
  int*eof, void *data)
{
	int len ;

	// don't allow the user to read using offset
	if (off > 0)
	{
		*eof = 1;
		return 0;
	}

	len = sprintf(page, "%d", ota);
	return len;
}

/* 
 *  Write by the user to low/high the gpio 8
 */
ssize_t
ota_proc_write(struct file *filp, const char __user *buff,
  unsigned long len, void *data)
{
	char k_buf[MAX_LEN] = "\0";
	int count = min(MAX_LEN, (int)len) ;
	int retval ;
	if(copy_from_user( k_buf, buff, count))
	{
		printk(KERN_ERR "Ant -- ota switch, copy from user failed!\n");
		return -1 ;
	} else {
		printk(KERN_ERR "Ant -- ota switch, user write %s\n", k_buf);
		retval = count ;
	}
	if(strncmp(k_buf, "start", 5) == 0)
	{
		ota = 1 ;
	}
	else if(strncmp(k_buf, "reboot", 6) == 0)
	{
		kernel_restart(NULL) ;
	} else {
		// do nothing
	}
	return retval ;
}

/* 
 *  Read by demond, if return 3 to start, return 4 to stop rild....
 */
ssize_t
rild_proc_read(char *page, char **start, off_t off, int count,
  int*eof, void *data)
{
	int len ;

	// don't allow the user to read using offset
	if (off > 0)
	{
		*eof = 1;
		return 0;
	}

	len = sprintf(page, "%d", rild);
	return len;
}

/* 
 *  Write by the user to stop/start rild
 */
ssize_t
rild_proc_write(struct file *filp, const char __user *buff,
  unsigned long len, void *data)
{
	char k_buf[MAX_LEN] = "\0";
	int count = min(MAX_LEN, (int)len) ;
	int retval ;
	if(copy_from_user( k_buf, buff, count))
	{
		printk(KERN_ERR "Ant -- rild stop/start, copy from user failed!\n");
		return -1 ;
	} else {
		printk(KERN_ERR "Ant -- rild stop/start, user write %s\n", k_buf);
		retval = count ;
	}
	if(strncmp(k_buf, "start", 5) == 0)
	{
		rild = 3 ; // tell the demond to start rild
	} else if (strncmp(k_buf, "stop", 4) ==0 )
	{
		rild = 4 ; // tell the demond to stop rild
	} else if (strncmp(k_buf, "done_start", 10) ==0 )
	{
		rild = 1 ; // start rild done
	} else if (strncmp(k_buf, "done_stop", 9) ==0 )
	{
		rild = 2 ; // stop rild done
	} else {
		// do nothing
	}
	
	return retval ;
}

/* 
 *  Read by user, return 1 if ite tp exist
 */
ssize_t
touch_pl_proc_read(char *page, char **start, off_t off, int count,
  int*eof, void *data)
{
	int len ;

	// don't allow the user to read using offset
	if (off > 0)
	{
		*eof = 1;
		return 0;
	}

	len = sprintf(page, "%d", ite_tp);

	return len;
}

/* 
 *  Write by the user to trigger touch panel calibration
 */
ssize_t
cal_proc_write(struct file *filp, const char __user *buff,
  unsigned long len, void *data)
{
	char k_buf[MAX_LEN] = "\0";
	int count = min(MAX_LEN, (int)len) ;
	int retval ;

	if(copy_from_user( k_buf, buff, count))
	{
		printk(KERN_ERR "-- calibration setting, copy from user failed!\n");
		return -1 ;
	} else {
		printk(KERN_ERR "-- calibration setting, user write %s\n", k_buf);
		retval = count ;
	}
	if(strncmp(k_buf, "0", 1) == 0)
   { 
       //enable calibration
       cal_on = 1;
       printk(KERN_INFO "enable calibration !!!\n") ;
	} else if (strncmp(k_buf, "1", 1) == 0 )
	{
		//finished calibration
       cal_on = 0;
       printk(KERN_INFO "finished calibration !!!\n") ;
	}
	
	return retval ;
}


/* 
 *  Read by demond, if return char is start, then ....
 */
ssize_t
cf_proc_read(char *page, char **start, off_t off, int count,
  int*eof, void *data)
{
	int len ;

	// don't allow the user to read using offset
	if (off > 0)
	{
		*eof = 1;
		return 0;
	}

	len = sprintf(page, "%d", cf);
	return len;
}

/* 
 *  Write by the user to change the frequency of camera
 */
ssize_t
cf_proc_write(struct file *filp, const char __user *buff,
  unsigned long len, void *data)
{
	char k_buf[MAX_LEN] = "\0";
	int count = min(MAX_LEN, (int)len) ;
	int retval ;
	if(copy_from_user( k_buf, buff, count))
	{
		printk(KERN_ERR "Ant -- camera frequency, copy from user failed!\n");
		return -1 ;
	} else {
		printk(KERN_ERR "Ant -- camera frequency, user write %s\n", k_buf);
		retval = count ;
	}
	if(strncmp(k_buf, "set50", 5) == 0)
	{
		cf = 3 ; // tell the demond to set 50 HZ
	} else if (strncmp(k_buf, "set60", 5) ==0 )
	{
		cf = 4 ; // tell the demond to set 50 HZ
	} else if (strncmp(k_buf, "done50", 6) ==0 )
	{
		cf = 1 ; // set 50 HZ is done
	} else if (strncmp(k_buf, "done60", 6) ==0 )
	{
		cf = 2 ; // set 60 HZ is done
	} else {
		// do nothing
	}
	
	return retval ;
}

/* 
 *  Write by the user to reboot
 */
//ssize_t
//reboot_proc_write(struct file *filp, const char __user *buff,
//  unsigned long len, void *data)
//{
//	char k_buf[MAX_LEN] = "\0";
//	int count = min(MAX_LEN, (int)len) ;
//	int retval ;
//	static int bat_toggle = 1 ;
//	if(copy_from_user( k_buf, buff, count))
//	{
//		printk(KERN_ERR "Ant -- reboot proc, copy from user failed!\n");
//		return -1 ;
//	} else {
//		printk(KERN_ERR "Ant -- reboot proc, user write %s\n", k_buf);
//		retval = count ;
//	}
//	kernel_restart(NULL) ;
//
//	return retval ;
//}

/* 
 *  Write by the user to turn on/off EC battery
 */
ssize_t
bat_proc_write(struct file *filp, const char __user *buff,
  unsigned long len, void *data)
{
	char k_buf[MAX_LEN] = "\0";
	int count = min(MAX_LEN, (int)len) ;
	int retval ;
	static int bat_toggle = 1 ;
	if(copy_from_user( k_buf, buff, count))
	{
		printk(KERN_ERR "Ant -- Battery switch, copy from user failed!\n");
		return -1 ;
	} else {
		printk(KERN_ERR "Ant -- Battery switch, user write %s\n", k_buf);
		retval = count ;
	}
	if(strncmp(k_buf, "stop", 4) == 0)
	{
		if(bat_toggle == 1)
		{
			printk(KERN_ERR "Ant -- stop nvec_battery\n") ;
			nvec_battery_stop() ;
			bat_toggle = 0 ;
		}
	} else {
		if(bat_toggle == 0)
		{
			printk(KERN_ERR "Ant -- start nvec_battery\n") ;
			nvec_battery_start() ;
			bat_toggle = 1 ;
		}
	}
	return retval ;
}

// GPIO_PAA7 -- low means not S3
//           -- high means S3
// according to .../kernel/arch/arm/mach-tegra/gpio-names.h
// The number of GPIO_PAA7 is 215
// GPIO 28 is to control USB camera
static void init_gpio_status(void)
{
	gpio_request(215, "Shuttle_S3flag") ;
	gpio_direction_output(215, 0) ;
	printk(KERN_INFO "Ant shuttle module init set GPIO_PAA7 to low\n") ;

	gpio_request(28, "Shuttle_USBCamera") ;
	gpio_direction_output( 28, 0);
	printk(KERN_INFO "Ant shuttle module init set GPIO 28 to low\n") ;

    //high is (host mode) --- TV_ON(GPIO_PB0 8)
	gpio_request(8, "USB0") ;
	gpio_direction_output( 8, 0);
	printk(KERN_INFO "Ant shuttle module init set GPIO 8 to high (host mode)\n") ;

}

// GPIO_PB1 -- Volup -- 9
// GPIO_PK7 -- Voldown -- 87 
// GPIO_PV2 -- Key_power -- 170
// GPIO_PH0 -- Key_back-- 56
static void init_shuttle_gpio_key(void)
{
	gpio_num[0] = 9 ;
	gpio_num[1] = 87 ;
	gpio_num[2] = 170;
	gpio_num[3] = 56 ;
}

static void init_input_dev(void)
{
	int error ;

	button_dev = input_allocate_device();
	if (!button_dev) {
		printk(KERN_ERR "Ant -- Can't allocate input device\n");
		error = -ENOMEM;
		return ;
	}

	set_bit(EV_KEY,button_dev->evbit) ;
	set_bit(KEY_VU,button_dev->keybit) ;
	set_bit(KEY_VD,button_dev->keybit) ;
	set_bit(KEY_PW,button_dev->keybit) ;
	set_bit(KEY_BK,button_dev->keybit) ;
	set_bit(KEY_F4,button_dev->keybit) ;

    set_bit(KEY_MU,button_dev->keybit) ;
	error = input_register_device(button_dev) ;
	if (error) {
		printk(KERN_ERR "Ant: Failed to register input device\n");
	}
}

static void timer2_handle(void)
{
	int v = -1 ;
	static int count = 0 ;
	static int prev_status = 0 ;

	gpio_request(gpio_num[2], "Shuttle_power") ;
	gpio_direction_input(gpio_num[2]) ;
	v = gpio_get_value(gpio_num[2]) ;

	if( v == 0 )
	{
		printk(KERN_INFO "Ant shuttle module -- PV2 is low\n") ;
		if(prev_status == 1)
		{
			count++ ;
			if(count == 10)
			{
				printk(KERN_INFO "Ant shuttle module -- emit KEY_POWER\n") ;
				emit_keycode(KEY_PW) ;
				count = 0 ;
				prev_status = 0 ;
			} 
		} else {
			prev_status = 1 ;
		}
	}  else {
		//printk(KERN_INFO "Ant shuttle module -- PV2 is high\n") ;
		prev_status = 0 ;
		count = 0 ;
	}
	mod_timer(&shuttle_timer2, jiffies + SHUTTLE_D2) ;
}



static int getGpioStatus(void)
{
    int currentState = 1;

	gpio_request(gpio_num[3], "Shuttle_back") ;
	gpio_direction_input(gpio_num[3]) ;
	currentState = gpio_get_value(gpio_num[3]) ;

    return currentState;
}



static void timer1_handle(void)
{
/*
	int i = 0 ;
	int v = -1 ;
	int key_code = 0 ;

	//	gpio_direction_input(gpio_num[i]) ;
	//	v = gpio_get_value(gpio_num[i]) ;
	//	if( v == 0 )
	//	{
	//		printk(KERN_INFO "Ant shuttle module -- set key_code to %d\n",
	//		  key_code) ;
	//		emit_keycode(key_code) ;
	//	}
	//}
	gpio_request(gpio_num[3], "Shuttle_back") ;
	gpio_direction_input(gpio_num[3]) ;
	v = gpio_get_value(gpio_num[3]) ;
	if( v == 0 )
	{
		key_code =  KEY_BK ;
		printk(KERN_INFO "Ant shuttle module -- set key_code to %d\n",
		  key_code) ;
		emit_keycode(key_code) ;
	}
   mod_timer(&shuttle_timer1, jiffies + SHUTTLE_D2) ;
*/
    if (0 == getGpioStatus())
    {
        
        range += SHUTTLE_D_TICK;
        // long press
        if ((range > SHUTTLE_D_MU) && (menuShow == 0) ) 
        {
            //send menu key
            emit_keycode(KEY_MU) ;
            printk(KERN_INFO "Ant shuttle module -- set key_code to %d\n", KEY_MENU);
            menuShow = 1; 
        }
        mod_timer(&shuttle_timer1, jiffies + SHUTTLE_D_TICK) ;
    }
	
}

static void init_shuttle_timer(void)
{
    init_timer(&shuttle_timer1) ;
    shuttle_timer1.function = &timer1_handle;
    shuttle_timer1.expires = jiffies - 1;
    add_timer(&shuttle_timer1) ;
    
	//init_timer(&shuttle_timer2) ;
	//shuttle_timer2.function = &timer2_handle ;
	//shuttle_timer2.expires = jiffies + SHUTTLE_D2 ;
	//add_timer(&shuttle_timer2) ;
}


static void start_shuttle_timer(void)
{
    if (!timerRunning)
    {
        printk(KERN_INFO "Ant shuttle module -- Start timer\n");
        mod_timer(&shuttle_timer1, jiffies + SHUTTLE_D_TICK) ;
    }
}


static void stop_shuttle_timer(void)
{
	mod_timer(&shuttle_timer1, jiffies - 1) ;
}  



static void init_shuttle_proc(void)
{
	// create gpio proc entry
	proc_entry1 = create_proc_entry(PROC_ENT1, 0666, NULL) ;
	if (proc_entry1 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT1) ;
	} else {
		proc_entry1->read_proc = ucs_proc_read ;
		proc_entry1->write_proc = ucs_proc_write ;
	//	proc_entry->owner = THIS_MODULE ;
	}

	// create s3 proc entry
	proc_entry2 = create_proc_entry(PROC_ENT2, 0666, NULL) ;
	if (proc_entry2 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT2) ;
	} else {
	//	proc_entry->read_proc = ucs_proc_read ;
		proc_entry2->write_proc = s3_proc_write ;
	//	proc_entry->owner = THIS_MODULE ;
	}

	// create batter switch proc entry
	proc_entry3 = create_proc_entry(PROC_ENT3, 0666, NULL) ;
	if (proc_entry3 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT3) ;
	} else {
		proc_entry3->write_proc = bat_proc_write ;
	}

	// create ec entry -- to display EC version
	proc_entry4 = create_proc_entry(PROC_ENT4, 0666, NULL) ;
	if (proc_entry4 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT4) ;
	} else {
		proc_entry4->read_proc = ec_proc_read ;
	}

	// create ota entry
	proc_entry5 = create_proc_entry(PROC_ENT5, 0666, NULL) ;
	if (proc_entry5 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT5) ;
	} else {
		proc_entry5->write_proc = ota_proc_write ;
		proc_entry5->read_proc = ota_proc_read ;
	}

	// create cf entry(adjust camera frequency)
	proc_entry6 = create_proc_entry(PROC_ENT6, 0666, NULL) ;
	if (proc_entry6 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT6) ;
	} else {
		proc_entry6->write_proc = cf_proc_write ;
		proc_entry6->read_proc = cf_proc_read ;
	}

	// create rild entry(stop/start rild)
	proc_entry7 = create_proc_entry(PROC_ENT7, 0666, NULL) ;
	if (proc_entry7 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT7) ;
	} else {
		proc_entry7->write_proc = rild_proc_write ;
		proc_entry7->read_proc = rild_proc_read ;
	}

	// create ite tp detect entry
	proc_entry8 = create_proc_entry(PROC_ENT8, 0666, NULL) ;
	if (proc_entry8 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT8) ;
	} else {
		proc_entry8->read_proc = touch_pl_proc_read ;
	}

	// create trigger capabration entry
	proc_entry9 = create_proc_entry(PROC_ENT9, 0666, NULL) ;
	if (proc_entry9 == NULL)
	{
		printk(KERN_ERR MOD_NAME "Shuttle module: Couldn't create proc entry: %s\n", PROC_ENT9) ;
	} else {
		proc_entry9->write_proc = cal_proc_write ;
	}

}


static void tp_cal_wq_func(struct work_struct *work)
{
    if (cal_on == 1)
    {
        cal_on = 0;
        sendCalibrationCmd();
    }
}


static irqreturn_t gpioKeyInput_interrupt_handler(int irq, void *dev_id)
{
    // filter 

    if (0 == getGpioStatus())
    {
        //printk(KERN_INFO "Ant shuttle module -- gpioKeyInput_interrupt_handler_[low]");
        start_shuttle_timer();
        timerRunning = 1;
    }
    else
    {
        stop_shuttle_timer();
        if ((range > SHUTTLE_D_BK) && ( range < SHUTTLE_D_MU))
        { 
           queue_work(tp_cal_wq, &work);

	        //send back key
	        emit_keycode(KEY_BK) ;
	        printk(KERN_INFO "Ant shuttle module -- set key_code to %d\n", KEY_BK);
        }

        range = 0;
        menuShow = 0;
        timerRunning = 0;
    }

    return IRQ_HANDLED;
}



static void init_interrupt(void)
{
 
    if (0 == request_irq( gpio_to_irq(gpio_num[3]),
                          gpioKeyInput_interrupt_handler,
                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                          "gpiokeyDevice",
                          NULL))
    {
        printk(KERN_INFO "Ant shuttle module -- Successed get irq handler\n");
    }

    tp_cal_wq = create_singlethread_workqueue("tp_cal_wq");
	if (!tp_cal_wq)
	{
       printk(KERN_INFO "tp cal workqueue init failed\n");
    }
    else
    {
	    INIT_WORK(&work, tp_cal_wq_func);
    }
}

static void uninit_interrupt(void)
{
    free_irq(gpio_to_irq(gpio_num[3]), NULL);
}

static int __init shuttle_gpio_init(void)
{

	init_shuttle_proc() ;
	init_input_dev() ;
	init_gpio_status() ;
	init_shuttle_gpio_key() ;
    init_shuttle_timer() ;

    init_interrupt() ;
	printk(KERN_INFO MOD_NAME "Module Init: Shuttle module\n");
	return 0;
}

static void __exit shuttle_gpio_exit(void)
{
	remove_proc_entry(PROC_ENT1, proc_entry1->parent) ; 
	remove_proc_entry(PROC_ENT2, proc_entry2->parent) ; 
	remove_proc_entry(PROC_ENT3, proc_entry3->parent) ; 
	remove_proc_entry(PROC_ENT4, proc_entry4->parent) ; 
	remove_proc_entry(PROC_ENT5, proc_entry5->parent) ; 
	remove_proc_entry(PROC_ENT6, proc_entry6->parent) ; 
	remove_proc_entry(PROC_ENT7, proc_entry7->parent) ; 
   remove_proc_entry(PROC_ENT8, proc_entry8->parent) ;
   remove_proc_entry(PROC_ENT9, proc_entry9->parent) ;
	printk(KERN_INFO MOD_NAME "Module Exit: Shuttle GPIO Switch\n");
	input_unregister_device(button_dev) ;
    
    uninit_interrupt();
}

module_init(shuttle_gpio_init);
module_exit(shuttle_gpio_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ant_Liao, antliao@gmail.com; Cheney_Ni, cheney_ni@tw.shuttle.com");
MODULE_VERSION("V0.8");

