/* drivers/input/keyboard/IT7260_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include "IT7260_ts.h"

extern void set_ite_i2c_nostop(int toggle) ;
extern int get_ite_i2c_nostop(void) ;
extern void set_tp_status(int number, int status) ;
extern int get_tp_status(int number) ;	

#define ENABLE_IME_IMPROVEMENT
#define IT7260_I2C_NAME "IT7260"
//Ant start
#include <linux/gpio.h>
extern void set_ite_i2c_nostop(int toggle) ;
extern int get_ite_i2c_nostop(void) ;
extern void set_tp_status(int number, int status) ;
extern int get_tp_status(int number) ;
//Ant end
//#define ABS_MT_AMPLITUDE 0x3a
//#define ABS_MT_POSITION 0x39
static int ite7260_major = 0; // dynamic major by default
static int ite7260_minor = 0;
static struct cdev ite7260_cdev;
static struct class *ite7260_class = NULL;
static dev_t ite7260_dev;
static struct input_dev *input_dev;
static struct timer_list timer, timer;
static int xres = IT7260_X_RESOLUTION;
static int yres = IT7260_Y_RESOLUTION;
static int xscreen = SCREEN_X_RESOLUTION;
static int yscreen = SCREEN_Y_RESOLUTION;

#ifdef DEBUG
	#define TS_DEBUG(fmt,args...)  printk( KERN_DEBUG "[it7260_i2c]: " fmt, ## args)
	#define DBG() printk("[%s]:%d => \n",__FUNCTION__,__LINE__)
#else
	#define TS_DEBUG(fmt,args...)
	#define DBG()
#endif
//#define JIM
//#ifdef JIM
//prt(fmt,args...)    pr_info("=====" fmt, ## args)
//#else
//prt(fmt,args...)
//#endif
//Ant end

static struct workqueue_struct *IT7260_wq;

struct IT7260_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	bool has_relative_report;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	int snap_state[2][2];
	int snap_down_on[2];
	int snap_down_off[2];
	int snap_up_on[2];
	int snap_up_off[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	int reported_finger_count;
	int8_t sensitivity_adjust;
	uint32_t dup_threshold;
	int (*power)(int on);
	struct early_suspend early_suspend;
	int display_width;	/* display width in pixel */
	int display_height;	/* display height in pixel */
	int ts_raw_pos[4];	/* raw data pos of left, right, top, and bottom */
	int ts_raw_width;
	int ts_raw_height;
#ifdef ENABLE_IME_IMPROVEMENT
	int ime_threshold_pixel;	/* threshold in pixel */
	int ime_threshold[2];		/* threshold X & Y in raw data */
	int ime_area_pixel[4];		/* ime area in pixel */
	int ime_area_pos[4];		/* ime area in raw data */
#endif
	int margin_inactive_pixel[4];		/* margin area in pixel */
	int margin_inactive_raw[4];		/* margin area in raw data */
	uint8_t disable_margin_filter[2];	/* local margin filter. enable/disable time by time */
	uint8_t last_finger2_pressed;
	uint8_t last_finger_result;
	uint8_t key_pressed[2];
	uint8_t debug_log_level;
};
#ifdef CONFIG_HAS_EARLYSUSPEND
static void IT7260_ts_early_suspend(struct early_suspend *h);
static void IT7260_ts_late_resume(struct early_suspend *h);
#endif


static struct IT7260_ts_data *gl_ts;
static struct kobject *android_touch_kobj;

static ssize_t debug_level_show(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct IT7260_ts_data *ts = gl_ts;

	return sprintf(buf, "%d\n", ts->debug_log_level);
}

static ssize_t debug_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct IT7260_ts_data *ts = gl_ts;

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts->debug_log_level = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(debug_level, 0644, debug_level_show, debug_level_store);

////////////////////////////////gl_ts  add dhb for Calibration
static int CalibrationCapSensor(struct IT7260_ts_data *ts)
{
	unsigned char ucQuery;
	unsigned char pucCmd[80];
	int ret = 0;
	int test_read_count=0;
	pr_info("=entry CalibrationCapSensor=\n");
	pr_info("=entry CalibrationCapSensor---name[%s]---addr[%x]-flags[%d]=\n",ts->client->name,ts->client->addr,ts->client->flags);
	set_ite_i2c_nostop(1);
	ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
	pr_info("=CalibrationCapSensor read 0x80 1=%d=\n",ucQuery);
	if(ucQuery<0)
	{
		msleep(250);
		ucQuery = 0xFF;
	}
	test_read_count=0;
	//while(ucQuery & 0x01)
	while((ucQuery & 0x01)&&(test_read_count<0x80000))
	{
		test_read_count++;
		set_ite_i2c_nostop(1);
		ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
		if(ucQuery<0)
		{
			ucQuery = 0xFF;
		}
	}
	pr_info("=CalibrationCapSensor write cmd=\n");
	memset(pucCmd,0x00,20);
	pucCmd[0] = 0x13;
	pucCmd[1] = 0x00;
	////0x13 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 
	pr_info("=CalibrationCapSensor write cmd--test=\n");
	set_ite_i2c_nostop(0);
	ret = i2c_smbus_write_i2c_block_data(ts->client, 0x20,12, pucCmd);
	pr_info("=CalibrationCapSensor write cmd--test end=\n");
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
		return ret;
	}
	///msleep(100);
	set_ite_i2c_nostop(1);
	ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
	if(ucQuery<0)
	{
		ucQuery = 0xFF;
	}
	test_read_count=0;
	pr_info("=CalibrationCapSensor read 0x80 2=%d=\n",ucQuery);
	while((ucQuery & 0x01)&&(test_read_count<0x80000))
	//while((ucQuery & 0x01))
	{
		test_read_count++;
		set_ite_i2c_nostop(1);
		ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
		if(ucQuery<0)
		{
			ucQuery = 0xFF;
		}
	}
	pr_info("=CalibrationCapSensor read 0x80 3=%d=\n",ucQuery);
	memset(pucCmd,0x00,20);
	set_ite_i2c_nostop(1);
	ret=i2c_smbus_read_i2c_block_data(ts->client,
							  0xA0,
							  1,
							  pucCmd);
	pr_info("=CalibrationCapSensor read id-1-[%d][%x]=\n",ret,pucCmd[0]);

	pr_info("=CalibrationCapSensor write end=\n");
	
}

int dhb_test_qdsp5_ver_error;
int dhb_test_qdsp5_ver_error1;

ssize_t IT7260_calibration_show_temp(char *buf)
{
    printk(KERN_INFO "IT7260_calibration_show\n");
	return sprintf(buf, "[%d][%d]\n", dhb_test_qdsp5_ver_error,dhb_test_qdsp5_ver_error1);
}
ssize_t IT7260_calibration_store_temp(char *buf)
{
    printk(KERN_INFO "IT7260_calibration_store\n");
	CalibrationCapSensor(gl_ts);
	return 0;
}


static ssize_t IT7260_calibration_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "%s():\n", __func__);
	return IT7260_calibration_show_temp(buf);
}

static ssize_t IT7260_calibration_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	printk(KERN_DEBUG "%s():\n", __func__);

	IT7260_calibration_store_temp(buf);

	return count;
}
static DEVICE_ATTR(calibration, 0666, IT7260_calibration_show, IT7260_calibration_store);

///////////////////////////////   add dhb for Calibration end

#ifdef ENABLE_IME_IMPROVEMENT
static ssize_t ime_threshold_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct IT7260_ts_data *ts = gl_ts;

	return sprintf(buf, "%d\n", ts->ime_threshold_pixel);
}

static ssize_t ime_threshold_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct IT7260_ts_data *ts = gl_ts;
	char *ptr_data = (char *)buf;
	unsigned long val;

	val = simple_strtoul(ptr_data, NULL, 10);

	if (val >= 0 && val <= max(ts->display_width, ts->display_height))
		ts->ime_threshold_pixel = val;
	else
		ts->ime_threshold_pixel = 0;

	ts->ime_threshold[0] = ts->ime_threshold_pixel * ts->ts_raw_width / ts->display_width;
	ts->ime_threshold[1] = ts->ime_threshold_pixel * ts->ts_raw_height / ts->display_height;

	return count;
}

static ssize_t ime_work_area_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct IT7260_ts_data *ts = gl_ts;

	return sprintf(buf, "%d,%d,%d,%d\n", ts->ime_area_pixel[0],
			ts->ime_area_pixel[1], ts->ime_area_pixel[2], ts->ime_area_pixel[3]);
}

static ssize_t ime_work_area_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct IT7260_ts_data *ts = gl_ts;
	char *ptr_data = (char *)buf;
	char *p;
	int pt_count = 0;
	unsigned long val[4];

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
			break;

		if (pt_count >= 4)
			break;

		val[pt_count] = simple_strtoul(p, NULL, 10);

		pt_count++;
	}

	if (pt_count >= 4 && ts->display_width && ts->display_height) {
		ts->ime_area_pixel[0] = val[0]; /* Left */
		ts->ime_area_pixel[1] = val[1]; /* Right */
		ts->ime_area_pixel[2] = val[2]; /* Top */
		ts->ime_area_pixel[3] = val[3]; /* Bottom */

		if (val[0] < 0 || val[0] > ts->display_width)
			ts->ime_area_pos[0] = 0;
		else
			ts->ime_area_pos[0] = val[0] * ts->max[0] / ts->display_width;

		if (val[1] < 0 || val[1] > ts->display_width)
			ts->ime_area_pos[1] = ts->max[0];
		else
			ts->ime_area_pos[1] = val[1] * ts->max[0] / ts->display_width;

		if (val[2] < 0 || val[2] > ts->display_height)
			ts->ime_area_pos[2] = 0;
		else
			ts->ime_area_pos[2] = val[2] * ts->max[1] / ts->display_height;

		if (val[3] < 0 || val[3] > ts->display_height)
			ts->ime_area_pos[3] = ts->max[1];
		else
			ts->ime_area_pos[3] = val[3] * ts->max[1] / ts->display_height;
	}

	return count;
}

static int ime_report_filter(struct IT7260_ts_data *ts, int pos[2][2], const int finger2_pressed, const int z)
{
	int dx = 0;
	int dy = 0;
	static int report_x;
	static int report_y;

	if (finger2_pressed)
		return 1;

	if ((pos[0][0] >= ts->ime_area_pos[0] && pos[0][0] <= ts->ime_area_pos[1]) &&
		(pos[0][1] >= ts->ime_area_pos[2] && pos[0][1] <= ts->ime_area_pos[3])) {
		dx = abs(pos[0][0] - report_x);
		dy = abs(pos[0][1] - report_y);

		if (dx < ts->ime_threshold[0] && dy < ts->ime_threshold[1] && z != 0) {
			return 1;
		}

		report_x = pos[0][0];
		report_y = pos[0][1];

		if (z == 0) {
			report_x = report_y = 0;
		}
	}

	return 0;
}

/* sys/class/input/inputX/ime_threshold */
static DEVICE_ATTR(ime_threshold, 0666, ime_threshold_show,
		ime_threshold_store);
static DEVICE_ATTR(ime_work_area, 0666, ime_work_area_show,
		ime_work_area_store);
#endif

static ssize_t margin_area_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct IT7260_ts_data *ts = gl_ts;

	return sprintf(buf, "#left,right,top,bottom\n"
			"%d,%d,%d,%d\n", ts->margin_inactive_pixel[0],
			ts->margin_inactive_pixel[1],
			ts->margin_inactive_pixel[2],
			ts->margin_inactive_pixel[3]);
}

static int IdentifyCapSensor(struct IT7260_ts_data *ts);
static int IdentifyCapSensor2(struct IT7260_ts_data *ts);

static void Read_Point(struct IT7260_ts_data *ts)
{
	unsigned char ucQuery = 0;
	unsigned char pucPoint[14];
#ifdef HAS_8_BYTES_LIMIT
	unsigned char cPoint[8];
	unsigned char ePoint[6];
#endif //HAS_8_BYTES_LIMIT
	int ret = 0;
	int finger2_pressed=0;
	int xraw, yraw, xtmp, ytmp;
	int i = 0 ;

	set_ite_i2c_nostop(1);
	ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
	// Ant start
	//pr_info("x=Read_Point read 0x80 =%x=\n",ucQuery);
	// Ant end
	if(ucQuery<0)
	{
	    //pr_info("=error Read_Point=\n");
		if (ts->use_irq)
		   enable_irq(ts->client->irq);
		return ;
	}
	else
	{
		if(ucQuery & 0x80)
		{
#ifdef HAS_8_BYTES_LIMIT
			set_ite_i2c_nostop(1);
			ret=i2c_smbus_read_i2c_block_data(ts->client,
							  0xC0,
							  8,
							  cPoint);
			ret=i2c_smbus_read_i2c_block_data(ts->client,
							  0xE0,
							  6,
							  ePoint);
			for(i=0; i<6; i++){
				pucPoint[i] = ePoint[i] ;
			}
			for(i=0; i<8; i++){
				pucPoint[i+6] = cPoint[i] ;
			}
#else //HAS_8_BYTES_LIMIT
			set_ite_i2c_nostop(1);
			ret=i2c_smbus_read_i2c_block_data(ts->client,
							  0xE0,
							  14,
							  pucPoint);
#endif //HAS_8_BYTES_LIMIT
			//pr_info("=Read_Point read ret[%d]--point[%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x]=\n",
			//	ret,pucPoint[0],pucPoint[1],pucPoint[2],
			//	pucPoint[3],pucPoint[4],pucPoint[5],pucPoint[6],pucPoint[7],pucPoint[8],
			//	pucPoint[9],pucPoint[10],pucPoint[11],pucPoint[12],pucPoint[13]);
			if(ret)
			{
				// gesture
				if(pucPoint[0] & 0xF0)
				{
				    if (ts->use_irq)
		               enable_irq(ts->client->irq);
					//pr_info("pucPoint 0 is 0xF0, it's a gesture\n") ;
					//pr_info("pucPoint[0]=%x\n", pucPoint[0]);
					return ;
				} 			
				// palm
				if(pucPoint[1] & 0x01)
				{
				    if (ts->use_irq)
	                   enable_irq(ts->client->irq);
					//pr_info("pucPoint 1 is 0x01, it's a palm\n") ;
					//continue ;
					return ;
				}
				// no more data
				if(!(pucPoint[0] & 0x08))
				{
			        input_report_key(ts->input_dev, BTN_TOUCH, 0);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_mt_sync(ts->input_dev);
					input_sync(ts->input_dev);
				    if (ts->use_irq)
		                enable_irq(ts->client->irq);
					//pr_info("pucPoint 0 is 0x08, no more data\n") ;
					set_ite_i2c_nostop(1);
					ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
					//pr_info("Read_Point read 0x80 =%x=\n",ucQuery);
					return;
				}
				// 3 fingers
				if(pucPoint[0] & 0x04) 
				{
			    	if (ts->use_irq)
	               		enable_irq(ts->client->irq);
					//pr_info("pucPoint 0 is 0x04, we don't support three fingers\n") ;
					return ;
				}

				if(pucPoint[0] & 0x01)
				{
					char pressure_point,z,w;

					xraw = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
					yraw = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];

					pressure_point = pucPoint[5] & 0x0f;
					//pr_info("=Read_Point1 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
					
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 1);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pressure_point);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, xraw);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, yraw);
					input_mt_sync(ts->input_dev);
					//pr_info("=input Read_Point1 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
				}

				if(pucPoint[0] & 0x02)
				{
                    char pressure_point,z,w;
					xraw = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];
					yraw = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];

					pressure_point=pucPoint[9]&0x0f;

					//pr_info("=Read_Point2 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 2);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, xraw);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, yraw);
					input_mt_sync(ts->input_dev);
					//pr_info("input Read_Point2 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
				}
				input_sync(ts->input_dev);

				if((pucPoint[0] & 0x07)==0)
				{
					input_report_key(ts->input_dev, BTN_TOUCH, 0);
					//input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
					input_mt_sync(ts->input_dev);
					input_sync(ts->input_dev);
					//pr_info("pucPoint 0 & 0x07 is false\n") ;
				}
			}
		}
	}
	if (ts->use_irq)
		enable_irq(ts->client->irq);
	//pr_info("=end Read_Point=\n");

//IdentifyCapSensor2(gl_ts);
}

///////////////////////////////////////////////////////////////////////////////////////

static void IT7260_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[15];
	//printk(KERN_INFO "=IT7260_ts_work_func=\n"); 
	struct IT7260_ts_data *ts = container_of(work, struct IT7260_ts_data, work);
	gl_ts = ts;
	int buf_len = ts->has_relative_report ? 15 : 13;
	Read_Point(ts);
}

static enum hrtimer_restart IT7260_ts_timer_func(struct hrtimer *timer)
{
	struct IT7260_ts_data *ts = container_of(timer, struct IT7260_ts_data, timer);
	// printk("=IT7260_ts_timer_func=\n"); 

	queue_work(IT7260_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 1250), HRTIMER_MODE_REL);//12500000
	return HRTIMER_NORESTART;
}

static irqreturn_t IT7260_ts_irq_handler(int irq, void *dev_id)
{
	struct IT7260_ts_data *ts = dev_id;

	//pr_info("=IT7260_ts_irq_handler=\n");
	disable_irq_nosync(ts->client->irq);
	queue_work(IT7260_wq, &ts->work);
	return IRQ_HANDLED;
}
/////////////////////////////////////////////////////////
void sendCalibrationCmd(void)
{
    int ret = 0;
    struct IT7260_ts_data *ts = gl_ts;
    unsigned char data[] = {0x20, 0x13, 0x00, 0x00};
  
    //CalibrationCapSensor(ts);
    set_ite_i2c_nostop(0);
    ret = i2c_master_send( ts->client ,data ,4 );
    
    printk(KERN_INFO "IT7260 sent calibration command [%d]!!!\n", ret);
}

EXPORT_SYMBOL(sendCalibrationCmd);


static int IdentifyCapSensor(struct IT7260_ts_data *ts)
{
	unsigned char ucQuery;
	unsigned char pucCmd[80];
	int ret = 0;
	int test_read_count=0;
	//pr_info("=entry IdentifyCapSensor=\n");
	//pr_info("=entry IdentifyCapSensor---name[%s]---addr[%x]-flags[%d]=\n",ts->client->name,ts->client->addr,ts->client->flags);
	set_ite_i2c_nostop(1);
    ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
	//pr_info("=IdentifyCapSensor read 0x80 =%d=\n",ucQuery);
	if(ucQuery<0)
	{
		msleep(250);
		ucQuery = 0xFF;
	}
	set_ite_i2c_nostop(1);
	while(ucQuery & 0x01)
	{
		ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
		if(ucQuery<0)
		{
			ucQuery = 0xFF;
		}
	}
    //pr_info("=IdentifyCapSensor write cmd=\n");
	pucCmd[0] = 0x00;
	set_ite_i2c_nostop(0);
	ret = i2c_smbus_write_byte_data(ts->client, 0x20, pucCmd[0]);
	//ret = i2c_smbus_write_i2c_block_data(ts->client, 0x20, 1, pucCmd);//Not work!
	//ret = i2c_smbus_write_block_data(ts->client, 0x20, 1, pucCmd);//Not work!
	//pucCmd[0] = 0x20;
	//pucCmd[1] = 0x00;
	//ret = i2c_master_send(ts->client, pucCmd, 2);//Work
	
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
		return ret;
	}

	set_ite_i2c_nostop(1);
	ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
	if(ucQuery<0)
	{
		ucQuery = 0xFF;
	}
	test_read_count=0;
	set_ite_i2c_nostop(1);
	while((ucQuery & 0x01)&&(test_read_count<0x2000))
	{
		test_read_count++;
		ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
		if(ucQuery<0)
		{
			ucQuery = 0xFF;
		}
	}
	//pr_info("=IdentifyCapSensor write read id=\n");
	set_ite_i2c_nostop(1);
    ret=i2c_smbus_read_i2c_block_data(ts->client,
							  0xA0,
							  //10,
							  8,
							  pucCmd);
	//pr_info("=IdentifyCapSensor read id--[%d][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x]=\n",ret,pucCmd[0],pucCmd[1],
	//	                                               pucCmd[2],pucCmd[3],pucCmd[4],pucCmd[5],
	//	                                               pucCmd[6],pucCmd[7],pucCmd[8],pucCmd[9]);
	
}


static int IT7260_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct IT7260_ts_data *ts;
	struct i2c_msg msg[2];
	int ret = 0;
	struct IT7260_i2c_platform_data *pdata;
	unsigned long irqflags;
	unsigned char ucQuery = 0 ;
	char tmp[] = "test" ; // Ant
	irqflags=IRQF_TRIGGER_HIGH;
	pr_info("=entry IT7260_ts_probe=\n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "IT7260_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_check_functionality_failed;
	}
	ts->client = client;
	
    ts->debug_log_level=0x3;
	
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	//ret=IdentifyCapSensor(ts);
	//if(ret<0)
	//	goto err_power_failed;
 
	// Ant start -- to identify if this device is exist
	ret = i2c_master_send(client, tmp, 4);
	mdelay(1000) ;
	if (get_tp_status(1) == 0)
	{
	    printk(KERN_ERR "The ite TP device is not exist\n");
	    ret = -ENODEV;
		goto err_power_failed;
	} else {
	    set_tp_status(1, 1) ;
	}
	// Ant end



	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "IT7260_ts_probe: Failed to allocate input device\n");
		goto err_check_functionality_failed;
	}
	ts->input_dev->name = "IT7260-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	//set_bit(BTN_2, ts->input_dev->keybit);

	//Ant start -- support two fingers
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 1024, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 600, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 4, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_X, 0, 1024, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, 600, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 1, 0, 0);
	//Ant end

	ret = input_register_device(ts->input_dev);
		if (ret) {
			printk(KERN_ERR "IT7260_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
			goto err_check_functionality_failed;
		}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
		ts->early_suspend.suspend = IT7260_ts_early_suspend;
		ts->early_suspend.resume = IT7260_ts_late_resume;
		register_early_suspend(&ts->early_suspend);
#endif

	IT7260_wq = create_singlethread_workqueue("IT7260_wq");
	if (!IT7260_wq)
		goto err_check_functionality_failed;
	INIT_WORK(&ts->work, IT7260_ts_work_func);

	pr_info("Ant IT7260_ts_probe-client->irq[%d]=\n",client->irq);
	if (client->irq) {
		ret = request_irq(client->irq, IT7260_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);
		pr_info("Ant IT7260_ts_probe-request_irq[%d]=\n",ret);
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	if (!ts->use_irq) {
		pr_info("Ant if not use irq for IT7260, start hrtimer_init=\n");
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = IT7260_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

    //////////////////////////////////////////////////////add dhb for Calibration
    gl_ts=ts;
    ret = device_create_file(&ts->input_dev->dev, &dev_attr_calibration);
	if (ret) {
		printk(KERN_ERR "IT7260_ts_probe: Error to create calibration attribute\n");
		///goto error_free_device;
	}
	///CalibrationCapSensor(gl_ts);
    /////////////////////////////////////////////////////
    
	pr_info("=end IT7260_ts_probe=\n");


	return 0;
err_power_failed:
	kfree(ts);

err_check_functionality_failed:
	return ret;

}

static int IT7260_ts_remove(struct i2c_client *client)
{
	return 0;
}

static int IT7260_ts_suspend(struct i2c_client *client, pm_message_t mesg)
	{
		int ret;
		struct IT7260_ts_data *ts = i2c_get_clientdata(client);
	    pr_info("=entry IT7260_ts_suspend=\n");
		if (ts->use_irq)
			disable_irq(client->irq);
		else
			hrtimer_cancel(&ts->timer);
		ret = cancel_work_sync(&ts->work);
		if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
			enable_irq(client->irq);
		#if 0
		set_ite_i2c_nostop(0);
		ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0); /* disable interrupt */
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");
	
		ts->key_pressed[0] = 0;
	
		set_ite_i2c_nostop(0);
		ret = i2c_smbus_write_byte_data(client, 0xf0, 0x86); /* deep sleep */
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");
		#endif
		if (ts->power) {
			ret = ts->power(0);
			if (ret < 0)
				printk(KERN_ERR "synaptics_ts_resume power off failed\n");
		}
		return 0;
	}


static int IT7260_ts_resume(struct i2c_client *client)
	{
		int ret;
		struct IT7260_ts_data *ts = i2c_get_clientdata(client);
	    pr_info("=entry IT7260_ts_resume=\n");
		if (ts->power) {
			ret = ts->power(1);
			if (ret < 0)
				printk(KERN_ERR "synaptics_ts_resume power on failed\n");
		}
	
		///synaptics_init_panel(ts);
	
		if (ts->use_irq)
			enable_irq(client->irq);
	
		if (!ts->use_irq)
			hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		#if 0
		else {
			set_ite_i2c_nostop(0);
			i2c_smbus_write_byte_data(ts->client, 0xf1, 0x01); /* enable abs int */
		}
		#endif
	
		return 0;
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void IT7260_ts_early_suspend(struct early_suspend *h)
{
	struct IT7260_ts_data *ts;
	ts = container_of(h, struct IT7260_ts_data, early_suspend);
	IT7260_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void IT7260_ts_late_resume(struct early_suspend *h)
{
	struct IT7260_ts_data *ts;
	ts = container_of(h, struct IT7260_ts_data, early_suspend);
	IT7260_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id IT7260_ts_id[] = {
	{ IT7260_I2C_NAME, 0 },
	{ }
};

bool IT7260_Init(void)
{
	int i;
	int tmp;
	unsigned char ucQuery = 0;
	unsigned char buffer[128];
	struct IT7260_ts_data *ts = gl_ts;

	// Identify Cap Sensor
	set_ite_i2c_nostop(1);
	do {
		ucQuery = i2c_smbus_read_byte_data(ts->client, 0x80);
	} while (ucQuery & 0x01);
	buffer[0] = 0x00;
	set_ite_i2c_nostop(0);
	i2c_smbus_write_byte_data(ts->client, 0x20, buffer[0]);
	set_ite_i2c_nostop(1);
	do {
		ucQuery = i2c_smbus_read_byte_data(ts->client, 0x80);
	} while (ucQuery & 0x01);

	memset(&buffer, 0, sizeof(buffer));
	set_ite_i2c_nostop(1);
	//i2c_smbus_read_i2c_block_data(ts->client, 0xA0, 10, buffer);
	i2c_smbus_read_i2c_block_data(ts->client, 0xA0, 8, buffer);

	pr_info("=IT7260_Init --[%x][%x][%x][%x][%x][%x]=\n",buffer[0],buffer[1],
		                                               buffer[2],buffer[3],buffer[4],buffer[5]);
	if (buffer[1] != 'I' || 
	   buffer[2] != 'T' ||
	   buffer[3] != 'E') {
	//	return false;
	}

	// Get firmware information
	set_ite_i2c_nostop(1);
	do {
		ucQuery = i2c_smbus_read_byte_data(ts->client, 0x80);
	} while (ucQuery & 0x01);
	buffer[0] = 0x01;
	buffer[1] = 0x00;
	set_ite_i2c_nostop(0);
	i2c_smbus_write_i2c_block_data(ts->client, 0x20, 2, buffer);
	set_ite_i2c_nostop(1);
	do {
		ucQuery = i2c_smbus_read_byte_data(ts->client, 0x80);
	} while (ucQuery & 0x01);
	memset(&buffer, 0, sizeof(buffer));
	set_ite_i2c_nostop(1);
	//i2c_smbus_read_i2c_block_data(ts->client, 0xA0, 9, buffer);
	i2c_smbus_read_i2c_block_data(ts->client, 0xA0, 8, buffer);
	tmp = 0;
	//for (i = 5; i < 9; i++) {
	for (i = 5; i < 8; i++) {
		tmp += buffer[i];
	}
	if (tmp == 0) {
	//	return false;
	}

	//// Get 2D Resolution
	//set_ite_i2c_nostop(1);
	//do {
	//	ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
	//} while (ucQuery & 0x01);
	//buffer[0] = 0x01;
	//buffer[1] = 0x02;
	//buffer[2] = 0x00;
	//set_ite_i2c_nostop(0);
	//i2c_smbus_write_i2c_block_data(ts->client, 0x20, 3, buffer);
	//set_ite_i2c_nostop(1);
	//do {
	//	ucQuery=i2c_smbus_read_byte_data(ts->client, 0x80);
	//} while (ucQuery & 0x01);
	//memset(&buffer, 0, sizeof(buffer));
	//set_ite_i2c_nostop(1);
	//i2c_smbus_read_i2c_block_data(ts->client, 0xA0, 11, buffer);
	//xres = (int)(buffer[2] + (buffer[3] << 8));
	//yres = (int)(buffer[4] + (buffer[5] << 8));
	//if (xres == 0) {
	//	xres = IT7260_X_RESOLUTION;
	//}
	//if (yres == 0) {
	//	yres = IT7260_Y_RESOLUTION;
	//}

	//// Reinitialize Firmware
	//set_ite_i2c_nostop(1);
	//do {
	//	ucQuery = i2c_smbus_read_byte_data(ts->client, 0x80);
	//} while (ucQuery & 0x01);
	//buffer[0] = 0x6F;
	//set_ite_i2c_nostop(0);
	//i2c_smbus_write_byte_data(ts->client, 0x20, buffer[0]);

	return true;
}

static struct i2c_driver IT7260_ts_driver = {
	.probe		= IT7260_ts_probe,
	.remove		= IT7260_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= IT7260_ts_suspend,
	.resume		= IT7260_ts_resume,
#endif
	.id_table	= IT7260_ts_id,
	.driver = {
		.name	= "IT7260-ts",
	},
};

struct ite7260_data {
	rwlock_t lock;
	//unsigned char bufferIndex;
	unsigned short bufferIndex;
	//unsigned int length;
	unsigned short length;
	//unsigned int buffer[MAX_BUFFER_SIZE];
	unsigned short buffer[MAX_BUFFER_SIZE];
};

int ite7260_ioctl(struct inode *inode, struct file *filp, 
					unsigned int cmd, unsigned long arg)
{
	struct ite7260_data *dev = filp->private_data;
	int retval = 0;
	int i;
	unsigned char ucQuery;
	unsigned char buffer[MAX_BUFFER_SIZE];
	struct ioctl_cmd168 data;
	unsigned char datalen;
	unsigned char ent[] = {0x60, 0x00, 0x49, 0x54, 0x37, 0x32};
	unsigned char ext[] = {0x60, 0x80, 0x49, 0x54, 0x37, 0x32};

    //pr_info("=ite7260_ioctl=\n");
	//memset(&data, 0, sizeof(data));
	memset(&data, 0, sizeof(struct ioctl_cmd168));

	switch (cmd) {
		case IOCTL_SET:
pr_info("=IOCTL_SET=\n");
//IdentifyCapSensor(gl_ts);
			if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
				retval = -EFAULT;
				goto done;
			}

			//if ( copy_from_user(&data, (int __user *)arg, sizeof(data)) ) {
			if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
				retval = -EFAULT;
				goto done;
			}
//pr_info("SET:bufferIndex=%x, dataLength=%d =>", (unsigned char)data.bufferIndex, (unsigned char)data.length);
//for(i = 0; i < data.length; i++)
//{
//pr_info("%.2X ", data.buffer[i]);
//} 
//pr_info("\n");
//      for(i = 0; i < 16; i++)
//      {
//          buffer[i] = (unsigned char)data.buffer[i];
//pr_info("%.2X ", buffer[i]);
//      }
buffer[0] = (unsigned char)data.bufferIndex;
//pr_info("%.2X ", buffer[0]);
      for(i = 1; i < 16; i++)
      {
          buffer[i] = (unsigned char)data.buffer[i - 1];
//pr_info("%.2X ", buffer[i]);
      }

//test 
if (!memcmp(&(buffer[1]), ent, sizeof(ent))) {

	pr_info("Disabling IRQ.\n");

	disable_irq(gl_ts->client->irq);

}

if (!memcmp(&(buffer[1]), ext, sizeof(ext))) {

	pr_info("Enabling IRQ.\n");

	enable_irq(gl_ts->client->irq);

}
// test

//pr_info("=================================================\n");      
//pr_info("name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);
	  datalen = (unsigned char)(data.length + 1);
//pr_info("datalen=%d\n", datalen);
			//write_lock(&dev->lock);
			set_ite_i2c_nostop(0);
			//retval = i2c_smbus_write_i2c_block_data(gl_ts->client, (unsigned char)data.bufferIndex, (unsigned char)data.length, (unsigned char*)buffer);
//retval = i2c_smbus_write_byte_data(gl_ts->client, 0x20, (unsigned char)0x00);//retval is 0xfff5

//retval = i2c_smbus_write_byte_data(gl_ts->client, 0x20, (unsigned char)0x00);
retval = i2c_master_send(gl_ts->client, buffer, datalen);
			//write_unlock(&dev->lock);
//pr_info("SET:retval=%x\n", retval);
			retval = 0;

			break;

		case IOCTL_GET:
//pr_info("=IOCTL_GET=\n");
//IdentifyCapSensor(gl_ts);
//set_ite_i2c_nostop(1);
//i2c_smbus_read_i2c_block_data(gl_ts->client, 0xA0, 8, buffer);
//pr_info("GET:After IdentifyCapSensor=buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", buffer[0], buffer[1], buffer[2], buffer[3]); 			
			if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
				retval = -EFAULT;
				goto done;
			}

			if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
				retval = -EFAULT;
				goto done;
			}

//pr_info("sizeof(struct ioctl_cmd168)=%d\n", sizeof(struct ioctl_cmd168));
			//if ( copy_from_user(&data, (int __user *)arg, sizeof(data)) ) {
			if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
				retval = -EFAULT;
				goto done;
			}

//pr_info("=================================================\n");      
//pr_info("name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);
			//read_lock(&dev->lock);
			set_ite_i2c_nostop(1);
			//i2c_smbus_read_i2c_block_data(gl_ts->client, data.bufferIndex, data.length, buffer);
			retval = i2c_smbus_read_i2c_block_data(gl_ts->client, (unsigned char)data.bufferIndex, (unsigned char)data.length, (unsigned char*)buffer);
			//retval = i2c_smbus_read_i2c_block_data(gl_ts->client, 0x80, 1, buffer);
			//read_unlock(&dev->lock);
//pr_info("GET:retval=%x\n", retval);
			//read_lock(&dev->lock);
//retval=i2c_smbus_read_byte_data(gl_ts->client, 0x80);
			//read_unlock(&dev->lock);
//pr_info("GET:retval=%x\n", retval);
			retval = 0;
			for(i = 0; i < data.length; i++) {
				data.buffer[i] = (unsigned short)buffer[i];
			}
//pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, buffer[0], buffer[1], buffer[2], buffer[3]); 			
//pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, data.buffer[0], data.buffer[1], data.buffer[2], data.buffer[3]); 			
//IdentifyCapSensor(gl_ts);
//if (data.bufferIndex == 0x80)
//	data.buffer[0] = 0x00;
			//if ( copy_to_user((int __user *)arg, &data, sizeof(data)) ) {
			if ( copy_to_user((int __user *)arg, &data, sizeof(struct ioctl_cmd168)) ) {
				retval = -EFAULT;
				goto done;
			}
			break;

		default:
			retval = -ENOTTY;
			break;
	}

done:
//pr_info("DONE! retval=%d\n", retval);
	return (retval);
}

int ite7260_open(struct inode *inode, struct file *filp)
{
	int i;
	struct ite7260_data *dev;

    pr_info("=ite7260_open=\n");
	dev = kmalloc(sizeof(struct ite7260_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	/* initialize members */
	rwlock_init(&dev->lock);
	for (i = 0; i < MAX_BUFFER_SIZE; i++) {
		dev->buffer[i] = 0xFF;
	}

	filp->private_data = dev;

	return 0;   /* success */
}

int ite7260_close(struct inode *inode, struct file *filp)
{
	struct ite7260_data *dev = filp->private_data;

	if (dev) {
		kfree(dev);
	}

	return 0;   /* success */
}

struct file_operations ite7260_fops = {
	.owner = THIS_MODULE,
	.open = ite7260_open,
	.release = ite7260_close,
	.ioctl = ite7260_ioctl,
};

static int __devinit IT7260_ts_init(void)
{
  dev_t dev = MKDEV(ite7260_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;
	int input_err = 0;
	struct device *class_dev = NULL;

	DBG();

//	if(!IT7260_Init()) {
//		TS_DEBUG("IT7260 cannot be connected or is in firmware upgrade mode.\n");
//		goto error;
//	}

	alloc_ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (alloc_ret)	{
		TS_DEBUG("IT7260 cdev can't get major number\n");
		goto error;
	}
	ite7260_major = MAJOR(dev);

	// allocate the character device
	cdev_init(&ite7260_cdev, &ite7260_fops);
	ite7260_cdev.owner = THIS_MODULE;
	ite7260_cdev.ops = &ite7260_fops;
	cdev_err = cdev_add(&ite7260_cdev, MKDEV(ite7260_major, ite7260_minor), 1);
	if(cdev_err) {
		goto error;
	}

	// register class
	ite7260_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(ite7260_class)) {
		TS_DEBUG("Err: failed in creating class.\n");
		goto error;
	}

  ite7260_dev = MKDEV(ite7260_major, ite7260_minor);
	class_dev = device_create(ite7260_class, NULL, ite7260_dev, NULL, DEVICE_NAME);
	if(class_dev == NULL)
	{
	   TS_DEBUG("Err: failed in creating device.\n");
	   goto error;
  }
  TS_DEBUG("=========================================\n");
	TS_DEBUG("register IT7260 cdev, major: %d, minor: %d \n", ite7260_major, ite7260_minor);
	TS_DEBUG("=========================================\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		input_err = -ENOMEM;
		goto error;
	}

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "I2C";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x7260;
	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SCREEN_X_RESOLUTION, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SCREEN_Y_RESOLUTION, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_FINGER_NUMBER + 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_X_RESOLUTION, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_Y_RESOLUTION, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);

	input_err = input_register_device(input_dev);
	if (input_err) goto error;
pr_info("it7260 driver is on###############################################################\n");

  i2c_add_driver(&IT7260_ts_driver);
	//return i2c_add_driver(&IT7260_ts_driver);
	return 0;

error:	
	if(cdev_err == 0) {
		cdev_del(&ite7260_cdev);
	}
	if(alloc_ret == 0) {
		unregister_chrdev_region(dev, 1);
	}
	if(input_dev) {
		input_free_device(input_dev);
	}
	if (IT7260_wq)
		destroy_workqueue(IT7260_wq);

	return -1;
}

static void __exit IT7260_ts_exit(void)
{
  dev_t dev = MKDEV(ite7260_major, ite7260_minor);
  
	// unregister class
	device_destroy(ite7260_class, ite7260_dev);
	class_destroy(ite7260_class);

	// unregister driver handle
	cdev_del(&ite7260_cdev);
	unregister_chrdev_region(dev, 1);

	i2c_del_driver(&IT7260_ts_driver);
	if (IT7260_wq)
		destroy_workqueue(IT7260_wq);
}

module_init(IT7260_ts_init);
module_exit(IT7260_ts_exit);

MODULE_DESCRIPTION("IT7260 Touchscreen Driver");
MODULE_LICENSE("GPL");
