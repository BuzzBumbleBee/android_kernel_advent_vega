/*
 * drivers/rtc/rtc-tegra-odm.c
 *
 * Tegra ODM kit wrapper for RTC functionality implemented in Tegra
 * ODM PMU adaptation
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include "nvodm_pmu.h"

#define PMU_IOCTL_ENABLE  1

/* Create a custom rtc structrue and move this to that structure */
static NvOdmPmuDeviceHandle hPmu = NULL;

static int tegra_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	NvU32 now;

	if (hPmu == NULL)
		return -1;

	if (!NvOdmPmuReadRtc(hPmu, &now)) {
		printk("NvOdmPmuReadRtc failed\n");
		return -1;
	}

	rtc_time_to_tm(now, tm);
	return 0;
}

static int tegra_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long now;
	int ret;

	if (hPmu == NULL)
		return -1;

	ret = rtc_tm_to_time(tm, &now);
	if (ret != 0)
		return -1;

	if (!NvOdmPmuWriteRtc(hPmu, (NvU32)now)) {
		printk("NvOdmPmuWriteRtc failed\n");
		return -1;
	}
	return 0;
}

#if (PMU_IOCTL_ENABLE)

/* Ant start
static int tegra_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct rtc_wkalrm	wkalrm;
	NvU32 count = 0;

	switch (cmd) {
		case RTC_ALM_READ:
			if(NvOdmPmuReadAlarm(hPmu, &count))
			{
				wkalrm.time.tm_hour		= count / 3600;
				wkalrm.time.tm_min	= (count - (wkalrm.time.tm_hour * 3600)) / 60;
				wkalrm.time.tm_sec	= (count - (wkalrm.time.tm_min * 60) - (wkalrm.time.tm_hour * 3600) );
				if (copy_to_user(argp, &count, sizeof(count)))
					return -EFAULT;
			}
			else
			{
				pr_debug("NvOdmPmuReadRtc failed\n");
				return -EINVAL;
			}
			break;

		case RTC_ALM_SET:
			if (copy_from_user(&wkalrm, argp, sizeof(wkalrm)))
				return -EFAULT;
			count = wkalrm.time.tm_hour * 3600 + wkalrm.time.tm_min * 60 + wkalrm.time.tm_sec;
			if(!NvOdmPmuWriteAlarm(hPmu, count))
			{
				pr_debug("NvOdmPmuWriteRtc failed\n");
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
	}

	return 0;
}
*/
// Ant end

static int tegra_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{

	struct rtc_time *time = &wkalrm->time;

	NvU32 alarm_sec = 0;
	if(!NvOdmPmuReadAlarm(hPmu, &alarm_sec))
		return -EINVAL;

	rtc_time_to_tm(alarm_sec, time);

	return 0;
}

static int tegra_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct rtc_time *time = &wkalrm->time;
	NvU32 now;
	NvU32 alarm_sec;
	struct rtc_time now_time;

	pr_debug("wkalrm->enabled = %d\n", wkalrm->enabled);
	if (wkalrm->enabled == 0)
		return 0;

	if (!NvOdmPmuReadRtc(hPmu, &now)) {
		pr_debug("NvOdmPmuReadRtc failed\n");
		return -1;
	}

	rtc_time_to_tm(now, &now_time);
	pr_debug( "read  now_time %02d:%02d:%02d %02d/%02d/%04d\n",
		now_time.tm_hour, now_time.tm_min, now_time.tm_sec,
		now_time.tm_mon + 1, now_time.tm_mday, now_time.tm_year + 1900);

	pr_debug("write alarm_time %02d:%02d:%02d %02d/%02d/%04d\n",
		time->tm_hour, time->tm_min, time->tm_sec,
		time->tm_mon+1, time->tm_mday, time->tm_yday+1900);

	alarm_sec = (NvU32)mktime(now_time.tm_year + 1900, time->tm_mon+1, time->tm_mday,
				time->tm_hour, time->tm_min, time->tm_sec);
	if (alarm_sec < now)
		alarm_sec = (NvU32)mktime(now_time.tm_year + 1901, time->tm_mon+1, time->tm_mday,
				time->tm_hour, time->tm_min, time->tm_sec);

	pr_debug("alarm_sec = %u\n", alarm_sec);

	if(!NvOdmPmuWriteAlarm(hPmu, alarm_sec-now))
		return -EINVAL;

	return 0;
}
#endif

static struct rtc_class_ops tegra_rtc_ops = {
	.read_time	= tegra_rtc_read_time,
	.set_time	= tegra_rtc_set_time,
#if (PMU_IOCTL_ENABLE)
	.read_alarm	= tegra_rtc_read_alarm,
	.set_alarm	= tegra_rtc_set_alarm,
#endif
};

static int __init tegra_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	NvU32 initial;

	if (NvOdmPmuDeviceOpen(&hPmu) == NV_FALSE) {
		pr_debug("%s: NvOdmPmuDeviceOpen failed\n", pdev->name);
		return -ENXIO;
	}

	/* if the SoCs PMU has't been properly initialized, a bogus large
	 * value may be returned which triggers the Y2038 bug in the kernel.
	 * work-around this issue by checking the initial value of the PMU
	 * and then clobbering it if the value is bogus */

	if (NvOdmPmuReadRtc(hPmu, &initial) && ((time_t)initial < 0))
	{
		if(!NvOdmPmuWriteRtc(hPmu, 0))
			return -EINVAL;
	}


	rtc = rtc_device_register(pdev->name, &pdev->dev,
		&tegra_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc)) {
		pr_debug("%s: can't register RTC device, err %ld\n",
			pdev->name, PTR_ERR(rtc));
		NvOdmPmuDeviceClose(hPmu);
		return -1;
	}
	platform_set_drvdata(pdev, rtc);

	return 0;
}

static int __exit tegra_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);;

	rtc_device_unregister(rtc);
	return 0;
}

#ifdef CONFIG_PM

static int tegra_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int tegra_rtc_resume(struct platform_device *pdev)
{
	return 0;
}

#endif

static void tegra_rtc_shutdown(struct platform_device *pdev)
{
}

MODULE_ALIAS("platform:tegra_rtc_odm");

static struct platform_driver tegra_rtc_driver = {
	.remove		= __exit_p(tegra_rtc_remove),
	.shutdown	= tegra_rtc_shutdown,
#ifdef CONFIG_PM
	.suspend	= tegra_rtc_suspend,
	.resume		= tegra_rtc_resume,
#endif
	.driver		=  {
		.name  = "tegra_rtc_odm",
		.owner = THIS_MODULE,
	},
};

static int __init rtc_init(void)
{
	return platform_driver_probe(&tegra_rtc_driver, tegra_rtc_probe);
}
module_init(rtc_init);

static void __exit rtc_exit(void)
{
	platform_driver_unregister(&tegra_rtc_driver);
}
module_exit(rtc_exit);
