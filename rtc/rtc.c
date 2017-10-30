#include "rtc-core.h"
#include <lib/timeconv.h>
#include <include/errno.h>
#include <string.h>


static int rtc_open(vnode_t *vnode, file_t* file)
{
	assert(vnode != NULL);
	assert(file != NULL);
	assert(vnode->type == vnodeDevice);
	vnode->flags |= VNODE_RTC;

	if(subdev_get(vnode->dev, &file->priv) != EOK)
	{
		file->priv = NULL;
		return -1;
	}
	//rtc_device *dev = (rtc_device *)file->priv;
	
	return 0;
}

static int rtc_release(vnode_t *vnode)
{
	assert(vnode != NULL);
	assert(vnode->type == vnodeDevice);
	assert((vnode->flags & VNODE_RTC) != 0);
	/*rtc_device *dev;
	if(subdev_get(vnode->dev, (void **)&dev) != EOK)
	{
		return -1;
	}*/
	
	return 0;
}


static int rtc_time2str(const struct rtc_time *time, char *buf, size_t len)
{

    static const char * dayLT[] =
    {
        "Fri",
        "Sat",
        "Sun",
        "Mon",
        "Tue",
        "Wed",
        "Thu"
    };

    static const char * monthLT[] =
    {
        "Jan",
        "Feb",
        "Mar",
        "Apr",
        "May",
        "Jun",
        "Jul",
        "Aug",
        "Sep",
        "Oct",
        "Nov",
        "Dec"
    };

    if(buf == NULL)
        return EOK;
    if(time == NULL)
        return -EINVAL;
    if(len < 9) //not enough to fit hh:mm:ss\0
        return -EINVAL;

    if(len < 11)
    {
        //write only hh:mm:sec
        main_snprintf(buf, len, "%02d:%02d:%02d", time->tm_hour % 24, time->tm_min % 60, time->tm_sec % 60);
    }
    else if(len < 20)
    {
        //write only dd.mm.YYYY
        main_snprintf(buf, len, "%02d.%02d.%04d", time->tm_mday % 31, time->tm_mon % 12, time->tm_year + 1970);
    }
    else if(len < 25)
    {
        //write only dd.mm.YYYY hh:mm:ss
        main_snprintf(buf, len, "%02d.%02d.%04d %02d:%02d:%02d", time->tm_mday % 31, time->tm_mon % 12, time->tm_year + 1970, time->tm_hour % 24, time->tm_min % 60, time->tm_sec % 60);
    }
    else
    {
        //write WeekDay Month DayOfMonth hh:mm:ss YYYY
        main_snprintf(buf, len, "%3s %3s %02d %02d:%02d:%02d %04d", dayLT[time->tm_wday], monthLT[time->tm_mon], time->tm_mday % 31, time->tm_hour % 24, time->tm_min % 60, time->tm_sec % 60, time->tm_year + 1970);
    }
    return EOK;
}

static int rtc_read(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	assert(file != NULL);
	assert(file->priv != NULL);
    rtc_device *dev = (rtc_device *) file->priv;
    struct rtc_time t;

    if(rtc_read_time(dev, &t) != EOK)
    {
        return -EFAULT;
    }
    return rtc_time2str(&t, buff, len);
}


static int rtc_poll(file_t *file, ktime_t timeout, int op)
{
	assert(file != NULL);
	assert(file->priv != NULL);
    //this should return only when rtc communication is possible
	return EOK;
}

static int rtc_select_poll(file_t *file, unsigned *ready)
{
	assert(file != NULL);
	assert(file->priv != NULL);
    //TODO - this should wait until rtc starts working, and after that it should always return immediately (until communication will be possible)
    *ready = FS_READY_READ | FS_READY_WRITE;
	return EOK;
}

int rtc_ioctl(file_t *file, unsigned int cmd, unsigned long arg)
{
	assert(file != NULL);
	assert(file->priv != NULL);
	rtc_device *dev = (rtc_device *) file->priv;
	
	switch(cmd)
	{
		//TODO case RTC_AIE_ON:/* Alarm int. enable on		*/
		//TODO case RTC_AIE_OFF:/* ... off			*/
		case RTC_UIE_ON: /* Update int. enable on	*/
		break;
		case RTC_UIE_OFF: /* ... off			*/
		break;
		//TODO case RTC_PIE_ON:/* Periodic int. enable on	*/
		//TODO case RTC_PIE_OFF:/* ... off			*/
		//TODO case RTC_WIE_ON: /* Watchdog int. enable on	*/
		//TODO case RTC_WIE_OFF: /* ... off			*/
		//TODO case RTC_ALM_SET:/* Set alarm time  */
		//TODO case RTC_ALM_READ:/* Read alarm time */
		case RTC_RD_TIME:/* Read RTC time   */
		{
			struct rtc_time *tm = (struct rtc_time *) arg;
			if(rtc_read_time(dev, tm) != EOK)
			{
				return -1;
			}
		}
		break;
		case RTC_SET_TIME:/* Set RTC time    */
		{
			struct rtc_time *tm = (struct rtc_time *) arg;
			if(rtc_set_time(dev, tm) != EOK)
			{
				return -1;
			}
		}
		break;
		//TODO case RTC_IRQP_READ:/* Read IRQ rate   */
		//TODO case RTC_IRQP_SET:/* Set IRQ rate    */
		case RTC_EPOCH_READ:/* Read epoch      */
		{
            struct rtc_time t;
            ktime_t tmp;
            unsigned long *result = (unsigned long *) arg;
            if(rtc_read_time(dev, &t) != EOK)
			{
				return -1;
			}
            tm_to_time((struct tm *)&t, &tmp);
            *result = (unsigned long) tmp;
		}
		break;
		case RTC_EPOCH_SET:/* Set epoch       */
		{
            struct rtc_time t;
            ktime_t tmp;
            tmp = *(ktime_t *) arg;
            memset(&t, 0, sizeof(t));
            time_to_tm(tmp, 0, (struct tm *)&t);
            if(rtc_set_time(dev, &t) != EOK)
            {
                return -1;
            }
		}
		break;
		//TODO case RTC_WKALM_SET:/* Set wakeup alarm*/
		//TODO case RTC_WKALM_RD:/* Get wakeup alarm*/
		//TODO case RTC_PLL_GET:/* Get PLL correction */
		//TODO case RTC_PLL_SET:/* Set PLL correction */
		//TODO case RTC_VL_READ:/* Voltage low detector */
		//TODO case RTC_VL_CLR:/* Clear voltage low information */
		default:
		{
			int ret;
			proc_semaphoreDown(&dev->ops_lock);
			ret = dev->ops->ioctl(dev, cmd, arg);
			proc_semaphoreUp(&dev->ops_lock);
			return ret;
		}
		break;
	}
	return EOK;
}

int rtc_getlocaltime(struct tm *time)
{
	//TODO - consider static file pointer, initialized once - not good if module dynamically loaded, but now...
	//read local time from first registered RTC
    file_t *fp=NULL;
    int ret;
	rtc_time t;
	assert(time != NULL);
	//open driver file
    ret = fs_open("dev/rtc", O_RDONLY, 0, &fp);
	if(ret != EOK)
	{
        ret = fs_open("dev/rtc0", O_RDONLY, 0, &fp);
        if(ret != EOK)
        {
            //failed to open rtc driver file
            return -1;
        }
	}
	//do read via ioctl
    fs_ioctl(fp, RTC_RD_TIME, (unsigned long)&t);
	time->tm_sec = t.tm_sec;
	time->tm_min = t.tm_min;
	time->tm_hour = t.tm_hour;
	time->tm_mday = t.tm_mday;
	time->tm_mon = t.tm_mon;
	time->tm_wday = t.tm_wday;
	time->tm_year = t.tm_year;
	time->tm_yday = t.tm_yday;
	//close driver file
    fs_close(fp);
	return 0;
}

int rtc_setlocaltime(struct tm *time)
{
	//TODO - consider static file pointer, initialized once - not good if module dynamically loaded, but now...
	//read local time from first registered RTC
	file_t *fp=NULL;
	int ret;
	rtc_time t;
	assert(time != NULL);
	//open driver file
	ret = fs_open("dev/rtc", O_RDONLY, 0, &fp);
	if(ret != EOK)
	{
		ret = fs_open("dev/rtc0", O_RDONLY, 0, &fp);
		if(ret != EOK)
		{
			//failed to open rtc driver file
			return -1;
		}
	}
	
	t.tm_sec = time->tm_sec;
	t.tm_min = time->tm_min;
	t.tm_hour = time->tm_hour;
	t.tm_mday = time->tm_mday;
	t.tm_mon = time->tm_mon;
	t.tm_wday = time->tm_wday;
	t.tm_year = time->tm_year;
	t.tm_yday = time->tm_yday;
	
	//do write via ioctl
	fs_ioctl(fp, RTC_SET_TIME, (unsigned long)&t);
	//close driver file
	fs_close(fp);
	return 0;
}

int rtc_gettimeofday(struct timespec *t)
{
	struct tm time;
    ktime_t tmp;
    if(t == NULL)
        return -EINVAL;

	rtc_getlocaltime(&time);
    tm_to_time(&time, &tmp);
    t->tv_nsec=0;
    t->tv_sec = (typeof(t->tv_sec)) tmp;
	return 0;
}

int _rtc_init(void)
{
	static const file_ops_t rtc_ops = {
		.open = rtc_open,
		.read = rtc_read,
		.release = rtc_release,
		.poll = rtc_poll,
		.ioctl = rtc_ioctl,
		.select_poll = rtc_select_poll
	};
	
	if (dev_register(MAKEDEV(MAJOR_RTC, 0), &rtc_ops) < 0) {
		//;main_printf(ATTR_ERROR, "rtc mcp7940: Can't register device for /dev/rtc!\n" );
		return -1;
	}

	return 0;
}
