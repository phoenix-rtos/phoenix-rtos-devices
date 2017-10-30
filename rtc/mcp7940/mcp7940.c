/**
 * MCP7940 RTC driver
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Pawel Krezolek <pawel.krezolek@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

/* I2C slave addresses in OmniChip's ADC board: Thermometer = 0x4C, ADCs = 0x48, 0x4A*/
/* In Tytus1: Thermometer orig = 0x48, later changed to 0x4C; Magnetometr = 0x57 or 0x56 ;
 * 				RTC = 0x6f for RTC + 0x57 for EEPROM;
 **/

#include <hal/if.h>
#include <main/if.h>
#include <dev/vybrid-i2c/if.h>
#include <dev/if.h>
#include <dev/rtc/rtc.h>
#include <dev/rtc/rtc-core.h>

#define REG_RTCSEC 0x0
#define REG_RTCMIN 0x1
#define REG_RTCHOUR 0x2
#define REG_RTCWKDAY 0x3
#define REG_RTCDATE 0x4
#define REG_RTCMTH 0x5
#define REG_RTCYEAR 0x6
#define REG_CONTROL 0x7
#define REG_OSCTRIM 0x8


static const int BCD2INT[16] =
{
	0,
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	8,
	9,
	-1, //this should never be accessed if bcd is proper
	-1, //this should never be accessed if bcd is proper
	-1, //this should never be accessed if bcd is proper
	-1, //this should never be accessed if bcd is proper
	-1, //this should never be accessed if bcd is proper
	-1  //this should never be accessed if bcd is proper
};

#define RTC_ST 0
#define RTC_AMPM 1
#define RTC_1224 2
#define RTC_OSCRUN 3
#define RTC_PWRFAIL 4 
#define RTC_VBATEN 5
#define RTC_LPYR 6
//Control reg start
#define OUT  14
#define SQWEN 13
#define ALM1EN 12
#define ALM0EN 11
#define EXTOSC 10
#define CRSTRIM 9
#define SQWFS1 8
#define SQWFS0 7
//Control reg end
//Osctrim reg start
#define SIGN 22
#define TRIMVAL6 21
#define TRIMVAL5 20
#define TRIMVAL4 19
#define TRIMVAL3 18
#define TRIMVAL2 17
#define TRIMVAL1 16
#define TRIMVAL0 15
//Osctrim reg end

static u32 uint2bcd(u32 number)
{
	u32 result = 0;
	int i = 0;
	
	do
	{
		int j = 0;
		u8 digit[8];
		for(j = 0; j < 8; ++j)
		{
			digit[j] = ((0xf << (j * 4)) &  result) >> (j * 4);
		}
		result = 0;
		for(j = 0; j < 8; ++j)
		{
		    if(digit[j] >= 5)
			{
				digit[j] += 3;
			}
			result += ((u32)digit[j]) << j*4;
		}
		result <<= 1;
		result |= ((1 << 31) & number) >> 31;
		number <<= 1;
		
		++i;
	}while(i < 32);
	return result;
}

#define RTC_MAX_FREQ	8192

static int mcp7940_open(rtc_device *dev)
{
	assert(dev != NULL);
	
	return 0;
}

static void mcp7940_release(rtc_device *dev)
{
	if(dev != NULL)
	{
		//TODO - do something
	}
}

static int mcp7940_read_time(rtc_device *dev, struct rtc_time *time)
{
	assert(dev != NULL);
	
	
	
		s32 res;
	u8 buf[1];
	u8 rtc_i2c_addr = I2C_MCP7940_RTC_ADDR;
	volatile u32 status = 0;
	assert(time != NULL);
	//TODO - calculate yday
	//tm_yday;//!< day in the year
	//TODO - find out what is this
	//tm_isdst;//!< daylight saving time
	//TODO - read status bits
	
	//read control register
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_CONTROL, buf, 1);
	if(res < 0)
		return -1;
	status |= ((u32)buf[0]) << 7;
	
	//read osctrim register
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_OSCTRIM, buf, 1);
	if(res < 0)
		return -1;
	status |= ((u32)buf[0]) << 15;	
	
	//read seconds
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_RTCSEC, buf, 1);
	if(res < 0)
		return -1;
	time->tm_sec = BCD2INT[buf[0] & 0xF] + 10 * BCD2INT[(buf[0] >> 4) & 0x7];
	status |= ((buf[0] >> 7) & 0x1) << RTC_ST;
	//read minutes
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_RTCMIN, buf, 1);
	if(res < 0)
		return -1;
	time->tm_min = BCD2INT[buf[0] & 0xF] + 10 * BCD2INT[(buf[0] >> 4) & 0x7];
	
	//read hours
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_RTCHOUR, buf, 1);
	if(res < 0)
		return -1;
	
	if(buf[0] & 0x40)//12 hour format
	{
		time->tm_hour = BCD2INT[buf[0] & 0xF] + 10 * BCD2INT[(buf[0] >> 4) & 0x1];
		//convert it into 24 hour format manually
		if(buf[0] & 0x20)//PM
		{
			time->tm_hour += 12;
		}
	}
	else//24 hour format
		time->tm_hour = BCD2INT[buf[0] & 0xF] + 10 * BCD2INT[(buf[0] >> 4) & 0x3];
	status |= ((buf[0] >> 5) & 0x1) << RTC_AMPM;
	status |= ((buf[0] >> 6) & 0x1) << RTC_1224;
	//read day of the week
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_RTCWKDAY, buf, 1);
	if(res < 0)
		return -1;
	time->tm_wday = BCD2INT[buf[0] & 0x7];
	status |= ((buf[0] >> 5) & 0x1) << RTC_OSCRUN;
	status |= ((buf[0] >> 4) & 0x1) << RTC_PWRFAIL;
	status |= ((buf[0] >> 3) & 0x1) << RTC_VBATEN;
	//read day of month
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_RTCDATE, buf, 1);
	if(res < 0)
		return -1;
	time->tm_mday = BCD2INT[buf[0] & 0xF] + 10 * BCD2INT[(buf[0] >> 4) & 0x3];
	
	//read month
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_RTCMTH, buf, 1);
	if(res < 0)
		return -1;
	time->tm_mon = BCD2INT[buf[0] & 0xF] + 10 * BCD2INT[(buf[0] >> 4) & 0x1];
	status |= ((buf[0] >> 5) & 0x1) << RTC_LPYR;
	//read year
	res = i2c_devRead(I2C_MCP7940_BUS, rtc_i2c_addr, REG_RTCYEAR, buf, 1);
	if(res < 0)
		return -1;
	time->tm_year = BCD2INT[buf[0] & 0xF] + 10 * BCD2INT[(buf[0] >> 4) & 0xF];
	

	
	return EOK;
}
static int mcp7940_set_time(rtc_device *dev, struct rtc_time *time)
{
	assert(dev != NULL);
	u8 cmd[1];
	s32 res;
	assert(time != NULL);
	//turn off external oscilator
	cmd[0]= 0;
	res = i2c_devWrite(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCSEC, cmd, 1);
	if(res != EOK)
	{
		main_printf(ATTR_FAILURE, "Rtc transfer failed!\n");
		return -1;
	}
	//wait until oscillator still running
	do
	{
		res = i2c_devRead(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCWKDAY, cmd, 1);
		if(res != EOK)
		{
			main_printf(ATTR_FAILURE, "Failed to set rtc! - RTC NOT RUNNING\n");
			return -1;
		}
		
	}
	while(cmd[0] & (1 << 5));
	//write new time setting
	//wait for 
	cmd[0] = uint2bcd(time->tm_min);
	res = i2c_devWrite(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCMIN, cmd, 1);
	assert(res == EOK);
	cmd[0] = uint2bcd(time->tm_hour);
	//TODO - take care about flags
	res = i2c_devWrite(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCHOUR, cmd, 1);
	assert(res == EOK);
	cmd[0] = uint2bcd(time->tm_mday);
	res = i2c_devWrite(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCDATE, cmd, 1);
	assert(res == EOK);
	cmd[0] = uint2bcd(time->tm_mon);
	res = i2c_devWrite(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCMTH, cmd, 1);
	assert(res == EOK);
	cmd[0] = uint2bcd(time->tm_year);
	res = i2c_devWrite(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCYEAR, cmd, 1);
	assert(res == EOK);
	cmd[0] = uint2bcd(time->tm_wday);
	res = i2c_devWrite(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCWKDAY, cmd, 1);
	assert(res == EOK);
	
	//turn on external oscillator
	cmd[0]=(u8) 0x1 << 7 | uint2bcd(time->tm_sec);
	res = i2c_devWrite(I2C_MCP7940_BUS, I2C_MCP7940_RTC_ADDR, REG_RTCSEC, cmd, 1);
	assert(res == EOK);
	return EOK;
}
int mcp7940_ioctl(rtc_device *dev, unsigned int cmd, unsigned long arg)
{
	assert(dev != NULL);
	switch(cmd)
	{
	case RTC_EPOCH_SET:
	case RTC_EPOCH_READ:
	default:
		return -1;
	};
	return -1;
}

int _mcp7940_init(void)
{
	static const rtc_class_ops_t mcp7940_ops = {
		.open=mcp7940_open,
		.release=mcp7940_release,
		.ioctl=mcp7940_ioctl,
		.read_time=mcp7940_read_time,
		.set_time=mcp7940_set_time,
		//int (*read_alarm)(dev_t *, struct rtc_wkalrm *);
		//int (*set_alarm)(dev_t *, struct rtc_wkalrm *);
		//int (*proc)(dev_t *, struct seq_file *);
		.set_mmss=NULL,
		.read_callback=NULL,
		.alarm_irq_enable=NULL,
	};
	static rtc_device mcp7940_dev;
	int minor = dev_minorAlloc(MAKEDEV(MAJOR_RTC, 0));
	char label[] = "rtc00";

	memset(&mcp7940_dev, 0, sizeof(mcp7940_dev));
	
	mcp7940_dev.ops = &mcp7940_ops;
	mcp7940_dev.uie_unsupported=1;
	mcp7940_dev.pie_enabled=0;
	mcp7940_dev.id=minor;
	mcp7940_dev.dev = MAKEDEV(MAJOR_RTC, minor);
	mcp7940_dev.flags=0;
	mcp7940_dev.irq_data=0;
	mcp7940_dev.irq_task=NULL;
	mcp7940_dev.irq_freq=0;
	mcp7940_dev.max_user_freq=0;
	strcpy(mcp7940_dev.name, "mcp7940");
	
	proc_semaphoreCreate(&mcp7940_dev.ops_lock, 1);
	proc_spinlockCreate(&mcp7940_dev.irq_lock, "mcp7940_irq");
	proc_spinlockCreate(&mcp7940_dev.irq_task_lock, "mcp7940_irq_task");
	//usually this will be done externally
	
	if(subdev_register(mcp7940_dev.dev, (void *)&mcp7940_dev) < 0)
	{
		//registration failed
		proc_semaphoreTerminate(&mcp7940_dev.ops_lock);
		proc_spinlockTerminate(&mcp7940_dev.irq_lock);
		proc_spinlockTerminate(&mcp7940_dev.irq_task_lock);
		return -1;
	}


	if(minor < 10) {
		label[3] = minor + '0';
		label[4] = 0;
	}
	else {
		label[3] = minor/10 + '0';
		label[4] = minor%10 + '0';
	}
	assert(EOK==dev_mknod(MAKEDEV(MAJOR_RTC, minor), label));

	return 0;
}
