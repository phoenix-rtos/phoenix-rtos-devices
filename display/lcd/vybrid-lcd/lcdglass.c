#include <dev/dev.h>
#include <dev/lcdglass/if.h>
#include <include/lcd.h>
#include <main/if.h>
static int lcdglass_open(vnode_t *vnode, file_t* file)
{
	assert(vnode != NULL);
	assert(file != NULL);
	assert(vnode->type == vnodeDevice);

	if(subdev_get(vnode->dev, &file->priv) != EOK)
	{
		file->priv = NULL;
		return -1;
	}

	return 0;
}

static int lcdglass_write(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	char tbuf[16];
	assert(file != NULL);
	assert(file->priv != NULL);
	lcdglass_device *dev = (lcdglass_device *) file->priv;

	if(len > 15) len=15;
	memcpy(tbuf, buff, len);
	tbuf[len]=0;
	dev->clear();
	dev->print(tbuf, 8);
	return len;
}


int lcdglass_ioctl(file_t *file, unsigned int cmd, unsigned long arg)
{
	assert(file != NULL);
	assert(file->priv != NULL);
	lcdglass_device *dev = (lcdglass_device *) file->priv;

	switch(cmd)
	{
		case CLEAR:
			dev->clear();
		break;
		case SET_SEGMENT:
			dev->setSegment((int)arg);
		break;
		case CLEAR_SEGMENT:
			dev->clearSegment((int)arg);
		break;
		case PRINT_ROW:
		{
			LcdRow_t *row = (LcdRow_t*)arg;
			char tbuff[16];
			int idx;
			if(row == NULL)
				return -EINVAL;
			if(row->buf > (char *)VADDR_KERNEL)
				return -EINVAL;
			for(idx=0; idx < 16; ++idx) {
				if(row->buf + idx > (char *)VADDR_KERNEL)
					return -EINVAL;
				tbuff[idx]=row->buf[idx];
				if(row->buf[idx] == 0)
					break;
			}
			tbuff[15]=0;
			if(dev->printRow(tbuff, row->row))
				return EOK;
			else
				return -EFAULT;
		}
		break;
		default:
			if(dev->ioctl)
				return dev->ioctl(file, cmd, arg);
			else
				return -EINVAL;
		break;
	};

	return EOK;
}

int lcdglass_truncate(vnode_t* vnode, unsigned int size)
{
	return EOK;
}

int lcdglass_init()
{
	static const file_ops_t lcdglass_ops = {
		.open = lcdglass_open,
		.write = lcdglass_write,
		.ioctl = lcdglass_ioctl,
		.truncate = lcdglass_truncate
	};

	if (dev_register(MAKEDEV(MAJOR_LCDGLASS, 0), &lcdglass_ops) < 0) {
		main_printf(ATTR_ERROR, "lcdglass: Can't register device for /dev/lcd!\n" );
		return -1;
	}

	return 0;
}
