#include "phoenix_firmware.h"
#include "phoenix_log.h"

int request_firmware_direct(const struct firmware **firmware, const char *file_name, struct device *dev)
{
    /* Open file and read it to firmware struct buffer */
    (void)dev;

    *firmware = (struct firmware *)malloc(sizeof(struct firmware));
    struct firmware *helper = *firmware;


    hailo_notice(NULL, "Trying to open: %s\n", file_name);
    FILE *f = fopen(file_name, "rb");
    if(NULL == f){
        pr_err("Failed to open firmware file\n");
        return -1;
    }

    if(fseek(f, 0, SEEK_END) < 0){
        pr_err("Failed to seek end\n");
        return -1;
    }

    size_t file_size = ftell(f);

    hailo_notice(NULL, "Size of file is %ld bytes\n", file_size);

    (helper)->data = (u8*)malloc(file_size);

    if(NULL == (helper)->data){
        pr_err("Failed to allocate buffer for firmware\n");
        return -1;
    }

    fseek(f, 0, 0);
    (helper)->size = fread((helper)->data, 1, file_size, f);

    hailo_notice(NULL, "Managed to read out %lu bytes\n", helper->size);

    if((helper)->size != file_size){
        pr_err("Failed to read whole firmware file %s:%s\n", __FILE__, __func__);
        return -1;
    }

    hailo_notice(NULL, "Managed to read whole firmware into the buffer (%lu)\n", helper->size);

	return 0;
}

void release_firmware(const struct firmware *firmware)
{
    /* Close file and free firmware struct */
    free(firmware->data);
    return;
}