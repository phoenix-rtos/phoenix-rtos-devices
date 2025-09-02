#include "phoenix_firmware.h"
#include "phoenix_log.h"

int request_firmware_direct(const struct firmware **firmware, const char *file_name, struct device *dev)
{
    /* Open file and read it to firmware struct buffer */
    (void)dev;

    *firmware = (struct firmware *)malloc(sizeof(struct firmware));
    struct firmware *helper = *firmware;


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

    (helper)->data = (u8*)malloc(file_size);

    if(NULL == (helper)->data){
        pr_err("Failed to allocate buffer for firmware\n");
        return -1;
    }

    fseek(f, 0, 0);
    (helper)->size = fread((helper)->data, file_size, file_size, f);

    if((helper)->size != file_size){
        pr_err("Failed to read whole firmware file %s:%s\n", __FILE__, __func__);
        return -1;
    }

	return 0;
}

void release_firmware(const struct firmware *firmware)
{
    /* Close file and free firmware struct */
    free(firmware->data);
    return;
}