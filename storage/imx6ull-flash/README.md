# imx6ull-flash driver for Phoenix-RTOS 3
This subdirectory contains the source of the driver for iMX6ULL flash controller and NAND memory

This work is licensed under a BSD license. See the LICENSE file for details.

# imx6ull-flash

This library gives abstraction layer for i.MX 6ULL NAND memory controller.

  typedef struct _flashdrv_dma_t flashdrv_dma_t;

Structure holding internal context of flash DMA.


	enum {
	  flash_reset = 0, flash_read_id, flash_read_parameter_page, flash_read_unique_id,
	  flash_get_features, flash_set_features, flash_read_status, flash_read_status_enhanced,
	  flash_random_data_read, flash_random_data_read_two_plane, flash_random_data_input,
	  flash_program_for_internal_data_move_column, flash_read_mode, flash_read_page,
	  flash_read_page_cache_sequential, flash_read_page_cache_random, flash_read_page_cache_last,
	  flash_program_page, flash_program_page_cache, flash_erase_block,
	  flash_read_for_internal_data_move, flash_program_for_internal_data_move,
	  flash_block_unlock_low, flash_block_unlock_high, flash_block_lock, flash_block_lock_tight,
	  flash_block_lock_read_status, flash_otp_data_lock_by_block, flash_otp_data_program,
	  flash_otp_data_read, flash_num_commands
  	};

List of operations which can be issued to the NAND controler.


	typedef struct _flashdrv_meta_t {
	  char metadata[16];
	  char errors[9];
	} flashdrv_meta_t;

Structure for holding NAND block metadata.


	extern flashdrv_dma_t *flashdrv_dmanew(void);

flashdrv_dma_t initializer.


    extern void flashdrv_dmadestroy(flashdrv_dma_t *dma);

flashdrv_dma_t deinitializer.


    extern int flashdrv_reset(flashdrv_dma_t *dma);

This function resets the NAND chip.


    extern int flashdrv_write(flashdrv_dma_t *dma, uint32_t paddr, void *data, char *metadata);

This function writes one page of data to the NAND.


    extern int flashdrv_read(flashdrv_dma_t *dma, uint32_t paddr, void *data, flashdrv_meta_t *meta);

This function reads one page of data from the NAND.


    extern int flashdrv_erase(flashdrv_dma_t *dma, uint32_t paddr);

This function erases one block of the NAND.


    extern int flashdrv_writeraw(flashdrv_dma_t *dma, uint32_t paddr, void *data, int sz);

Analogue to flashdrv_write, but ignores metadata.


    extern int flashdrv_readraw(flashdrv_dma_t *dma, uint32_t paddr, void *data, int sz);

Analogue to flashdrv_read, but ignores metadata.


    extern void flashdrv_init(void);

Library and NAND controler initialization.

