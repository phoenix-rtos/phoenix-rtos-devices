#ifndef _IMX6ULL_USB_HOST_DMA_H_
#define _IMX6ULL_USB_HOST_DMA_H_


typedef struct dma_buf {
	union {
		struct {
			struct dma_buf *next;
			unsigned freesz;
			void **free;
		};
		char padding[64];
	};

	char start[];
} dma_buf_t;


extern void dma_free64(void *ptr);


extern void *dma_alloc64(void);

#endif
