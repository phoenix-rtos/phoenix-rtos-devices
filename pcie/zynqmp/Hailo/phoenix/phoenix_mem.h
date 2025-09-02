#ifndef PHOENIX_MEM
#define PHOENIX_MEM

#include "../../core/pci_utils.h"
#include "phoenix_vm.h"
#include "phoenix_scatter.h"
#include <stdint.h>
#include <stdbool.h>

#define DMA_ALIGMENT 4096


#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define __ALIGN_MASK(x, mask) (((x) + (mask)) & ~(mask))
#define ALIGN(x, a)           __ALIGN_MASK(x, (typeof(x))(a) - 1)

#define DMA_BIT_MASK(a) 1

void *dma_alloc_coherent(struct device *dev, size_t buffer_size, dma_addr_t *dma_address, int flags);
void dma_free_coherent(struct device *dev, size_t buffer_size, void *addr, dma_addr_t dma_handle);

void *vmalloc(size_t size);
void *kmalloc(size_t size, int flags);

void *kvmalloc_array(size_t n, size_t size, gfp_t flags);

struct page *vmalloc_to_page(void *v_addr);

void *kzalloc(size_t size, int flag);

int dma_set_mask_and_coherent(struct device *dev, int f);

void *devm_kmalloc_array(struct device *dev, size_t num, size_t size, int flags);

void *get_dma_ops(struct device *dev);

unsigned long virt_to_phys(void *add);

/* This function is used to map already allocated DMA capable buffer to the user space */
int dma_mmap_coherent(struct device *dev, struct vm_area_struct *vma, void *cpu_addr, dma_addr_t dma_addr, size_t size);

/* Since this is only book-keeping function used by Linux we can skip this  */
int remap_pfn_range(struct vm_area_struct *vma, unsigned long vm_start, unsigned long a, int b, int c);

void kfree(void *add);


bool is_kmalloc_dma_capable(struct device *dev);

#endif
