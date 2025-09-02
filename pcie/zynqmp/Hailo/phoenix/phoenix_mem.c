#include "phoenix_mem.h"
#include <sys/mman.h>
#include <stdlib.h>
#include <string.h>

#include "phoenix_log.h"


/* 
 This function allocates buffer_size data via mmap, that is alligned to 4KiB
 It also keeps a bit of trailling data at the end of the buffer since
 whole page has to be mapped. It will deallocate preceiding part, which is not
 alligned, and trailing part which is redundant.

 Also, dma_alloc_coherent has to provide address that is to be translated via IOMMU.
 Since for current implementation, no bus to physcial translation is enabled.
*/
void *dma_alloc_coherent(struct device *dev, size_t buffer_size, dma_addr_t *dma_address, int flags)
{
    /* Ignore device and flags */
	(void)dev;
    (void)flags;

    void *temp = mmap(NULL, buffer_size + DMA_ALIGMENT, PROT_READ | PROT_WRITE, 
        MAP_CONTIGUOUS | MAP_ANONYMOUS | MAP_UNCACHED, -1, (off_t)0);

    /* 
     Since DMA alligment is quite huge, bigger than page size
     unmapping range from dma_address_base to dma_address_alligned
     should not unmap any of the buffer that is relevant
     to us (dma_address_alligned should be at beggining of the page anyways).
    */

    if(NULL == temp){
        pr_err("mmap failed: %s:%s\n", __FILE__, __func__);
    }

    uint64_t dma_address_base = (uint64_t)va2pa(temp);
    uint64_t dma_addres_alligned = ALIGN(dma_address_base, DMA_ALIGMENT);

    size_t to_deallocate_head = dma_addres_alligned - dma_address_base;

    uint64_t tail = (uint64_t)(dma_addres_alligned) + buffer_size;

    /* Allign to make sure not to unmap used page */
    uint64_t tail_to_trim = ALIGN(tail, DMA_PAGE_SIZE);

    munmap((void *)temp, to_deallocate_head);
    munmap((void *)tail_to_trim, 4096 - to_deallocate_head);

    *dma_address = dma_addres_alligned;

    return (void *)((uint64_t)temp + to_deallocate_head);
}


void dma_free_coherent(struct device *dev, size_t buffer_size, void *addr, dma_addr_t dma_handle)
{
    (void)dev;
    (void)dma_handle;

    munmap(addr, buffer_size);
}


void *vmalloc(size_t size)
{
	return malloc(size);
}

void *kmalloc(size_t size, int flags)
{
	return malloc(size);
}

void *kvmalloc_array(size_t n, size_t size, gfp_t flags)
{
	(void)flags; //ignore
    return vmalloc(n * size);
}


struct page *vmalloc_to_page(void *v_addr)
{
	/* This function shall convert virtual address into pointer to a structure page structure */
    /* We have no way to check how much of this "page" - block of 512 bytes is used by the buffer with v_addr */
    struct page *p = (struct page *)malloc(sizeof(struct page));

    if(NULL == p){
        pr_err("malloc failed: %s\n", __func__);
    }
    
    p->virt_add = v_addr;
    p->size = DMA_PAGE_SIZE;

    return p;
}

void *kzalloc(size_t size, int flag)
{
	void *ret = malloc(size);
    if(NULL == ret){
        return ret;
    }

    memset(ret, 0, size);
    return ret;
}

int dma_set_mask_and_coherent(struct device *dev, int f)
{
	return 0;
}

void *devm_kmalloc_array(struct device *dev, size_t num, size_t size, int flags)
{
    /* Check for overflow */
    if((size_t)(~0ul) / num < size){
        return NULL;
    }
	return malloc(num * size);
}

void *get_dma_ops(struct device *dev)
{
	return NULL;
}

unsigned long virt_to_phys(void *add)
{
    (unsigned long)va2pa(add);
}


int dma_mmap_coherent(struct device *dev,struct vm_area_struct *vma, void *cpu_addr, dma_addr_t dma_addr, size_t size)
{
    vma->dummy = cpu_addr;
    vma->vm_start = (unsigned long)cpu_addr;
    vma->vm_end = vma->vm_start + size;
    return 0;
}

int remap_pfn_range(struct vm_area_struct *vma, unsigned long vm_start, unsigned long a, int b, int c)
{
	return 0;
}

bool is_kmalloc_dma_capable(struct device *dev)
{
	return false;
}


void kfree(void *add){
    free(add);
}