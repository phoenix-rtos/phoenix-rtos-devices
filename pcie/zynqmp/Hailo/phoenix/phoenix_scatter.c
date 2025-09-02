#include "phoenix_scatter.h"
#include <string.h>
#include <sys/mman.h>

#include "phoenix_log.h"

/* Function allocates actual scatterlist table and populates it */
int sg_alloc_table_from_pages(struct sg_table *sgt, struct page **pages,
		unsigned int n_pages, unsigned long offset, unsigned long size, gfp_t gfp_mask)
{
	(void)offset;
	(void)size;
	(void)gfp_mask;
	/* This implementation is most, basic - can optimize by collapsing neighboring pages */

	sgt->nents = n_pages;
	sgt->sgl = (struct scatterlist *)malloc(sizeof(struct scatterlist) * n_pages);

	if (NULL == sgt->sgl) {
		pr_err("malloc failed, %s:%s\n", __FILE__, __func__);
	}

	for (int i = 0; i < n_pages; i++) {
		/* Assing offset to  */
		sgt->sgl[i].p = pages[i];
	}

	sgt->nents = 0; /* This tells us how many entries have already been mapped */
	sgt->orig_nents = n_pages; /* This tells us how many entries there are */

	return 0;
}

unsigned int dma_map_sg(struct device *dev, struct scatterlist *table, unsigned int n_entries, int flags)
{
	/* Since we have physical address inside page */
	(void)flags;

	for (int i = 0; i < n_entries; i++) {
		table[i].dma_address = (uint64_t)va2pa(table[i].p->virt_add);
        table[i].length = table[i].p->size;
	}

	return 0;
}

void dma_unmap_sg(struct device *dev, struct scatterlist *table, unsigned int n_entries, int flag)
{
	/* In this use case we have one virtually contigous buffer - one free call should be sufficient */
	free(table[0].p->virt_add);
}

size_t sg_pcopy_from_buffer(struct scatterlist *table, unsigned int n_entries,
		const void *buffer, size_t buffer_size, off_t offset)
{
	/* Skip lists until offset */
	int i = 0;
	uint8_t *buf = (uint8_t *)buffer;
    size_t n_bytes_to_copy = buffer_size;

	/* Find page which has some memory after moving to offset */
	while (offset > table[i].length && i < n_entries) {
		offset -= table[i].length;
		i++;
	}

    if(i == n_entries - 1 && offset > table[i].length){
        /* We reached last sg_entry and still did not meet offset */
        return 0;
    }

	/* Now we are at sg_entry which has not been fully filled yet */
	/* Fill this entry and go further */
	size_t n_bytes = table->p->size - offset > buffer_size ? buffer_size : table->p->size - offset;
	memcpy(((uint8_t *)table[i].p->virt_add + offset), buf, n_bytes);
	buffer_size -= n_bytes;
	buf += n_bytes;
    i++;

    /* While there is data left in input buffer and we have sg_entry copy data */
    while(buffer_size > 0 && i < n_entries){
        n_bytes = table[i].p->size > buffer_size ? buffer_size : table[i].p->size;
        memcpy(table[i].p->virt_add, buf, n_bytes);
        buffer_size -= n_bytes;
        buf += n_bytes;
        i++;
    }

    /* Return how much data was copied */
    return n_bytes_to_copy - buffer_size;
}

inline struct scatterlist *sg_next(struct scatterlist *prev)
{
	if(prev->is_last){
		return NULL;
	}
	if(prev->is_chain){
		return sg_chain_ptr(prev);
	}
	return prev + 1;
}

void sg_free_table(struct sg_table *sg)
{
	free(sg->sgl);
	sg->sgl = NULL;
	sg->nents = 0;
	sg->orig_nents = 0;
}
