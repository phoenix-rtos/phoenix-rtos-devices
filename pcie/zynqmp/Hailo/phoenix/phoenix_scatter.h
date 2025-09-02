#ifndef PHOENIX_SCATTER
#define PHOENIX_SCATTER
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include "phoenix_comp.h"

/* Here we need to calculate shift from page size */
#define PAGE_SHIFT 12
#define DMA_PAGE_SIZE 512//_PAGE_SIZE


struct page
{
    void *virt_add;
    uint64_t size;
};


struct scatterlist {
	struct page *p;
	unsigned int	length;
	bool is_last;
	bool is_chain;
	dma_addr_t	dma_address;
};


#define sg_dma_address(sg)	((sg)->dma_address)
#define sg_dma_len(sg)		((sg)->length)


struct sg_table {
	struct scatterlist *sgl;	/* the list */
	unsigned int nents;		/* number of mapped entries */
	unsigned int orig_nents;	/* original size of list */
};


#define sg_is_chain(sg)		((sg)->is_chain)
#define sg_is_last(sg)		((sg)->is_last)
#define sg_chain_ptr(sg)	\
	((struct scatterlist *) ((struct scatterlist *)(sg)->p))

	
#define for_each_sg(sglist, sg, nr, __i)	\
	for (__i = 0, sg = (sglist); __i < (nr); __i++, sg = sg_next(sg))



static inline void sg_chain(struct scatterlist *prv, unsigned int prv_nents,
			    struct scatterlist *sgl)
{
	prv[prv_nents - 1].length = 0;
	prv[prv_nents - 1].is_chain= true;
	prv[prv_nents - 1].p = (void *)sgl;
}


static inline void sg_mark_end(struct scatterlist *sg)
{
	sg->is_last = true;
}


static inline void sg_unmark_end(struct scatterlist *sg)
{
	sg->is_last = false;
}

unsigned int dma_map_sg(struct device *dev, struct scatterlist *table, unsigned int n_entries, int flags);

void dma_unmap_sg(struct device *dev, struct scatterlist *table, unsigned int n_entries, int flag);


struct scatterlist *sg_next(struct scatterlist *);

void sg_free_table(struct sg_table *);

int sg_alloc_table_from_pages(struct sg_table *sgt,
	struct page **pages, unsigned int n_pages,
	unsigned long offset, unsigned long size,
	gfp_t gfp_mask);

size_t sg_pcopy_from_buffer(struct scatterlist *sgl, unsigned int nents,
			    const void *buf, size_t buflen, off_t skip);
/*
 * Maximum number of entries that will be allocated in one piece, if
 * a list larger than this is required then chaining will be utilized.
 */
#define SG_MAX_SINGLE_ALLOC		(PAGE_SIZE / sizeof(struct scatterlist))
/*
 * The maximum number of SG segments that we will put inside a
 * scatterlist (unless chaining is used). Should ideally fit inside a
 * single page, to avoid a higher order allocation.  We could define this
 * to SG_MAX_SINGLE_ALLOC to pack correctly at the highest order.  The
 * minimum value is 32
 */
#define SG_CHUNK_SIZE	128
/*
 * Like SG_CHUNK_SIZE, but for archs that have sg chaining. This limit
 * is totally arbitrary, a setting of 2048 will get you at least 8mb ios.
 */
#ifdef CONFIG_ARCH_HAS_SG_CHAIN
#define SG_MAX_SEGMENTS	2048
#else
#define SG_MAX_SEGMENTS	SG_CHUNK_SIZE
#endif
#ifdef CONFIG_SG_POOL
void sg_free_table_chained(struct sg_table *table, bool first_chunk);
int sg_alloc_table_chained(struct sg_table *table, int nents,
			   struct scatterlist *first_chunk);
#endif

/* This is needed smh */
/*
 * sg page iterator
 *
 * Iterates over sg entries page-by-page.  On each successful iteration,
 * you can call sg_page_iter_page(@piter) and sg_page_iter_dma_address(@piter)
 * to get the current page and its dma address. @piter->sg will point to the
 * sg holding this page and @piter->sg_pgoffset to the page's page offset
 * within the sg. The iteration will stop either when a maximum number of sg
 * entries was reached or a terminating sg (sg_last(sg) == true) was reached.
 */
struct sg_page_iter {
	struct scatterlist	*sg;		/* sg holding the page */
	unsigned int		sg_pgoffset;	/* page offset within the sg */
	/* these are internal states, keep away */
	unsigned int		__nents;	/* remaining sg entries */
	int			__pg_advance;	/* nr pages to advance at the
						 * next step */
};
bool __sg_page_iter_next(struct sg_page_iter *piter);
void __sg_page_iter_start(struct sg_page_iter *piter,
			  struct scatterlist *sglist, unsigned int nents,
			  unsigned long pgoffset);
/**
 * sg_page_iter_page - get the current page held by the page iterator
 * @piter:	page iterator holding the page
 */
static inline struct page *sg_page_iter_page(struct sg_page_iter *piter)
{
	return NULL;//nth_page(sg_page(piter->sg), piter->sg_pgoffset);
}
/**
 * sg_page_iter_dma_address - get the dma address of the current page held by
 * the page iterator.
 * @piter:	page iterator holding the page
 */
static inline dma_addr_t sg_page_iter_dma_address(struct sg_page_iter *piter)
{
	return sg_dma_address(piter->sg) + (piter->sg_pgoffset << PAGE_SHIFT);
}
/**
 * for_each_sg_page - iterate over the pages of the given sg list
 * @sglist:	sglist to iterate over
 * @piter:	page iterator to hold current page, sg, sg_pgoffset
 * @nents:	maximum number of sg entries to iterate over
 * @pgoffset:	starting page offset
 */
#define for_each_sg_page(sglist, piter, nents, pgoffset)		   \
	for (__sg_page_iter_start((piter), (sglist), (nents), (pgoffset)); \
	     __sg_page_iter_next(piter);)
#endif 