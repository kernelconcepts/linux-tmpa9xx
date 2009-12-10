//typedef unsigned long a32; 

// mem sizes we are dealing with
#define CACHELINESIZE       0x20
#define CACHELINEMASK       0x1F
#define SRAMSIZE      (8*1024)

// used constants
#define ALLOC_BITMAP_FREE    0
#define ALLOC_BITMAP_USED    1
#define ALLOC_BITMAP_LAST    3

// with byte bitmap we need 512 bytes versus 32 byte for 2 bit entry
// additionaly code for 2bit entry will exceed savings in array size
#define ALLOC_BITMAP_SIZE (SRAMSIZE/CACHELINESIZE)

static unsigned char g_sram_alloc_bitmap[ALLOC_BITMAP_SIZE];
static a32 g_sram_base = 0xFFFFFFFF;
static a32 g_sram_phys = 0xFFFFFFFF;
/*
 * init allocator
 * set all to free, init base
 *
 */

static spinlock_t sram_lock;

// convert virt sram address to phys
static unsigned int tmpa9x0_sram_to_phys(void *virt_sram)
{
	return ((unsigned int) virt_sram - (unsigned int) g_sram_base  ) + g_sram_phys;
}


static void tmpa9x0_sram_init(a32 sram_base, a32 phys_base){
  int i;

  if (g_sram_base != 0xFFFFFFFF)
    return;


	g_sram_phys = phys_base;

  // set global base
  g_sram_base=sram_base;

	spin_lock_init(&sram_lock);

  // set global alloc bitmap
  for( i = 0 ; i < ALLOC_BITMAP_SIZE ; i++)
    g_sram_alloc_bitmap[i]=ALLOC_BITMAP_FREE;

  // done
  return;
}

/*
 * alloc from sram pool with given alignment
 * minimum size is CACHELINESIZE with CACHELINE align
 * NOTE: time to find a free memory range raises with
 *       bitmap size/and !free entries
 */
static a32 tmpa9x0_sram_alloc(unsigned long size, unsigned long align){
  unsigned long s_current,step,needed,i,last;
	unsigned long	flags;

  // init?
  if( g_sram_base == 0xFFFFFFFF )
    return 0;

  // check size
  if( size == 0 )
    return 0;

  else if( size & CACHELINEMASK )
    size = (size + CACHELINEMASK) & ~CACHELINEMASK;

  // see your local sram dealer first
  if( size  > SRAMSIZE )
    return 0;

  //check align
  if( align == 0)
    align = CACHELINESIZE;
  else if( align & CACHELINEMASK )
    align = (align + CACHELINEMASK) & ~CACHELINEMASK;

  // assume sram_base is aligned to SRAMSIZE, we can allways align within SRAMSIZE
  if( align > SRAMSIZE )
    return 0;

  // scan bitmap for a valid/free block
  step   = align/CACHELINESIZE;
  needed = size/CACHELINESIZE;
  last   = ALLOC_BITMAP_SIZE - needed + 1 ;

	spin_lock_irqsave (&sram_lock, flags);

  for( s_current = 0 ; s_current < last ; s_current += step ){
    // check free
    for( i = 0; i < needed ; i++ )
    {
      if( g_sram_alloc_bitmap[s_current + i] != ALLOC_BITMAP_FREE )
      {
         goto next_block;
      }
    }
    // mark used
    for( i = 0; i < needed ; i++ ){
      g_sram_alloc_bitmap[s_current + i] = ALLOC_BITMAP_USED;
    }
    g_sram_alloc_bitmap[s_current + i - 1 ] = ALLOC_BITMAP_LAST;

    // return result
   // NPRINTK("return 0x%8.8x, s_current=%d, i=%d\n", (g_sram_base + s_current*CACHELINESIZE), s_current, i);
		spin_unlock_irqrestore (&sram_lock, flags);

    return (g_sram_base + s_current*CACHELINESIZE);
next_block:;
  }

#if 0
	if (printk_ratelimit())
		printk("Out of SRAM %d\n", size);
#endif
	spin_unlock_irqrestore (&sram_lock, flags);
	return 0;
}

/*
 * free block allocated from sram pool
 *
 */
static void tmpa9x0_sram_free(a32 block){
  unsigned long tofree,i,size;
	unsigned long	flags;
  //NPRINTK("block=%p\n",block);

  // zero block
  if( !block )
    return;
  
  // cheating address
  if( block & CACHELINEMASK )
    return;
  
  // out of range (low)
  if( block < g_sram_base )
    return;
  
  // out of range (hi)
  if( (tofree = ((block - g_sram_base)/CACHELINESIZE)) >= ALLOC_BITMAP_SIZE )
    return;
  
  // check bitmap valid at start pos
  if( tofree && (g_sram_alloc_bitmap[tofree-1] == ALLOC_BITMAP_USED ))
    return;
    

spin_lock_irqsave (&sram_lock, flags);

  // find size of block
  for( size = 0 , i = tofree ; i < ALLOC_BITMAP_SIZE ; i++ ){
    // done bitmap corrupted ?!
    if( g_sram_alloc_bitmap[i] == ALLOC_BITMAP_FREE )
      goto end;
    // used entry
    if( g_sram_alloc_bitmap[i] == ALLOC_BITMAP_USED )
      size++;
    // last entry
    if( g_sram_alloc_bitmap[i] == ALLOC_BITMAP_LAST ){
      size++;
      // mark free
      for( i = 0; i < size ; i++ )
				g_sram_alloc_bitmap[tofree + i] = ALLOC_BITMAP_FREE;
      // done OK

      goto end;
    }
  }
  
end:
	spin_unlock_irqrestore (&sram_lock, flags);
}
