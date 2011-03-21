/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sysrq.h>

#undef SRAM_DEBUG

#ifdef SRAM_DEBUG
#define mprintk printk
#else
#define mprintk(...)
#endif

static void *g_virt;
static void *g_phys;

struct memory_item
{
	unsigned int size;
	void *virt;
	unsigned char in_use;
};

#define MAX_NUM_ITEMS 256 /* 64 */

static struct memory_item memory_map[MAX_NUM_ITEMS];
static spinlock_t sram_lock;

unsigned long tmpa9xx_sram_to_phys(void *virt_sram)
{
	return ((unsigned int)virt_sram - (unsigned int)g_virt  ) + (unsigned int)g_phys;
}

EXPORT_SYMBOL(tmpa9xx_sram_to_phys);

static void tmpa9xx_sram_print_stats(void)
{
	unsigned long flags;
	int i;
	void *adr;

	spin_lock_irqsave (&sram_lock, flags);

	adr = g_virt;
	for (i = 0; i < MAX_NUM_ITEMS; i++) {
		struct memory_item *m = &memory_map[i];
		if (!m->size)
			break;
		printk("%3d: %4d|%d @ %p\n", i, m->size, m->in_use, m->virt);
		adr += m->size;
	}
	printk("%d bytes free in sram\n", (g_virt + 8192) - adr);

	spin_unlock_irqrestore (&sram_lock, flags);
}

void *tmpa9xx_sram_alloc(int o_size)
{
	unsigned long flags;
	struct memory_item *m;
	void *virt = NULL;
	int i;
	int n_size;

	n_size = (((o_size - 1) / 32) + 1) * 32;

	spin_lock_irqsave (&sram_lock, flags);

	for (i = 0; i < MAX_NUM_ITEMS; i++) {
		m = &memory_map[i];

		if (!m->size)
			break;
		if (m->in_use)
			continue;
		if (m->size != n_size)
			continue;

		m->in_use = 1;
		virt = m->virt;
		break;
	}
	if (i == MAX_NUM_ITEMS)
	{
		printk("increase MAX_NUM_ITEMS\n");
		virt = NULL;
		goto out;
	}

	/* try allocate new item */
	if (!m->size) {
		void *adr = g_virt;
		if (i) {
			adr = memory_map[i-1].virt + memory_map[i-1].size;
		}
		if (adr + n_size > g_virt + 8192) {
			/* out of memory */
			printk("out of memory for size %d\n", n_size);
			virt = NULL;
			goto out;
		}

		mprintk("allocating new item @ %2d with size %d @ adr %p (original size %d)\n", i, n_size, adr, o_size);
		m = &memory_map[i];
		m->virt = adr;
		m->size = n_size;
		m->in_use = 1;
		virt = m->virt;

	}

	mprintk("using item %2d, size %4d @ virt %p\n", i, memory_map[i].size, virt);

out:
	spin_unlock_irqrestore (&sram_lock, flags);
	return virt;
}

EXPORT_SYMBOL(tmpa9xx_sram_alloc);

void tmpa9xx_sram_free(void *virt)
{
	unsigned long flags;
	int i, j;
	int last_in_use;

	spin_lock_irqsave (&sram_lock, flags);

	for (i = 0; i < MAX_NUM_ITEMS; i++) {
		struct memory_item *m = &memory_map[i];

		if (m->virt != virt)
			continue;

		mprintk("clr'g item %2d, size %4d @ virt %p\n", i, m->size, virt);

		m->in_use = 0;
		break;
	}
	BUG_ON(i == MAX_NUM_ITEMS);

	/* erase all unused items after the last item in use */
	last_in_use = -1;
	for (i = 0; i < MAX_NUM_ITEMS; i++) {
		struct memory_item *m = &memory_map[i];
		if(m->in_use)
			last_in_use = i;
		if (!m->size)
			break;
	}
	last_in_use++;
	if(last_in_use != i) {
		mprintk("flushing items %d to %d\n", last_in_use, i);
		for(j = last_in_use; j < i; j++) {
			struct memory_item *m = &memory_map[j];
			m->size = 0;
		}
	}

	spin_unlock_irqrestore (&sram_lock, flags);
}

EXPORT_SYMBOL(tmpa9xx_sram_free);

#ifdef CONFIG_MAGIC_SYSRQ
static void sysrq_handle_tmpa(int key)
{
	tmpa9xx_sram_print_stats();
}

static struct sysrq_key_op sysrq_tmpa_op = {
	.handler	= sysrq_handle_tmpa,
	.help_msg	= "tmpa(A)",
	.action_msg	= "DEBUG",
};
#endif

#define SRAM_PHYS 0xF8008000
#define SRAM_LEN  0x2000

static int __init tmpa9xx_sram_init(void)
{
	g_phys = (void *)SRAM_PHYS;

	g_virt = ioremap(SRAM_PHYS, SRAM_LEN);
	if (!g_virt) {
		pr_err("ioremap() failed\n");
		return -ENOMEM;
	}
	spin_lock_init(&sram_lock);

#ifdef CONFIG_MAGIC_SYSRQ
	register_sysrq_key('a', &sysrq_tmpa_op);
#endif

	return 0;
}

arch_initcall(tmpa9xx_sram_init);

