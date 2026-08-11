// SPDX-License-Identifier: GPL-2.0-only
/*
 * jagar_fbcon - panel console for the Daylight DC-1 (jagar, MT8781).
 *
 * The device has no accessible UART and no working USB gadget yet, so the only
 * way to read kernel output is to photograph the panel. LK initialises the DSI
 * panel and leaves a framebuffer scanning out at a fixed physical address; that
 * scanout keeps running as long as nothing reprograms the display controller.
 * This console renders printk text straight into that buffer.
 *
 * The framebuffer lives at mblock-13-framebuffer in LK's generated DT. That
 * reservation has no "no-map", so the range stays in the kernel linear map and
 * must be reached with __va() -- ioremap() of linear-mapped RAM is refused.
 * Because the mapping is cacheable but the display controller reads DRAM
 * directly, every glyph is cleaned to the point of coherency after it is drawn.
 *
 * Deliberately independent of simpledrm/fbcon: those need a simple-framebuffer
 * DT node and a working ioremap of the range, neither of which holds under the
 * stock LK device tree.
 */

#include <linux/console.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <asm/cacheflush.h>

/* Geometry from DEVICE.md: atag,videolfb-fb_base_l and the vramSize arithmetic. */
#define JAGAR_FB_PHYS		0xfe8c1000UL
#define JAGAR_FB_WIDTH		1200
#define JAGAR_FB_HEIGHT		1600
#define JAGAR_FB_STRIDE		4864		/* bytes: (1200 + 16 pad) * 4 */
#define JAGAR_FB_BPP		4

#define JAGAR_FG			0xffffffffu	/* a8r8g8b8 white */
#define JAGAR_BG			0xff000000u	/* a8r8g8b8 black */

#define GLYPH_W			8
#define GLYPH_H			16
#define JAGAR_COLS		(JAGAR_FB_WIDTH / GLYPH_W)	/* 150 */
#define JAGAR_ROWS		(JAGAR_FB_HEIGHT / GLYPH_H)	/* 100 */

static u8 *jagar_fb;
static unsigned int jagar_col, jagar_row;

void jagar_fbcon_puts(const char *s);

static void jagar_fb_clear_row(unsigned int row)
{
	unsigned int y;

	if (row >= JAGAR_ROWS)
		return;

	for (y = 0; y < GLYPH_H; y++) {
		unsigned int dy = JAGAR_FB_HEIGHT - 1 - (row * GLYPH_H + y);
		u8 *line = jagar_fb + dy * JAGAR_FB_STRIDE;
		u32 *px = (u32 *)line;
		unsigned int x;

		for (x = 0; x < JAGAR_FB_WIDTH; x++)
			px[x] = JAGAR_BG;

		dcache_clean_poc((unsigned long)line,
				 (unsigned long)line + JAGAR_FB_WIDTH * JAGAR_FB_BPP);
	}
}

static void jagar_fb_draw_glyph(unsigned int row, unsigned int col, unsigned char c)
{
	const u8 *glyph = font_data_buf(font_vga_8x16.data) + c * GLYPH_H;
	unsigned int y;

	/*
	 * The panel is mounted 180 degrees from the framebuffer's origin, so
	 * every pixel is written to (W-1-x, H-1-y). Without this the log comes
	 * out upside down and mirrored on the screen.
	 */
	for (y = 0; y < GLYPH_H; y++) {
		unsigned int dy = JAGAR_FB_HEIGHT - 1 - (row * GLYPH_H + y);
		unsigned int dx_end = JAGAR_FB_WIDTH - 1 - col * GLYPH_W;
		u8 *line = jagar_fb + dy * JAGAR_FB_STRIDE;
		u32 *px = (u32 *)line;
		u8 bits = glyph[y];
		unsigned int x;

		for (x = 0; x < GLYPH_W; x++)
			px[dx_end - x] = (bits & (0x80u >> x)) ? JAGAR_FG : JAGAR_BG;

		dcache_clean_poc((unsigned long)(px + dx_end - (GLYPH_W - 1)),
				 (unsigned long)(px + dx_end + 1));
	}
}

static void jagar_fb_newline(void)
{
	jagar_col = 0;
	jagar_row++;
	if (jagar_row >= JAGAR_ROWS)
		jagar_row = 0;

	/*
	 * Wrap instead of scrolling: shifting 7.4 MiB per line would be far too
	 * slow against the watchdog. Clearing one row ahead leaves a visible gap
	 * marking where the newest text ends and the oldest begins.
	 */
	jagar_fb_clear_row(jagar_row);
	jagar_fb_clear_row((jagar_row + 1) % JAGAR_ROWS);
}

static void jagar_fb_putc(unsigned char c)
{
	if (c == '\n') {
		jagar_fb_newline();
		return;
	}
	if (c == '\r')
		return;
	if (c == '\t') {
		do {
			jagar_fb_putc(' ');
		} while (jagar_col % 8);
		return;
	}
	if (c < 32 || c > 126)
		c = '?';

	if (jagar_col >= JAGAR_COLS)
		jagar_fb_newline();

	jagar_fb_draw_glyph(jagar_row, jagar_col, c);
	jagar_col++;
}

/*
 * Second channel: mirror the same text into ramoops' console zone so it can be
 * read back over ADB from stock slot A after the watchdog reset, without anyone
 * having to photograph the panel.
 *
 * LK's bootargs give ramoops.mem_address=0x48090000 mem_size=0xe0000
 * console_size=0x80000 pmsg_size=0x10000. ramoops lays out dmesg records first
 * (mem_size - console_size - pmsg_size = 0x50000), so the console zone starts at
 * 0x480e0000. A zone is a persistent_ram_buffer: {sig, start, size, data[]},
 * with sig = 0 ^ PERSISTENT_RAM_SIG. ECC is off (no ramoops.ecc on the cmdline),
 * so the data needs no parity bytes.
 *
 * This deliberately does not use the pstore driver: ramoops probes far later
 * than boot currently survives. Text accumulates in a BSS buffer and is flushed
 * once ioremap is usable -- the zone is "no-map", so it is absent from the
 * linear map and __va() would not work here (the opposite of the framebuffer).
 */
#define JAGAR_PSTORE_CONSOLE_PHYS	0x480e0000UL
#define JAGAR_PSTORE_CONSOLE_SIZE	0x80000UL
#define PERSISTENT_RAM_SIG		0x43474244u	/* DBGC */
#define JAGAR_LOG_MAX			(64 * 1024)

struct jagar_prb {
	u32 sig;
	u32 start;
	u32 size;
	u8 data[];
};

static char jagar_log[JAGAR_LOG_MAX];
static unsigned int jagar_log_len;
static void __iomem *jagar_prz;		/* full zone, valid once slab is up */
static void __iomem *jagar_prz_early;	/* 4K early_ioremap window */
static unsigned int jagar_prz_limit;

/*
 * Map a 4K window of the zone before the allocators exist. setup_arch() runs
 * long before mm_init(), so ioremap() is unavailable exactly where boot is
 * currently dying; early_ioremap works there (it already maps the watchdog).
 */
void __init jagar_pstore_early_init(void)
{
	jagar_prz_early = early_ioremap(JAGAR_PSTORE_CONSOLE_PHYS, SZ_4K);
	jagar_prz_limit = SZ_4K - sizeof(struct jagar_prb);
}

/* early_ioremap_reset() invalidates the window; stop using it. */
void __init jagar_pstore_drop_early(void)
{
	jagar_prz_early = NULL;
	jagar_prz_limit = 0;
}

static void jagar_pstore_flush(void)
{
	struct jagar_prb __iomem *prb;
	unsigned int len = jagar_log_len;
	unsigned int limit;

	if (jagar_prz_early) {
		prb = (struct jagar_prb __iomem *)jagar_prz_early;
		limit = jagar_prz_limit;
	} else {
		if (!jagar_prz) {
			/* ioremap needs the allocators; slab is a safe gate. */
			if (!slab_is_available())
				return;
			jagar_prz = ioremap(JAGAR_PSTORE_CONSOLE_PHYS,
					    JAGAR_PSTORE_CONSOLE_SIZE);
			if (!jagar_prz)
				return;
		}
		prb = (struct jagar_prb __iomem *)jagar_prz;
		limit = JAGAR_PSTORE_CONSOLE_SIZE - sizeof(*prb);
	}

	if (len > limit)
		len = limit;

	memcpy_toio(prb->data, jagar_log, len);
	writel(len, &prb->size);
	writel(len, &prb->start);
	writel(PERSISTENT_RAM_SIG, &prb->sig);
}

static void jagar_log_putc(unsigned char c)
{
	if (jagar_log_len < JAGAR_LOG_MAX)
		jagar_log[jagar_log_len++] = c;
}

/*
 * Breadcrumb trail that does not go through printk. During setup_arch the
 * console is not registered yet, so pr_info() output sits in printk's ring
 * where this code cannot reach it; writing here instead means the checkpoint
 * sequence reaches ramoops even if boot never gets as far as console_init().
 */
void jagar_note_checkpoint(unsigned int n)
{
	char buf[16];
	int i = 0;

	if (n >= 100)
		buf[i++] = '0' + (n / 100) % 10;
	if (n >= 10)
		buf[i++] = '0' + (n / 10) % 10;
	buf[i++] = '0' + n % 10;
	buf[i] = '\0';

	jagar_fbcon_puts("jagar cp ");
	jagar_fbcon_puts(buf);
	jagar_fbcon_puts("\n");
	jagar_pstore_flush();
}

/* Called from each watchdog checkpoint so the ring tracks how far boot got. */
void jagar_pstore_checkpoint(void)
{
	jagar_pstore_flush();
}

void jagar_fbcon_puts(const char *s)
{
	while (*s) {
		jagar_log_putc(*s);
		if (jagar_fb)
			jagar_fb_putc(*s);
		s++;
	}
}

/*
 * Map the framebuffer and paint a marker. Called from setup_arch() once the
 * linear map exists, so a photograph shows whether the kernel got this far even
 * if console registration never happens.
 */
void __init jagar_fbcon_early_init(void)
{
	unsigned int row;

	jagar_fb = (u8 *)__va(JAGAR_FB_PHYS);

	for (row = 0; row < JAGAR_ROWS; row++)
		jagar_fb_clear_row(row);

	jagar_col = 0;
	jagar_row = 0;
	jagar_fbcon_puts("jagar: framebuffer console up\n");
}

static void jagar_console_write(struct console *co, const char *s,
				unsigned int count)
{
	while (count--) {
		jagar_log_putc(*s);
		if (jagar_fb)
			jagar_fb_putc(*s);
		s++;
	}
	jagar_pstore_flush();
}

static struct console jagar_console = {
	.name	= "jagarfb",
	.write	= jagar_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

/*
 * console_initcall, not early_initcall: console_init() runs from start_kernel
 * itself, whereas early initcalls only run later from the init thread. Boot
 * currently dies inside start_kernel, so an early_initcall would never fire.
 *
 * CON_PRINTBUFFER makes printk replay everything buffered since boot, so
 * registering here still yields the full log from the very first message.
 */
static int __init jagar_fbcon_register(void)
{
	if (jagar_fb)
		register_console(&jagar_console);
	return 0;
}
console_initcall(jagar_fbcon_register);
