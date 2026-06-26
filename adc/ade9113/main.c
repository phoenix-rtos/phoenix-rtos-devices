/*
 * Phoenix-RTOS
 *
 * ADC driver continuously reading samples from 4 chained ADE9113
 *
 * This code was written for specific HW configuration. This is included here to serve as an example / base that can
 * be adapted to work with other platforms.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <sysexits.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <stdlib.h>

#include <signal.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/time.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <posix/utils.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include <phoenix/arch/armv7a/imx6ull/imx6ull.h>

#include <imx6ull-ecspi.h>

#define LOG_TAG "ADE9113-DRV: "

#include "log.h"
#include "dma.h"
#include "sample.h"
#include "msgapi.h"
#include "imx6ull.h"
#include "ade9113.h"
#include "ade9113_regs.h"
#include "iomux_mux_enums.h"


#define SAMPLE_SIZE 32


/* value written to SCRATCH registers to verify reads (arbitrary unique value for each ADC chip) */
static const uint8_t scratchValues[4] = { 0xa0, 0xa1, 0xa2, 0xa3 };


struct adcCtx {
	volatile struct imx6ull_regs_ecspi *spi;

	volatile unsigned rxCount;
	volatile unsigned crcErrors;
	volatile unsigned valueErrors;

	struct sampleCtx samples;
	struct msgapiCtx msgapi;
};


static struct adcCtx common_adc;


/* Basic PWM3 configuration that generates 16.5 MHz (IPG_CLK / 4) */
static int initPwm(void)
{
	volatile struct imx6ull_regs_pwm *pwm = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, IMX_PWM3_BASE);
	if (pwm == MAP_FAILED) {
		printf("Failed to map: %p\n", pwm);
		return -1;
	}

	/* enable pwm clock */
	platformctl(&(platformctl_t) {
		.action = pctl_set,
		.type = pctl_devclock,
		.devclock.dev = pctl_clk_pwm3,
		.devclock.state = 3 });

	pwm->PWMCR = 0; /* disable PWM */

	/* 50% duty cycle - output frequency is half of the prescaller output */
	pwm->PWMPR = 0;
	pwm->PWMSAR = 1;

	const uint32_t clockSource_ipgClkHighfreq = 2;

	uint32_t clockSource = clockSource_ipgClkHighfreq;
	uint32_t prescaller = 1; /* divide IPG by 2 */
	uint32_t enable = 1;

	pwm->PWMCR = (0 | ((clockSource & 0x03) << 16) | ((prescaller & 0xfff) << 4) | ((enable & 0x01) << 0x00));

	munmap((void *)pwm, _PAGE_SIZE);
	return 0;
}


static void configureEcspi(volatile struct imx6ull_regs_ecspi *spi)
{
	/* defaults from ecspi_init */
	spi->CONREG = 0x007000F1;
	spi->CONFIGREG = 0x00000F00;

	const uint16_t burst = 16 * 4 * 8;
	spi->CONREG = (spi->CONREG & ~(0xFFF << 20)) | ((burst - 1) << 20);

	spi->CONFIGREG &= ~(0x01 << 12); /* set SS_POL[0] to invert SS on channel 0 (buffered with schmitt-trigger inverter) */
	spi->CONFIGREG &= ~(0x01 << 4);  /* set SCLK_POL[0] */
	spi->CONFIGREG |= (0x01 << 0);   /* set SCLK_PHA[0] */

	/*
	 * spi_freq = 66 Mhz / (pre + 1) * (post + 1)
	 * for 8k sample rate we need SPI clock >= 5 Mhz (8000 * 64 * 8 + margin)
	 *
	 * pre=4  post=0 -> ~13.2 Mhz (current value)
	 * pre=12 post=0 ->  ~5.0 Mhz (required minimum)
	 *
	 * For now using higher than needed to see if it is stable enough. Can be lowered later to improve robustness
	 */
	/* TODO: consider changing this to minimum once this is validated on semi-final HW in different conditions */
	uint8_t divPre = 4;
	uint8_t divPost = 0;
	ecspi_setClockDiv(ecspi1, divPre, divPost);
}


static void resetEcspi(volatile struct imx6ull_regs_ecspi *spi)
{
	/* reset ecspi1 */
	usleep(1000);
	spi->CONREG = 0;
	usleep(1000);

	configureEcspi(spi);
}


bool dmaRxCb(const uint8_t *data, size_t size)
{
	struct adcCtx *ctx = &common_adc;
	sample_writeStart(&ctx->samples);
	for (int sample = 0; sample < size / 64; ++sample) {
		sample_write(&ctx->samples, (const uint8_t *)"\x00\x00\x00\x00\x00", 5); /* header (aligns full sample to 32 bytes) */
		for (int chip = 0; chip < 4; ++chip) {
			/* response from first chip in chain is received last */
			size_t offs = (sample * 64) + (3 - chip) * 16;
			uint8_t rsp[16];
			for (int i = 0; i + 4 <= sizeof(rsp); i += 4) {
				uint8_t *dst = rsp + i;
				const uint8_t *src = data + offs + i;
				dst[0] = src[3];
				dst[1] = src[2];
				dst[2] = src[1];
				dst[3] = src[0];
			}

			const char *error = ade9113_checkResponse(rsp, sizeof(rsp));
			if (error != NULL) {
				ctx->crcErrors += 1;
				memset(rsp, 0, sizeof(rsp));
			}
			else if (rsp[13] != scratchValues[chip]) {
				ctx->valueErrors += 1;
				memset(rsp, 0, sizeof(rsp));
			}
			else {
			}
			sample_write(&ctx->samples, &rsp[1], 3);  // I LO,MID,HI
			sample_write(&ctx->samples, &rsp[5], 3);  // V1 LO,MID,HI
			if (chip == 3) {
				// TODO: make configurable
				sample_write(&ctx->samples, &rsp[9], 3);  // V2 LO,MID,HI
			}
		}
	}
	ctx->rxCount += size;
	sample_writeEnd(&ctx->samples);
	return true; /* true - keep reading samples */
}


static int spiExchangeCb(void *userData, const uint8_t *dataIn, uint8_t *dataOut, size_t len)
{
	return (ecspi_exchangeBusy(ecspi1, dataIn, dataOut, len) == 0) ? 0 : -1;
}


static void msgThread(void *voidCtx)
{
	struct adcCtx *ctx = (struct adcCtx *)voidCtx;
	msgapi_serve(&ctx->msgapi);
	endthread();
}

volatile sig_atomic_t pendingTerm = 0;


static void signalHandler(int signum)
{
	pendingTerm = 1;
}


struct args {
	unsigned verbosity;
	bool debug;
	enum ade9113_config0_stream_dbg dbgMode;
	enum ade9113_config_filt_datapath_config datapath;
	unsigned bufferSize; /* sample buffer size expressed as power of 2 */
	const char *configCrc;
};


struct strEnumDef {
	const char *name;
	unsigned value;
};

const struct strEnumDef modeEnumDefs[] = {
	{ "normal", (unsigned)ADE9113_CONFIG0_STREAM_DBG__NORMAL_MODE },
	{ "static", (unsigned)ADE9113_CONFIG0_STREAM_DBG__STATIC_MODE },
	{ "count", (unsigned)ADE9113_CONFIG0_STREAM_DBG__COUNT_MODE },
	{ NULL, 0 }
};


static int parseStrEnum(char opt, const char *value, const struct strEnumDef *defs)
{
	for (const struct strEnumDef *def = defs; def->name != NULL; ++def) {
		if (strcmp(def->name, value) == 0) {
			return def->value;
		}
	}
	log_error("value for option `-%c` is not recognised. Allowed options:", opt);
	for (const struct strEnumDef *def = defs; def->name != NULL; ++def) {
		fprintf(stderr, "* \"%s\"", def->name);
	}
	return -1;
}


static int parseIntEnum(char opt, const char *value, unsigned min, unsigned max)
{
	char *end = NULL;
	int result = strtoul(value, &end, 10);
	if (end == NULL || *end != '\0') {
		log_error("failed to parse int value for option `-%c`", opt);
		return -1;
	}
	if ((result < min) || (result > max) || (result > INT_MAX)) {
		log_error("value for option `-%c` is out of allowed range <%u;%u>", opt, min, max);
		return -1;
	}
	return result;
}


static void printUsage(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", (progname != NULL) ? progname : "prog");
	puts("\t-h             This help message");
	puts("\t-v             Increase verbosity");
	puts("\t-d             Debug mode");
	puts("\t-b [12-28]     Sample buffer size (power of 2)");
	puts("\t-m {normal,static,count} Sample stream mode");
	puts("\t-s [0-7]       ADE9113 datapath. Default is 4 (8 kHz + LPF + comp)");
	puts("\t-c <crc>       CRC values used to verify ADC config: `0123,4567,89ab,cdef`. Defaults to no CRC check");
}


static int parseArgs(struct args *parsed, int argc, char **argv)
{
	*parsed = (struct args) {
		.verbosity = 0,
		.bufferSize = 18,
		.debug = false,
		.dbgMode = ADE9113_CONFIG0_STREAM_DBG__NORMAL_MODE,
		.datapath = ADE9113_CONFIG_FILT_DATAPATH_CONFIG__COMP_LPF_8_KHZ,
		.configCrc = NULL
	};
	while (true) {
		int c = getopt(argc, argv, "hdvm:s:b:c:");
		if (c == -1) {
			break;
		}
		switch (c) {
			case 'm': {
				int value = parseStrEnum(c, optarg, modeEnumDefs);
				if (value < 0) {
					log_error("invalid stream mode (option `-m`)");
					return -1;
				}
				parsed->dbgMode = value;
				break;
			}
			case 's': {
				int value = parseIntEnum(c, optarg, 0, 7);
				if (value < 0) {
					return -1;
				}
				parsed->datapath = value;
				break;
			}
			case 'b': {
				int value = parseIntEnum(c, optarg, 12, 28);
				if (value < 0) {
					return -1;
				}
				parsed->bufferSize = value;
				break;
			}
			case 'c': {
				parsed->configCrc = optarg;
				break;
			}
			case 'h':
			default: {
				printUsage(argv[0]);
				return -1;
			}
		}
	}
	return 0;
}


int gpioWrite(const char *path, uint32_t value, uint32_t mask)
{
	int fd = open(path, O_RDWR);
	if (fd < 0) {
		return -1;
	}
	const uint32_t data[2] = { value, mask }; /* no endianness - same as in driver */
	int ret = write(fd, (const uint8_t *)data, 8);
	close(fd);
	return (ret > 0) ? 0 : -1;
}


int adcInit(struct adcCtx *ctx, addr_t spiBase, unsigned bufferSize)
{
	ctx->spi = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, spiBase);
	if (ctx->spi == MAP_FAILED) {
		log_error("failed to allocate spi1 buffer");
		return -1;
	}

	return sample_init(&ctx->samples, bufferSize, SAMPLE_SIZE);
}


int adcConfigure(struct adcCtx *ctx, const struct args *args)
{
	resetEcspi(ctx->spi);

	struct ade9113_ctx ade = { spiExchangeCb, NULL };

	ade9113_writeRegs(&ade, ADE9113_WR_LOCK, ADE9113_WR_LOCK__UNLOCK_KEY);
	ade9113_writeRegs(&ade, ADE9113_SWRST, ADE9113_SWRST__SOFTWARE_RESET_COMMAND);

	usleep(300 * 1000);

	/* enable clock passthrough from first chip */
	ade9113_writeRegsDifferent(
			&ade,
			ADE9113_CONFIG0,
			ADE9113_CONFIG0_ENC(.crc_en_spi_write = 1, .clkout_en = 1),
			ADE9113_CONFIG0_ENC(.crc_en_spi_write = 1, .clkout_en = 0),
			ADE9113_CONFIG0_ENC(.crc_en_spi_write = 1, .clkout_en = 0),
			ADE9113_CONFIG0_ENC(.crc_en_spi_write = 1, .clkout_en = 0));

	/* now that remaining chips were provided with clock we can reset them */
	ade9113_writeRegsDifferent(
			&ade,
			ADE9113_SWRST,
			0,
			ADE9113_SWRST__SOFTWARE_RESET_COMMAND,
			ADE9113_SWRST__SOFTWARE_RESET_COMMAND,
			ADE9113_SWRST__SOFTWARE_RESET_COMMAND);
	usleep(300 * 1000);

	/* prevent interrupts from driving IRQ pin */
	ade9113_writeRegs(&ade, ADE9113_MASK0, 0);
	ade9113_writeRegs(&ade, ADE9113_MASK1, 0);
	ade9113_writeRegs(&ade, ADE9113_MASK2, 0);

	/* synchronize sampling between chips (two separate writes to different bits as described in datasheet) */
	ade9113_writeRegs(&ade, ADE9113_SYNC_SNAP, ADE9113_SYNC_SNAP_ENC(.align = 1));
	ade9113_writeRegs(&ade, ADE9113_SYNC_SNAP, ADE9113_SYNC_SNAP_ENC(.snapshot = 1));

	if (args->dbgMode != ADE9113_CONFIG0_STREAM_DBG__NORMAL_MODE) {
		/* clear all counters */
		for (uint8_t reg = ADE9113_I_WAV_HI; reg <= ADE9113_V2_WAV_LO; ++reg) {
			ade9113_writeRegs(&ade, reg, 0);
		}
	}

	ade9113_writeRegs(&ade, ADE9113_CONFIG_FILT, ADE9113_CONFIG_FILT_ENC(.datapath_config = args->datapath));

	ade9113_writeRegsDifferent(
			&ade,
			ADE9113_CONFIG0,
			ADE9113_CONFIG0_ENC(.stream_dbg = args->dbgMode, .crc_en_spi_write = 1, .clkout_en = 1),
			ADE9113_CONFIG0_ENC(.stream_dbg = args->dbgMode, .crc_en_spi_write = 1, .clkout_en = 0),
			ADE9113_CONFIG0_ENC(.stream_dbg = args->dbgMode, .crc_en_spi_write = 1, .clkout_en = 0),
			ADE9113_CONFIG0_ENC(.stream_dbg = args->dbgMode, .crc_en_spi_write = 1, .clkout_en = 0));

	/* scratch values will be checked in DMA responses to ensure there is no shift */
	ade9113_writeRegsDifferent(&ade, ADE9113_SCRATCH, scratchValues[0], scratchValues[1], scratchValues[2], scratchValues[3]);

	ade9113_writeRegs(&ade, ADE9113_WR_LOCK, ADE9113_WR_LOCK__LOCK_KEY);
	ade9113_writeRegs(&ade, ADE9113_CONFIG_CRC, ADE9113_CONFIG_CRC_ENC(.crc_force = 1));

	for (int try = 0; try < 10; ++try) {
		uint8_t values[4];
		ade9113_readRegs(&ade, ADE9113_CONFIG_CRC, values, sizeof(values));
		bool ready = true;
		for (int i = 0; i < 4; ++i) {
			if ((values[i] & 0x01) == 0) {
				ready = false;
				break;
			}
		}
		if (ready) {
			break;
		}
		usleep(50000);
	}

	{
		uint8_t crcHigh[4];
		ade9113_readRegs(&ade, ADE9113_CRC_RESULT_HI, crcHigh, sizeof(crcHigh));

		uint8_t crcLow[4];
		ade9113_readRegs(&ade, ADE9113_CRC_RESULT_LO, crcLow, sizeof(crcLow));

		uint16_t crc[4];
		for (int i = 0; i < 4; ++i) {
			crc[i] = (crcHigh[i] << 8) | crcLow[i];
		}

		char strCrc[5 * 4];
		sprintf(strCrc, "%04x,%04x,%04x,%04x", crc[0], crc[1], crc[2], crc[3]);
		assert(strCrc[sizeof(strCrc) - 1] == '\0');

		log_info("config   CRC: %s", strCrc);
		if ((args->configCrc != NULL) && (strcmp(args->configCrc, strCrc) != 0)) {
			log_info("expected CRC: %s", args->configCrc);
			log_error("ERROR: CRC mismatch");
			return -1;
		};
	}
	return 0;
}


int main(int argc, char **argv)
{
	struct args args;
	if (parseArgs(&args, argc, argv) < 0) {
		return EX_USAGE;
	}

	struct sigaction sa;
	sa.sa_handler = signalHandler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESTART; /* Restart functions if interrupted by handler */

	if (sigaction(SIGINT, &sa, NULL) == -1) {
		log_error("sigaction failed");
		return -1;
	}

	if (sigaction(SIGTERM, &sa, NULL) == -1) {
		log_error("sigaction failed");
		return -1;
	}

	struct muxConfig {
		uint16_t mux;
		uint8_t mode;
	};
	const struct muxConfig muxConfigs[] = {
		/* PWM */
		{ .mux = pctl_mux_lcd_d2, .mode = MUX_PAD_LCD_DATA02__PWM3_OUT },

		/* GPIO */
		{ .mux = pctl_mux_tamper2, .mode = MUX_PAD_SNVS_TAMPER2__GPIO5_IO02 }, /* ADC_RESET (connected to RESET2) */
		{ .mux = pctl_mux_lcd_rst, .mode = MUX_PAD_LCD_RESET__GPIO3_IO04 },    /* ADC_RESET2 */
		{ .mux = pctl_mux_lcd_hsync, .mode = MUX_PAD_LCD_HSYNC__GPIO3_IO02 },  /* ADC_IRQN */
		{ .mux = pctl_mux_lcd_vsync, .mode = MUX_PAD_LCD_VSYNC__GPIO3_IO03 },  /* ADC_DREADY_L2 */
		{ .mux = pctl_mux_lcd_en, .mode = MUX_PAD_LCD_ENABLE__GPIO3_IO01 },    /* ADC_IRQ1 */
		{ .mux = pctl_mux_lcd_clk, .mode = MUX_PAD_LCD_CLK__GPIO3_IO00 },      /* ADC_IRQ2 */
		{ .mux = pctl_mux_lcd_d0, .mode = MUX_PAD_LCD_DATA00__GPIO3_IO05 },    /* ADC_IRQ3 */
		{ .mux = pctl_mux_lcd_d12, .mode = MUX_PAD_LCD_DATA12__ECSPI1_RDY },   /* ADC_DREADY */
	};
	for (int i = 0; i < sizeof(muxConfigs) / sizeof(*muxConfigs); ++i) {
		const struct muxConfig *cfg = &muxConfigs[i];
		platformctl(&(platformctl_t) { .action = pctl_set, .type = pctl_iomux, .iomux = { .mux = cfg->mux, .sion = 0, .mode = cfg->mode } });
	}

	if (initPwm()) {
		log_error("failed to initialize PWM");
		return EX_OSERR;
	}

	/* set RESET pin to HIGH (push-pull output) */
	const int resetPin = 2;
	gpioWrite("/dev/gpio5/dir", 1 << resetPin, 1 << resetPin);
	gpioWrite("/dev/gpio5/port", 1 << resetPin, 1 << resetPin);

	/* Initialize ecspi1 with channel 0 */
	if (ecspi_init(ecspi1, (1 << 0)) < 0) {
		log_error("failed to initialize ecspi");
		return EX_OSERR;
	}

	/* pull up reset2 pin */
	platformctl(&(platformctl_t) {
		.action = pctl_set,
		.type = pctl_iopad,
		.iopad = { .pad = pctl_pad_tamper2, .pus = 0b11, .pke = 0, .speed = 2 } });

	/* SPI pads - fast, strong and with hysteresis */
	uint8_t spiPads[] = { pctl_mux_lcd_d20, pctl_mux_lcd_d21, pctl_mux_lcd_d22, pctl_mux_lcd_d23 };
	for (int i = 0; i < sizeof(spiPads) / sizeof(*spiPads); ++i) {
		platformctl(&(platformctl_t) {
			.action = pctl_set,
			.type = pctl_iopad,
			.iopad = {
				.pad = spiPads[i],
				.hys = 1,    /* enable hysteresis */
				.pus = 0b00, /* 100kOhm pull down */
				.pue = 1,    /* pull on power down */
				.pke = 1,
				.ode = 0, /* open drain disabled */
				.speed = 2,
				.dse = 0b111, /* strong drive */
				.sre = 1,     /* fast slew rate */
			} });
	}

	/* IRQN pad - enable internal pullup */
	platformctl(&(platformctl_t) {
		.action = pctl_set,
		.type = pctl_iopad,
		.iopad = {
			.pad = pctl_pad_lcd_hsync,
			.hys = 1,    /* enable hysteresis */
			.pus = 0b11, /* 22kOhm pull up */
			.pue = 1,    /* pull on power down */
			.pke = 1,
			.ode = 0, /* open drain disabled */
			.speed = 2,
			.dse = 0b111, /* strong drive */
			.sre = 1,     /* fast slew rate */
		} });

	struct adcCtx *ctx = &common_adc;

	if (adcInit(ctx, IMX_ECSPI1_BASE, args.bufferSize) < 0) {
		return EX_OSERR;
	};
	adcConfigure(ctx, &args);

	if (dma_init(_PAGE_SIZE * 2, 2, dmaRxCb) < 0) {
		log_error("failed to init DMA");
		return -1;
	}

	msgapi_init(&ctx->msgapi, &ctx->samples, "ade9113");

	static uint8_t msgStacks[2][_PAGE_SIZE] __attribute__((aligned(8)));
	beginthread(msgThread, 2, msgStacks[0], _PAGE_SIZE, ctx);
	beginthread(msgThread, 2, msgStacks[1], _PAGE_SIZE, ctx);

	resetEcspi(ctx->spi);

	/*
	 * Configure eCSPI for DMA usage:
	 * - TX burst will be triggered from DREADY pin
	 * - DMA will write to TX FIFO when it reaches 0 words (empty)
	 * - DMA will read from RX FIFO when it reaches 16 words (full 64 byte response from all chips)
	 */
	ctx->spi->CONREG = (ctx->spi->CONREG & ~(0b11 << 16)) | (0b01 << 16); /* TRIGGER by falling edge */
	ctx->spi->CONREG = (ctx->spi->CONREG & ~(0b1 << 3)) | (0b1 << 3);     /* enable auto trigger */

	/* clang-format off */
	ctx->spi->DMAREG = (
		0
		| (0x01 << 23) /* RXDEN = 1 */
		| (15 << 16)   /* RX_THRESHOLD = 15 */
		| (0x01 << 7)  /* TXDEN = 1 */
		| (0 << 0)     /* TX_THRESHOLD = 0 */
	);
	/* clang-format on */

	dma_enable();

	time_t now;
	gettime(&now, NULL);
	const time_t start = now;

	while (pendingTerm == 0) {
		if (ctx->crcErrors || true) {
			if ((ctx->crcErrors > 0) || (ctx->valueErrors > 0)) {
				gettime(&now, NULL);
				fprintf(stdout, LOG_TAG "[%llu] crcErrors:%u valueErrors:%u\n", (long long unsigned)((now - start) / 1000), ctx->crcErrors, ctx->valueErrors);
				fflush(stderr);
			}
		}
		sleep(60);
	};

	fprintf(stderr, "dma_disable()\n");
	dma_disable();
	usleep(200000);
	fprintf(stderr, "resetEcspi()\n");
	resetEcspi(ctx->spi);

	return 0;
}
