# imx6ull-ecspi

This library API provides direct access to i.MX 6ULL ECSPI hardware. Currently maximum burst length is 256 bytes and Slave Select is asserted for the whole transfer.

## Initialization and configuration

To initialize one of four ECSPI instances use
```c
int ecspi_init(int dev_no, uint8_t chan_msk);
```
where `dev_no` ∊ {1, 2, 3, 4} indicates which instance to initialize, and `chan_msk` is a 4-bit bitmask stating which Slave Select lines (channels) for a given instance will be enabled. I/O multiplexers and daisies for ECSPI pins are implicitly set-up to correct values — but only for the enabled instance and enabled channels.

Instead of specifying numbers for `dev_no` one could use predefined enums: `ecspi1`, `ecspi2`, `ecspi3`, `ecspi4` which correspond to 1, 2, 3, and 4 respectively.

Default configuration set after calling this procedure is as follows: SPI mode 0, channel 0, all channels as masters, no clock division, and no CS-to-SCLK delay. And thus default clock is 60 MHz.


In order to use [asynchronous versions](#Asynchronous-data-exchange) of read/write procedures, a context registration shall occur first. Use
```c
int ecspi_registerContext(int dev_no, ecspi_ctx_t *ctx, handle_t cond);
```
where `ctx` is a pointer to a user-allocated `ecspi_ctx_t` structure, and `cond` is a handle to an already-created conditional variable. The `ctx` context shall be used in asynchronous procedures from now on.


To set the current channel use
```c
int ecspi_setChannel(int dev_no, uint8_t chan);
```
where `chan` ∊ {0, 1, 2, 3} indicates a channel to use. Any subsequent exchange operation will use this channel. Only enabled channel can be selected — doing otherwise will abort with an error value.


To set the current mode of SPI operation use
```c
int ecspi_setMode(int dev_no, uint8_t chan, uint8_t mode);
```
where `chan` is a channel to configure (no need to set the current one beforehand), and `mode` ∊ {0, 1, 2, 3} is a (CPOL, CPHA) value to use.


To set the clock dividers use
```c
int ecspi_setClockDiv(int dev_no, uint8_t pre, uint8_t post);
```
where `pre` ∊ \[0x0, 0xF\] is a pre-divider value and `post` ∊ \[0x0, 0xF\] is a post-divider value for the clock signal supplied to ECSPI instance. The values' interpretation can be found in i.MX reference manual.


To set a delay between asserting CS (Chip Select, Slave Select) and the first SPI clock edge (a CS-to-SCLK delay) use
```c
int ecspi_setCSDelay(int dev_no, uint8_t delay);
```
where `delay` ∊ \[0, 63\] is how many SPI clocks shall be inserted.


To set a period between consecutive transactions (from the end of the previous transaction to the beginning of the next transaction) use
```c
int ecspi_setSSDelay(int dev_no, uint16_t delay);
```
where `delay` ∊ \[0, 32767\] is how many SPI clocks shall be inserted. SS during this time will be inactive.


### Example

```c
/* Enable 4th instance, and channels 0 and 1. */
ecspi_init(ecspi4, 0x03);

ecspi_setMode(ecspi4, 1, 2);
ecspi_setMode(ecspi4, 0, 0);

ecspi_setChannel(ecspi4, 1);
/* Set clock to ~114 Hz, assuming clock root was 60 MHz. */
ecspi_setClockDiv(ecspi4, 0xF, 0xF);
/* Set CS-to-SCLK delay to ~87 ms. */
ecspi_setCSDelay(ecspi4, 10);
/* Set SS-to-SS  */
ecspi_setSSDelay(ecspi4, 500);
```

## Data exchange

Data read and write proceed simultaneously so only one procedure is used for data transmission:
```c
int ecspi_exchange(int dev_no, const uint8_t *out, uint8_t *in, size_t len);
```
where `out` is a pointer to outgoing data, `in` is a pointer for incoming data, and `len` is the length of `out` (`in` must be at least `len` in length). The `in` buffer is written to after all bytes from `out` are consumed, so it is safe to pass the same pointer as `out` and `in`. This procedure is blocking but employs sleeping on a conditional variable so it is suitable for long transactions.

For very short transactions (a few bytes at a relatively high speed), where the overhead of synchronization primitives and interrupt handling is greater than the actual transaction time, there is a busy-waiting version:
```c
int ecspi_exchangeBusy(int dev_no, const uint8_t *out, uint8_t *in, size_t len);
```
where parameters have the same meaning as in the `ecspi_exchange()` procedure.


Both these procedures wait until a previous transaction has completed in hardware so mixing them with asynchronous procedures should not corrupt transactions.


### Example

```c
uint8_t data[16] = {0x66, 0x12, 0};
uint8_t in[16];

ecspi_exchange(ecspi4, data, data, sizeof(data));
ecspi_exchangeBusy(ecspi4, data, in, sizeof(data));
```

## Asynchronous data exchange

When data has to be sent without awaiting for a response, an asynchronous write can be used:
```c
int ecspi_writeAsync(ecspi_ctx_t *ctx, const uint8_t *out, size_t len);
```
where `ctx` is a [registered](#Initialization-and-configuration) asynchronous context, `out` is outgoing data, and `len` is the length of `out`. Data is written into a queue, the transaction is started and the procedure immediately returns the number of bytes written, while the hardware performs an actual transfer. If there is already a pending `ecspi_writeAsync()` transaction with the same length of data, the data is put immediately and its length returned. If the length is different, there is no space in the hardware queue, or there is an operation other than `ecspi_writeAsync()` pending, then no data is put and `0` is returned. The received data is discarded.


When a response is required, an asynchronous exchange can be used:
```c
int ecspi_exchangeAsync(ecspi_ctx_t *ctx, const uint8_t *out, size_t len);
```
where parameters have the same meaning as in the `ecspi_writeAsync()` procedure. Data is written into a queue, the transaction is started and the procedure immediately returns the number of bytes written, while the hardware performs an actual transfer. If there is any operation pending, then no data is put and `0` is returned.

When the received data is available, its length is put into the `rx_count` member of the `ctx` and the registered conditional variable is signalled. Data has to be retrieved using `ecspi_readFifo()`:
```c
int ecspi_readFifo(ecspi_ctx_t *ctx, uint8_t *buf, size_t len);
```
where `buf` is the buffer for the data being retrieved and `len` is the length of data to retrieve (should be the same as used in `ecspi_exchangeAsync()`).


For exchanging data periodically at a fixed rate, an asynchronous periodical exchange can be used:
```c
int ecspi_exchangePeriodically(ecspi_ctx_t *ctx, uint8_t *out, uint8_t *in, size_t len, unsigned int wait_states, ecspi_writerProc_t writer_proc);
```
where `out` is the data to write, `in` is a temporary buffer (at least of `len` length), `len` is the length of `out`, `wait_states` is a number of wait states (in SPI clocks) to be inserted between transactions (as in `ecspi_setSSDelay()`), and `writer_proc` is a procedure used to save data of a single transaction and to provide a stop condition. `writer_proc` has the following prototype:
```c
typedef int ecspi_writerProc_t(const uint8_t *rx, size_t len, uint8_t *out);
```
where `rx` is basically the buffer passed as `in` in `ecspi_exchangePeriodically()` and contains the received data of `len` length, and `out` is the output buffer passed as `out` in `ecspi_exchangePeriodically()` so that it can be modified between periodical transactions (its length cannot be changed). If this procedure returns a value greater or equal to 0, then the process of periodical transactions continues, and else it stops (no more transactions are performed after that procedure returns). When the process is stopped, the registered conditional variable is signalled (only then – not after every transaction). This procedure is called by an interrupt handler.


### Example

```c
int res;
handle_t cond_spi;
handle_t irqlock;
ecspi_ctx_t ctx;
condCreate(&cond_spi);
mutexCreate(&irqlock);
ecspi_registerContext(ecspi4, &ctx, cond_spi);

uint8_t data[4] = {0x33, 0x44, 0x55, 0x66};
uint8_t in[4];

/* Enqueue two separate transactions. */
do {
	res = ecspi_writeAsync(&ctx, &data[0], 2);
} while (res == 0);
do {
	res = ecspi_writeAsync(&ctx, &data[2], 2);
} while (res == 0);


/* Transaction with a response. This will return 0 until the previous ecspi_writeAsync() has completed. */
do {
	res = ecspi_exchangeAsync(&ctx, data, sizeof(data));
} while (res == 0);

/* Await the response. */
mutexLock(irqlock);
while (ctx.rx_count < sizeof(data)) {
	condWait(cond_spi, irqlock, 0);
}
mutexUnlock(irqlock);

/* Get the response. */
ecspi_readFifo(&ctx, in, sizeof(data));


static size_t rx_len = 0;
/* A callback procedure for handling periodical transactions. */
/* In this example, it saves every second byte of transaction and increments
the third byte of output for the next transaction. It cancels exchanges when
12 bytes are saved. */
static int handler(const uint8_t *rx, size_t len, uint8_t *out)
{
	in[rx_len] = rx[1];
	out[2] += 1;
	rx_len += 1;

	return (rx_len < 12) ? len : -1;
}

/* Periodical transactions. */
do {
	res = ecspi_exchangePeriodically(&ctx, data, in, sizeof(data), 400, handler);
} while (res == 0);

/* Wait for all 12 transactions to complete. */
mutexLock(irqlock);
while (rx_len < 12) {
	condWait(cond_spi, irqlock, 0);
}
mutexUnlock(irqlock);

rx_len = 0;

```
