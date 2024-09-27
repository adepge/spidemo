/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// SPI parameters
#define SPI_FREQ 200000
#define STORE_SIZE 2048

// Thread stack size and priority
#define STACK_SIZE 2048
#define THREAD_PRIORITY 5

// Devicetree node identifiers
#define LED0_NODE DT_ALIAS(led0)
#define SPI_MASTER DT_NODELABEL(my_spi_master)
#define SPI_MASTER_CS_DT_SPEC SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)) // Node identifier for Chip Select
#define SPI_SLAVE DT_NODELABEL(my_spi_slave)

// SPI slave thread
K_THREAD_STACK_DEFINE(spi_slave_stack, STACK_SIZE);	
struct k_thread spi_slave_thread;

const struct device *spi_slave_dev;
bool spi_slave_ready = false;

static uint8_t store[STORE_SIZE];

// SPI device configuration
const struct device *spi_dev;

static struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_OP_MODE_MASTER,
	.frequency = SPI_FREQ,
	.cs = {.gpio = SPI_MASTER_CS_DT_SPEC, .delay = 0},
};
static const struct spi_config spi_slave_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_OP_MODE_SLAVE,
	.frequency = SPI_FREQ,
};

// SPI device initialization functions
static void spi_init(void)
{
	spi_dev = DEVICE_DT_GET(SPI_MASTER);
	if (!device_is_ready(spi_dev))
	{
		printk("SPI master device not ready!\n");
	}
	struct gpio_dt_spec spim_cs_gpio = SPI_MASTER_CS_DT_SPEC;
	if (!device_is_ready(spim_cs_gpio.port))
	{
		printk("SPI master chip select device not ready!\n");
	}
}

static void spi_slave_init(void)
{
	spi_slave_dev = DEVICE_DT_GET(SPI_SLAVE);
	if (!device_is_ready(spi_slave_dev))
	{
		printk("SPI slave device not ready!\n");
	}
}

// Read from SPI slave
static int spi_read()
{
    uint8_t rx_buffer[1024] = {0};

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = sizeof(rx_buffer),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

	int ret;
    ret = spi_read(spi_dev, &spi_cfg, &rx);
    if(ret != 0){
        printk("SPI transceive error: %i\n", ret);
        return ret;
    }

	int count = 0;
    do {
        printk("%c", rx_buffer[count]);
        count++;
    } while (rx_buffer[count] != 0x00);  // While EOT != NULL, keep reading
	printk("\n");
	return 0;
}

// Write to SPI slave
static int spi_write(uint8_t *bytes, int length)
{	
	// Block access byte
	uint8_t tx_buffer[1024] = {0};
    memcpy(tx_buffer, bytes, length);

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = sizeof(tx_buffer),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    if(ret != 0){
        printk("SPI transceive error: %i\n", ret);
        return ret;
    }
	return 0;
}

/* SPI slave thread */
static void spi_slave_thread_entry(void *p1, void *p2, void *p3)
{
    spi_slave_init();

    uint8_t rx_buffer[1024] = {0};
    uint8_t tx_buffer[1024];
    memcpy(tx_buffer, store, STORE_SIZE);

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = sizeof(tx_buffer),
    };
    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = sizeof(rx_buffer),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    while (1) {
        int ret = spi_transceive(spi_slave_dev, &spi_slave_cfg, &tx, &rx);
        if(ret < 0){
            printk("SPI transceive error: %i\n", ret);
        }
        memcpy(store, rx_buffer, STORE_SIZE);
        k_msleep(100);
    }
}

void start_spi_slave_thread(void)
{
    k_thread_create(&spi_slave_thread, spi_slave_stack,
                    K_THREAD_STACK_SIZEOF(spi_slave_stack),
                    spi_slave_thread_entry, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&spi_slave_thread, "spi_slave");
}

// Shell command functions
static int cmd_spi_read(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 1) {
		shell_print(shell, "Usage: spi read");
		return -EINVAL;
    }
	spi_read();
	return 0;
}

static int cmd_spi_write(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(shell, "Usage: spi write <values>");
        return -EINVAL;
    }

    // Parse address
    if (!isxdigit(argv[1][0]) || !isxdigit(argv[1][1])) {
        shell_error(shell, "Invalid address format");
        return -EINVAL;
    }

    // Calculate total length of arguments
    size_t arg_len = 0;
    for (int x = 1; x < argc; x++)
    {
        arg_len += strlen(argv[x]);
    }

    int total_len = arg_len + (argc - 2) + 1; // Account for space between arguments and null terminator
    uint8_t *tx_buffer = (uint8_t *)malloc(total_len * sizeof(uint8_t));
    if (!tx_buffer) {
        shell_error(shell, "Memory allocation error");
        return -ENOMEM;
    }

    // Concatenates each string argument into tx_buffer
    tx_buffer[0] = '\0';
    for (int i = 1; i < argc; i++)
    {
        for (char *p = argv[i]; *p != '\0'; p++) {
            if ((*p == '/' || *p == '\\') && *(p + 1) == 'n') { // Check for newline characters
                strcat((char *)tx_buffer, "\n");
                p++;
                total_len--;
            } else {
                strncat((char *)tx_buffer, p, 1);
            }
        }
        if (i < argc - 1) {
            strcat((char *)tx_buffer, " ");	// Add space between arguments
        }
    }
    spi_master_write_text_block(target_addr, (uint8_t *)tx_buffer, total_len);
    return 0;
}

static int cmd_spi_list(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "SPI master device: %s", spi_dev->name);
	shell_print(shell, "SPI slave device: %s", spi_slave_dev->name);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(	sub_spi,
							   	SHELL_CMD(read, &sub_spi_rd, "SPI master read", cmd_spi_read),
							   	SHELL_CMD(write, &sub_spi_wr, "SPI master write", cmd_spi_write),
								SHELL_CMD(list, NULL, "List SPI devices", cmd_spi_list),
							   	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(spi, &sub_spi, "SPI read/write commands", NULL);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;

	if (!device_is_ready(led.port))
	{
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 0;
	}

	spi_init();
	start_spi_slave_thread();

    // Clear shell
	printk("\033[2J");
    printk("\033[H");

	return 0;
}
