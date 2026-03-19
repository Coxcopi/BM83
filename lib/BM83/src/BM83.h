#pragma once
#include "soc/gpio_num.h"
#include "stdint.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <bit>
#include <iostream>
#include <driver/uart.h>

static_assert(std::endian::native == std::endian::little);

// TODO: Use correct timeout from bm83 uart specs
constexpr time_t BM83_CMD_TIMEOUT_MS = 100;
// ~ 10 kB with a full queue
constexpr size_t BM83_CMD_QUEUE_SIZE = 8;
// might need adjustment
constexpr size_t BM83_MAX_TX_PACKET_LENGTH = 64;
// might need adjustment
constexpr size_t BM83_MAX_RX_PACKET_LENGTH = 1024;
// Used to avoid heap fragmentation for small packages.
// TODO: Could probably be smaller.
constexpr size_t BM83_PACKET_BUFFER_SIZE = 32;

constexpr uint8_t BM83_CMD_START_BYTE = 0xAA;

struct bm83_cmd {
    uint8_t tx_data[BM83_MAX_TX_PACKET_LENGTH];
    size_t tx_len;

    uint8_t rx_data[BM83_MAX_RX_PACKET_LENGTH];
    size_t rx_len;

    SemaphoreHandle_t done_sem;

    bm83_cmd() {
        done_sem = xSemaphoreCreateBinary();
    }

    ~bm83_cmd() {
        if (done_sem) vSemaphoreDelete(done_sem);
    }
};

class BM83 {
    public:
        BM83();
        bool begin(gpio_num_t pin_rx, gpio_num_t pin_tx, gpio_num_t pin_mfb, uint32_t baud_rate = 115200);
        void send(uint8_t opcode, uint8_t *params = nullptr, uint16_t params_length = 0);

    private:
        QueueHandle_t cmd_queue;
        uart_port_t uart_port = UART_NUM_1;

        bm83_cmd *current_command = nullptr;

        void init_uart(gpio_num_t pin_rx, gpio_num_t pin_tx);

        uint16_t build_packet(uint8_t opcode, uint8_t *params, uint16_t params_length, uint8_t *packet);
        uint8_t calc_checksum(uint8_t *packet_wo_checksum, size_t len_total);

        bm83_cmd build_cmd(uint8_t *packet, uint16_t packet_len);

        static void queue_process_task_wrapper(void *arg);
        void queue_process_task();

        static void rx_task_wrapper(void *arg);
        void rx_task();

        void clear_current_cmd();

        bool uart_read_packet(bm83_cmd *cmd);

        void buf_to_hex_string(const uint8_t* buf, size_t len, char* out, size_t out_size);
};