#include "BM83.h"
#include "cstring"
#include "driver/uart.h"
#include "esp_log.h"

BM83::BM83()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    cmd_queue = xQueueCreate(BM83_CMD_QUEUE_SIZE, sizeof(bm83_cmd*));
}

bool BM83::begin(gpio_num_t pin_rx, gpio_num_t pin_tx, gpio_num_t pin_mfb, uint32_t baud_rate)
{
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    uart_param_config(uart_port, &config);

    uart_set_pin(
        uart_port,
        pin_tx,
        pin_rx,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    uart_driver_install(uart_port, 1024, 1024, 0, NULL, 0);

    xTaskCreate(
        queue_process_task_wrapper,
        "BM83QueueProcessTask",
        4096,
        this,
        5,
        nullptr
    );

    xTaskCreate(
        rx_task_wrapper,
        "BM83UartRxTask",
        4096,
        this,
        5,
        nullptr
    );

    // FIXME
    return true;
}

void BM83::send(uint8_t opcode, uint8_t *params, uint16_t params_length)
{
    uint8_t packet[BM83_MAX_TX_PACKET_LENGTH];
    uint16_t packet_len = build_packet(opcode, params, params_length, packet);
    char hex_str[256];
    buf_to_hex_string(packet, packet_len, hex_str, sizeof(hex_str));
    ESP_LOGE("BM83", "#Sending Packet:");
    ESP_LOGE("BM83", "Packet length: %d", packet_len);
    ESP_LOGE("BM83", "Packet: %s", hex_str);
    ESP_LOGE("BM83", "----------");
    bm83_cmd cmd = build_cmd(packet, packet_len);
    bm83_cmd *cmd_ptr = &cmd;
    xQueueSend(cmd_queue, &cmd_ptr, portMAX_DELAY);
    xSemaphoreTake(cmd.done_sem, portMAX_DELAY);
    ESP_LOGE("BM83", "--- Command response: ---");
    ESP_LOGE("BM83", "Packet length: %d", cmd.rx_len);
    char hex_res_str[512];
    buf_to_hex_string(cmd.rx_data, cmd.rx_len, hex_res_str, sizeof(hex_res_str));
    ESP_LOGE("BM83", "Packet: %s", hex_res_str);
    ESP_LOGE("BM83", "");
    clear_current_cmd();
}

void BM83::init_uart(gpio_num_t pin_rx, gpio_num_t pin_tx)
{
}

uint16_t BM83::build_packet(uint8_t opcode, uint8_t *params, uint16_t params_length, uint8_t *packet)
{
    if (params_length == UINT16_MAX) {
        // TODO: Log err and abort
    }
    if (!params) {
        params_length = 0;
    }
    uint16_t len = 1 + params_length;
    uint32_t len_total = len + 4;
    uint8_t len_l = len & 0xFF;
    uint8_t len_h = (len >> 8) & 0xFF;
    packet[0] = 0xAA;
    packet[1] = len_h;
    packet[2] = len_l;
    packet[3] = opcode;
    if (params && params_length > 0) {
        std::memcpy(packet + 4, params, params_length);
    }
    packet[len_total - 1] = calc_checksum(packet, len_total);
    return len_total;
}

uint8_t BM83::calc_checksum(uint8_t *packet, size_t len_total)
{
    uint8_t checksum = 0;
    // ignore start byte and ignore last byte (= checksum byte)
    for (uint8_t i = 1; i < len_total - 1; i++) {
        checksum += packet[i];
    }
    return (~checksum) + 1;
}

bm83_cmd BM83::build_cmd(uint8_t *packet, uint16_t packet_len)
{
    bm83_cmd cmd;
    cmd.tx_len = packet_len;
    memcpy(cmd.tx_data, packet, packet_len);
    return cmd;
}

void BM83::queue_process_task_wrapper(void *arg)
{
    auto bm83 = static_cast<BM83*>(arg);
    bm83->queue_process_task();
}

void BM83::queue_process_task()
{
    while(true) {

        bm83_cmd *cmd;
        xQueueReceive(cmd_queue, &cmd, portMAX_DELAY);
        current_command = cmd;
        uart_write_bytes(uart_port, cmd->tx_data, cmd->tx_len);
    }
}

void BM83::rx_task_wrapper(void *arg)
{
    auto bm83 = static_cast<BM83*>(arg);
    bm83->rx_task();
}

void BM83::rx_task()
{
    uint8_t packet[BM83_MAX_RX_PACKET_LENGTH];
    while (true)
    {
        int len_rx_header = uart_read_bytes(uart_port, packet, 4, portMAX_DELAY);

        // corrupt packet (maybe we're reading in between?), discard
        if (len_rx_header != 4 || packet[0] != BM83_CMD_START_BYTE)
        {
            ESP_LOGE("BM83", "Discarding packet due to invalid header");
            continue;
        }


        uint16_t len_payload = (packet[1] << 8) | packet[2];

        // either the packet is corrupt or BM83_MAX_RX_PACKET_LENGTH is too small
        if (len_payload > BM83_MAX_RX_PACKET_LENGTH)
        {
            ESP_LOGE("BM83", "Discarding packet (payload size too large)");
            continue;
        }

        int len_rx_payload = uart_read_bytes(uart_port, packet + 4, len_payload, portMAX_DELAY);

        // incomplete packet
        if (len_rx_payload != len_payload)
        {
            ESP_LOGE("BM83", "Discarding packet (incomplete)");
            continue;
        }

        uint16_t len_total = len_rx_header + len_rx_payload;

        // corrupt packet
        if (calc_checksum(packet, len_total) != packet[len_total - 1])
        {
            ESP_LOGE("BM83", "Discarding packet (checksums don't match)");
            continue;
        }
    
        if (current_command)
        {
            // TODO: Check if the packet is actually the correct response to current_command
            // and handle accordingly
            current_command->rx_len = len_total;
            memcpy(current_command->rx_data, packet, len_total);
            xSemaphoreGive(current_command->done_sem);
        } else // no current command, must be a random event
        {
            ESP_LOGE("BM83", "--- Random event received: ---");
            ESP_LOGE("BM83", "Packet length: %d", len_total);
            char hex_res_str[512];
            buf_to_hex_string(packet, len_total, hex_res_str, sizeof(hex_res_str));
            ESP_LOGE("BM83", "Packet: %s", hex_res_str);
            ESP_LOGE("BM83", "");
            // TODO: Handle event
        }
    }
}

void BM83::clear_current_cmd()
{
    current_command = nullptr;
}

// bool BM83::uart_read_packet(bm83_cmd *cmd)
// {
//     uint8_t header[4];
//     int h_rx_len = uart_read_bytes(uart_port, header, 4, pdMS_TO_TICKS(BM83_CMD_TIMEOUT_MS));

//     // header rx timeout / invalid
//     if (h_rx_len != 4) {
//         return false;
//     }

//     // invalid start byte
//     if (header[0] != BM83_CMD_START_BYTE) {
//         return false;
//     }

//     uint16_t payload_len = (header[1] << 8) | header[2];

//     // corrupt packet, return early to avoid large buffer allocation/uart read
//     if (payload_len > BM83_MAX_RX_PACKET_LENGTH) {
//         return false;
//     }

//     uint16_t total_len = payload_len + 4;

//     cmd->res_len = total_len;

//     if (total_len <= BM83_PACKET_BUFFER_SIZE) {
//         cmd->res_data = cmd->res_buf;
//     } else {
//         cmd->res_data = (uint8_t*)malloc(cmd->res_len);
//         if (!cmd->res_data) return false;
//     }

//     memcpy(cmd->res_data, header, 4);

//     int pl_rx_len = uart_read_bytes(uart_port, cmd->res_data + 4, payload_len, pdMS_TO_TICKS(BM83_CMD_TIMEOUT_MS));

//     // payload rx timeout / invalid
//     if (pl_rx_len != payload_len) {
//         return false;
//     }

//     return true;
// }

void BM83::buf_to_hex_string(const uint8_t* buf, size_t len, char* out, size_t out_size)
{
    size_t pos = 0;
    for (size_t i = 0; i < len && pos + 3 < out_size; i++) {
        pos += snprintf(out + pos, out_size - pos, "%02X ", buf[i]);
    }
    if (pos > 0 && pos < out_size) out[pos-1] = '\0'; // remove last space
}
