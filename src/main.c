#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "argtable3/argtable3.h"

typedef struct {
    struct arg_int *kbps;
    struct arg_end *end;
} set_baudrate_args_t;

typedef struct {
    struct arg_int *seconds;
    struct arg_end *end;
} listen_can_args_t;

typedef struct {
    struct arg_int *period;
    struct arg_int *identifier;
    struct arg_int *data;
    struct arg_end *end;
} send_message_args_t;

static set_baudrate_args_t set_baudrate_args;
static listen_can_args_t listen_can_args;
static send_message_args_t send_message_args;

static twai_timing_config_t config_100kbps = TWAI_TIMING_CONFIG_100KBITS();
static twai_timing_config_t config_125kbps = TWAI_TIMING_CONFIG_125KBITS();
static twai_timing_config_t config_250kbps = TWAI_TIMING_CONFIG_250KBITS();
static twai_timing_config_t config_500kbps = TWAI_TIMING_CONFIG_500KBITS();

static int* baudrate = NULL;

esp_err_t init_timing_config(int bd, twai_timing_config_t *t_config)
{
    switch (bd)
    {
    case 100:
        *t_config = config_100kbps;
        break;
    case 125:
        *t_config = config_125kbps;
        break;
    case 250:
        *t_config = config_250kbps;
        break;
    case 500:
        *t_config = config_500kbps;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

void print_can_message(int time_ms, twai_message_t message)
{
    printf("{'time':%d,'id':'0x%X','len':'%d','data':[", time_ms, message.identifier, message.data_length_code);
    for (int i = 0; i < message.data_length_code; i++) {
        if (i + 1 == message.data_length_code)
        {
            printf("'0x%.2X'", message.data[i]);
        } else {
            printf("'0x%.2X',", message.data[i]);
        }
    }
    printf("]}\n");
}

void listen_can_task(int seconds)
{
    int64_t end_listening_time = esp_timer_get_time() + (seconds * 1000000);
    int64_t time = 0;

    for (;;) {
        if (time > end_listening_time) {
            break;
        }

        twai_message_t rx_msg;
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(10));
        time = esp_timer_get_time();
        if (ret == ESP_OK) {
            print_can_message(time / 1000, rx_msg);
        } else if (ret == ESP_ERR_TIMEOUT) {
            continue;
        } else {
            ESP_ERROR_CHECK(ret);
        }
    }
}

void send_can_message_task(int period, int id, int count, int *data)
{
    printf("%X ", id);
    for (int i = 0; i < count; i++) {
        printf("%X ", data[i]);
    }

    int n_iter = 100;
    for (int i = 0; i < n_iter; i++) {
        twai_message_t tx_msg;
        tx_msg.identifier = id;
        tx_msg.extd = 0;
        tx_msg.data_length_code = count;
        for (int i = 0; i < count; i++)
        {
            tx_msg.data[i] = data[i];
        }
        
        ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}

static int set_baudrate(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &set_baudrate_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, set_baudrate_args.end, argv[0]);
        return 1;
    }

    baudrate = set_baudrate_args.kbps->ival;
    return 0;
}

static int listen_can(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &listen_can_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, listen_can_args.end, argv[0]);
        return 1;
    }

    if (baudrate == NULL) {
        printf("error: need to set baudrate first\n");
        return 1;
    }
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config;
    ESP_ERROR_CHECK(init_timing_config(*baudrate, &t_config));
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    listen_can_task(listen_can_args.seconds->ival[0]);

    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());
    return 0;
}

static int send_message(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &send_message_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, send_message_args.end, argv[0]);
        return 1;
    }

    if (baudrate == NULL) {
        printf("error: need to set baudrate first\n");
        return 1;
    }
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config;
    ESP_ERROR_CHECK(init_timing_config(*baudrate, &t_config));
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    send_can_message_task(send_message_args.period->ival[0], send_message_args.identifier->ival[0], send_message_args.data->count, send_message_args.data->ival);

    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());
    return 0;
}

static void register_baudrate(void)
{
    set_baudrate_args.kbps = arg_int1(NULL, NULL, "<kbps>", "Baudrate in kbps (100, 125, 500)");
    set_baudrate_args.end = arg_end(5);

    const esp_console_cmd_t cmd = {
        .command = "baudrate",
        .help = "Set CAN bus baudrate",
        .hint = NULL,
        .func = &set_baudrate,
        .argtable = &set_baudrate_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_listen(void)
{
    listen_can_args.seconds = arg_int1(NULL, NULL, "<seconds>", "Time to listen in seconds");
    listen_can_args.end = arg_end(5);

    const esp_console_cmd_t cmd = {
        .command = "listen",
        .help = "Listen CAN bus",
        .hint = NULL,
        .func = &listen_can,
        .argtable = &listen_can_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_send(void)
{
    send_message_args.period = arg_int1(NULL, NULL, "<period>", "Period sending in ms");
    send_message_args.identifier = arg_int1(NULL, NULL, "<id>", "Message identifier");
    send_message_args.data = arg_intn(NULL, NULL, "<data>", 1, TWAI_FRAME_MAX_DLC, "Data bytes");
    send_message_args.end = arg_end(5);

    const esp_console_cmd_t cmd = {
        .command = "send",
        .help = "Send CAN bus message",
        .hint = NULL,
        .func = &send_message,
        .argtable = &send_message_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void app_main(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    esp_console_register_help_command();
    register_baudrate();
    register_listen();
    register_send();

    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
