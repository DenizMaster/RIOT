/*
 * Copyright (C) 2022 Inria
 * Copyright (C) 2020-2022 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests für tests
 * @{
 *
 * @file
 * @brief       Test Application For SX1280 Driver
 *
 * @author      Aymeric Brochier <aymeric.brochier@univ-grenoble-alpes.fr>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "msg.h"
#include "thread.h"
#include "shell.h"
#include "mutex.h"

#include "net/lora.h"
#include "net/netdev.h"
#include "net/netdev/lora.h"
#include "periph/gpio.h"

#include "sx1280.h"
#include "sx1280_params.h"
#include "sx1280_netdev.h"

#define SX1280_MSG_QUEUE        (8U)
#define SX1280_STACKSIZE        (THREAD_STACKSIZE_DEFAULT)
#define SX1280_MSG_TYPE_ISR     (0x3456)
#define SX1280_MAX_PAYLOAD_LEN  (128U)

static char stack[SX1280_STACKSIZE];
static kernel_pid_t _recv_pid;

static char message[SX1280_MAX_PAYLOAD_LEN];
const char fix_payoad[SX1280_MAX_PAYLOAD_LEN]="Lorem ipsum dolor sit amet, consectetuer adipiscing elit. Aenean commodo ligula eget dolor. Aenean massa. Cum sociis natoque pe";



static sx1280_t sx1280;

mutex_t m;
int po =0;
int pi =15;
gpio_mode_t mode = GPIO_OUT;

static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;
        msg.type = SX1280_MSG_TYPE_ISR;
        if (msg_send(&msg, _recv_pid) <= 0) {
            puts("sx1280_netdev: possibly lost interrupt.");
        }
    }
    else {
        switch (event) {
        case NETDEV_EVENT_RX_STARTED:
            puts("Data reception started");
            break;

        case NETDEV_EVENT_RX_COMPLETE:
        {
            uint16_t var;
            
            size_t len = dev->driver->recv(dev, NULL, 0, 0);
            netdev_lora_rx_info_t packet_info;
            dev->driver->recv(dev, message, len, &packet_info);
            memcpy(&var,message,sizeof(var));
            printf(
                //"Received: \"%s\" (%d bytes) - [RSSI: %i, SNR: %i]\n",
                //message, (int)len, packet_info.rssi, (int)packet_info.snr);
                "payload:%d RSSI:%i SNR:%i bytes:%i \n",var,packet_info.rssi,(int)packet_info.snr,len);
                
        }
        break;

        case NETDEV_EVENT_TX_COMPLETE:
            puts("Transmission completed");
            mutex_unlock(&m);
            break;

        case NETDEV_EVENT_TX_TIMEOUT:
            puts("Transmission timeout");
            break;

        default:
            printf("Unexpected netdev event received: %d\n", event);
            break;
        }
    }
}

void *_recv_thread(void *arg)
{
    netdev_t *netdev = arg;

    static msg_t _msg_queue[SX1280_MSG_QUEUE];

    msg_init_queue(_msg_queue, SX1280_MSG_QUEUE);

    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == SX1280_MSG_TYPE_ISR) {
            netdev->driver->isr(netdev);
        }
        else {
            puts("Unexpected msg type");
        }
    }
}

static void _get_usage(const char *cmd)
{
    printf("Usage: %s get <type|freq|bw|sf|cr>\n", cmd);
}


static void _usage_freq(void)
{
    printf("Usage: use freq between 2400000000 + (bw/2) and 2500000000 - (bw/2) (Hz) !\n");
}

static void _usage_bw(void)
{
    printf("Usage: use 200, 400, 800, 1600 (kHz)\n");
}

static void _usage_sf(void)
{
    printf("Usage: use SF between 5 and 12\n");
}
static void _usage_pwr(void)
{
    printf("Usage: use power between -18 and 13 (dBm)\n");
}
static void _usage_cr(void)
{
    printf(
        "Usage: use\n \
    LORA_CR_4_5 = 1\n \
    LORA_CR_4_6 = 2\n \
    LORA_CR_4_7 = 3\n \
    LORA_CR_4_8 = 4\n \
    LORA_CR_LI_4_5 = 5\n \
    LORA_CR_LI_4_6 = 6\n \
    LORA_CR_LI_4_8 = 7\n");
}


static int sx1280_get_cmd(netdev_t *netdev, int argc, char **argv)
{
    if (argc == 2) {
        _get_usage(argv[0]);
        return -1;
    }

    if (!strcmp("type", argv[2])) {
        uint16_t type;
        netdev->driver->get(netdev, NETOPT_DEVICE_TYPE, &type, sizeof(uint16_t));
        printf("Device type: %s\n", (type == NETDEV_TYPE_LORA) ? "lora" : "fsk");
    }
    else if (!strcmp("freq", argv[2])) {
        uint32_t freq;
        netdev->driver->get(netdev, NETOPT_CHANNEL_FREQUENCY, &freq, sizeof(uint32_t));
        printf("Frequency: %" PRIu32 " Hz\n", freq);
    }
    else if (!strcmp("bw", argv[2])) {
        uint32_t bw_val = 0;
        netdev->driver->get(netdev, NETOPT_BANDWIDTH, &bw_val, sizeof(uint32_t));
        printf("Bandwidth: %" PRIu32 " kHz\n", bw_val / 1000);
    }
    else if (!strcmp("sf", argv[2])) {
        uint8_t sf;
        netdev->driver->get(netdev, NETOPT_SPREADING_FACTOR, &sf, sizeof(uint8_t));
        printf("Spreading factor: %d\n", sf);
    }
    else if (!strcmp("cr", argv[2])) {
        uint8_t cr;
        netdev->driver->get(netdev, NETOPT_CODING_RATE, &cr, sizeof(uint8_t));
        printf("Coding rate: %d\n", cr);
        _usage_cr();
    }
    else {
        _get_usage(argv[0]);
        return -1;
    }

    return 0;
}

static void _set_usage(const char *cmd)
{
    printf("Usage: %s set <freq|bw|sf|cr|pwr> <value>\n", cmd);
}

static int sx1280_set_cmd(netdev_t *netdev, int argc, char **argv)
{
    if (argc == 3) {
        if (!strcmp("freq", argv[2])) {
            _usage_freq();
        }
        if (!strcmp("bw", argv[2])) {
            _usage_bw();
        }
        if (!strcmp("sf", argv[2])) {
            _usage_sf();
        }
        if (!strcmp("cr", argv[2])) {
            _usage_cr();
        }
        if (!strcmp("pwr", argv[2])) {
            _usage_pwr();
        }
    }
    if (argc != 4) {
        _set_usage(argv[0]);
        return -1;
    }

    int ret = 0;

    if (!strcmp("freq", argv[2])) {
        uint32_t freq = strtoul(argv[3], NULL, 10);
        ret = netdev->driver->set(netdev, NETOPT_CHANNEL_FREQUENCY, &freq, sizeof(uint32_t));
    }
    else if (!strcmp("bw", argv[2])) {
        uint16_t bw = atoi(argv[3]);
        ret = netdev->driver->set(netdev, NETOPT_BANDWIDTH, &bw, sizeof(uint16_t));
    }
    else if (!strcmp("sf", argv[2])) {
        uint8_t sf = atoi(argv[3]);
        ret = netdev->driver->set(netdev, NETOPT_SPREADING_FACTOR, &sf, sizeof(uint8_t));
    }
    else if (!strcmp("cr", argv[2])) {
        uint8_t cr = atoi(argv[3]);
        ret = netdev->driver->set(netdev, NETOPT_CODING_RATE, &cr, sizeof(uint8_t));
    }
    else if (!strcmp("pwr", argv[2])) {
        int8_t pwr = atoi(argv[3]);
        ret = netdev->driver->set(netdev, NETOPT_TX_POWER, &pwr, sizeof(int8_t));
    }
    else {
        _set_usage(argv[0]);
        return -1;
    }

    if (ret < 0) {
        printf("cannot set %s\n", argv[2]);
        return ret;
    }

    printf("%s set\n", argv[2]);
    return 0;
}

static void _rx_usage(const char *cmd)
{
    printf("Usage: %s rx <start|stop>\n", cmd);
}

static int sx1280_rx_cmd(netdev_t *netdev, int argc, char **argv)
{
    

    if (gpio_init(GPIO_PIN(po, pi), mode) < 0) {
            printf("Error to initialize GPIO_PIN(%i, %02i)\n", po, pi);
            return 1;
        }
    gpio_clear(GPIO_PIN(po,pi));


    if (argc == 2) {
        _rx_usage(argv[0]);
        return -1;
    }

  
    if (!strcmp("start", argv[2])) {
        /* Switch to RX (IDLE) state */
        netopt_state_t state = NETOPT_STATE_IDLE;
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(state));
        printf("Listen mode started\n");
    }
    else if (!strcmp("stop", argv[2])) {
        /* Switch to STANDBY state */
        netopt_state_t state = NETOPT_STATE_STANDBY;
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(state));
        printf("Listen mode stopped\n");
    }
    else {
        _rx_usage(argv[0]);
        return -1;
    }

    return 0;
}

static int sx1280_tx_cmd(netdev_t *netdev, int argc, char **argv)
{
   
    if (argc == 2) {
        printf("Usage: %s tx <payload>\n", argv[0]);
        return -1;
    }
    
    if (gpio_init(GPIO_PIN(po, pi), mode) < 0) {
        printf("Error to initialize GPIO_PIN(%i, %02i)\n", po, pi);
        return 1;
    }
    gpio_set(GPIO_PIN(po,pi));

    printf("sending \"%s\" payload (%u bytes)\n",
           argv[2], (unsigned)strlen(argv[2]) + 1);
    iolist_t iolist = {
        .iol_base = argv[2],// pointer übergeben
        .iol_len = (strlen(argv[2]) + 1)//länge des pointer
    };

    if (netdev->driver->send(netdev, &iolist) == -ENOTSUP) {
        puts("Cannot send: radio is still transmitting");
        return -1;
    }

    return 0;
}

int len_helper(int x){
	if (x>=10000)	return 5;
	if (x>=1000) 	return 4;
	if (x>=100)	return 3;
	if (x>=10)	return 2;
	return 1;
}

int sx1280_flood_cmd(netdev_t *netdev, int argc, char **argv)
{
    // sx1280 tx_flooding 1000 32
	(void)argc;
    
	//(void)argv;
    //char int_as_string[20];
	uint16_t i =0;
    if (gpio_init(GPIO_PIN(po, pi), mode) < 0) {
        printf("Error to initialize GPIO_PIN(%i, %02i)\n", po, pi);
        return 1;
    }
    //gpio_set(GPIO_PIN(po,pi));
	int j = atoi(argv[2]);
    int payload_len = atoi(argv[3]);
	//printf("%s\n",argv[2]);
	printf("amount: %d\n",j);
	//int output_test[2];
	while(i<=j){
		//output_test[0]=i;
        printf("test %d\n",i);
        //sprintf(int_as_string,"%d",i);
        memcpy(message,&fix_payoad,sizeof(fix_payoad));
        memcpy(message,&i,sizeof(i));
		iolist_t iolist ={
			//.iol_base =&output_test[0],
            
			//.iol_base=int_as_string,
			.iol_base=&message,
            .iol_len=payload_len};
		i++;
        //printf("test:zeile 382\n");
	
		if (netdev->driver->send(netdev, &iolist) == -ENOTSUP) {
        		puts("Cannot send: radio is still transmitting");
        		return -1;
    		}
    	mutex_lock(&m);
        if (i>j){
            printf("the End\n");
        }
	}
    //printf("test:the End");
	return 0;
	
}
/*
int sx1280_constwave_cmd(netdev_t *netdev, int argc, char **argv)
{
	int exit_value=0;
    	(void)argc;
	iolist_t iolist ={
	.iol_base =argv[0],
	.iol_len=(unsigned)strlen(argv[0])};
	
	while(exit_value!=-1){
		if (netdev->driver->send(netdev, &iolist) == -ENOTSUP) {
        		puts("Cannot send: radio is still transmitting");
        		exit_value= -1;
        	}
        	mutex_lock(&m); 
        	
	}

	
	return exit_value;
	
}
*/
int const_wave(netdev_t *netdev, int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    //netopt_rf_testmode_t mode = NETOPT_RF_TESTMODE;
    netopt_state_t state = NETOPT_STATE_RESET;;
    if (gpio_init(GPIO_PIN(po, pi), mode) < 0) {
        printf("Error to initialize GPIO_PIN(%i, %02i)\n", po, pi);
        return 1;
    }
    gpio_set(GPIO_PIN(po,pi));

    puts("starting constant wave");
    netdev->driver->set(netdev,NETOPT_RF_TESTMODE,&state, sizeof(state));
    return 0;
}

static int sx1280_reset_cmd(netdev_t *netdev, int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("resetting sx1280...");
    netopt_state_t state = NETOPT_STATE_RESET;

    netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    return 0;

}
int sx1280_cmd(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: %s <get|set|rx|tx|tx_constwave|tx_flooding|reset>\n", argv[0]);
        return -1;
    }

    netdev_t *netdev = &sx1280.netdev;

    if (!strcmp("get", argv[1])) {
        return sx1280_get_cmd(netdev, argc, argv);
    }
    else if (!strcmp("set", argv[1])) {
        return sx1280_set_cmd(netdev, argc, argv);
    }
    else if (!strcmp("rx", argv[1])) {
        return sx1280_rx_cmd(netdev, argc, argv);
    }
    else if (!strcmp("tx", argv[1])) {
        return sx1280_tx_cmd(netdev, argc, argv);
    }
    else if (!strcmp("reset", argv[1])) {
        return sx1280_reset_cmd(netdev, argc, argv);
    }
    else if (!strcmp("tx_constwave",argv[1])) {
    	return const_wave(netdev, argc, argv); //TODO: anpassen
    }
    else if (!strcmp("tx_flooding",argv[1])) {
    	if(argc < 3){
    		printf("needed: amount of msgs (<=10000)\n like: sx1280 tx_flooding 1000");
    		return -1;
    	}
    	else {
    		return sx1280_flood_cmd(netdev, argc, argv); //TODO: anpassen
    	}
    }
    else {
        printf("Unknown cmd %s\n", argv[1]);
        printf("Usage: %s <get|set|rx|tx|tx_constwave|tx_flooding|reset>\n", argv[0]);
        return -1;
    }

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "sx1280", "Control the SX1280 radio",     sx1280_cmd },
    { NULL, NULL, NULL }
};

int main(void)
{
    sx1280_setup(&sx1280, &sx1280_params[0], 0);

    mutex_init(&m);
    mutex_lock(&m);

    netdev_t *netdev = &sx1280.netdev;

    netdev->driver = &sx1280_driver;

    netdev->event_callback = _event_cb;

    if (netdev->driver->init(netdev) < 0) {
        puts("Failed to initialize SX1280 device, exiting");
        return 1;
    }

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, netdev,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

    puts("Initialization successful - starting the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
