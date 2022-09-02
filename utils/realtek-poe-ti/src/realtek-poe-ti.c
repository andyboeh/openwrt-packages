// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 Andreas Böhler <dev@aboehler.at>
 * Copyright (c) realtek-poe authors
 *
 * User-space daemon for monitoring and managing PoE ports with
 * TI TPS23861 chips. Heavily based on realtek-poe for MCU-based
 * PoE PSE systems.
 *
 * Author: Andreas Böhler <dev@aboehler.at>
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <libubox/ustream.h>
#include <libubox/uloop.h>
#include <libubox/list.h>
#include <libubox/ulog.h>
#include <libubus.h>

#include <uci.h>
#include <uci_blob.h>
#include "tps23861.h"
#include "tps23861_mappings.h"

#define MAX_PORT        48 // This needs to be more than the max defined mapping
#define MAX_CHIP        12 // This needs to be more than the max defined mapping
#define PORT_PER_CHIP   4
#define ULOG_DBG(fmt, ...) ulog(LOG_DEBUG, fmt, ## __VA_ARGS__)
#define MAX(a, b)	(((a) > (b)) ? (a) : (b))

typedef int (*poe_reply_handler)(unsigned char *reply);

struct port_config {
    char name[16];
    unsigned int valid : 1;
    unsigned int enable : 1;
    unsigned char priority;
    unsigned char poe_plus;
    unsigned char power_budget;
};

struct config {
    float budget;
    float budget_guard;

    unsigned int port_count;
    struct port_config ports[MAX_PORT];
};

struct port_state {
    const char *status;
    float watt;
    const char *poe_mode;
};

struct state {
    float power_consumption;
    unsigned int num_detected_ports;

    struct port_state ports[MAX_PORT];
};

struct tps_map {
    struct tps23861_dev *dev;
    unsigned char channel;
};

struct tps23861_config {
    unsigned int num_chips;
    unsigned int num_ports;
    
    struct tps23861_dev tps[MAX_CHIP];
    struct tps_map tps_map[MAX_PORT];
};

//static struct ustream_fd stream;
static struct state state;
static struct blob_buf b;
static struct tps23861_config tps23861_config;


static struct config config = {
    .budget = 384,
    .budget_guard = 7,
};

static struct tps23861_dev *get_tps23861_dev_for_port(int id) {
    return tps23861_config.tps_map[id].dev;
}

static unsigned int get_tps23861_channel_for_port(int id) {
    return tps23861_config.tps_map[id].channel;
}

static char *get_board_compatible(void)
{
    char name[128];
    int fd, ret;

    fd = open("/sys/firmware/devicetree/base/compatible", O_RDONLY);
    if (fd < 0)
        return NULL;

    ret = read(fd, name, sizeof(name));
    if (ret < 0)
        return NULL;

    close(fd);

    return strndup(name, ret);
}

static void load_port_config(struct uci_context *uci, struct uci_section *s)
{
    const char * name, *id_str, *enable, *priority, *poe_plus;
    unsigned long id;

    id_str = uci_lookup_option_string(uci, s, "id");
    name = uci_lookup_option_string(uci, s, "name");
    enable = uci_lookup_option_string(uci, s, "enable");
    priority = uci_lookup_option_string(uci, s, "priority");
    poe_plus = uci_lookup_option_string(uci, s, "poe_plus");

    if (!id_str || !name) {
        ULOG_ERR("invalid port with missing name and id");
        return;
    }

    id = strtoul(id_str, NULL, 0);
    if (!id || id > tps23861_config.num_ports) {
        ULOG_ERR("invalid port id=%lu for %s", id, name);
        return;
    }
    config.port_count = MAX(config.port_count, id);
    id--;

    strncpy(config.ports[id].name, name, sizeof(config.ports[id].name));
    config.ports[id].valid = 1;
    config.ports[id].enable = enable ? !strcmp(enable, "1") : 0;
    config.ports[id].priority = priority ? strtoul(priority, NULL, 0) : 0;
    if (config.ports[id].priority > 3)
        config.ports[id].priority = 3;

    if (poe_plus && !strcmp(poe_plus, "1"))
        config.ports[id].poe_plus = 1;
    else
        config.ports[id].poe_plus = 0;

    ULOG_DBG("Config Port %lu: TPS %x Channel %d\n", id,
             tps23861_config.tps_map[id].dev->addr,
             tps23861_config.tps_map[id].channel
            );
}

static void load_global_config(struct uci_context *uci, struct uci_section *s)
{
    const char *budget, *guardband;
    char *compatible;
    size_t i, j;

    budget = uci_lookup_option_string(uci, s, "budget");
    guardband = uci_lookup_option_string(uci, s, "guard");

    config.budget = budget ? strtof(budget, NULL) : 31.0;
    config.budget_guard = config.budget / 10;
    if (guardband)
        config.budget_guard = strtof(guardband, NULL);

    compatible = get_board_compatible();
    if(!compatible) {
        ULOG_ERR("Can't get 'compatible': %s\n", strerror(errno));
        return;
    }

    if(!strcmp(compatible, "tplink,sg2452p-v4")) {
        ULOG_DBG("Found TP-Link TL-SG2452P v4, loading static mapping\n");
        tps23861_config.num_chips = SG2452P_NUM_CHIPS;
        tps23861_config.num_ports = SG2452P_NUM_PORTS;

        for(i=0; i<tps23861_config.num_chips; i++) {
            tps23861_config.tps[i].addr = SG2452P_DEV_LIST[i];
            tps23861_config.tps[i].bus_number = SG2452P_BUS_NUMBER;
            tps23861_config.tps[i].shunt_resistor = SG2452P_SHUNT_RESISTOR;
        }

        for(i=0; i<tps23861_config.num_ports; i++) {
            tps23861_config.tps_map[i].channel = SG2452P_CHANNEL_MAP[i];
            for(j=0; j<tps23861_config.num_chips; j++)
                if(tps23861_config.tps[j].addr == SG2452P_DEV_MAP[i])
                    tps23861_config.tps_map[i].dev = &tps23861_config.tps[j];
        }
    }
}

static void config_load() {
    struct uci_context *uci = uci_alloc_context();
    struct uci_package *package = NULL;

    memset(config.ports, 0, sizeof(config.ports));

    if (!uci_load(uci, "poe", &package)) {
        struct uci_element *e;


        uci_foreach_element(&package->sections, e) {
            struct uci_section *s = uci_to_section(e);

            if (!strcmp(s->type, "global"))
                load_global_config(uci, s);
        }

        uci_foreach_element(&package->sections, e) {
            struct uci_section *s = uci_to_section(e);

            if (!strcmp(s->type, "port"))
                load_port_config(uci, s);
        }
    }

    uci_unload(uci, package);
    uci_free_context(uci);

}

static void poe_init() {
    // We initialize all chips, even if the port is not configured
    size_t i;

    ULOG_DBG("PoE Init called\n");

    for(i=0; i<tps23861_config.num_chips; i++)
        if(tps23861_init(&tps23861_config.tps[i]))
            ULOG_ERR("Error initializing TPS23861 with address %x\n", tps23861_config.tps[i].addr);

    // Only if the port is configured, we check if we need to enable power
    for(i=0; i<config.port_count; i++) {
        if(config.ports[i].valid) {
            tps23861_set_poe_plus_mode(get_tps23861_dev_for_port(i),
                                       get_tps23861_channel_for_port(i), 
                                       config.ports[i].poe_plus);

            tps23861_set_port_power(get_tps23861_dev_for_port(i),
                                    get_tps23861_channel_for_port(i), 
                                    config.ports[i].enable);
        }
    }
}

static void poe_get_port_status() {
    size_t i;
    long voltage, current;
    float watt;
    const char *class_status;
    const char *detect_status;
    float total_consumption = 0;

    for(i=0; i<config.port_count; i++) {
        tps23861_get_voltage(tps23861_config.tps_map[i].dev, tps23861_config.tps_map[i].channel, &voltage);
        tps23861_get_current(tps23861_config.tps_map[i].dev, tps23861_config.tps_map[i].channel, &current);
        detect_status = tps23861_get_detect_status(tps23861_config.tps_map[i].dev, tps23861_config.tps_map[i].channel);
        class_status = tps23861_get_class_status(tps23861_config.tps_map[i].dev, tps23861_config.tps_map[i].channel);
        watt = (voltage * current) / 1000000.0;
        ULOG_DBG("Port %zu status: %s, %s, %ld mV, %ld mA, %f W\n", i+1,
                 detect_status,
                 class_status,
                 voltage,
                 current,
                 watt
                );
        if(!config.ports[i].valid)
            state.ports[i].status = "Disabled";
        else {
            if(voltage == 0 || current == 0) {
                state.ports[i].status = "Searching";
                state.ports[i].watt = 0;
            } else {
                state.ports[i].status = "Delivering power";
                state.ports[i].watt = (voltage * current) / 1000000.0;
                total_consumption += watt;
            }
            state.ports[i].poe_mode = class_status;
        }
    }
    state.power_consumption = total_consumption;
}

static void state_timeout_cb(struct uloop_timeout *t) {
    poe_get_port_status();

    uloop_timeout_set(t, 2 * 1000);
}

static int ubus_poe_info_cb(struct ubus_context *ctx, struct ubus_object *obj,
                            struct ubus_request_data *req, const char *method,
                            struct blob_attr *msg)
{
    //char tmp[16];
    size_t i;
    void *c;

    blob_buf_init(&b, 0);

    blobmsg_add_double(&b, "budget", config.budget);
    blobmsg_add_double(&b, "consumption", state.power_consumption);

    c = blobmsg_open_table(&b, "ports");
    for (i = 0; i < config.port_count; i++) {
        void *p;

        if (!config.ports[i].valid)
            continue;

        p = blobmsg_open_table(&b, config.ports[i].name);

        blobmsg_add_u32(&b, "priority", config.ports[i].priority);

        if (state.ports[i].poe_mode)
            blobmsg_add_string(&b, "mode", state.ports[i].poe_mode);
        if (state.ports[i].status)
            blobmsg_add_string(&b, "status", state.ports[i].status);
        else
            blobmsg_add_string(&b, "status", "unknown");
        if (state.ports[i].watt)
            blobmsg_add_double(&b, "consumption", state.ports[i].watt);

    blobmsg_close_table(&b, p);
    }
    blobmsg_close_table(&b, c);

    ubus_send_reply(ctx, req, b.head);

    return UBUS_STATUS_OK;
}

static int ubus_poe_reload_cb(struct ubus_context *ctx, struct ubus_object *obj,
                              struct ubus_request_data *req, const char *method,
                              struct blob_attr *msg)
{
    config_load();
    poe_init();

    return UBUS_STATUS_OK;
}

static const struct blobmsg_policy ubus_poe_manage_policy[] = {
    { "port", BLOBMSG_TYPE_STRING },
    { "enable", BLOBMSG_TYPE_BOOL },
};

static int ubus_poe_manage_cb(struct ubus_context *ctx, struct ubus_object *obj,
                              struct ubus_request_data *req, const char *method,
                              struct blob_attr *msg)
{
    struct blob_attr *tb[ARRAY_SIZE(ubus_poe_manage_policy)];
    const struct port_config *port;
    const char *port_name;
    size_t i;

    blobmsg_parse(ubus_poe_manage_policy,
                  ARRAY_SIZE(ubus_poe_manage_policy),
                  tb, blob_data(msg), blob_len(msg));
    if (!tb[0] || !tb[1])
        return UBUS_STATUS_INVALID_ARGUMENT;

    port_name = blobmsg_get_string(tb[0]);
    for (i = 0; i < config.port_count; i++) {
        port = &config.ports[i];
        if (port->valid && !strcmp(port_name, port->name))
            return tps23861_set_port_power(tps23861_config.tps_map[i].dev, tps23861_config.tps_map[i].channel, blobmsg_get_bool(tb[1]));
    }

    return UBUS_STATUS_INVALID_ARGUMENT;
}

static const struct ubus_method ubus_poe_methods[] = {
    UBUS_METHOD_NOARG("info", ubus_poe_info_cb),
    UBUS_METHOD_NOARG("reload", ubus_poe_reload_cb),
    UBUS_METHOD("manage", ubus_poe_manage_cb, ubus_poe_manage_policy),
};

static struct ubus_object_type ubus_poe_object_type =
	UBUS_OBJECT_TYPE("poe", ubus_poe_methods);

static struct ubus_object ubus_poe_object = {
    .name = "poe",
    .type = &ubus_poe_object_type,
    .methods = ubus_poe_methods,
    .n_methods = ARRAY_SIZE(ubus_poe_methods),
};

static void ubus_connect_handler(struct ubus_context *ctx) {
    int ret;

    ret = ubus_add_object(ctx, &ubus_poe_object);
    if (ret)
        ULOG_ERR("Failed to add object: %s\n", ubus_strerror(ret));
}

static int config_generate() {
    char *compatible;
    size_t i;

    compatible = get_board_compatible();
    if(!compatible) {
        ULOG_ERR("Can't get 'compatible': %s\n", strerror(errno));
        return 1;
    }

    if(!strcmp(compatible, "tplink,sg2452p-v4")) {
        ULOG_DBG("Found TP-Link TL-SG2452P v4, generating UCI config\n");
        printf("config global\n");
        printf("\toption budget    '384'\n");
        for(i=0; i<SG2452P_NUM_PORTS; i++) {
            printf("\n");
            printf("config port\n");
            printf("\toption enable    '0'\n");
            printf("\toption id        '%d'\n", i+1);
            printf("\toption name      'lan%d'\n", i+1);
            printf("\toption poe_plus  '1'\n");
            printf("\toption priority  '2'\n");
        }
        return 0;
    }

    return 1;
}

int main(int argc, char *argv[]) {
    int ch;

    struct uloop_timeout state_timeout = {
        .cb = state_timeout_cb,
    };

    struct ubus_auto_conn conn = {
        .cb = ubus_connect_handler,
    };

    ulog_open(ULOG_STDIO | ULOG_SYSLOG, LOG_DAEMON, "realtek-poe-ti");
    ulog_threshold(LOG_INFO);

    while ((ch = getopt(argc, argv, "dg")) != -1) {
        switch (ch) {
        case 'd':
            ulog_threshold(LOG_DEBUG);
            break;
        case 'g':
            return config_generate();
        }
    }

    config_load();

    uloop_init();
    ubus_auto_connect(&conn);

    poe_init();
    uloop_timeout_set(&state_timeout, 1000);
    uloop_run();
    uloop_done();

    return 0;
}
