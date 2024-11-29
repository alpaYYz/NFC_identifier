#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mosquitto.h>
#include <json-c/json.h>

// payload frame struct
typedef struct { 
    unsigned char securityID[2];
    unsigned char address;
    unsigned char seq;
    unsigned char command;
    unsigned char parameter;
    unsigned char reserved[2];
} Frame;

Frame tx_frame = {
    .seq = 0
}; 
void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        printf("Connected to broker\n");
        mosquitto_subscribe(mosq, NULL, "test/topic", 0);
    } else {
        printf("Failed to connect, return code %d\n", rc);
    }
}

void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    printf("Received message: %s\n", (char *)msg->payload);

    json_object *jobj = json_tokener_parse((char *)msg->payload);
    json_object *address, *command, *led, *params, *continuous;

    unsigned char r_address, r_command, r_LED, r_time, r_cycle, r_count, r_continuous;

    if (json_object_object_get_ex(jobj, "address", &address)) {
        r_address = (unsigned char)json_object_get_int(address);
    }

    if (json_object_object_get_ex(jobj, "command", &command)) {
        r_command = (unsigned char)json_object_get_int(command);
    }

    if (json_object_object_get_ex(jobj, "led", &led)) {
        json_object *red, *blue, *green, *motor;
        if (json_object_object_get_ex(led, "RED", &red)) {
            r_LED |= json_object_get_int(red);
        }
        if (json_object_object_get_ex(led, "BLUE", &blue)) {
            r_LED |= json_object_get_int(blue) << 1;
        }
        if (json_object_object_get_ex(led, "GREEN", &green)) {
            r_LED |= json_object_get_int(green) << 2;
        }
        if (json_object_object_get_ex(led, "MOTOR", &motor)) {
            r_LED |= json_object_get_int(motor) << 3;
        }
    }

    if (json_object_object_get_ex(jobj, "params", &params)) {
        json_object *turn_on_time, *cycle, *count;
        if (json_object_object_get_ex(params, "turn_on_time", &turn_on_time)) {
            r_time = (unsigned char)json_object_get_int(turn_on_time);
        }
        if (json_object_object_get_ex(params, "cycle", &cycle)) {
            r_cycle = (unsigned char)json_object_get_int(cycle);
        }
        if (json_object_object_get_ex(params, "count", &count)) {
            r_count = (unsigned char)json_object_get_int(count);
        }
    }

    if (json_object_object_get_ex(jobj, "continuous", &continuous)) {
        r_continuous = (unsigned char)json_object_get_int(continuous);
    }

    tx_frame.address = r_address;
    tx_frame.command = (r_command << 4) | r_LED;
    if (r_command == 3) { tx_frame.parameter = r_time; }
    else if (r_command == 1) { tx_frame.parameter = (r_time << 6) | (r_cycle << 3) | r_count; }
    else { tx_frame.parameter = 0; }

    printf("Security ID: %02x %02x Address: %02x Seq: %d Command: %02x Parameter: %02x\n",  
    tx_frame.securityID[0], tx_frame.securityID[1], tx_frame.address, tx_frame.seq, tx_frame.command, tx_frame.parameter);

    json_object_put(jobj);
}

int main() {
    struct mosquitto *mosq;
    int rc;

    mosquitto_lib_init();

    mosq = mosquitto_new("subscriber-client", true, NULL);
    if (!mosq) {
        fprintf(stderr, "Error: Out of memory.\n");
        return 1;
    }

    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);

    rc = mosquitto_connect(mosq, "localhost", 1883, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Unable to connect: %s\n", mosquitto_strerror(rc));
        return 1;
    }

    mosquitto_loop_forever(mosq, -1, 1);

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}