#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mosquitto.h>
#include <json-c/json.h>
#include <unistd.h>

void on_connect(struct mosquitto *mosq, void *obj, int rc) {
if (rc == 0) {
printf("Connected to broker\n");
} else {
printf("Failed to connect, return code %d\n", rc);
}
}

int main() {
struct mosquitto *mosq;
int rc;

mosquitto_lib_init();

mosq = mosquitto_new("publisher-client", true, NULL);
if (!mosq) {
fprintf(stderr, "Error: Out of memory.\n");
return 1;
}

mosquitto_connect_callback_set(mosq, on_connect);

rc = mosquitto_connect(mosq, "localhost", 1883, 60);
if (rc != MOSQ_ERR_SUCCESS) {
fprintf(stderr, "Unable to connect: %s\n", mosquitto_strerror(rc));
return 1;
}

json_object *jobj = json_object_new_object();
json_object_object_add(jobj, "address", json_object_new_int(1));
json_object_object_add(jobj, "command", json_object_new_int(1));

json_object *led = json_object_new_object();
json_object_object_add(led, "RED", json_object_new_int(1));
json_object_object_add(led, "BLUE", json_object_new_int(1));
json_object_object_add(led, "GREEN", json_object_new_int(1));
json_object_object_add(led, "MOTOR", json_object_new_int(1));
json_object_object_add(jobj, "led", led);

json_object *params = json_object_new_object();
json_object_object_add(params, "turn_on_time", json_object_new_double(2));
json_object_object_add(params, "cycle", json_object_new_int(5));
json_object_object_add(params, "count", json_object_new_int(3));
json_object_object_add(jobj, "params", params);

json_object_object_add(jobj, "period", json_object_new_int(0));

const char *json_str = json_object_to_json_string(jobj);

mosquitto_publish(mosq, NULL, "lora/command", strlen(json_str), json_str, 0, false);

mosquitto_loop_start(mosq);
sleep(1); // Wait for message to be sent
mosquitto_loop_stop(mosq, true);

json_object_put(jobj);
mosquitto_destroy(mosq);
mosquitto_lib_cleanup();

return 0;
}