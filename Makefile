all: master slave mqtt_publisher mqtt_subscriber

LoRa.o: LoRa.c
	gcc -c LoRa.c -o LoRa.o -lpigpio -lrt -pthread -lm

master.o: master.c
	gcc -c master.c -o master.o -lpigpio -lrt -pthread -lm -lnfc -lmosquitto -ljson-c

master: LoRa.o master.o
	gcc -o master master.o LoRa.o -lpigpio -lrt -pthread -lm -lnfc -lmosquitto -ljson-c

slave.o: slave.c
	gcc -c slave.c -o slave.o -lpigpio -lrt -pthread -lm -lwiringPi

slave: LoRa.o slave.o
	gcc -o slave slave.o LoRa.o -lpigpio -lrt -pthread -lm -lwiringPi

mqtt_publisher: mqtt_publisher.c
	gcc -o mqtt_publisher mqtt_publisher.c -lmosquitto -ljson-c

mqtt_subscriber: mqtt_subscriber.c
	gcc -o mqtt_subscriber mqtt_subscriber.c -lmosquitto -ljson-c

clean:
	rm *.o master slave mqtt_subscriber mqtt_publisher
