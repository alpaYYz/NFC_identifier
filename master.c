// update : 24.08.22 AM 10:58
// update : 24.08.22 AM 09:15
// update : 24.08.23 PM 16:34
// update : 24.09.05 PM 16:52
// update : 24.09.06 am 11:12
// update : 24.09.09 am 10:56
// update : 24.09.10 Am 11:46
// update : 24.09.11 AM 09:16
// update : 24.09.11 AM 09:45
// update : 24.09.19 AM 05:32


#include "LoRa.h"
#include <nfc/nfc.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <mosquitto.h>
#include <json-c/json.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <fcntl.h>   // open(), O_CREAT, O_RDWR 등을 위한 헤더
#include <unistd.h>  // close()를 위한 헤더

// Defining Flash Memory File Name
#define FLASH_FILE "flash_memory.bin"  
#define MAX_FRAME_LEN 264 // NDEF

// Master's security ID and address and next slave address
unsigned char MySecurityID[2]; 
unsigned char ADDRESS = 127; 
unsigned char next_slaveADDRESS = 1; 

// Paths and log messages to store logs
const char *folderPath = "./logs";  // Device Log File Path
const char *m_logPath = "./m_logs"; // Master-to-master communication log file path
const char *ndef_logPath = "./ndef_logs";  // NDEF log file path (newly added path)
char logMessage[256];               // Buffer to store log messages


// pthread variables for NFC tag detection and data transfer
pthread_t threads[2];
unsigned char thread_status = 0;

// Defining Transport Frame Structures
typedef struct {
    unsigned char securityID[2]; // Security ID (2 bytes)
    unsigned char address; // Device Address (1 byte)
    unsigned char seq; // Transfer sequence (1 byte, representing the order of messages sent and received)
    unsigned char command; // Command (1 byte)
    unsigned char parameter; // Additional parameters according to the command (1 byte)
    unsigned char reserved[2]; // Reserved space (2 bytes)
} Frame;

// Define the NDEFRecord data structure
typedef struct {
    unsigned char SecurityID[2];   // Security ID (2 bytes)
    unsigned char DeviceID[2];     // Device ID (2 bytes)
    unsigned char DeviceAddress;   // Device Address (1 byte)
} NDEFRecord;

// Variables for transmitting and receiving
unsigned char ack_receive;
unsigned char is_broadcast;
unsigned char transmit_period = 0;

// Initialize transmit frame
Frame tx_frame = {
    .seq = 0};

// Variables for LoRa and MQTT
LoRa_ctl modem;
struct mosquitto *mosq;

// Maximum number and arrangement to store registered slave addresses
#define MAX_DEVICES 100

// Define a data structure to store in flash memory
typedef struct {
    unsigned char registered_addresses[MAX_DEVICES];
    unsigned int registered_count;
    unsigned char MySecurityID[2];
} FlashData;

FlashData flash_data;

// Declaration of functions for parsing NDEF messages
void print_hex(const uint8_t *data, size_t len);


void parse_ndef_message(uint8_t *ndef_message, size_t length, NDEFRecord *record);
void write_ndef_message(nfc_device *pnd, uint8_t *ndef_message, size_t length);
void read_ndef_message(nfc_device *pnd, nfc_context *context, NDEFRecord *record);
void writeLog(const char *folderPath, const char *tag, const char *logMessage);
unsigned char hexCharToByte(char high, char low);

// Flash memory initialization function
void flash_init() {
    int fd = open(FLASH_FILE, O_RDWR | O_CREAT, 0644);
    if (fd == -1) {
        perror("Failed to open flash memory file");
        exit(1);
    }

    // Flash memory initialization: record new data if file size is 0
    if (lseek(fd, 0, SEEK_END) == 0) {
        write(fd, &flash_data, sizeof(FlashData));  // Flash memory initialization
    }

    close(fd);
}

// Data read function from flash memory
void flash_read() {
    int fd = open(FLASH_FILE, O_RDONLY);
    if (fd == -1) {
        perror("Failed to open flash memory file for reading");
        return;
    }

    read(fd, &flash_data, sizeof(FlashData));
    close(fd);
}


// Write data to flash memory function
void flash_write() {
    int fd = open(FLASH_FILE, O_WRONLY);
    if (fd == -1) {
        perror("Failed to open flash memory file for writing");
        return;
    }

    write(fd, &flash_data, sizeof(FlashData));
    close(fd);
}

// Function to register a new address (flash memory interworking)
void register_address(unsigned char address) {
    for (unsigned int i = 0; i < flash_data.registered_count; i++) {
        if (flash_data.registered_addresses[i] == address) {
            printf("Address 0x%02x is already registered.\n", address);
            return;
        }
    }

    if (flash_data.registered_count < MAX_DEVICES) {
        flash_data.registered_addresses[flash_data.registered_count++] = address;
        flash_write();  // Write to Flash memory after registration
        printf("Address 0x%02x registered successfully.\n", address);
    } else {
        printf("Address list is full. Cannot register new address.\n");
    }
}

// List of registered devices
unsigned char registered_addresses[MAX_DEVICES];
unsigned int registered_count = 0;

// Master Identification ID and Status Definition
#define MASTER1_ID 0x01
#define MASTER2_ID 0x02
unsigned char MASTER_ID = MASTER1_ID; // ID of the current master
char *master_status = "idle";         // Default state is idle

// Master communication start/end status
bool master_communication_active = false;

// Defining LED control related constants
#define LED3_PIN 3                // Pin number corresponding to LED 3
#define COMMAND_LED_CONTROL 0x18  // Defining LED Control Commands

// 로그 작성 함수 선언
void writeLog(const char *folderPath, const char *tag, const char *logMessage);
void writeMasterLog(const char *tag, const char *logMessage);  // 마스터 통신 로그 함수 선언

// 메시지 전송 함수 선언
void send_message(const char *message, unsigned char address, const char *mac, const unsigned char *uid, size_t uid_len);

// NDEF 파싱 함수 선언
void parse_ndef(uint8_t *ndef_data, int length);

// NDEF 데이터를 읽는 함수
void read_ndef_data(nfc_device *pnd);

// LED 밝기 설정 함수
void process_led_brightness(json_object *led);

// LED 밝기 설정을 포함한 패킷 처리 함수
void process_led_brightness(json_object *brightness) {
    int brightness_level = json_object_get_int(brightness);

    // 밝기 레벨 설정 (Message Definition 3/4에 따라 설정)
    switch (brightness_level) {
        case 1:
            tx_frame.reserved[0] = 0x00;  // 약한 밝기
            break;
        case 2:
            tx_frame.reserved[0] = 0x01;  // 중간 밝기
            break;
        case 3:
            tx_frame.reserved[0] = 0x02;  // 강한 밝기
            break;
        default:
            tx_frame.reserved[0] = 0x00;  // 기본값은 약한 밝기
            break;
    }

    // 명령어를 LED 제어로 설정
    tx_frame.command = (COMMAND_LED_CONTROL & 0x0F) | (0 << 4);  // LED 제어, 4비트

    // 로그 작성 및 전송
    snprintf(logMessage, sizeof(logMessage), "LED Brightness set to %d", brightness_level);
    writeLog(folderPath, "TX", logMessage);
    LoRa_send(&modem);  // LoRa 전송
}


// 로그 파일 작성 함수
void writeLog(const char *folderPath, const char *tag, const char *logMessage) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char filePath[256];
    snprintf(filePath, sizeof(filePath), "%s/log_%04d-%02d-%02d.txt", folderPath, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);

    struct stat st = {0};
    if (stat(folderPath, &st) == -1) {
        mkdir(folderPath, 0777);
    }

    FILE *file = fopen(filePath, "a");
    if (file == NULL) {
        perror("Unable to open log file");
        return;
    }
    fprintf(file, "[%04d-%02d-%02d %02d:%02d:%02d] %s %s\n",
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
            t->tm_hour, t->tm_min, t->tm_sec, tag, logMessage);

    fclose(file);
}

// 마스터 통신 로그 작성 함수
void writeMasterLog(const char *tag, const char *logMessage) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char filePath[256];
    snprintf(filePath, sizeof(filePath), "%s/m_log_%04d-%02d-%02d.txt", m_logPath, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);

    struct stat st = {0};
    if (stat(m_logPath, &st) == -1) {
        mkdir(m_logPath, 0777);
    }

    FILE *file = fopen(filePath, "a");
    if (file == NULL) {
        perror("Unable to open master log file");
        return;
    }
    fprintf(file, "[%04d-%02d-%02d %02d:%02d:%02d] %s %s\n",
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
            t->tm_hour, t->tm_min, t->tm_sec, tag, logMessage);

    fclose(file);
}

// 전송 프레임 데이터를 포함한 로그 메시지를 생성하는 함수
void make_logMessage(Frame *frame, char * logMessage) {
    sprintf(logMessage, "Security ID: %02x %02x Address: %02x Seq: %d Command: %02x Parameter: %02x Reserved: %02x",  
    frame->securityID[0], frame->securityID[1], frame->address, frame->seq, frame->command, frame->parameter, frame->reserved[0]);
}

// // 새로운 주소를 등록하는 함수
// void register_address(unsigned char address) {
//     for (unsigned int i = 0; i < registered_count; i++) {
//         if (registered_addresses[i] == address) {
//             printf("Address 0x%02x is already registered.\n", address);
//             return;
//         }
//     }

//     if (registered_count < MAX_DEVICES) {
//         registered_addresses[registered_count++] = address;
//         printf("Address 0x%02x registered successfully.\n", address);
//     } else {
//         printf("Address list is full. Cannot register new address.\n");
//     }
// }

// 통신 완료 후 slave에게 LED 제어 명령을 전송하는 함수
void notify_slave_to_blink_led() {
    tx_frame.command = COMMAND_LED_CONTROL;           // LED 제어 명령 설정
    tx_frame.parameter = (1 << 6) | (0 << 3) | 3;     // 1초 켜짐, 반복 없음, 3번 반복
    make_logMessage(&tx_frame, logMessage);           // 로그 메시지 생성
    writeLog(folderPath, "TX", logMessage);           // 로그 기록 추가
    LoRa_send(&modem);                                // LoRa 전송
}

////////////// 스레드 함수 정의 //////////////
// 데이터 전송을 위한 스레드 함수
void *transmit(void *arg) {
    struct timespec ts;  // 타이머 구조체
    clock_gettime(CLOCK_MONOTONIC, &ts);  // 현재 시간을 모노토닉 시계로 가져오기
    while (true) {
        memcpy(modem.tx.data.buf, &tx_frame, sizeof(Frame));  // 전송 프레임을 버퍼에 복사
        make_logMessage(&tx_frame, logMessage);  // 로그 메시지 생성

        writeLog(folderPath, "TX", logMessage);  // 전송 로그 기록

        struct timeval tv;  // 현재 시간 저장
        struct tm *tm;  // 시간 구조체
        char time_str[100];  // 시간 문자열 버퍼
        gettimeofday(&tv, NULL);  // 현재 시간 가져오기
        tm = localtime(&tv.tv_sec);  // 시간을 로컬 시간으로 변환
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm);  // 시간 문자열 생성
        printf("  Time(sec): %ld.%06ld\n", tv.tv_sec, tv.tv_usec);  // 전송 시간을 출력
        LoRa_send(&modem);  // LoRa를 통해 데이터 전송

        tx_frame.seq++;  // 프레임 시퀀스 증가
        while (ack_receive == 0) { }  // ACK 메시지를 기다림
        ack_receive = 0;  // ACK 수신 상태 초기화
        if (transmit_period == 0) { break; }  // 한 번 전송 후 종료
        ts.tv_sec += transmit_period;  // 다음 전송 시간을 설정
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);  // 지정된 시간 동안 대기
    }
    thread_status = 0;  // 전송 스레드 상태 초기화
    return NULL;
}

// 데이터를 16진수로 출력
void print_hex(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0)
            printf("\n");
    }
    if (len % 16 != 0)
        printf("\n");
}

unsigned char hexCharToByte(char high, char low) {
    unsigned char byte = 0;

    // Convert high nibble
    if (high >= '0' && high <= '9') {
        byte = (high - '0') << 4;
    } else if (high >= 'A' && high <= 'F') {
        byte = (high - 'A' + 10) << 4;
    } else if (high >= 'a' && high <= 'f') {
        byte = (high - 'a' + 10) << 4;
    } else {
        fprintf(stderr, "Invalid hex character: %c\n", high);
        exit(EXIT_FAILURE);
    }

    // Convert low nibble
    if (low >= '0' && low <= '9') {
        byte |= (low - '0');
    } else if (low >= 'A' && low <= 'F') {
        byte |= (low - 'A' + 10);
    } else if (low >= 'a' && low <= 'f') {
        byte |= (low - 'a' + 10);
    } else {
        fprintf(stderr, "Invalid hex character: %c\n", low);
        exit(EXIT_FAILURE);
    }

    return byte;
}

void parse_ndef_message(uint8_t *ndef_message, size_t length, NDEFRecord *record) {
    printf("NDEF Message (Hex):\n");
    print_hex(ndef_message, length);

    size_t offset = 0;

    while (offset < length) {
        if (offset + 1 > length) {
            printf("Incomplete NDEF header.\n");
            return;
        }

        uint8_t header = ndef_message[offset++];
        uint8_t tnf = header & 0x07;
        uint8_t mb = (header >> 7) & 0x01;
        uint8_t me = (header >> 6) & 0x01;
        uint8_t cf = (header >> 5) & 0x01;
        uint8_t sr = (header >> 4) & 0x01;
        uint8_t il = (header >> 3) & 0x01;

        if (offset + 1 > length) {
            printf("Incomplete Type Length.\n");
            return;
        }
		        uint8_t type_length = ndef_message[offset++];

        uint32_t payload_length = 0;
        if (sr) {
            if (offset + 1 > length) {
                printf("Incomplete Payload Length for SR.\n");
                return;
            }
            payload_length = ndef_message[offset++];
        } else {
            if (offset + 4 > length) {
                printf("Incomplete Payload Length for normal record.\n");
                return;
            }
            payload_length = (ndef_message[offset] << 24) | (ndef_message[offset + 1] << 16) |
                             (ndef_message[offset + 2] << 8) | ndef_message[offset + 3];
            offset += 4;
        }

        uint8_t id_length = 0;
        if (il) {
            if (offset + 1 > length) {
                printf("Incomplete ID Length.\n");
                return;
            }
            id_length = ndef_message[offset++];
        }

        // Type field
        if (offset + type_length > length) {
            printf("Type Length exceeds message length.\n");
            return;
        }
        char type_field[256];
        memcpy(type_field, &ndef_message[offset], type_length);
        type_field[type_length] = '\0';
        offset += type_length;

        // ID field (if present)
        if (il) {
            if (offset + id_length > length) {
                printf("ID Length exceeds message length.\n");
                return;
            }
            // Skipping ID field as it's not used in NDEFRecord
            offset += id_length;
        }

        // Payload field
        if (offset + payload_length > length) {
            printf("Payload Length exceeds message length.\n");
            return;
        }
        uint8_t *payload = &ndef_message[offset];
        offset += payload_length;

        printf("Record Type: %s\n", type_field);
        printf("Payload (Hex):\n");
        print_hex(payload, payload_length);

        // Handle Text record
        if (tnf == 0x01 && strcmp(type_field, "T") == 0) {
            if (payload_length < 3) { // Minimum 3 bytes: status + lang (2)
                printf("Invalid Text record payload length.\n");
                continue;
            }

            uint8_t status_byte = payload[0];
            uint8_t encoding = (status_byte & 0x80) ? 1 : 0; // UTF-16 if bit 7 is set
            uint8_t lang_length = status_byte & 0x3F; // bits 0-5

            if (payload_length < 1 + lang_length) {
                printf("Language length exceeds payload length.\n");
                continue;
            }

            char lang_code[64];
            memcpy(lang_code, &payload[1], lang_length);
            lang_code[lang_length] = '\0';

            char text_string[256];
            size_t text_length = payload_length - 1 - lang_length;
            memcpy(text_string, &payload[1 + lang_length], text_length);
            text_string[text_length] = '\0';

            printf("Status Byte: 0x%02X\n", status_byte);
            printf("Encoding: %s\n", encoding ? "UTF-16" : "UTF-8");
            printf("Language Code: %s\n", lang_code);
            printf("Text Data: %s\n", text_string);

            // Ensure the text string has the expected length (10 characters)
            if (strlen(text_string) != 10) {
                printf("Invalid text string length for mapping to NDEFRecord.\n");
                continue;
            }

            // Map text_string to NDEFRecord
            record->SecurityID[0] = hexCharToByte(text_string[0], text_string[1]);
            record->SecurityID[1] = hexCharToByte(text_string[2], text_string[3]);
            record->DeviceID[0] = hexCharToByte(text_string[4], text_string[5]);
            record->DeviceID[1] = hexCharToByte(text_string[6], text_string[7]);
            record->DeviceAddress = hexCharToByte(text_string[8], text_string[9]);

        } else if (strcmp(type_field, "application/vnd.myapp.deviceinfo") == 0) {
            // Handle your custom MIME type
            if (payload_length >= 5) {
                memcpy(record->SecurityID, payload, 2);
                memcpy(record->DeviceID, payload + 2, 2);
                record->DeviceAddress = payload[4];
            } else {
                printf("Payload too short to parse NDEFRecord.\n");
                continue;
            }
        } else {
            printf("Unknown Record Type.\n");
            continue;
        }

        // Check ME flag (Message End)
        if (me) {
            printf("End of NDEF Message.\n");
            break;
        }
    }
}

void read_ndef_message(nfc_device *pnd, nfc_context *context, NDEFRecord *record) {
    uint8_t abtRx[MAX_FRAME_LEN];
    int res;

    // Select NDEF Application
    uint8_t ndef_select_app[] = {
        0x00, 0xA4, 0x04, 0x00, 0x07,
        0xD2, 0x76, 0x00, 0x00, 0x85,
        0x01, 0x01, 0x00
    };
    res = nfc_initiator_transceive_bytes(pnd, ndef_select_app, sizeof(ndef_select_app), abtRx, sizeof(abtRx), 0);
    if (res < 0) {
        nfc_perror(pnd, "Error selecting NDEF application");
        return;
    }

    // Select NDEF File (EF.NDEF)
    uint8_t ndef_select_file[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04};
    res = nfc_initiator_transceive_bytes(pnd, ndef_select_file, sizeof(ndef_select_file), abtRx, sizeof(abtRx), 0);
    if (res < 0) {
        nfc_perror(pnd, "Error selecting NDEF file");
        return;
    }

    // Read NDEF message length (first 2 bytes)
    uint8_t read_length_cmd[] = {0x00, 0xB0, 0x00, 0x00, 0x02};
    res = nfc_initiator_transceive_bytes(pnd, read_length_cmd, sizeof(read_length_cmd), abtRx, sizeof(abtRx), 0);
    if (res < 0 || res < 2) {
        nfc_perror(pnd, "Error reading NDEF length");
        return;
    }

    // Extract the NDEF message length
    uint16_t ndef_length = (abtRx[0] << 8) | abtRx[1];
    printf("NDEF Message Length: %d bytes\n", ndef_length);

    if (ndef_length == 0 || ndef_length > (MAX_FRAME_LEN - 2)) {
        printf("Invalid NDEF message length.\n");
        return;
    }

    // Read the NDEF message
    uint8_t read_ndef_cmd[] = {
        0x00, 0xB0, 0x00, 0x02,    // Offset starts after the length bytes
        (uint8_t)(ndef_length & 0xFF)
    };
    res = nfc_initiator_transceive_bytes(pnd, read_ndef_cmd, sizeof(read_ndef_cmd), abtRx, sizeof(abtRx), 0);
    if (res < 0 || res < ndef_length) {
        nfc_perror(pnd, "Error reading NDEF message");
        return;
    }

    // Parse the NDEF message and map it to NDEFRecord
    parse_ndef_message(abtRx, ndef_length, record);
}

void write_ndef_message(nfc_device *pnd, uint8_t *ndef_message, size_t length) {
    uint8_t abtRx[MAX_FRAME_LEN];
    int res;

    if (!pnd || !ndef_message) {
        fprintf(stderr, "Invalid arguments: pnd or ndef_message is NULL\n");
        return;
    }

    // Select NDEF Application
    uint8_t ndef_select_app[] = {
        0x00, 0xA4, 0x04, 0x00, 0x07,
        0xD2, 0x76, 0x00, 0x00, 0x85,
        0x01, 0x01, 0x00
    };
    res = nfc_initiator_transceive_bytes(pnd, ndef_select_app, sizeof(ndef_select_app), abtRx, sizeof(abtRx), 0);

    if (res < 2 || abtRx[res - 2] != 0x90 || abtRx[res - 1] != 0x00) {
        fprintf(stderr, "Failed to select NDEF application. SW1 SW2: %02X %02X\n",
                res >= 2 ? abtRx[res - 2] : 0x00,
                res >= 2 ? abtRx[res - 1] : 0x00);
        return;
    }
    usleep(100000);

    // Select NDEF File (EF.NDEF)
    uint8_t ndef_select_file[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04};
    res = nfc_initiator_transceive_bytes(pnd, ndef_select_file, sizeof(ndef_select_file), abtRx, sizeof(abtRx), 0);
    if (res < 0) {
        nfc_perror(pnd, "Error selecting NDEF file");
        return;
    }
    usleep(100000);

    // Combine header and payload for a single Short Record message
    size_t cmd_length = 5 + length;
    uint8_t *write_ndef_cmd = malloc(cmd_length);
    if (!write_ndef_cmd) {
        fprintf(stderr, "Memory allocation error\n");
        return;
    }

    // Prepare UpdateBinary command to write the NDEF message
    write_ndef_cmd[0] = 0x00;           // CLA
    write_ndef_cmd[1] = 0xD6;           // INS (Write Binary)
    write_ndef_cmd[2] = 0x00;           // P1 (Offset High Byte)
    write_ndef_cmd[3] = 0x00;           // P2 (Offset Low Byte for writing the whole message starting at offset 0)
    write_ndef_cmd[4] = (uint8_t)(length & 0xFF); // Lc (length of the NDEF message in short format)

    // Copy the actual NDEF message into the command buffer
    memcpy(&write_ndef_cmd[5], ndef_message, length);

    // Send the entire NDEF message in one command
    res = nfc_initiator_transceive_bytes(pnd, write_ndef_cmd, cmd_length, abtRx, sizeof(abtRx), 0);
    free(write_ndef_cmd);
    if (res < 0) {
        nfc_perror(pnd, "Error writing NDEF message");
        return;
    }

    printf("NDEF message written successfully.\n");
}
/*
void write_ndef_message(nfc_device *pnd, uint8_t *ndef_message, size_t length) {
	uint8_t abtRx[MAX_FRAME_LEN];
    int res;

    if (!pnd || !ndef_message) {
        fprintf(stderr, "Invalid arguments: pnd or ndef_message is NULL\n");
        return;
    }
	// UpdateBinary command to write NDEF message length (first 2 bytes)
    uint8_t write_length_0[] = {
        0x00, 0xD6, 0x00, 0x00, 0x02,0x00, 0x00
    };
	// Select NDEF Application
    uint8_t ndef_select_app[] = {
        0x00, 0xA4, 0x04, 0x00, 0x07,
        0xD2, 0x76, 0x00, 0x00, 0x85,
        0x01, 0x01, 0x00
    };

	res = nfc_initiator_transceive_bytes(pnd, ndef_select_app, sizeof(ndef_select_app), abtRx, sizeof(abtRx), 0);

    if (res < 2 || abtRx[res - 2] != 0x90 || abtRx[res - 1] != 0x00) {
        fprintf(stderr, "Failed to select NDEF application. SW1 SW2: %02X %02X\n",
                res >= 2 ? abtRx[res - 2] : 0x00,
                res >= 2 ? abtRx[res - 1] : 0x00);
        return;
    }
    //usleep(100000);

    // Select NDEF File (EF.NDEF)
    uint8_t ndef_select_file[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04};
    res = nfc_initiator_transceive_bytes(pnd, ndef_select_file, sizeof(ndef_select_file), abtRx, sizeof(abtRx), 0);
    if (res < 0) {
        nfc_perror(pnd, "Error selecting NDEF file");
        return;
    }
    //usleep(100000);

    // UpdateBinary command to write NDEF message length (first 2 bytes)
    uint8_t write_length_cmd[] = {
        0x00, 0xD6, 0x00, 0x00, 0x02,
        (uint8_t)((length >> 8) & 0xFF),  // Length High Byte
        (uint8_t)(length & 0xFF)          // Length Low Byte
    };
    //res = nfc_initiator_transceive_bytes(pnd, write_length_cmd, sizeof(write_length_cmd), abtRx, sizeof(abtRx), 0);
    //if (res < 0) {
    //    nfc_perror(pnd, "Error writing NDEF length");
    //    return;
    //}
    //usleep(100000);
	//res = nfc_initiator_transceive_bytes(pnd, write_ndef_cmd, 0, abtRx, sizeof(abtRx), 0);
	
	res = nfc_initiator_transceive_bytes(pnd, write_length_cmd, sizeof(write_length_cmd), abtRx, sizeof(abtRx), 0);
    if (res < 0) {
        nfc_perror(pnd, "Error writing NDEF length");
        return;
    }
    // UpdateBinary command to write entire NDEF message in one go
    size_t cmd_length = 5 + length;
    uint8_t *write_ndef_cmd = malloc(cmd_length);
    if (!write_ndef_cmd) {
        fprintf(stderr, "Memory allocation error\n");
        return;
    }

    // Prepare command to write NDEF message
    write_ndef_cmd[0] = 0x00;  // CLA
    write_ndef_cmd[1] = 0xD6;  // INS (Write Binary)
    write_ndef_cmd[2] = 0x00;  // P1 (Offset High Byte)
    write_ndef_cmd[3] = 0x02;  // P2 (Offset Low Byte after length bytes)
    write_ndef_cmd[4] = (uint8_t)(length & 0xFF);  // Lc (length of NDEF message data)
    memcpy(&write_ndef_cmd[5], ndef_message, length);

    // Send the NDEF message in one command
    res = nfc_initiator_transceive_bytes(pnd, write_ndef_cmd, cmd_length, abtRx, sizeof(abtRx), 0);
    free(write_ndef_cmd);
    if (res < 0) {
        nfc_perror(pnd, "Error writing NDEF message");
        return;
    }

    printf("NDEF message written successfully.\n");
	    
	uint8_t abtRx[MAX_FRAME_LEN];
    int res;
	if (!pnd || !ndef_message) {
        fprintf(stderr, "Invalid arguments: pnd or ndef_message is NULL\n");
        return;
    }
	
    // Select NDEF Application
    uint8_t ndef_select_app[] = {
        0x00, 0xA4, 0x04, 0x00, 0x07,
        0xD2, 0x76, 0x00, 0x00, 0x85,
        0x01, 0x01, 0x00
    };
    res = nfc_initiator_transceive_bytes(pnd, ndef_select_app, sizeof(ndef_select_app), abtRx, sizeof(abtRx), 0);
    
	if (res < 2 || abtRx[res - 2] != 0x90 || abtRx[res - 1] != 0x00) {
        fprintf(stderr, "Failed to select NDEF application. SW1 SW2: %02X %02X\n", 
                res >=2 ? abtRx[res - 2] : 0x00, 
                res >=2 ? abtRx[res - 1] : 0x00);
        return;
    }
	usleep(100000);

	if (res < 0) {
        nfc_perror(pnd, "Error selecting NDEF application");
        return;
    }

    // Select NDEF File (EF.NDEF)
    uint8_t ndef_select_file[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04};
    res = nfc_initiator_transceive_bytes(pnd, ndef_select_file, sizeof(ndef_select_file), abtRx, sizeof(abtRx), 0);
    if (res < 0) {
        nfc_perror(pnd, "Error selecting NDEF file");
        return;
    }
	
	usleep(100000);
    // UpdateBinary command to write NDEF message length (first 2 bytes)
    uint8_t write_length_cmd[] = {
        0x00, 0xD6, 0x00, 0x00, 0x02,
        (uint8_t)((length >> 8) & 0xFF),  // Length High Byte
        (uint8_t)(length & 0xFF)          // Length Low Byte
    };
    res = nfc_initiator_transceive_bytes(pnd, write_length_cmd, sizeof(write_length_cmd), abtRx, sizeof(abtRx), 0);
    if (res < 0) {
        nfc_perror(pnd, "Error writing NDEF length");
        return;
    }
	usleep(100000);
    // UpdateBinary command to write NDEF message
    size_t offset = 0x02;  // Offset after the length bytes
    size_t chunk_size = 0xFF;  // Maximum data per command
    size_t remaining = length;
    uint8_t *data_ptr = ndef_message;

    while (remaining > 0) {
        size_t send_length = (remaining > chunk_size) ? chunk_size : remaining;
        size_t cmd_length = 5 + send_length;
        uint8_t *write_ndef_cmd = malloc(cmd_length);
        if (!write_ndef_cmd) {
            fprintf(stderr, "Memory allocation error\n");
            return;
        }
        write_ndef_cmd[0] = 0x00;
        write_ndef_cmd[1] = 0xD6;
        write_ndef_cmd[2] = (uint8_t)((offset >> 8) & 0xFF);
        write_ndef_cmd[3] = (uint8_t)(offset & 0xFF);
        write_ndef_cmd[4] = (uint8_t)(send_length & 0xFF);
        memcpy(&write_ndef_cmd[5], data_ptr, send_length);

        res = nfc_initiator_transceive_bytes(pnd, write_ndef_cmd, cmd_length, abtRx, sizeof(abtRx), 0);
        free(write_ndef_cmd);
        if (res < 0) {
            nfc_perror(pnd, "Error writing NDEF message");
            return;
        }

        offset += send_length;
        data_ptr += send_length;
        remaining -= send_length;
		usleep(500000);
    }

    printf("NDEF message written successfully.\n");


}
*/
// Global flag for graceful termination
int keep_running = 1;

// Signal handler to catch Ctrl+C
void int_handler(int dummy) {
    keep_running = 0;
}

void build_data(NDEFRecord record, size_t *length, unsigned char *ndef_message){
	// Prepare the NDEF message as a Text record
	size_t offset = 0; 

	// Header byte: MB=1, ME=1, CF=0, SR=0, IL=0, TNF=0x01 (Well-Known Type)
	ndef_message[offset++] = 0xD1;

	// Type Length: 1 byte (Type Field is 'T')
	ndef_message[offset++] = 0x01;

	// Payload Length: 13 bytes (specified as 4 bytes because SR=0)
	uint32_t payload_length = 13;
	ndef_message[offset++] = payload_length;
	//uint8_t payload_length = 13;  // Short record, so use 1 byte
    //ndef_message[offset++] = payload_length;
	// Type Field: 'T'
	ndef_message[offset++] = 0x54;

	// Convert NDEFRecord back to text string
	char text_string[11];
	snprintf(text_string, sizeof(text_string), "%02X%02X%02X%02X%02X",
		 record.SecurityID[0], record.SecurityID[1],
		 record.DeviceID[0], record.DeviceID[1],
		 record.DeviceAddress);

	// Payload:
	unsigned char payload[15];
	payload[0] = 0x02; // Status byte
	payload[1] = 'e';
	payload[2] = 'n';
	memcpy(&payload[3], text_string, 10); // "0000F02304"
	

	memcpy(&ndef_message[offset], payload, payload_length);
	offset += payload_length;
	
	// 6. Set the total length
    *length = offset;


}

// NFC 태그 감지 및 데이터 전송을 위한 스레드 함수
void *nfc_tag() {
    nfc_device *pnd = NULL;
    nfc_context *context = NULL;
    nfc_target nt;
	// Register signal handler for graceful termination
    signal(SIGINT, int_handler);

	// Main loop to continuously read NFC tags
   
    uint8_t abtRx[MAX_FRAME_LEN];
    int res;
	
	const nfc_modulation nm = {
        .nmt = NMT_ISO14443A,
        .nbr = NBR_106,
    };

	NDEFRecord record;
	sleep(1);// waith other thread and main work
	while (keep_running) {
retry_init:
		nfc_init(&context);
		if (context == NULL) {
        	fprintf(stderr, "Unable to init libnfc\n");
        	exit(EXIT_FAILURE);
    	}
    	// Open the NFC device
		pnd = nfc_open(context, NULL);
		if (pnd == NULL) {
			fprintf(stderr, "Error opening NFC device, will be retry in 1 second ...\n");
			nfc_exit(context);
			sleep(1);
			goto retry_init;
		}
		// Initiate the device
		if (nfc_initiator_init(pnd) < 0) {
			nfc_perror(pnd, "nfc_initiator_init");
			//nfc_close(pnd);
			nfc_exit(context);
			sleep(1);
            goto retry_init;
			//exit(EXIT_FAILURE);
		}
		//printf("NFC reader: %s opened\n", nfc_device_get_name(pnd));
			
		record.SecurityID[0] = 0;
        record.SecurityID[1] = 0;
        record.DeviceAddress = 0;

		//printf("\nWaiting for an NFC tag...\n");
		
		// Poll for a passive target (with a timeout of 2 seconds)
        //res = nfc_initiator_select_passive_target(pnd, nm, NULL, 0, &nt);
        if (nfc_initiator_select_passive_target(pnd, nm, NULL, 0, &nt) > 0) {
            printf("NFC Tag detected.\n");
			unsigned char detected_address = 0;
			if(nt.nti.nai.szUidLen > 0) {
				
				for (size_t szPos = 0; szPos < nt.nti.nai.szUidLen; szPos++) {
					printf("%02x  ", nt.nti.nai.abtUid[szPos]);  // UID의 각 바이트를 출력
					detected_address ^= nt.nti.nai.abtUid[szPos];  // UID의 각 바이트를 XOR하여 address 생성
				}	
			}
			printf("\nDetected Address: 0x%02x\n", detected_address);  // 생성된 주소 출력
            
			register_address(detected_address);
			
			// Read and parse the NDEF message from the tag
            ///read_ndef_message(pnd, context, &record);

            // Print the contents of the NDEFRecord
            printf("Parsed NDEFRecord:\n");
            printf("SecurityID: %02X %02X\n", record.SecurityID[0], record.SecurityID[1]);
            printf("DeviceID: %02X %02X\n", record.DeviceID[0], record.DeviceID[1]);
            printf("DeviceAddress: %02X\n", record.DeviceAddress);
			if(( record.SecurityID[0] == 0 ) && ( record.SecurityID[1] == 0))
            {
				printf("Start register:\n");
				record.SecurityID[0] = MySecurityID[0];;
                record.SecurityID[1] = MySecurityID[1];;
				record.DeviceAddress = detected_address;
				unsigned char ndef_message[50]; // Allocate sufficient space
            	size_t length = 0;

            	build_data(record,&length, ndef_message);


            	printf("NDEF Message (Hex):\n");
            	for (size_t i = 0; i < length; i++) {
                	printf("%02X ", ndef_message[i]);
            	}
            	printf("\nTotal Length: %zu bytes\n", length);
            	write_ndef_message(pnd, ndef_message, length);
				printf("Updated NDEF message written successfully.\n");
            	printf("----------------------------------------\n");
            	nfc_close(pnd);
            	nfc_exit(context);


			}
			else{
				printf("Already registered ...\n");
				/*
				record.SecurityID[0] = 0;
                record.SecurityID[1] = 0;
                record.DeviceAddress = 0;
				unsigned char ndef_message[50]; // Allocate sufficient space
                size_t length = 0;

                build_data(record,&length, ndef_message);


                printf("NDEF Message (Hex):\n");
                for (size_t i = 0; i < length; i++) {
                    printf("%02X ", ndef_message[i]);
                }
                printf("\nTotal Length: %zu bytes\n", length);
                write_ndef_message(pnd, ndef_message, length);
                printf("Updated NDEF message written successfully.\n");
                printf("----------------------------------------\n");
                nfc_close(pnd);
                nfc_exit(context);
				return;
				*/	
			}
            
						
			// 메시지 전송: "Register" 대신 원하는 type ("Tx", "Rx", "NoAck")을 설정
            send_message("Tx", record.DeviceAddress, "", record.DeviceID, sizeof(record.DeviceID));

            // MQTT에 등록 메시지 전송
            char uid[16];
            snprintf(uid, sizeof(uid), "%02X%02X", record.DeviceID[0], record.DeviceID[1]);
            send_message("Register", record.DeviceAddress, "", (unsigned char *)uid, strlen(uid));

			
        }
		// Close the NFC device and exit libnfc
        //nfc_close(pnd);
        //nfc_exit(context);

		sleep(5);
	}
    return 0;
}

////////////// Mosquitto 함수 정의 //////////////
// MQTT 브로커에 연결되었을 때 호출되는 함수
void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        mosquitto_subscribe(mosq, NULL, "lora/command", 0);  // 토픽 구독
    } else {
        printf("Failed to connect, return code %d\n", rc);  // 연결 실패 시 오류 코드 출력
    }
}

// MQTT 메시지를 수신했을 때 호출되는 함수
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    printf("Received raw message from server: %s\n", (char *)msg->payload);
    
    // 수신된 메시지로 스레드가 이미 동작 중이면 중단
    if (thread_status == 1) {
        pthread_cancel(threads[1]);
        pthread_join(threads[1], NULL);
    }

    // JSON 메시지 파싱
    json_object *jobj = json_tokener_parse((char *)msg->payload);
    json_object *address, *command, *led, *params, *period, *led_bright, *ir, *gps, *transmission;  // ir 추가

    unsigned char r_address, r_command, r_LED = 0, r_time = 0, r_cycle = 0, r_count = 0, r_period = 0, r_brightness = 0, r_IR = 0, r_GPS = 0, r_transmission = 0;

    // address 필드 추출
    if (json_object_object_get_ex(jobj, "address", &address)) {
        r_address = (unsigned char)json_object_get_int(address);
    }
    is_broadcast = (r_address == 0xFF) ? 1 : 0;

    // command 필드 추출
    if (json_object_object_get_ex(jobj, "command", &command)) {
        r_command = (unsigned char)json_object_get_int(command);
    }
	
    // 명령이 5인 경우 송신을 중단하고 idle 상태로 복귀
    if (r_command == 5) {
        json_object_put(jobj);
        printf("Command 5 received, returning to idle state.\n");
        return;
    }
	if (r_command == 1) {
		// GPS
		if (json_object_object_get_ex(jobj, "gps", &gps)) {
			r_GPS = (unsigned char)json_object_get_int(gps);
		}
		// transmission for Lora power
		if (json_object_object_get_ex(jobj, "transmission", &transmission)) {
			r_transmission = (unsigned char)json_object_get_int(transmission);
		}
	}
    // params 필드 추출
    if (json_object_object_get_ex(jobj, "params", &params)) {
        json_object *turn_on_time, *cycle, *count;
        if (json_object_object_get_ex(params, "turn_on_time", &turn_on_time)) {
            r_time = (unsigned char)(json_object_get_double(turn_on_time) / 0.25);
        }
        if (json_object_object_get_ex(params, "cycle", &cycle)) {
            r_cycle = (unsigned char)json_object_get_int(cycle) - 1;
        }
        if (json_object_object_get_ex(params, "count", &count)) {
            r_count = (unsigned char)json_object_get_int(count);
        }
    }

    // LED 밝기 처리
    if (json_object_object_get_ex(params, "led_bright", &led_bright)) {
        const char *bright_str = json_object_get_string(led_bright);
        if (strcmp(bright_str, "00") == 0) {
            r_brightness = 0x00;  // 약한 밝기
        } else if (strcmp(bright_str, "01") == 0) {
            r_brightness = 0x01;  // 중간 밝기
        } else if (strcmp(bright_str, "10") == 0) {
            r_brightness = 0x02;  // 강한 밝기
        }
    }

    // period 필드 추출
    if (json_object_object_get_ex(jobj, "period", &period)) {
        r_period = (unsigned char)json_object_get_int(period);
    }

    // LED 및 진동 모터 제어 값 추출
    if (json_object_object_get_ex(jobj, "led", &led)) {
        json_object *red, *green, *blue, *motor;

        if (json_object_object_get_ex(led, "RED", &red)) {
            r_LED |= json_object_get_int(red);  // RED LED control
        }
        if (json_object_object_get_ex(led, "GREEN", &green)) {
            r_LED |= json_object_get_int(green) << 1;  // GREEN LED control
        }
        if (json_object_object_get_ex(led, "BLUE", &blue)) {
            r_LED |= json_object_get_int(blue) << 2;  // BLUE LED control
        }
        if (json_object_object_get_ex(led, "MOTOR", &motor)) {
            r_LED |= json_object_get_int(motor) << 3;  // 진동 모터 control
        }
    }

    // IR 값 추출 및 설정
    if (json_object_object_get_ex(jobj, "IR", &ir)) {
        if (json_object_get_int(ir) == 1) {
            r_IR = 1 << 4;  // IR이 켜졌을 때 비트를 설정
            printf("IR is active\n");
        } else {
            printf("IR is inactive\n");
        }
    } else {
        printf("No IR data found\n");  // IR 필드가 없을 경우 로그 출력
    }

    // 전송 프레임 설정
    tx_frame.address = r_address;

    if (r_time == 0) {
        r_command = 3;  // turn on 명령으로 설정
    } else {
        r_time -= 1;  // 시간 값 설정 (0~3)
    }
	printf("tx_frame:\n");
    // LED, 진동 모터, IR을 포함한 command 설정
    tx_frame.command = (r_command << 4) | r_LED | r_IR;  // 명령 및 LED/모터/IR 설정
	printf("	tx_frame.command: 0x%02X \n",tx_frame.command);
    // parameter 설정
    if (r_command == 3) { 
        tx_frame.parameter = ((r_cycle + 1) * r_count * 4) + 1;  // parameter에 주기 설정
    } else if (r_command == 1) { 
        tx_frame.parameter = (r_time << 6) | (r_cycle << 3) | r_count;  // 시간, 주기, 반복 횟수 설정
		tx_frame.reserved[1] = (r_GPS & 0x01)  | (r_transmission << 1);
		//printf("	reserved[1]: 0x%02X \n",tx_frame.reserved[1]);
    } else { 
        tx_frame.parameter = 0;  // 기타 명령의 경우 parameter는 0으로 설정
    }
	printf("    tx_frame.parameter: 0x%02X \n",tx_frame.parameter);
    // reserved[0]에 LED 밝기 값 설정 (reserved에 LED 밝기 반영)
    tx_frame.reserved[0] = r_brightness;  // led_bright 값 설정
	printf("    tx_frame.reserved[1]: 0x%02X \n",tx_frame.reserved[1]);
	printf("	tx_frame.reserved[0]: 0x%02X \n",tx_frame.reserved[0]);
    // 전송 주기 설정
    transmit_period = r_period;

    // 전송할 frame을 버퍼로 복사
    memcpy(modem.tx.data.buf, &tx_frame, sizeof(Frame));

    // JSON 객체 해제
    json_object_put(jobj);

    // 전송 스레드 시작
    thread_status = 1;
    pthread_create(&threads[1], NULL, transmit, NULL);
    pthread_detach(threads[1]);
}

// 메시지를 JSON 형식으로 변환하여 전송하는 함수
void send_message(const char *message, unsigned char address, const char *mac, const unsigned char *uid, size_t uid_len) {
    char payload[512];  // 메시지를 저장할 버퍼
    int rc;

    // UID를 문자열로 변환 (DeviceID에 해당)
    char uid_str[64] = {0};
    for (size_t i = 0; i < uid_len; i++) {
        snprintf(uid_str + i * 2, sizeof(uid_str) - i * 2, "%02x", uid[i]);
    }

    // Master ID를 문자열로 설정 ("1" 또는 "2")
    const char *master_id_str = (MASTER_ID == MASTER1_ID) ? "1" : "2";

    // 메시지 형식에 맞게 JSON 메시지 생성
    snprintf(payload, sizeof(payload), "{\"type\": \"%s\", \"address\": %d, \"mac\": \"%s\", \"uid\": \"%s\", \"masterID\": \"%s\"}",
             message, address, mac, uid_str, master_id_str);

    // MQTT를 통해 메시지 전송
    rc = mosquitto_publish(mosq, NULL, "lora/status", strlen(payload), payload, 0, false);  // 지정된 토픽으로 전송
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Failed to publish message: %s\n", mosquitto_strerror(rc));  // 전송 실패 시 오류 메시지 출력
    }
    printf("Sent message: %s\n", payload);  // 전송된 메시지 출력
}


////////////// Ack Timeout 함수 정의 //////////////
// Ack 메시지 수신 타임아웃 시 호출되는 함수
void timeout_handler(int signum) {
    if (!ack_receive) {  // ACK 수신 실패
        printf("Timeout: No response received.\n");  // 타임아웃 메시지 출력
        sprintf(logMessage, "Address %02x device is inactive", tx_frame.address);  // 로그 메시지 생성
        writeLog(folderPath, "No Ack", logMessage);  // 로그 파일에 기록
        send_message("NoAck", tx_frame.address, "", NULL, 0);  // "NoAck" 메시지 전송
    }
}

// ACK 수신 타임아웃 설정 함수
void set_timeout(int seconds) {
    struct sigaction sa;  // 신호 처리 구조체
    struct itimerval timer;  // 타이머 구조체

    // 타임아웃 핸들러 설정
    sa.sa_handler = &timeout_handler;
    sa.sa_flags = SA_RESTART;
    sigaction(SIGALRM, &sa, NULL);

    // 타이머 설정
    timer.it_value.tv_sec = seconds;  // 타이머 값 설정
    timer.it_value.tv_usec = 0;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    setitimer(ITIMER_REAL, &timer, NULL);  // 타이머 시작
}

////////////// SecurityID 및 주소 처리 함수 //////////////
// 보안 ID를 무작위로 생성하는 함수
void set_securityID(unsigned char *securityID) {
    srand(time(NULL));  // 난수 생성기 초기화
    for (int i = 0; i < 2; i++) {
        // 보안 ID를 랜덤 값으로 설정
        securityID[i] = rand() % 256;// 테스트를 위해 고정된 값으로 설정
		//securityID[i] = 0xFF;  // 테스트를 위해 고정된 값으로 설정
    }
	securityID[0] = 0x5a;
    securityID[1] = 0x5a;	
}

// 수신된 보안 ID를 확인하는 함수
int check_ID(unsigned char receiveID[2]) {
    return ((receiveID[0] == MySecurityID[0] && receiveID[1] == MySecurityID[1]) || 
            (receiveID[0] == 0xFF && receiveID[1] == 0xFF));  // 받은 ID가 내 ID와 같거나 0xFF이면 true 반환
}

// 수신된 주소를 확인하는 함수
int check_address(unsigned char receive_address) {
    if (receive_address == ADDRESS) { return 1; }  // 받은 주소가 내 주소와 같으면 true 반환
    return 0;
}

///////////////Master to Slave/////////////////////////
////////////// 전송 및 수신 콜백 함수 정의 //////////////
// 전송 콜백 함수
void tx_f(txData *tx) {
    LoRa_ctl *modem = (LoRa_ctl *)(tx->userPtr);  // LoRa 모뎀 포인터 가져오기
    writeLog(folderPath, "TX", logMessage);  // 전송 로그 작성
    send_message("Tx", tx_frame.address, "", NULL, 0);  // 전송 메시지 전송

    // 송신 직후에 전송된 프레임 정보 출력
    printf("Sent message to address: 0x%02x\n", tx_frame.address);
    printf("Security ID: %02x %02x, Command: %02x, Seq: %d\n", 
        tx_frame.securityID[0], tx_frame.securityID[1], tx_frame.command, tx_frame.seq);


    if (is_broadcast == 0) {  // 브로드캐스트 메시지가 아니면 ACK 대기
        LoRa_receive(modem);  // 수신 모드로 전환
        set_timeout(6);  // 6초 타임아웃 설정
    } else { 
        ack_receive = 1;  // 브로드캐스트인 경우 ACK 수신 상태로 설정
    } 
}

// 수신 콜백 함수
void *rx_f(void *p){
    rxData *rx = (rxData *)p;  // 수신된 데이터 포인터 가져오기
    LoRa_ctl *modem = (LoRa_ctl *)(rx->userPtr);  // LoRa 모뎀 포인터 가져오기
    Frame * frame = (Frame *)(rx->buf);  // 수신된 프레임 데이터 포인터 가져오기

    // 수신된 프레임 정보 출력
    printf("Received message from address: 0x%02x\n", frame->address);
    printf("Security ID: %02x %02x, Command: %02x, Seq: %d\n", 
        frame->securityID[0], frame->securityID[1], frame->command, frame->seq);


    // 보안 ID 및 주소 확인
    if ((!check_ID(frame->securityID)) || (!check_address(frame->address))) { 
        free(p);  // 내 ACK가 아닌 경우 메모리 해제
        return NULL;  // 함수 종료
    }  
    ack_receive = 1;  // ACK 수신 상태로 설정
    set_timeout(0);  // 타이머 중지
    printf("  Address: 0x%02x",frame->address);
    printf("  Ack: %d\n\n",frame->seq);
    make_logMessage(frame, logMessage);  // 수신된 프레임 데이터를 로그 메시지로 변환
    writeLog(folderPath, "RX", logMessage);  // 로그 파일에 기록
    send_message("Rx", tx_frame.address, "", NULL, 0);  // 수신 메시지 전송
    free(p);  // 메모리 해제
    return NULL;
}


// 송신 콜백 함수 (마스터 간 통신)
void mtx(txData *tx) {
    LoRa_ctl *modem = (LoRa_ctl *)(tx->userPtr);
    make_logMessage(&tx_frame, logMessage);        // 로그 메시지 생성
    writeMasterLog("MTX", logMessage);  // 송신 로그 저장
    send_message("MTx", tx_frame.address, "", NULL, 0);
    if (is_broadcast == 0) {
        LoRa_receive(modem);
        set_timeout(6);
    } else {
        ack_receive = 1;
    }
}

// 수신 콜백 함수 (마스터 간 통신)
void *mrx(void *p) {
    rxData *rx = (rxData *)p;
    LoRa_ctl *modem = (LoRa_ctl *)(rx->userPtr);
    Frame *frame = (Frame *)(rx->buf);

    if ((!check_ID(frame->securityID)) || (!check_address(frame->address))) {
        free(p);
        return NULL;
    }

    ack_receive = 1;
    set_timeout(0);
    printf("  Address: 0x%02x", frame->address);
    printf("  Ack: %d\n\n", frame->seq);
    make_logMessage(frame, logMessage);
    writeMasterLog("MRX", logMessage);  // 수신 로그 저장
    free(p);
    return NULL;
}
////////////// 메인 함수 //////////////
int main() {
     // 플래시 메모리 초기화 및 기존 데이터 로드
    flash_init();
    flash_read();


    pthread_create(&threads[0], NULL, nfc_tag, NULL);
    pthread_detach(threads[0]);


    set_securityID(MySecurityID);
    tx_frame.securityID[0] = MySecurityID[0];
    tx_frame.securityID[1] = MySecurityID[1];

    char txbuf[500];
    memset(txbuf, 0, sizeof(txbuf));

    modem.spiCS = 0;
    modem.tx.callback = mtx;  // 마스터 송신 콜백 설정
    modem.tx.data.buf = txbuf;
    modem.rx.callback = mrx;  // 마스터 수신 콜백 설정
    modem.rx.data.userPtr = (void *)(&modem);  // To handle with chip from rx callback
    modem.tx.data.userPtr = (void *)(&modem); // To handle with chip from tx callback
    modem.tx.data.size = sizeof(Frame); // Payload len for tx on Explicit Header Mode
    modem.eth.preambleLen = 6;          //Preable 6symbols
    modem.eth.bw = BW125;               // Bandwidth 125KHz
    modem.eth.sf = SF8;                 //Spreading Factor 8   
    modem.eth.ecr = CR8;                //Error coding rate CR4/8
    modem.eth.CRC = 1;                  //Turn on CRC checking
    modem.eth.freq = 915000000;         //Frequency 915MHz
    modem.eth.resetGpioN = 4;           //GPI04 on lora RESET pi
    modem.eth.dio0GpioN = 17;           // GPI017 on lora DIO0 pin to control Rxdone and Txdone interrupts
    modem.eth.outPower = OP20;          //Output power
    modem.eth.powerOutPin = PA_BOOST;   //Power Amplifire pin
    modem.eth.AGC = 1;                  //Auto Gain Control
    modem.eth.OCP = 240;                //45 to 240 mA. 0 to turn off protection
    modem.eth.implicitHeader = 0;       //Explicit Header Mode
    modem.eth.syncWord = 0x12;          //Sync Word

    LoRa_begin(&modem);

    // Setting Mosquitto
    int rc;
    // intit mosquitto library
    mosquitto_lib_init();
    // generate client
    mosq = mosquitto_new("subscriber-client", true, NULL);
    if (!mosq) {
        fprintf(stderr, "Error: Out of memory.\n");
        return 1;
    }
    // setting callback function
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);
    rc = mosquitto_connect(mosq, "localhost", 1883, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Unable to connect: %s\n", mosquitto_strerror(rc));
        return 1;
    }
    // Loop Forever until end
    mosquitto_loop_forever(mosq, -1, 1);

    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    LoRa_end(&modem);
}
