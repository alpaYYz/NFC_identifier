
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

#define FLASH_FILE "flash_memory.bin"  // 플래시 메모리 파일 이름 정의
#define MAX_FRAME_LEN 264 // NDEF

// 마스터의 보안 ID 및 주소와 다음 슬레이브 주소
unsigned char MySecurityID[2]; 
unsigned char ADDRESS = 127; 
unsigned char next_slaveADDRESS = 1; 

// 로그를 저장할 경로와 로그 메시지
const char *folderPath = "./logs";  // 장치 로그 파일 경로
const char *m_logPath = "./m_logs"; // 마스터 간 통신 로그 파일 경로
const char *ndef_logPath = "./ndef_logs";  // NDEF 로그 파일 경로 (새롭게 추가된 경로)
char logMessage[256];               // 로그 메시지를 저장할 버퍼


// NFC 태그 감지 및 데이터 전송을 위한 pthread 변수
pthread_t threads[2];
unsigned char thread_status = 0;

// 전송 프레임 구조체 정의
typedef struct {
    unsigned char securityID[2]; // 보안 ID(2바이트)
    unsigned char address; // 장치주소 (1바이트)
    unsigned char seq; // 전송 시퀀스(1바이트, 송수신되는 메시지의 순서를 나타냄)
    unsigned char command; // 명령어(1바이트)
    unsigned char parameter; // 명령어에 따른 추가 매개변수(1바이트)
    unsigned char reserved[2]; // 예약된 공간(2바이트)
} Frame;


// Define the NDEFRecord data structure
typedef struct {
    unsigned char SecurityID[2];   // 보안 ID (2바이트)
    unsigned char DeviceID[2];     // 장치 ID (2바이트)
    unsigned char DeviceAddress;   // 장치 주소 (1바이트)
} NDEFRecord;

// 전송 및 수신을 위한 변수들
unsigned char ack_receive;
unsigned char is_broadcast;
unsigned char transmit_period = 0;

// 송신 프레임 초기화
Frame tx_frame = {
    .seq = 0};

// LoRa 및 MQTT용 변수들
LoRa_ctl modem;
struct mosquitto *mosq;

// 등록된 슬레이브 주소를 저장할 최대 개수 및 배열
#define MAX_DEVICES 100

// 플래시 메모리에 저장할 데이터 구조체 정의
typedef struct {
    unsigned char registered_addresses[MAX_DEVICES];
    unsigned int registered_count;
    unsigned char MySecurityID[2];
} FlashData;

FlashData flash_data;

// NDEF 메시지 파싱을 위한 함수 선언
void print_hex(const uint8_t *data, size_t len);
void parse_ndef_message(uint8_t *ndef_message, size_t length);
void write_ndef_message(nfc_device *pnd, uint8_t *ndef_message, size_t length);

void writeLog(const char *folderPath, const char *tag, const char *logMessage);

// 플래시 메모리 초기화 함수
void flash_init() {
    int fd = open(FLASH_FILE, O_RDWR | O_CREAT, 0644);
    if (fd == -1) {
        perror("Failed to open flash memory file");
        exit(1);
    }

    // 플래시 메모리 초기화: 파일 크기가 0이면 새로운 데이터를 기록
    if (lseek(fd, 0, SEEK_END) == 0) {
        write(fd, &flash_data, sizeof(FlashData));  // 플래시 메모리 초기화
    }

    close(fd);
}

// 플래시 메모리에서 데이터 읽기 함수
void flash_read() {
    int fd = open(FLASH_FILE, O_RDONLY);
    if (fd == -1) {
        perror("Failed to open flash memory file for reading");
        return;
    }

    read(fd, &flash_data, sizeof(FlashData));
    close(fd);
}

// 플래시 메모리에 데이터 쓰기 함수
void flash_write() {
    int fd = open(FLASH_FILE, O_WRONLY);
    if (fd == -1) {
        perror("Failed to open flash memory file for writing");
        return;
    }

    write(fd, &flash_data, sizeof(FlashData));
    close(fd);
}

// 새로운 주소를 등록하는 함수 (플래시 메모리 연동)
void register_address(unsigned char address) {
    for (unsigned int i = 0; i < flash_data.registered_count; i++) {
        if (flash_data.registered_addresses[i] == address) {
            printf("Address 0x%02x is already registered.\n", address);
            return;
        }
    }

    if (flash_data.registered_count < MAX_DEVICES) {
        flash_data.registered_addresses[flash_data.registered_count++] = address;
        flash_write();  // 등록 후 플래시 메모리에 기록
        printf("Address 0x%02x registered successfully.\n", address);
    } else {
        printf("Address list is full. Cannot register new address.\n");
    }
}

// 등록된 장치 리스트
unsigned char registered_addresses[MAX_DEVICES];
unsigned int registered_count = 0;

// 마스터 식별 ID 및 상태 정의
#define MASTER1_ID 0x01
#define MASTER2_ID 0x02
unsigned char MASTER_ID = MASTER1_ID; // 현재 마스터의 ID
char *master_status = "idle";         // 기본 상태는 idle

// 마스터 통신 시작/종료 상태
bool master_communication_active = false;

// LED 제어 관련 상수 정의
#define LED3_PIN 3                // LED 3에 해당하는 핀 번호
#define COMMAND_LED_CONTROL 0x18  // LED 제어 명령어 정의

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

    // 밝기 레벨을 reserved에 저장 (2비트로)
    tx_frame.reserved[0] = (brightness_level & 0x03) << 6;  // 상위 2비트에 LED 밝기 값을 저장
    tx_frame.parameter = 0;  // parameter 값은 다른 용도로 설정 가능 (예: 0으로 설정)

    // 로그 파일에 기록
    snprintf(logMessage, sizeof(logMessage), "LED Brightness set in Reserved[0]: %02x", tx_frame.reserved[0]);
    writeLog(folderPath, "TX", logMessage);

    // LoRa 전송
    LoRa_send(&modem);  
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

// 전송 프레임 데이터를 포함한 로그 메시지를 생성하는 함수 (Reserved 비트 포함)
void make_logMessage(Frame *frame, char *logMessage) {
    sprintf(logMessage, "Security ID: %02x %02x Address: %02x Seq: %d Command: %02x Parameter: %02x Reserved: %02x %02x",
            frame->securityID[0], frame->securityID[1], frame->address, frame->seq, frame->command, frame->parameter,
            frame->reserved[0]);
}


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


void write_ndef_message(nfc_device *pnd, uint8_t *ndef_message, size_t length) {
    uint8_t abtRx[MAX_FRAME_LEN];
    int res;

    // UpdateBinary command to write NDEF message length (first 2 bytes)
    uint8_t write_length_cmd[] = {
        0x00, 0xD6, 0x00, 0x00, 0x02,
        (uint8_t)((length >> 8) & 0xFF),  // Length High Byte
        (uint8_t)(length & 0xFF)          // Length Low Byte
    };
    res = nfc_initiator_transceive_bytes(pnd, write_length_cmd, sizeof(write_length_cmd), abtRx, sizeof(abtRx), 0);
    if (res < 0) {
        nfc_perror(pnd, "Error writing NDEF length");
        nfc_close(pnd);
        nfc_exit(NULL);
        exit(EXIT_FAILURE);
    }

    // UpdateBinary command to write NDEF message
    size_t offset = 0x02;  // Offset after the length bytes
    size_t chunk_size = 0xFF;  // Maximum data per command
    size_t remaining = length;
    uint8_t *data_ptr = ndef_message;

    while (remaining > 0) {
        size_t send_length = (remaining > chunk_size) ? chunk_size : remaining;
        size_t cmd_length = 5 + send_length;
        uint8_t *write_ndef_cmd = malloc(cmd_length);
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
            nfc_close(pnd);
            nfc_exit(NULL);
            exit(EXIT_FAILURE);
        }

        offset += send_length;
        data_ptr += send_length;
        remaining -= send_length;
    }

    printf("NDEF message written successfully.\n");
}

void parse_ndef_message(uint8_t *ndef_message, size_t length) {
    printf("NDEF Message (Hex):\n");
    print_hex(ndef_message, length);

    // Parse the NDEF message
    size_t offset = 0;

    while (offset < length) {
        uint8_t header = ndef_message[offset++];
        uint8_t tnf = header & 0x07;
        uint8_t type_length = ndef_message[offset++];
        uint32_t payload_length = 0;

        // Check SR flag
        if (header & 0x10) {
            // Short Record
            payload_length = ndef_message[offset++];
        } else {
            // Normal Record (not handled in this example)
            printf("Non-short records are not supported in this example.\n");
            return;
        }

        // Type field
        char type_field[256];
        memcpy(type_field, &ndef_message[offset], type_length);
        type_field[type_length] = '\0';
        offset += type_length;

        // Payload
        uint8_t payload[256];
        memcpy(payload, &ndef_message[offset], payload_length);
        offset += payload_length;

        printf("Record Type: %s\n", type_field);
        printf("Payload (Hex):\n");
        print_hex(payload, payload_length);

        // If the type matches our custom MIME type, parse the payload
        if (strcmp(type_field, "application/vnd.myapp.deviceinfo") == 0) {
            if (payload_length >= 5) {
                NDEFRecord record;
                memcpy(record.SecurityID, payload, 2);
                memcpy(record.DeviceID, payload + 2, 2);
                record.DeviceAddress = payload[4];

                printf("Parsed NDEFRecord:\n");
                printf("SecurityID: %02X%02X\n", record.SecurityID[0], record.SecurityID[1]);
                printf("DeviceID: %02X%02X\n", record.DeviceID[0], record.DeviceID[1]);
                printf("DeviceAddress: %02X\n", record.DeviceAddress);
            } else {
                printf("Payload too short to parse NDEFRecord.\n");
            }
        } else {
            printf("Unknown Record Type.\n");
        }

        // Check ME flag (Message End)
        if (header & 0x40) {
            break; // Last record
        }
    }
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

// // NFC 태그 감지를 위한 스레드 함수
// void *nfc_tag() {
//     nfc_device *pnd = NULL;
//     nfc_context *context = NULL;
//     nfc_target nt;

// retry_init:    
//     nfc_init(&context);

//     if (context == NULL) {
//         fprintf(stderr, "Unable to init libnfc\n");
//         exit(EXIT_FAILURE);
//     }
//     // Open the NFC device
//     pnd = nfc_open(context, NULL);
//     if (pnd == NULL) {
//         fprintf(stderr, "Error opening NFC device, will be retry in 1 second ...\n");
//         nfc_exit(context);
// 	sleep(1);
// 	goto retry_init;
//         //exit(EXIT_FAILURE);
//     }
//     // Initiate the device
//     if (nfc_initiator_init(pnd) < 0) {
//         nfc_perror(pnd, "nfc_initiator_init");
//         nfc_close(pnd);
//         nfc_exit(context);
//         exit(EXIT_FAILURE);
//     }
//     printf("NFC reader: %s opened\n", nfc_device_get_name(pnd));

//     // Configure the NFC device
//     const nfc_modulation nm = {
//         .nmt = NMT_ISO14443A,
//         .nbr = NBR_106,
//     };
//     // Poll for a target
//     if (nfc_initiator_select_passive_target(pnd, nm, NULL, 0, &nt) <= 0) {
//         printf("No target found.\n");
//         nfc_close(pnd);
//         nfc_exit(context);
//         exit(EXIT_FAILURE);
//     }
//     printf("NFC Tag detected.\n");
//     uint8_t abtRx[MAX_FRAME_LEN];
//     int res;

//     // Select NDEF Application
//     uint8_t ndef_select_app[] = {
//         0x00, 0xA4, 0x04, 0x00, 0x07,
//         0xD2, 0x76, 0x00, 0x00, 0x85,
//         0x01, 0x01, 0x00
//     };
//     res = nfc_initiator_transceive_bytes(pnd, ndef_select_app, sizeof(ndef_select_app), abtRx, sizeof(abtRx), 0);
//     if (res < 0) {
//         nfc_perror(pnd, "Error selecting NDEF application");
//         nfc_close(pnd);
//         nfc_exit(context);
//         exit(EXIT_FAILURE);
//     }

//     // Select NDEF File (EF.NDEF)
//     uint8_t ndef_select_file[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04};
//     res = nfc_initiator_transceive_bytes(pnd, ndef_select_file, sizeof(ndef_select_file), abtRx, sizeof(abtRx), 0);
//     if (res < 0) {
//         nfc_perror(pnd, "Error selecting NDEF file");
//         nfc_close(pnd);
//         nfc_exit(context);
//         exit(EXIT_FAILURE);
//     }

//     // Prepare your custom NDEF message using the NDEFRecord structure
//     NDEFRecord record;
//     record.SecurityID[0] = 0x01; // Example values
//     record.SecurityID[1] = 0x02;
//     record.DeviceID[0] = 0x03;
//     record.DeviceID[1] = 0x04;
//     record.DeviceAddress = 0x05;

//     // Build the payload (5 bytes)
//     unsigned char payload[5];
//     memcpy(payload, record.SecurityID, 2);
//     memcpy(payload + 2, record.DeviceID, 2);
//     payload[4] = record.DeviceAddress;

//     // Build the NDEF message
//     unsigned char ndef_message[256];
//     size_t offset = 0;

//     // Header byte: MB=1, ME=1, CF=0, SR=1, IL=0, TNF=0x02 (MIME Media Type)
//     ndef_message[offset++] = 0xD2;

//     // Type Length: Length of MIME type string
//     const char mime_type[] = "application/vnd.myapp.deviceinfo";
//     size_t type_length = strlen(mime_type);
//     ndef_message[offset++] = (unsigned char)type_length;

//     // Payload Length: 5 bytes for your data
//     ndef_message[offset++] = 5;

//     // Type field: MIME type string
//     memcpy(&ndef_message[offset], mime_type, type_length);
//     offset += type_length;

//     // Payload field: Serialized data
//     memcpy(&ndef_message[offset], payload, 5);
//     offset += 5;

//     size_t ndef_length = offset;

//     // Debug: Print the NDEF message being written
//     printf("Writing NDEF Message (Hex):\n");
//     print_hex(ndef_message, ndef_length);

//     // Write the NDEF message
//     write_ndef_message(pnd, ndef_message, ndef_length);

//     // Read back the NDEF message to verify

//     // ReadBinary command to read NDEF message length (first 2 bytes)
//     uint8_t read_length_cmd[] = {0x00, 0xB0, 0x00, 0x00, 0x02};
//     res = nfc_initiator_transceive_bytes(pnd, read_length_cmd, sizeof(read_length_cmd), abtRx, sizeof(abtRx), 0);
//     if (res < 0 || res < 2) {
//         nfc_perror(pnd, "Error reading NDEF length");
//         nfc_close(pnd);
//         nfc_exit(context);
//         exit(EXIT_FAILURE);
//     }
//     // Extract the NDEF message length
//     uint16_t read_ndef_length = (abtRx[0] << 8) | abtRx[1];
//     printf("NDEF Message Length: %d bytes\n", read_ndef_length);

//     // Prepare to read the NDEF message
//     uint8_t read_ndef_cmd[] = {
//         0x00, 0xB0, 0x00, 0x02,    // Offset starts after the length bytes
//         (uint8_t)(read_ndef_length & 0xFF)
//     };
//     // Read the NDEF message
//     res = nfc_initiator_transceive_bytes(pnd, read_ndef_cmd, sizeof(read_ndef_cmd), abtRx, sizeof(abtRx), 0);
//     if (res < 0 || res < read_ndef_length) {
//         nfc_perror(pnd, "Error reading NDEF message");
//         nfc_close(pnd);
//         nfc_exit(context);
//         exit(EXIT_FAILURE);
//     }

//     // Parse the NDEF message
//     parse_ndef_message(abtRx, read_ndef_length);
//     while (1){
//     	sleep(1);
    
//     }
//     // Close the NFC device
//     nfc_close(pnd);
//     nfc_exit(context);

//     return 0;
// }

// NFC 태그 감지를 위한 스레드 함수
void *nfc_tag() {
    nfc_device *pnd = NULL;
    nfc_context *context = NULL;
    nfc_target nt;

    // NFC 라이브러리 초기화
    nfc_init(&context);
    if (context == NULL) {
        fprintf(stderr, "Unable to init libnfc\n");
        exit(EXIT_FAILURE);
    }

    // NFC 장치 열기 및 재시도 루프
    while (1) {
        // Open the NFC device
        pnd = nfc_open(context, NULL);
        if (pnd == NULL) {
            fprintf(stderr, "Error opening NFC device, retrying in 1 second...\n");
            nfc_exit(context);
            sleep(1);
            continue; // NFC 장치를 열지 못한 경우 다시 시도
        }

        // NFC 장치 초기화
        if (nfc_initiator_init(pnd) < 0) {
            nfc_perror(pnd, "nfc_initiator_init");
            nfc_close(pnd);
            nfc_exit(context);
            sleep(1);
            continue; // 장치 초기화 실패 시 다시 시도
        }
        printf("NFC reader: %s opened\n", nfc_device_get_name(pnd));

        // NFC 장치가 성공적으로 열리고 초기화되었으므로 NFC 태그 감지를 시작
        const nfc_modulation nm = {
            .nmt = NMT_ISO14443A,
            .nbr = NBR_106,
        };

        // NFC 태그 감지 루프
        while (1) {
            // Poll for a target
            if (nfc_initiator_select_passive_target(pnd, nm, NULL, 0, &nt) > 0) {
                printf("NFC Tag detected.\n");
                uint8_t abtRx[MAX_FRAME_LEN];
                int res;

                // NDEF 애플리케이션 선택
                uint8_t ndef_select_app[] = {
                    0x00, 0xA4, 0x04, 0x00, 0x07,
                    0xD2, 0x76, 0x00, 0x00, 0x85,
                    0x01, 0x01, 0x00
                };
                res = nfc_initiator_transceive_bytes(pnd, ndef_select_app, sizeof(ndef_select_app), abtRx, sizeof(abtRx), 0);
                if (res < 0) {
                    nfc_perror(pnd, "Error selecting NDEF application");
                    break; // 에러 발생 시 다음 태그로 넘어감
                }

                // NDEF 파일 선택
                uint8_t ndef_select_file[] = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04};
                res = nfc_initiator_transceive_bytes(pnd, ndef_select_file, sizeof(ndef_select_file), abtRx, sizeof(abtRx), 0);
                if (res < 0) {
                    nfc_perror(pnd, "Error selecting NDEF file");
                    break;
                }

                // NDEF 메시지 작성 및 전송 (커스텀 데이터)
                NDEFRecord record;
                record.SecurityID[0] = 0x01;
                record.SecurityID[1] = 0x02;
                record.DeviceID[0] = 0x03;
                record.DeviceID[1] = 0x04;
                record.DeviceAddress = 0x05;

                unsigned char payload[5];
                memcpy(payload, record.SecurityID, 2);
                memcpy(payload + 2, record.DeviceID, 2);
                payload[4] = record.DeviceAddress;

                unsigned char ndef_message[256];
                size_t offset = 0;

                ndef_message[offset++] = 0xD2;
                const char mime_type[] = "application/vnd.myapp.deviceinfo";
                size_t type_length = strlen(mime_type);
                ndef_message[offset++] = (unsigned char)type_length;
                ndef_message[offset++] = 5;
                memcpy(&ndef_message[offset], mime_type, type_length);
                offset += type_length;
                memcpy(&ndef_message[offset], payload, 5);
                offset += 5;

                size_t ndef_length = offset;
                printf("Writing NDEF Message (Hex):\n");
                print_hex(ndef_message, ndef_length);
                write_ndef_message(pnd, ndef_message, ndef_length);

                // NDEF 메시지 확인을 위해 읽기
                uint8_t read_length_cmd[] = {0x00, 0xB0, 0x00, 0x00, 0x02};
                res = nfc_initiator_transceive_bytes(pnd, read_length_cmd, sizeof(read_length_cmd), abtRx, sizeof(abtRx), 0);
                if (res < 0 || res < 2) {
                    nfc_perror(pnd, "Error reading NDEF length");
                    break;
                }

                uint16_t read_ndef_length = (abtRx[0] << 8) | abtRx[1];
                printf("NDEF Message Length: %d bytes\n", read_ndef_length);

                uint8_t read_ndef_cmd[] = {
                    0x00, 0xB0, 0x00, 0x02,
                    (uint8_t)(read_ndef_length & 0xFF)
                };

                res = nfc_initiator_transceive_bytes(pnd, read_ndef_cmd, sizeof(read_ndef_cmd), abtRx, sizeof(abtRx), 0);
                if (res < 0 || res < read_ndef_length) {
                    nfc_perror(pnd, "Error reading NDEF message");
                    break;
                }

                parse_ndef_message(abtRx, read_ndef_length);

            } else {
                printf("No NFC Tag found, retrying...\n");
            }

            sleep(1); // NFC 태그가 감지되지 않으면 1초 대기 후 다시 감지
        }

        // NFC 장치 닫기
        nfc_close(pnd);
        nfc_exit(context);

        // NFC 장치 다시 열기 시도
        printf("NFC device closed, attempting to reopen...\n");
        sleep(1);
    }

    return NULL;
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

void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    printf("Received message: %s\n", (char *)msg->payload);
    if (thread_status == 1) {
        pthread_cancel(threads[1]);
        pthread_join(threads[1], NULL);
    }
    
    json_object *jobj = json_tokener_parse((char *)msg->payload);
    json_object *address, *command, *led, *params, *period, *led_bright;

    unsigned char r_address, r_command, r_LED = 0, r_time, r_cycle, r_count, r_period, r_brightness = 0;

    if (json_object_object_get_ex(jobj, "address", &address)) {
        r_address = (unsigned char)json_object_get_int(address);
    }
    if (r_address == 0xFF) { is_broadcast = 1; }
    else { is_broadcast = 0; }

    // command 필드 추출
    if (json_object_object_get_ex(jobj, "command", &command)) {
        const char *cmd_str = json_object_get_string(command);

        if (strcmp(cmd_str, "start") == 0) {
            if (!master_communication_active) {
                master_communication_active = true;
                printf("Communication between masters started.\n");
                send_message("CommStart", r_address, "", NULL, 0);
            }
            json_object_put(jobj);
            return;
        } else if (strcmp(cmd_str, "stop") == 0) {
            if (master_communication_active) {
                master_communication_active = false;
                printf("Communication between masters stopped.\n");
                send_message("CommStop", r_address, "", NULL, 0);
            }
            json_object_put(jobj);
            return;
        } else {
            r_command = (unsigned char)atoi(cmd_str);
        }
    }

    // 명령이 5인 경우 데이터 송신 중단
    if (r_command == 5) {
        json_object_put(jobj);
        printf("Command 5 received, returning to idle state.\n");
        return;
    }

    if (json_object_object_get_ex(jobj, "params", &params)) {
        json_object *turn_on_time, *cycle, *count;
        if (json_object_object_get_ex(params, "turn_on_time", &turn_on_time)) {
            r_time = (unsigned char)(json_object_get_double(turn_on_time)/0.25);
        }
        if (json_object_object_get_ex(params, "cycle", &cycle)) {
            r_cycle = (unsigned char)json_object_get_int(cycle) - 1;
        }
        if (json_object_object_get_ex(params, "count", &count)) {
            r_count = (unsigned char)json_object_get_int(count);
        }
    }

    // JSON에서 led_bright 값 추출
    if (json_object_object_get_ex(params, "led_bright", &led_bright)) {
        const char* bright_str = json_object_get_string(led_bright);
        if (strcmp(bright_str, "00") == 0) {
            r_brightness = 0x00;  // 약한 밝기
        } else if (strcmp(bright_str, "01") == 0) {
            r_brightness = 0x01;  // 중간 밝기
        } else if (strcmp(bright_str, "10") == 0) {
            r_brightness = 0x02;  // 강한 밝기
        }
    }

    if (json_object_object_get_ex(jobj, "period", &period)) {
        r_period = (unsigned char)json_object_get_int(period);
    }

    // frame에 필요한 값들 설정
    tx_frame.address = r_address;
    if (r_time == 0) { r_command = 3; }
    else { r_time -= 1; }
    tx_frame.command = (r_command << 4) | r_LED;
    
    if (r_command == 3 ) { 
        tx_frame.parameter = ((r_cycle + 1) * r_count * 4) + 1; 
    } else if (r_command == 1) { 
        tx_frame.parameter = (r_time << 6) | (r_cycle << 3) | r_count; 
    } else { 
        tx_frame.parameter = 0; 
    }

    // reserved[0]에 LED 밝기 값 설정
    tx_frame.reserved[0] = r_brightness;  // led_bright 값 설정

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
    
    // UID를 문자열로 변환
    char uid_str[64] = {0};
    for (size_t i = 0; i < uid_len; i++) {
        snprintf(uid_str + i * 2, sizeof(uid_str) - i * 2, "%02x", uid[i]);
    }

    // Master ID를 문자열로 설정 ("1" 또는 "2")
    const char *master_id_str = (MASTER_ID == MASTER1_ID) ? "1" : "2";

    // Master 간 통신 메시지인 경우 "node" 필드를 추가하여 Master 1 또는 Master 2를 구분
    if (strcmp(message, "MTx") == 0 || strcmp(message, "MRx") == 0) {
        const char *node = (MASTER_ID == MASTER1_ID) ? "master_1" : "master_2";  // Master ID에 따라 노드를 설정
        snprintf(payload, sizeof(payload), "{\"type\": \"%s\", \"Maddress\": %d, \"node\": \"%s\", \"masterID\": \"%s\"}", message, address, node, master_id_str);
    } 
    // NFC 태그 등록 메시지의 경우 "masterID"를 추가
    else if (strcmp(message, "Register") == 0) {
        snprintf(payload, sizeof(payload), "{\"type\": \"%s\", \"address\": %d, \"mac\": \"%s\", \"uid\": \"%s\", \"masterID\": \"%s\"}", message, address, mac, uid_str, master_id_str);
    } 
    // 일반 메시지인 경우 "MTx"를 "Tx"로 수정
    else {
        if (strcmp(message, "MTx") == 0) {
            message = "Tx";  // MTx 메시지를 Tx로 변경
        }
        snprintf(payload, sizeof(payload), "{\"type\": \"%s\", \"address\": %d, \"mac\": \"%s\", \"uid\": \"%s\"}", message, address, mac, uid_str);
    }

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
        securityID[i] = 0xFF;  // 테스트를 위해 고정된 값으로 설정
    }
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
    modem.rx.data.userPtr = (void *)(&modem);
    modem.tx.data.userPtr = (void *)(&modem);
    modem.tx.data.size = sizeof(Frame);
    modem.eth.preambleLen = 6;
    modem.eth.bw = BW125;
    modem.eth.sf = SF8;
    modem.eth.ecr = CR8;
    modem.eth.CRC = 1;
    modem.eth.freq = 915000000;
    modem.eth.resetGpioN = 4;
    modem.eth.dio0GpioN = 17;
    modem.eth.outPower = OP20;
    modem.eth.powerOutPin = PA_BOOST;
    modem.eth.AGC = 1;
    modem.eth.OCP = 240;
    modem.eth.implicitHeader = 0;
    modem.eth.syncWord = 0x12;

    LoRa_begin(&modem);

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

    LoRa_end(&modem);
}
