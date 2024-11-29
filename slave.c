// Last update : 24.08.19 AM 11:55
// Last update : 24.08.23 PM 16:35


#include "LoRa.h"
#include <wiringPi.h> // RGB 제어 라이브러리
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <curl/curl.h>
#include <json-c/json.h>
#include <nfc/nfc.h>

#define NUM_LEDS 4  // LED 개수

unsigned char MySecurityID[2];  // NFC Tag로 읽어온 Security ID
unsigned char MyDeviceAddress;  // NFC Tag로 생성된 Device Address
unsigned char Group_Address[8];  // 그룹 주소 저장

// LED 스레드와 상태 변수
pthread_t led_threads[NUM_LEDS];
int thread_status[NUM_LEDS] = {0};

// 실제 LED Pin 번호 설정
enum LED {  
    LED0 = 29,
    LED1 = 28,
    LED2 = 27,
    LED3 = 24
    
};

// payload frame struct 정의
typedef struct { 
    unsigned char securityID[2];
    unsigned char address;
    unsigned char seq;
    unsigned char command;
    unsigned char parameter;
    unsigned char reserved[2];
} Frame;

// LED 리스트
int list_LED[4] = { LED0, LED1, LED2, LED3 };

struct timeval tv;  // 현재 시간을 저장하기 위한 구조체

// LED 제어를 위한 함수 (깜빡임)
void* led_control(void* arg) { 
    int* args = (int*)arg;
    int led = args[0];
    int time = args[1];
    int cycle = args[2];
    int count = args[3];
    int is_zero = (count == 0);
    thread_status[led] = 1;

    while (count != 0 || is_zero) {
        digitalWrite(list_LED[led], 1);  // LED 켜기
        usleep((time + 1) * 250000);
        digitalWrite(list_LED[led], 0);  // LED 끄기
        usleep((time + 1) * 250000 * (cycle+1));
        count--;
    }
    thread_status[led] = 0;
    free(arg);
    return NULL;
}

// LED 제어를 위한 함수 (켜짐 유지)
void* led_control2(void* arg) { 
    int* args = (int*)arg;
    int led = args[0];
    int sec = args[1];
    thread_status[led] = 1;

    digitalWrite(list_LED[led], 1);  // LED 켜기
    sleep(sec);  // 설정된 시간 동안 유지
    digitalWrite(list_LED[led], 0);  // LED 끄기
    thread_status[led] = 0;
    free(arg);
    return NULL;
}

// LED 깜빡임 패턴 실행
void blinking_LED(unsigned char led_bits, unsigned char parameter) { 
    for (int i = 0; i < 4; i++) {
        if (led_bits & (1 << i)) {
            int *args = malloc(4 * sizeof(int));
            args[0] = i;
            args[1] = parameter >> 6;       // turn on time, 2bit
            args[2] = (parameter >> 3) & 0x07;  // cycle, 3bit
            args[3] = parameter & 0x07;     // count, 3bit
            pthread_create(&led_threads[i], NULL, led_control, args);
            pthread_detach(led_threads[i]);
        }
    }
}   

// 모든 LED 끄기 및 깜빡임 종료
void blinking_off_LED() { 
    for(int i = 0; i < NUM_LEDS; i++) {
        if(thread_status[i] == 1) {
            pthread_cancel(led_threads[i]);   // 해당 LED의 스레드 종료
            pthread_join(led_threads[i], NULL);
            thread_status[i] = 0;
        } 
        digitalWrite(list_LED[i], 0);  // LED 끄기
    }
}

// LED 켜짐 유지 패턴 실행
void turn_on_LED(unsigned char led_bits, unsigned char parameter) { 
    for (int i = 0; i < 4; i++) {
        if (led_bits & (1 << i)) {  
            int* args = malloc(2 * sizeof(int));
            args[0] = i;  // LED 번호
            args[1] = parameter;  // 유지할 시간 (초)
            pthread_create(&led_threads[i], NULL, led_control2, args);
            pthread_detach(led_threads[i]);
        }
    }
}

// Security ID 확인 함수
int check_ID(unsigned char receiveID[2]) {
    return ((receiveID[0] == MySecurityID[0] && receiveID[1] == MySecurityID[1]) || 
            (receiveID[0] == 0xFF && receiveID[1] == 0xFF));
}

// Address 확인 함수
int check_address(unsigned char receive_address) {
    if (receive_address == 0xFF) { return 1; }  // 브로드캐스트 주소 확인
    if (receive_address < 128 && receive_address != MyDeviceAddress) { return 0; }
    return 1;
}

// 명령어 처리 함수
void check_command(unsigned char command, unsigned char parameter) {
    switch(command >> 4) {
        case 1:  // Blinking
            blinking_off_LED(); 
            blinking_LED(command & 0x0F, parameter);
            break;            
        case 2:  // Blinking off
            blinking_off_LED();
            break;        
        case 3:  // turn on
            blinking_off_LED();
            turn_on_LED(command & 0x0F, parameter);
            break;        
        case 4:  // Scenario Blinking
            blinking_off_LED(); 
            printf("Scenario Blinking\n"); 
            break;
        case 5:  // Reserved
            printf("Reserved\n"); 
            break;
    }
}

// NFC로부터 MAC 주소를 생성하는 함수
void generate_mac_from_nfc(unsigned char* mac_address) {
    nfc_device *pnd;
    nfc_target nt;
    nfc_context *context;

    nfc_init(&context);
    if (context == NULL) {
        printf("Unable to init libnfc (malloc)\n");
        exit(EXIT_FAILURE);
    }

    pnd = nfc_open(context, NULL);
    if (pnd == NULL) {
        printf("ERROR: %s\n", "Unable to open NFC device.");
        exit(EXIT_FAILURE);
    }

    if (nfc_initiator_init(pnd) < 0) {
        nfc_perror(pnd, "nfc_initiator_init");
        exit(EXIT_FAILURE);
    }

    const nfc_modulation nmMifare = {
        .nmt = NMT_ISO14443A,
        .nbr = NBR_106,
    };

    if (nfc_initiator_select_passive_target(pnd, nmMifare, NULL, 0, &nt) > 0) {
        for (size_t i = 0; i < nt.nti.nai.szUidLen && i < 6; i++) {
            mac_address[i] = nt.nti.nai.abtUid[i];
        }
    } else {
        printf("No NFC tag detected.\n");
    }

    nfc_close(pnd);
    nfc_exit(context);
}

// JSON 데이터를 웹 서버에 전송하는 함수
void send_json_to_server(const char* url, const char* mac_address) {
    CURL *curl;
    CURLcode res;

    struct json_object *jobj = json_object_new_object();
    struct json_object *jtype = json_object_new_string("Register");
    struct json_object *jmac = json_object_new_string(mac_address);

    json_object_object_add(jobj, "type", jtype);
    json_object_object_add(jobj, "mac", jmac);

    const char *json_string = json_object_to_json_string(jobj);

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_string);

        res = curl_easy_perform(curl);

        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        } else {
            printf("Successfully sent JSON to server.\n");
        }

        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();
    json_object_put(jobj);
}

// 송신 콜백 함수
void tx_f(txData *tx){ 
    LoRa_ctl *modem = (LoRa_ctl *)(tx->userPtr);
    LoRa_receive(modem);  // 수신 모드로 전환
}

// 수신 콜백 함수
void * rx_f(void *p){
    rxData *rx = (rxData *)p;
    LoRa_ctl *modem = (LoRa_ctl *)(rx->userPtr);
    Frame * frame = (Frame *)(rx->buf);

    // Security ID, Address 확인
    if ((!check_ID(frame->securityID)) || (!check_address(frame->address))) { 
	    free(p);  
	    return NULL; 
    }

    // 수신 데이터 출력
    printf("Security ID: ");
    for (int i = 0; i < 2; i++) { printf("%02x ", frame->securityID[i]); }
    printf("\nAddress: %02x\nSeq: %d\nCommand: %02x\nParameter: %02x\n",
           frame->address, frame->seq, frame->command, frame->parameter);

    // 명령 실행
    check_command(frame->command, frame->parameter);
    
    if (frame->address == 0xFF) { free(p); return NULL; }  // ACK가 필요 없는 경우
    // ACK 전송
    Frame ack_frame = {
        .address = 127,  // Master Address
        .seq = frame->seq,  // ack
        .command = 0x60,  // ack response
        .parameter = frame->parameter
    };
    ack_frame.securityID[0] = MySecurityID[0];
    ack_frame.securityID[1] = MySecurityID[1];
    
    memcpy(modem->tx.data.buf, &ack_frame, sizeof(Frame)); 
    LoRa_send(modem);  // ACK 전송
    free(p);
    return NULL;
}

// main 함수
int main(){
    
    // NFC 태그로부터 MAC 주소 생성
    unsigned char mac_address[6] = {0};
    char mac_str[18];

    generate_mac_from_nfc(mac_address);

    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_address[0], mac_address[1], mac_address[2],
             mac_address[3], mac_address[4], mac_address[5]);

    send_json_to_server("http://your-server-url.com/api/register", mac_str);

    // Security ID 및 Device Address 설정
    MySecurityID[0] = 0xFF;
    MySecurityID[1] = 0xFF;
    MyDeviceAddress = 0x01;
    
    // wiringPi 설정 및 LED 초기화
    if (wiringPiSetup() == -1) { return 1; }
    pinMode(LED0, OUTPUT);  // TODO: 실제 LED 핀 번호로 변경
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    digitalWrite(LED0, 0);  // LED 끄기
    digitalWrite(LED1, 0);
    digitalWrite(LED2, 0);
    digitalWrite(LED3, 0);

    // LoRa 모뎀 설정
    LoRa_ctl modem;
    modem.spiCS = 0;  // 라즈베리 파이의 SPI CE 핀 번호 설정
    modem.tx.callback = tx_f;  // 송신 콜백 함수 설정
    modem.tx.data.buf = txbuf;  // 송신 데이터 버퍼 설정
    modem.rx.callback = rx_f;  // 수신 콜백 함수 설정
    modem.rx.data.userPtr = (void *)(&modem);  // 수신 콜백에서 모뎀을 제어하기 위해 포인터 설정
    modem.tx.data.userPtr = (void *)(&modem);  // 송신 콜백에서 모뎀을 제어하기 위해 포인터 설정
    modem.tx.data.size = sizeof(Frame);  // 명시적 헤더 모드에서 전송 시 페이로드 길이 설정
    modem.eth.preambleLen = 6;  // 프리앰블 길이 설정 (6심볼)
    modem.eth.bw = BW125;  // 대역폭 설정 (125kHz)
    modem.eth.sf = SF10;  // 스프레딩 팩터 설정 (10)
    modem.eth.ecr = CR45;  // 오류 수정율 설정 (4/5)
    modem.eth.CRC = 1;  // CRC 체크 활성화
    modem.eth.freq = 915000000;  // 주파수 설정 (915MHz)
    modem.eth.resetGpioN = 4;  // 라즈베리 파이의 GPIO4에 연결된 LoRa 리셋 핀 설정
    modem.eth.dio0GpioN = 17;  // 라즈베리 파이의 GPIO17에 연결된 LoRa DIO0 핀 설정 (수신 및 송신 완료 인터럽트 제어)
    modem.eth.outPower = OP20;  // 출력 전력 설정 (20dBm)
    modem.eth.powerOutPin = PA_BOOST;  // 전력 증폭기 출력 핀 설정
    modem.eth.AGC = 1;  // 자동 이득 제어 활성화
    modem.eth.OCP = 240;  // 과전류 보호 설정 (45~240mA, 0은 보호 비활성화)
    modem.eth.implicitHeader = 0;  // 명시적 헤더 모드 비활성화 (암묵적 헤더 모드 비활성화)
    modem.eth.syncWord = 0x12;  // 동기화 워드 설정

    LoRa_begin(&modem);  // LoRa 모듈 초기화
    LoRa_receive(&modem);  // 수신 모드로 전환

    // main 함수 종료 방지
    pause();
    LoRa_end(&modem);  // LoRa 모듈 종료
}
