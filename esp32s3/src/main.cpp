
#include <Arduino.h>

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#define IIC_SDA 7
#define IIC_SCL 6
#define PMU_IRQ 2
static bool pmu_flag = false;
static void setFlag(void) {
    pmu_flag = true;
}

#include <Wire.h>
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);

#define PIR_DET 17
static inline void ir_init(void) {
    pinMode(PIR_DET, INPUT_PULLDOWN);
}
static inline bool ir_read(void) {
    return digitalRead(PIR_DET);
}

#include <esp_wifi.h>
#include "esp_camera.h"
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 39
#define XCLK_GPIO_NUM  38
#define SDA_GPIO_NUM    5
#define SCL_GPIO_NUM    4
#define D7_GPIO_NUM     9
#define D6_GPIO_NUM    10
#define D5_GPIO_NUM    11
#define D4_GPIO_NUM    13
#define D3_GPIO_NUM    21
#define D2_GPIO_NUM    48
#define D1_GPIO_NUM    47
#define D0_GPIO_NUM    14
#define VSYNC_GPIO_NUM  8
#define HREF_GPIO_NUM  18
#define PCLK_GPIO_NUM  12

#include "USB.h"
#include "USBHIDMouse.h"
#include "USBHIDKeyboard.h"
USBHIDMouse Mouse;
USBHIDKeyboard Keyboard;

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#define SERVER_HOST "192.168.0.103"
#define SERVER_PORT 8080
#define SERVER_CAMERA_POST_URI  "/hw_image"
#define SERVER_INFO_GET_URI  ("/hw_info?chip_id=" + hw_info.chip_id)
#define SERVER_INFO_POST_URI ("/hw_info?chip_id=" + hw_info.chip_id)



static String get_chip_id(void) {
    uint64_t chipid = ESP.getEfuseMac();
    char chip_id[40];
    snprintf(chip_id, 40, "%04X%08X", (uint16_t) (chipid >> 32), (uint32_t) chipid);
    return chip_id;
}
typedef struct {
    String chip_id;
    String chip_model;
} HwInfo_t;
static HwInfo_t hw_info = {
    .chip_id = get_chip_id(),
    .chip_model = ESP.getChipModel(),
};



typedef struct {
    String hw_id;
    int keyboard;
    int mouse;
    int sta_intvl;
    int cam_intvl;
    int res_val;
    bool led;
    bool reboot;
    String time;
    String res_str;
} HttpGet_t;
static HttpGet_t http_get;
static void info_get_handler(String json) {
    static StaticJsonDocument<1024> json_doc;
    Serial0.printf("get json: %s \r\n", json.c_str());
    deserializeJson(json_doc, json);
    http_get.hw_id      = json_doc["hw_id"].as<String>();
    http_get.keyboard   = json_doc["keyboard"].as<int>();
    http_get.mouse      = json_doc["mouse"].as<int>();
    http_get.sta_intvl  = json_doc["sta_intvl"].as<int>();
    http_get.cam_intvl  = json_doc["cam_intvl"].as<int>();
    http_get.res_val    = json_doc["res_val"].as<int>();
    http_get.led        = json_doc["led"].as<bool>();
    http_get.reboot     = json_doc["reboot"].as<bool>();
    http_get.time       = json_doc["time"].as<String>();
    http_get.res_str    = json_doc["res_str"].as<String>();
}
typedef struct {
    String hw_id;
    int cam_href;
    int cam_vref;
    int res_val;
    bool ir;
    String ssid;
    String psw;
    String local_ip;
    String gate_ip;
    String res_str;
} HttpPost_t;
static HttpPost_t http_post;
static String post_info_handler(void) {
    static StaticJsonDocument<1024> json_doc;
    String json;
    json_doc["hw_id"]       = http_post.hw_id       = hw_info.chip_id;
    json_doc["cam_href"]    = http_post.cam_href    = 1600;
    json_doc["cam_vref"]    = http_post.cam_vref    = 1200;
    json_doc["res_val"]     = http_post.res_val     = 0;
    json_doc["ir"]          = http_post.ir          = ir_read();
    json_doc["ssid"]        = http_post.ssid        = WiFi.SSID();
    json_doc["psw"]         = http_post.psw         = WiFi.psk();
    json_doc["local_ip"]    = http_post.local_ip    = WiFi.localIP().toString();
    json_doc["gate_ip"]     = http_post.gate_ip     = WiFi.gatewayIP().toString();
    json_doc["res_str"]     = http_post.res_str;
    serializeJson(json_doc, json);
    Serial0.printf("post json: %s \r\n", json.c_str());
    return json;
}

static esp_err_t info_get(void) { // get json
    HTTPClient http;
    if (!http.begin(String(SERVER_HOST), SERVER_PORT, String(SERVER_INFO_GET_URI))) {
        http.end();
        Serial0.println("http.begin error");
        return ESP_FAIL;
    }
    http.setUserAgent(hw_info.chip_model);
    if (http.GET() !=  HTTP_CODE_OK) {
        http.end();
        Serial0.println("http.GET error");
        return ESP_FAIL;
    }
    info_get_handler(http.getString());
    http.end();
    return ESP_OK;
}
static esp_err_t info_post(void) { // post json
    HTTPClient http;
    if (!http.begin(String(SERVER_HOST), SERVER_PORT, String(SERVER_INFO_POST_URI))) {
        http.end();
        Serial0.println("http.begin error");
        return ESP_FAIL;
    }
    http.setUserAgent(hw_info.chip_model);
    http.addHeader("Content-Type", "application/json");
    if (http.POST(post_info_handler()) !=  HTTP_CODE_OK) {
        http.end();
        Serial0.println("http.POST error");
        return ESP_FAIL;
    }
    // Serial0.println(http.getString());
    http.end();
    return ESP_OK;
}

static esp_err_t camera_init(void) {
    esp_err_t ret = ESP_OK;
    camera_config_t ov2640_config = {
        .pin_pwdn     = PWDN_GPIO_NUM,
        .pin_reset    = RESET_GPIO_NUM,
        .pin_xclk     = XCLK_GPIO_NUM,
        .pin_sscb_sda = SDA_GPIO_NUM,
        .pin_sscb_scl = SCL_GPIO_NUM,
        .pin_d7       = D7_GPIO_NUM,
        .pin_d6       = D6_GPIO_NUM,
        .pin_d5       = D5_GPIO_NUM,
        .pin_d4       = D4_GPIO_NUM,
        .pin_d3       = D3_GPIO_NUM,
        .pin_d2       = D2_GPIO_NUM,
        .pin_d1       = D1_GPIO_NUM,
        .pin_d0       = D0_GPIO_NUM,
        .pin_vsync    = VSYNC_GPIO_NUM,
        .pin_href     = HREF_GPIO_NUM,
        .pin_pclk     = PCLK_GPIO_NUM,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_UXGA,
        .jpeg_quality = 10,
        .fb_count     =  2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST
    };
    if (esp_camera_init(&ov2640_config) != ESP_OK) {
        Serial0.println("esp_camera_init error");
        ret = ESP_FAIL;
    }
    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_UXGA);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    return ret;
}
static esp_err_t camera_post(void) { // post multipart/form-data
    // int64_t fr_start = esp_timer_get_time();

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial0.println("esp_camera_fb_get error");
        return ESP_FAIL;
    }

    WiFiClient client;
    if (!client.connect(SERVER_HOST, SERVER_PORT)) {
        client.stop();
        esp_camera_fb_return(fb);
        Serial0.println("client.connect error");
        return ESP_FAIL;
    }

    client.println("POST " + String(SERVER_CAMERA_POST_URI) + " HTTP/1.1");
    client.println("User-Agent: " + hw_info.chip_model);
    client.println("Cache-Control: no-cache");
    client.println("Accept: */*");
    client.println("Accept-Encoding: gzip, deflate, br");
    client.println("Connection: keep-alive");
    client.println("Host: " + String(SERVER_HOST) + ':' + String(SERVER_PORT));
    client.println("Content-Type: multipart/form-data; boundary=--------------------------" + hw_info.chip_id);
    client.println("Content-Length: " + String(fb->len + 1024));
    client.println();

    client.print(" ----------------------------" + hw_info.chip_id + "\r\n");
    client.print("Content-Disposition: form-data; name=\"hw_id\"\r\n\r\n");
    client.write(hw_info.chip_id.c_str());
    client.print(" \r\n----------------------------" + hw_info.chip_id + "\r\n");
    client.print("Content-Disposition: form-data; name=\"hw_image\"; filename=\"" + hw_info.chip_id + ".jpeg\"\r\n");
    client.print("Content-Type: image/jpeg\r\n\r\n");
    client.write(fb->buf, fb->len);
    client.print(" \r\n----------------------------" + hw_info.chip_id + "--\r\n");

#if 0
    String response;
    uint16_t timeout = 0;
    while (client.connected() || client.available()) { 
        if (client.available()) {
            response = client.readStringUntil('\n');
            Serial0.println(response);
            if (response == "\r") {
                if (client.available()) {
                    response = client.readStringUntil('\n');
                    Serial0.println(response);
                }
                break;
            }
        } else {
            vTaskDelay(10);
            if (++timeout >= 500) {
                Serial0.println("get response timeout");
                break;
            }
        }
    }
#endif

    client.stop();
    esp_camera_fb_return(fb);
    // int64_t fr_end = esp_timer_get_time();
    // Serial0.println("time: %ums", (uint32_t) ((fr_end - fr_start) / 1000));
    return ESP_OK;
}



static void usb_init(void) {
    Mouse.begin();
    Keyboard.begin();
    USB.begin();
}

static void serial_init(void) {
    Serial0.begin(115200);
    while (!Serial0);
    delay(3000);
    Serial0.println();
}
static void serial_deinit(void) {
    Serial0.end();
}

static void oled_init(void) {
    u8g2.begin();
    u8g2.enableUTF8Print();
}
static String oled_convert(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;
    for (int i = 0; i <= maxIndex && found <= index; ++i) {
        if (data.charAt(i) == separator || i == maxIndex){
            ++found;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
static void oled_show(String str) {
    String tzn, year, month, day, week, time;
    String time_no_secd;
    week    = oled_convert(str, ' ', 0);
    month   = oled_convert(str, ' ', 1);
    day     = oled_convert(str, ' ', 2);
    time    = oled_convert(str, ' ', 3);
    tzn     = oled_convert(str, ' ', 4);
    year    = oled_convert(str, ' ', 5);
    time_no_secd = time.substring(0, 5);
    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_fub35_tn);
        u8g2.setCursor(0, 48);
        u8g2.print(time_no_secd);
    } while (u8g2.nextPage());
}

static void controller_thread(void * parameter) {
    oled_init();
    ir_init();
    usb_init();    
    for (;;) {
        if (ESP_OK == info_get()) {
            oled_show(http_get.time);
            http_post.res_str = "ESP_OK";
        } else {
            http_post.res_str = "ESP_FAIL";
        }
        info_post();

        if (http_get.mouse || http_get.keyboard) {
            Keyboard.write(KEY_ESC);
        }

        if (http_get.led) {
            http_get.led = false;
        }
        if (http_get.reboot) {
            http_get.reboot = false;
            ESP.restart();
        }

        vTaskDelay(http_get.sta_intvl);
    }
}
static void camera_thread(void * parameter) {
    camera_init();
    for (;;) {
        while (ESP_OK == camera_post()) {
            vTaskDelay(http_get.cam_intvl);
        }
        vTaskDelay(10000);
    }
}

#include "OneButton.h"
OneButton btn(0);
static void btn_click_handler(void)  {
    Serial0.println("reboot");
    ESP.restart();
}
static void btn_press_handler(void)  {
    Serial0.println("smartconfig");
    ESP.restart();
}
static void btn_thread(void * parameter) {
    btn.reset();
    btn.attachClick(btn_click_handler);
    btn.attachDuringLongPress(btn_press_handler);
    for (;;) {
        btn.tick();
        vTaskDelay(10);
    }
}

static void pmu_init(void) {
    XPowersPMU PMU;

    /*********************************
     *  step 1 : Initialize power chip,
     *  turn on modem and gps antenna power channel
    ***********************************/
    PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  
    // Set the minimum common working voltage of the PMU VBUS input,
    // below this value will turn off the PMU
    PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);

    // Set the maximum current of the PMU VBUS input,
    // higher than this value will turn off the PMU
    PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);

    // Set VSY off voltage as 2600mV , Adjustment range 2600mV ~ 3300mV
    PMU.setSysPowerDownVoltage(2600);

    //Set the working voltage of the modem, please do not modify the parameters
    // PMU.setDC1Voltage(3300);         //ESP32s3 Core VDD
    // PMU.enableDC1();                 //It is enabled by default, please do not operate

    // Board 5 Pin socket 3.3V power output control
    PMU.setDC3Voltage(3100);         //Extern 3100~ 3400V
    PMU.enableDC3();

    // Camera working voltage, please do not change
    PMU.setALDO1Voltage(1800);      // CAM DVDD
    PMU.enableALDO1();

    // Camera working voltage, please do not change
    PMU.setALDO2Voltage(2800);      // CAM DVDD
    PMU.enableALDO2();

    // Camera working voltage, please do not change
    PMU.setALDO4Voltage(3000);      // CAM AVDD
    PMU.enableALDO4();

    // Pyroelectric sensor working voltage, please do not change
    PMU.setALDO3Voltage(3300);        // PIR VDD
    PMU.enableALDO3();

    // Microphone working voltage, please do not change
    PMU.setBLDO1Voltage(3300);       // MIC VDD
    PMU.enableBLDO1();

    // TS Pin detection must be disable, otherwise it cannot be charged
    PMU.disableTSPinMeasure();



    /*********************************
     * step 2 : Enable internal ADC detection
    ***********************************/
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();


    /*********************************
     * step 3 : Set PMU interrupt
    ***********************************/
    pinMode(PMU_IRQ, INPUT);
    attachInterrupt(PMU_IRQ, setFlag, FALLING);

    // Disable all interrupts
    PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    // Clear all interrupt flags
    PMU.clearIrqStatus();
    // Enable the required interrupt function
    PMU.enableIRQ(
        XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
        XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |   //POWER KEY
        XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
    );

    /*********************************
     * step 4 : Set PMU Charger params
    ***********************************/
    // Set the precharge charging current
    PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    // Set constant current charge current limit
    PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
    // Set stop charging termination current
    PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

    // Set charge cut-off voltage
    PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

    /*********************************
     * step 4 : Set PMU pwrkey params
    ***********************************/
    // Set the time of pressing the button to turn off
    PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = PMU.getPowerKeyPressOffTime();
    Serial0.print("PowerKeyPressOffTime:");
    switch (opt) {
    case XPOWERS_POWEROFF_4S: Serial0.println("4 Second");
        break;
    case XPOWERS_POWEROFF_6S: Serial0.println("6 Second");
        break;
    case XPOWERS_POWEROFF_8S: Serial0.println("8 Second");
        break;
    case XPOWERS_POWEROFF_10S: Serial0.println("10 Second");
        break;
    default:
        break;
    }
    // Set the button power-on press time
    PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
    opt = PMU.getPowerKeyPressOnTime();
    Serial0.print("PowerKeyPressOnTime:");
    switch (opt) {
    case XPOWERS_POWERON_128MS: Serial0.println("128 Ms");
        break;
    case XPOWERS_POWERON_512MS: Serial0.println("512 Ms");
        break;
    case XPOWERS_POWERON_1S: Serial0.println("1 Second");
        break;
    case XPOWERS_POWERON_2S: Serial0.println("2 Second");
        break;
    default:
        break;
    }


    /*
    The default setting is CHGLED is automatically controlled by the PMU.
    - XPOWERS_CHG_LED_OFF,
    - XPOWERS_CHG_LED_BLINK_1HZ,
    - XPOWERS_CHG_LED_BLINK_4HZ,
    - XPOWERS_CHG_LED_ON,
    - XPOWERS_CHG_LED_CTRL_CHG,
    * */
    PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
}
static void wait_config(void) {
    pinMode(0, INPUT_PULLUP);
    if (0 == digitalRead(0)) {
        esp_wifi_restore();
        while(digitalRead(0)) {}
        ESP.restart();
    }
}
static void smart_config(void) {
    WiFi.mode(WIFI_STA);
    WiFi.beginSmartConfig();
    for (;;) {
        Serial0.print(".");
        delay(500);
        if (WiFi.smartConfigDone()) {
            esp_wifi_set_storage(WIFI_STORAGE_RAM);
            WiFi.setAutoConnect(true);
            Serial0.println("");
            ESP.restart();
            break;      
        }
    } 
}
static bool auto_config(void) {
    WiFi.begin();
    for (uint8_t i = 0; i < 10; ++i) {
        if (WL_CONNECTED == WiFi.status()) {
            Serial0.println("");
            Serial0.printf("SSID:%s", WiFi.SSID().c_str());
            Serial0.printf(", PSW:%s\r\n", WiFi.psk().c_str());
            Serial0.print("LocalIP:");
            Serial0.print(WiFi.localIP());
            Serial0.print(", GateIP:");
            Serial0.println(WiFi.gatewayIP());
            Serial0.println("");
            return true;
        } else {
            Serial0.print(".");
            delay(1000);
        }   
    }
    return false;
}
void setup() {
    serial_init();
    pmu_init();

    if (!auto_config()) {
        smart_config();
    }  
    wait_config();

    xTaskCreate(btn_thread,        "btn_thread",        1024, NULL, 20, NULL);
    xTaskCreate(controller_thread, "controller_thread", 4096, NULL, 10, NULL);
    xTaskCreate(camera_thread,     "camera_thread",     4096, NULL,  5, NULL);



}
void loop(void) {
    return ;
}
