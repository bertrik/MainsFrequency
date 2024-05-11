#include <WiFiManager.h>
#include <PubSubClient.h>
#include <MiniShell.h>

#define printf Serial.printf

// whether we publish over WiFi or not
//#define WIFI_ENABLED

#define BASE_FREQUENCY  50.0

#define PIN_50HZ_INPUT      0
#define PIN_LED             12
#define SAMPLE_FREQUENCY    5000

#define MQTT_HOST   "stofradar.nl"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "bertrik/mains"

static hw_timer_t *timer = nullptr;

static char line[120];
static char esp_id[] = "esp32-50hz";

static volatile uint32_t int_count = 0;

// sample buffer
#define BUF_SIZE    2048
static uint16_t buffer[BUF_SIZE];
static volatile uint32_t bufr = 0;
static volatile uint32_t bufw = 0;
static volatile bool overflow = false;
static volatile uint16_t latest_reading = 0;

static MiniShell shell(&Serial);
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);

static void show_help(const cmd_t * cmds)
{
    for (const cmd_t * cmd = cmds; cmd->cmd != NULL; cmd++) {
        printf("%10s: %s\r\n", cmd->name, cmd->help);
    }
}

static void IRAM_ATTR adc_int()
{
    latest_reading = analogRead(PIN_50HZ_INPUT);
    uint32_t next = (bufw + 1) % BUF_SIZE;
    if (next != bufr) {
        latest_reading = buffer[bufw] = latest_reading;
        bufw = next;
    }
    int_count++;
}

static void adc_init()
{
    analogReadResolution(12);
    analogSetAttenuation(ADC_0db);
    analogSetPinAttenuation(PIN_50HZ_INPUT, ADC_0db);
    adcAttachPin(PIN_50HZ_INPUT);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &adc_int, true);
    timerAlarmWrite(timer, 1000000 / SAMPLE_FREQUENCY, true);   // 5kHz timer
    timerAlarmEnable(timer);
}

static int do_adc(int argc, char *argv[])
{
    printf("interrupts: %d\r\n", int_count);
    printf("latest reading: %d\r\n", latest_reading);
    return 0;
}

static int compare_uint16(const void *v1, const void *v2)
{
    uint16_t u1 = *((uint16_t *) v1);
    uint16_t u2 = *((uint16_t *) v2);
    return u1 - u2;
}

static void sample_reset(void)
{
    bufr = 0;
    bufw = 0;
    overflow = false;
}

static bool sample_get(uint16_t * pval)
{
    if (bufr == bufw) {
        return false;
    }
    int next = (bufr + 1) % BUF_SIZE;
    *pval = buffer[bufr];
    bufr = next;
    return true;
}

static int do_help(int argc, char *argv[]);

const cmd_t commands[] = {
    { "help", do_help, "Show help" },
    { "a", do_adc, "ADC functions" },
    { NULL, NULL, NULL }
};

static int do_help(int argc, char *argv[])
{
    show_help(commands);
    return 0;
}

static bool mqtt_send(const char *topic, const char *value, bool retained)
{
    bool result = false;
    if (!mqttClient.connected()) {
        Serial.print("Connecting MQTT...");
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
        result = mqttClient.connect(esp_id, topic, 0, retained, "offline");
        Serial.println(result ? "OK" : "FAIL");
    }
    if (mqttClient.connected()) {
        Serial.print("Publishing ");
        Serial.print(value);
        Serial.print(" to ");
        Serial.print(topic);
        Serial.print("...");
        result = mqttClient.publish(topic, value, retained);
        Serial.println(result ? "OK" : "FAIL");
    }
    return result;
}

void setup(void)
{
    pinMode(PIN_LED, OUTPUT);
    adc_init();

    Serial.begin(115200);
    Serial.println("\nESP32PHASE");
#ifdef WIFI_ENABLED
    WiFiManager wm;
    wm.autoConnect("AutoConnectAP");
#endif
}

void loop(void)
{
    // run the frequency algorithm continuously
    static double sum_i = 0.0;
    static double sum_q = 0.0;
    static int index = 0;
    uint16_t value;
    static double prev_angle = 0.0;
    if (sample_get(&value)) {
        sum_i += value * cos(2.0 * M_PI * BASE_FREQUENCY * index / SAMPLE_FREQUENCY);
        sum_q += value * sin(2.0 * M_PI * BASE_FREQUENCY * index / SAMPLE_FREQUENCY);
        index++;
        if (index >= SAMPLE_FREQUENCY) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));

            double angle = 180.0 * atan2(sum_i, sum_q) / M_PI;
            double ampl = sqrt(sum_i * sum_i + sum_q * sum_q) / SAMPLE_FREQUENCY;
            sum_q = 0.0;
            sum_i = 0.0;
            index = 0;
            double d = angle - prev_angle;
            if (d < -180.0) {
                d += 360.0;
            } else if (d > 180.0) {
                d -= 360.0;
            }
            double t = 1.0 - (d / 360.0) / BASE_FREQUENCY;
            double freq = BASE_FREQUENCY / t;
            prev_angle = angle;

            printf("Angle:%8.3f, dAngle:%7.3f, Freq:%7.3f, Ampl:%5.1f\n", angle, d, freq, ampl);

#ifdef WIFI_ENABLED
            char value[64];
            sprintf(value, "{\"frequency\":%.3f,\"phase\":%.3f,\"amplitude\":%.1f}", freq, angle,
                    ampl);
            mqtt_send(MQTT_TOPIC, value, false);
            // calculate median and send over mqtt
            double median;
            if (median_add(freq, &median)) {
                char value[64];
                sprintf(value, "%.3f Hz", median);
                mqtt_send(MQTT_TOPIC, value, true);
            }
#endif
        }
    }
    // command line processing
    shell.process(">", commands);
}

