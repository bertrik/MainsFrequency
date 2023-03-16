#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <include/WiFiState.h>
#include <PubSubClient.h>

#include "editline.h"
#include "cmdproc.h"

#include "statemachine.h"

#define PIN_50HZ_INPUT      A0
#define SAMPLE_FREQUENCY    5000

#define BUF_SIZE            2048
#define STATS_SIZE          SAMPLE_FREQUENCY / 10

#define MQTT_HOST   "mosquitto.space.revspace.nl"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "revspace/sensors/ac/frequency"

#define printf Serial.printf

// editor
static char line[120];
static char esp_id[32];

// sample buffer
static uint16_t buffer[BUF_SIZE];
static uint16_t stats[STATS_SIZE];
static volatile uint32_t bufr = 0;
static volatile uint32_t bufw = 0;
static volatile bool overflow = false;

// wifi / mqtt
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);

static void IRAM_ATTR timer_isr(void)
{
    uint16_t value = analogRead(PIN_50HZ_INPUT);

    uint32_t next = (bufw + 1) % BUF_SIZE;
    if (!overflow) {
        if (next != bufr) {
            buffer[bufw] = value;
            bufw = next;
        } else {
            overflow = true;
        }
    }
}

static void sample_init(int frequency)
{
    // set up timer interrupt
    timer1_disable();
    timer1_isr_init();
    timer1_attachInterrupt(timer_isr);
    timer1_write(5000000 / frequency);
}

static void sample_start(void)
{
    digitalWrite(LED_BUILTIN, 0);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
}

static void sample_stop(void)
{
    timer1_disable();
    digitalWrite(LED_BUILTIN, 1);
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

static int compare_uint16(const void *v1, const void *v2)
{
    uint16_t u1 = *((uint16_t *) v1);
    uint16_t u2 = *((uint16_t *) v2);
    return u1 - u2;
}

static void stats_calculate(uint16_t * q1, uint16_t * q2, uint16_t * q3)
{
    sample_reset();
    int index = 0;
    while (index < STATS_SIZE) {
        uint16_t value;
        if (sample_get(&value)) {
            stats[index++] = value;
        }
    }
    qsort(stats, STATS_SIZE, sizeof(uint16_t), compare_uint16);
    *q1 = stats[1 * STATS_SIZE / 4];
    *q2 = stats[2 * STATS_SIZE / 4];
    *q3 = stats[3 * STATS_SIZE / 4];
}

static void show_help(const cmd_t * cmds)
{
    for (const cmd_t * cmd = cmds; cmd->cmd != NULL; cmd++) {
        printf("%10s: %s\n", cmd->name, cmd->help);
    }
}

// sends a value on the specified topic (retained), attempts to connect if not connected
static bool mqtt_send(const char *topic, const char *value, bool retained)
{
    bool result = false;
    if (!mqttClient.connected()) {
        printf("Connecting MQTT...");
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
        result = mqttClient.connect(esp_id, topic, 0, retained, "offline");
        printf("%s\n", result ? "OK" : "FAIL");
    }
    if (mqttClient.connected()) {
        printf("Publishing %s to %s...", value, topic);
        result = mqttClient.publish(topic, value, retained);
        printf("%s\n", result ? "OK" : "FAIL");
    }
    return result;
}

static bool measure_frequency(double *frequency)
{
    // WiFi down
    WiFiState state;
    WiFi.shutdown(state);

    // take stats
    uint16_t q1, med, q3;
    sample_start();
    stats_calculate(&q1, &med, &q3);

    // determine zero crossings
    sample_reset();
    StateMachine sm(q1 - med, q3 - med);
    int t = 0;
    uint32_t start = millis();
    double first = 0.0;
    int count = 0;
    bool done = false;
    while (!done && ((millis() - start) < 1500)) {
        uint16_t value;
        if (sample_get(&value)) {
            double v = value - med;
            double time = (double) t++ / SAMPLE_FREQUENCY;
            if (sm.process(time, v)) {
                switch (count) {
                case 0:
                    first = sm.get_result();
                    break;
                case 50:
                    *frequency = count / (sm.get_result() - first);
                    done = true;
                    break;
                default:
                    break;
                }
                count++;
            }
        }
    }

    // WiFi resume
    sample_stop();
    WiFi.resumeFromShutdown(state);

    return done;
}

static int do_freq(int argc, char *argv[])
{
    double frequency;
    if (measure_frequency(&frequency)) {
        printf("Frequency = %.3f\n", frequency);
    }

    return 0;
}

static int do_stats(int argc, char *argv[])
{
    uint16_t q1, q2, q3;
    stats_calculate(&q1, &q2, &q3);
    printf("q1=%u,q2=%u,q3=%u\n", q1, q2, q3);
    return 0;
}

static int do_help(int argc, char *argv[]);

const cmd_t commands[] = {
    { "help", do_help, "Show help" },
    { "stats", do_stats, "Stats" },
    { "f", do_freq, "Measure frequency" },
    { NULL, NULL, NULL }
};

static int do_help(int argc, char *argv[])
{
    show_help(commands);
    return CMD_OK;
}

void setup(void)
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);

    Serial.begin(115200);
    Serial.println("\nESP8266 SAMPLER");
    EditInit(line, sizeof(line));

    // get ESP id as mqtt client id
    sprintf(esp_id, "%08X", ESP.getChipId());
    printf("ESP ID: %s\n", esp_id);

    WiFi.begin("revspace-pub-2.4ghz", "");

    sample_init(SAMPLE_FREQUENCY);
}

void loop(void)
{
    static int last_period = 0;
    static double frequency = 0.0;
    static bool have_data = false;

    // handle console commands
    bool haveLine = false;
    if (Serial.available()) {
        char c;
        haveLine = EditLine(Serial.read(), &c);
        Serial.print(c);
    }
    if (haveLine) {
        int result = cmd_process(commands, line);
        switch (result) {
        case CMD_OK:
            printf("OK\n");
            break;
        case CMD_NO_CMD:
            break;
        case CMD_UNKNOWN:
            printf("Unknown command, available commands:\n");
            show_help(commands);
            break;
        default:
            printf("%d\n", result);
            break;
        }
        printf(">");
    }
    // measure every X seconds
    int period = millis() / 5000;
    if (period != last_period) {
        last_period = period;
        if (measure_frequency(&frequency)) {
            printf("Frequency: %.3f\n", frequency);
            have_data = true;
        }
    }
    // publish if we have data and are connected
    if (have_data) {
        if (WiFi.status() == WL_CONNECTED) {
            char payload[16];
            sprintf(payload, "%.3f Hz", frequency);
            mqtt_send(MQTT_TOPIC, payload, true);
            have_data = false;
        }
    }
}
