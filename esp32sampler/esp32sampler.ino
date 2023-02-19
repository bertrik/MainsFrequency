#include "print.h"
#include "cmdproc.h"
#include "editline.h"

#include "statemachine.h"

#define PIN_50HZ_INPUT      36
#define SAMPLE_FREQUENCY    5000

static hw_timer_t *timer = nullptr;

static char line[120];

static uint32_t value = 0;
static volatile uint32_t int_count = 0;

// sample buffer
#define BUF_SIZE    2048
static uint16_t buffer[BUF_SIZE];
static volatile uint32_t bufr = 0;
static volatile uint32_t bufw = 0;
static volatile bool overflow = false;
static volatile uint16_t latest_reading = 0;

static void show_help(const cmd_t * cmds)
{
    for (const cmd_t * cmd = cmds; cmd->cmd != NULL; cmd++) {
        print("%10s: %s\r\n", cmd->name, cmd->help);
    }
}

static int tt = 0;

void IRAM_ATTR adc_int()
{
    int_count++;
    uint32_t next = (bufw + 1) % BUF_SIZE;
    if (!overflow) {
        if (next != bufr) {
            latest_reading = buffer[bufw] = analogRead(PIN_50HZ_INPUT);
            digitalWrite(LED_BUILTIN, !(micros() & 0x40000));
            tt++;
            bufw = next;
        } else {
            overflow = true;
        }
    }
}

static void adc_init()
{
    analogSetClockDiv(2);  // 12.5 k sample/sec
    analogReadResolution(12);
    analogSetWidth(12);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &adc_int, true);
    timerAlarmWrite(timer, 200, true);  // 5kHz timer
    timerAlarmEnable(timer);
}

static int do_adc(int argc, char *argv[])
{
    print("interrupts: %d\r\n", int_count);
    print("value: %d\r\n", value);
    print("latest reading: %d\r\n", latest_reading);
    return 0;
}

static int compare_uint16(const void *v1, const void *v2)
{
    uint16_t u1 = *((uint16_t *) v1);
    uint16_t u2 = *((uint16_t *) v2);
    return u1 - u2;
}

static int do_stats(int argc, char *argv[])
{
    bufr = 0;
    bufw = 0;
    overflow = false;
    uint32_t start = millis();
    while (!overflow) {
        // wait
    }
    print("done: %d, took: %d\r\n", overflow, millis() - start);

    qsort(buffer, BUF_SIZE, sizeof(uint16_t), compare_uint16);
    uint16_t q1 = buffer[BUF_SIZE / 4];
    uint16_t med = buffer[BUF_SIZE / 2];
    uint16_t q3 = buffer[3 * BUF_SIZE / 4];

    print("q1=%u,med=%u,q3=%u\r\n", q1, med, q3);
    return 0;
}

static void sample_reset(void)
{
    bufr = 0;
    bufw = 0;
    overflow = false;
}

static bool sample_get(double * pval)
{
    if (bufr == bufw) {
        return false;
    }
    int next = (bufr + 1) % BUF_SIZE;
    *pval = buffer[bufr];
    bufr = next;
    return true;
}

static int do_freq(int argc, char *argv[])
{
    // take stats
    sample_reset();
    while (!overflow) yield();
    qsort(buffer, BUF_SIZE, sizeof(uint16_t), compare_uint16);
    uint16_t q1 = buffer[2 * BUF_SIZE / 8];
    uint16_t med = buffer[4 * BUF_SIZE / 8];
    uint16_t q3 = buffer[6 * BUF_SIZE / 8];
    print("stats: %u-%u-%u\r\n", q1, med, q3);

    // determine zero crossings
    sample_reset();
    StateMachine sm(q1 - med, q3 - med);
    int t = 0;
    uint32_t start = millis();
    double first = 0.0;
    double last = 0.0;
    int count = 0;
    bool done = false;
    while (!done && ((millis() - start) < 3000)) {
        double value = 0;
        if (sample_get(&value)) {
            double time = (double)t / SAMPLE_FREQUENCY;
            if (sm.process(time, value - med)) {
                switch (count) {
                case 0:
                    first = sm.get_result();
                    break;
                case 50:
                    last = sm.get_result();
                    done = true;
                    break;
                default:
                    break;
                }
                count++;
            }
            t++;
        }
    }
    double freq = 50.0 / (last - first);
    print("n=%d,first=%f,last=%f,frequency=%f\r\n", count, first, last, freq);

    return 0;
}

static int do_reboot(int argc, char *argv[])
{
    return 0;
}

static int do_help(int argc, char *argv[]);

const cmd_t commands[] = {
    { "help", do_help, "Show help" },
    { "adc", do_adc, "ADC functions" },
    { "stats", do_stats, "Stats" },
    { "f", do_freq, "Measure frequency" },
    { "reboot", do_reboot, "Reboot" },
    { NULL, NULL, NULL }
};

static int do_help(int argc, char *argv[])
{
    show_help(commands);
    return CMD_OK;
}

void setup(void)
{
    PrintInit();
    EditInit(line, sizeof(line));

    pinMode(LED_BUILTIN, OUTPUT);

    adc_init();

    Serial.begin(115200);
    Serial.println("\nSTM32SAMPLER");
}

void loop(void)
{
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
            print("OK\r\n");
            break;
        case CMD_NO_CMD:
            break;
        case CMD_UNKNOWN:
            print("Unknown command, available commands:\r\n");
            show_help(commands);
            break;
        default:
            print("%d\r\n", result);
            break;
        }
        print(">");
    }
}
