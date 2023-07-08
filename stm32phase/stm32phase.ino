#include <Arduino.h>
#include <STM32ADC.h>

#include "print.h"
#include "cmdproc.h"
#include "editline.h"

#define PIN_50HZ_INPUT      PA0
#define SAMPLE_FREQUENCY    5000

static STM32ADC adc(ADC1);
static char line[120];

static uint32_t value = 0;
static volatile uint32_t int_count = 0;

// sample buffer
#define BUF_SIZE    2048
static uint16_t buffer[BUF_SIZE];

static volatile uint32_t bufr = 0;
static volatile uint32_t bufw = 0;
static volatile bool overflow = false;

static void show_help(const cmd_t * cmds)
{
    for (const cmd_t * cmd = cmds; cmd->cmd != NULL; cmd++) {
        print("%10s: %s\n", cmd->name, cmd->help);
    }
}

static int tt = 0;

static void adc_int(void)
{
    int_count++;
    uint32_t next = (bufw + 1) % BUF_SIZE;
    if (next != bufr) {
#if 1
        buffer[bufw] = adc.getData();
#else
        buffer[bufw] = 1000 + 800 * cos(tt * 2.0 * M_PI * 50.123 / SAMPLE_FREQUENCY);
        tt++;
#endif
        bufw = next;
    }
}

static void adc_init(uint8_t pin, int sample_rate)
{
    Timer3.setPeriod(1000000 / sample_rate);
    Timer3.setMasterModeTrGo(TIMER_CR2_MMS_UPDATE);

    adc.calibrate();
    adc.setSampleRate(ADC_SMPR_13_5);
    adc.setPins(&pin, 1);
    adc.setTrigger(ADC_EXT_EV_TIM3_TRGO);
    adc.attachInterrupt(adc_int, ADC_EOC);
    adc.startConversion();
}

static int do_adc(int argc, char *argv[])
{
    print("interrupts: %d\n", int_count);
    print("value: %d\n", value);
    return 0;
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

static int do_reboot(int argc, char *argv[])
{
    nvic_sys_reset();
    return 0;
}

static int do_help(int argc, char *argv[]);

const cmd_t commands[] = {
    { "help", do_help, "Show help" },
    { "a", do_adc, "ADC functions" },
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
    pinMode(LED_BUILTIN, OUTPUT);

    PrintInit();
    EditInit(line, sizeof(line));

    pinMode(PIN_50HZ_INPUT, INPUT_ANALOG);
    adc_init(PIN_50HZ_INPUT, SAMPLE_FREQUENCY);

    Serial.begin(115200);
    Serial.println("\nSTM32SAMPLER");

    sample_reset();
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
        sum_i += value * cos(2.0 * M_PI * 50 * index / SAMPLE_FREQUENCY);
        sum_q += value * sin(2.0 * M_PI * 50 * index / SAMPLE_FREQUENCY);
        index++;
        if (index >= SAMPLE_FREQUENCY) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

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
            double t = 1.0 - 0.02 * (d / 360.0);
            double freq = 50.0 / t;
            prev_angle = angle;
            print("Angle:%8.3f, dAngle:%7.3f, Freq:%7.3f, Ampl:%5.1f\n", angle, d, freq, ampl);
        }
    }
    // command line processing
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
            print("OK\n");
            break;
        case CMD_NO_CMD:
            break;
        case CMD_UNKNOWN:
            print("Unknown command, available commands:\n");
            show_help(commands);
            break;
        default:
            print("%d\n", result);
            break;
        }
        print(">");
    }
}
