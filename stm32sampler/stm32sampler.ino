#include <Arduino.h>
#include <STM32ADC.h>

#include "print.h"
#include "cmdproc.h"
#include "editline.h"

static STM32ADC adc(ADC1);
static char line[120];

static uint8 pins = 0;

static uint32_t value = 0;

#define SAMPLE_FREQUENCY       10000


static void show_help(const cmd_t * cmds)
{
    for (const cmd_t * cmd = cmds; cmd->cmd != NULL; cmd++) {
        print("%10s: %s\n", cmd->name, cmd->help);
    }
}

static volatile uint32_t int_count = 0;

static void adc_int(void)
{
    int_count++;
    value = adc.getData();
}

static int do_adc(int argc, char *argv[])
{
    if (argc > 1) {
        char *cmd = argv[1];
        if (strcmp(cmd, "timer") == 0) {
            Timer3.setPeriod(1000000 / SAMPLE_FREQUENCY);
            Timer3.setMasterModeTrGo(TIMER_CR2_MMS_UPDATE);
        }
        if (strcmp(cmd, "init") == 0) {
            adc.calibrate();
            adc.setSampleRate(ADC_SMPR_1_5);    // ?
            adc.setPins(&pins, 1);
            adc.setTrigger(ADC_EXT_EV_TIM3_TRGO);
            adc.attachInterrupt(adc_int, ADC_EOC);
            print("done\n");
        }
        if (strcmp(cmd, "start") == 0) {
            print("start\n");
            adc.startConversion();
        }
    }
    print("interrupts: %d\n", int_count);
    print("value: %d\n", value);
    return 0;
}

static int do_help(int argc, char *argv[]);

const cmd_t commands[] = {
    { "help", do_help, "Show help" },
    { "adc", do_adc, "ADC functions" },
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

    pinMode(pins, INPUT_ANALOG);

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

