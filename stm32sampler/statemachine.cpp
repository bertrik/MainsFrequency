#include "statemachine.h"

StateMachine::StateMachine(double low, double high)
{
    this->low = low;
    this->high = high;
    this->state = STATE_IDLE;
}

bool StateMachine::process(double time, double v)
{
    switch (state) {
    case STATE_IDLE:
        if (v < low) {
            this->state = STATE_LOW;
        }
        break;
    case STATE_LOW:
        if (v > low) {
            // start linear regression
            this->regression.reset();
            this->regression.add(time, v);
            // start collecting slope data
            this->state = STATE_SLOPE;
        }
        break;
    case STATE_SLOPE:
        if (v > high) {
            // end linear regression
            this->regression.calculate();
            this->result = this->regression.getC();
            this->state = STATE_IDLE;
            return true;
        } else {
            this->regression.add(time, v);
        }
        break;
    default:
        this->state = STATE_IDLE;
        break;
    }
    return false;
}

double StateMachine::get_result()
{
    return result;
}
