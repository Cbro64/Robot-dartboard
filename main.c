/* ========================================
 *
 * Cameron Brown 2019
 * PSoC code for robotic darboard final year project
 * S1 2019 Monash University
 *
 * Project partner: James Rowe
 *
 * ========================================
*/
#include "project.h"
#include "stdlib.h"

void setup();
void home(); // homing routine that runs every time psoc is reset. Measures range of motion of system, in motor pulses.
void setMotion(int, int, int); // sends motion system to target location (x, y, speed)
void updatePos(); // reads the pwm counters and updates the current position
void stopPWMs(); // stops sending position commands to motors
void clearCounters();
int atTarget();
void tripRelay();


// Global variables
int width;
int height;
int mode = 0; // 0 = startup, 1 = homingWait, 2=homingContinue, 3 = operation, 4 = stop
int currentX = 0;
int currentY = 0;
int pwmRangeX;
int pwmRangeY;
int piRangeX = 255;
int piRangeY = 255;
int leftDir;
int rightDir;
int leftReached = 0;
int rightReached = 0;

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    LED_Write(1);
    CyDelay(1000);
    LED_Write(0);
    
    int button = 0;
    int buttonPre = 0;
    
    setup();
    //home();
    mode = 3;
    while(1) {
        button = SW1_Read();
        if((button == 0) && (buttonPre !=0)) LED_Write(0); // turn off led when button is released
        buttonPre = button;
    }
}

CY_ISR(leftISR) {
    if(mode != 1) {
        tripRelay();
    } else {
        stopPWMs();
        clearCounters();
        currentX = 0;
        mode = 2;
    }
}

CY_ISR(rightISR) {
    if(mode != 1) {
        tripRelay();
    } else {;
        stopPWMs();
        pwmRangeX = Counter_left_ReadCounter(); // read pwm count from counter
        clearCounters();
        currentX = pwmRangeX;
        mode = 2;
    }
}

CY_ISR(topISR) {
    if(mode != 1) {
        tripRelay();
    } else {
        stopPWMs();
        clearCounters();
        currentY = 0;
        mode = 2;
    }
}

CY_ISR(bottomISR) {
    if(mode != 1) {
        tripRelay();
    } else {
        stopPWMs();
        pwmRangeY = Counter_left_ReadCounter(); // read pwm count from counter
        clearCounters();
        currentY = pwmRangeY;
        mode = 2;
    }
}

CY_ISR(FloorISR) {
    // reads 1 byte - turn on LED when byte received
    uint8 rxData;
    if(UART_FLOOR_RX_STS_FIFO_NOTEMPTY != 0u) {
        rxData = UART_FLOOR_RXDATA_REG;
        LED_Write(1);
    }
}

CY_ISR(SideISR) {
    // reads 1 byte - turn on LED when byte received
    uint8 rxData;
    if(UART_SIDE_RX_STS_FIFO_NOTEMPTY != 0u) {
        rxData = UART_SIDE_RXDATA_REG;
        LED_Write(1);
    }
}

CY_ISR(LeftCountISR) {
    PWM_left_Stop();
    leftReached = 1;
}

CY_ISR(RightCountISR) {
    PWM_right_Stop();
    rightReached = 1;
}

void setup() {
    // assign interrupt service routines to interrupts
    isr_left_limit_StartEx(leftISR);
    isr_right_limit_StartEx(rightISR);
    isr_top_limit_StartEx(topISR);
    isr_bottom_limit_StartEx(bottomISR);
    isr_floorpi_StartEx(FloorISR);
    isr_sidepi_StartEx(SideISR);
    isr_left_count_StartEx(LeftCountISR);
    isr_right_count_StartEx(RightCountISR);
    
    // start hardware components
    UART_SIDE_Start();
    UART_FLOOR_Start();
    Clock_PWM_Start();
    Clock_COUNT_Start();
    Counter_left_Start();
    Counter_right_Start();
}

void home() {
    mode = 1;
    // set pwm to send carriage far left
    setMotion(-20000, 0, 20);
    while(mode != 2) { CyDelay(10); } // wait for mode change
    mode = 1;
    // set pwm to send carriage far right
    setMotion(20000, 0, 20);
    while(mode != 2) { CyDelay(10); } // wait for mode change
    mode = 1;
    setMotion(pwmRangeX/2, 0, 20);
    while(atTarget() != 1) {
        CyDelay(10);
    }
    setMotion(pwmRangeX/2, -20000, 20);
    while(mode != 2) { CyDelay(10); } // wait for mode change
    mode = 1;
    // set pwm to send carriage far bottom
    setMotion(pwmRangeX/2, 20000, 20);
    while(mode != 2) { CyDelay(10); } // wait for mode change
    
    // travel to centre
    setMotion(pwmRangeX/2, pwmRangeY/2, 20);
}


void setMotion(int targetX, int targetY, int speed) { // note: currently deals in motor pulses
    if (mode != 4) { // do not run if in emergency stop mode
        // todo: deal with speed settings
        stopPWMs();
        updatePos();
        leftReached = 0;
        rightReached = 0;
        int x_travel = targetX - currentX;
        int y_travel = targetY - currentY;
        // check for out of bounds movement here
        int leftPulses = x_travel + y_travel;
        int rightPulses = x_travel - y_travel;
        if(leftPulses > 0) {
            // todo set left dir pin here
            leftDir = 1;
        } 
        else {
            leftDir = -1;
        }
        if(rightPulses > 0) {
            // todo set right dir pin here
            rightDir = 1;
        }
        else {
            rightDir = -1;
        }
        leftPulses = abs(leftPulses);
        rightPulses = abs(rightPulses);
        // set hardware PWM pulse counts, and start/update pwms here
        Counter_left_WriteCompare(leftPulses);
        Counter_right_WriteCompare(rightPulses);
        PWM_left_Start();
        PWM_right_Start();
    }
}

void updatePos() {
    // Reads counters to update current position and clears counters
    int left_counter = Counter_left_ReadCounter() * leftDir;
    int right_counter = Counter_right_ReadCounter() * rightDir;
    currentX += (left_counter + right_counter) /2;
    currentY += (left_counter - right_counter) /2;
}

void stopPWMs() {
    PWM_left_Stop();
    PWM_right_Stop();
}

void clearCounters() {
    Counter_left_WriteCounter(0);
    Counter_right_WriteCounter(0);
}

int atTarget() {
    if (leftReached == 1 && rightReached == 1) {
        return 1;
    }
    return 0;
}

void tripRelay() {
    // trip power relay
    Pin_relay_Write(0);
    mode = 4;
    PWM_left_Stop();
    PWM_right_Stop();
}


/* [] END OF FILE */
