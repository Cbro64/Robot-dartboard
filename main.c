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
void waitMotion(); // waits until current motion is complete
void tripRelay(int);
void tripRelay2();
void triangulate(float[3],float[3],float[3],float[3],float*);

// Global variables
int width;
int height;
int mode = 0; // 0 = startup, 1 = homing, 2=waiting, 3 = throwing, 4 = stop, 5 = calibrating
// control signals: 251 = finished homing, 252 get ready to throw, 253 throw ended , 254 calibrating
int currentX;
int currentY;
int pwmRangeX;
int pwmRangeY;
int piRangeX = 250;
int piRangeY = 250;
int leftDir;
int rightDir;
volatile int leftReached;
volatile int rightReached;
int targetX;
int targetY;
int Ltriggered=0, Rtriggered=0, Ttriggered=0, Btriggered=0;
int rangeOffset = 700;
int floorRecvd = 0;
int sideRecvd = 0;
int speedRamp = 220;
int speedRampInit = 220;
int speedInc = 5;
int goNext = 0;
int calibrate = 0;

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    LED_Write(1);
    CyDelay(1000);
    LED_Write(0);
    
    currentX = 0;
    currentY = 0;
    targetX = 0;
    targetY = 0;
    
    int button = 0;
    int buttonPre = 0;
    
    setup();
    home();
    //CyDelay(1000);
    //mode = 3;
    //mode = 2;
    //uint8 txData = 251;
    //UART_FLOOR_PutChar(txData);
    //UART_SIDE_PutChar(txData);
    
    //setMotion(0,0,240);
    //waitMotion();
    //CyDelay(1000);
    //setMotion(pwmRangeX,pwmRangeY,240);
    //waitMotion();
   // CyDelay(1000);
   // setMotion(pwmRangeX, 0, 240);
    //waitMotion(); 
   // CyDelay(1000);
   // setMotion(0, pwmRangeY,240);
    //waitMotion(); 
   // CyDelay(1000);
   // setMotion(0, pwmRangeY/2,240);
   // waitMotion(); 
   // CyDelay(1000);
  //  setMotion(pwmRangeX/2, pwmRangeY/2,240); 
  //  CyDelay(1000);
    //waitMotion();
    while(1) {
        if (mode == 2 && goNext && calibrate) {
            switch (goNext) {
                case 1: setMotion(0, pwmRangeY, 150);
                break;
                case 2: setMotion(0, 0, 150);
                break;
                case 3: setMotion(pwmRangeX, pwmRangeY, 150);
                break;
                case 4: setMotion(pwmRangeX, 0, 150);
                break;
                case 5: setMotion(0, pwmRangeY, 150);
                break;
                case 6: setMotion(pwmRangeX, pwmRangeY, 150);
                break;
                case 7: setMotion(0, pwmRangeY, 150);
                break;
                case 8: setMotion(pwmRangeX, 0, 150);
                break;
                case 9: setMotion(0, 0, 150);
                break;
                case 10: setMotion(0, pwmRangeY, 150);
                break;
            }
            calibrate = 0;
        }
    }
}

CY_ISR(leftISR) {
    if(mode == 0) return;
    else if(mode == 1) {
        Ltriggered = 1;
    } else {
        tripRelay(1);
    }
}

CY_ISR(rightISR) {
    if(mode == 0) return;
    else if(mode == 1) {
        Rtriggered = 1;
    } else {
        tripRelay(2);
    }
}

CY_ISR(topISR) {
    if(mode == 0) return;
    else if(mode == 1) {
        Ttriggered = 1;
    } else {
        tripRelay(3);
    }
}

CY_ISR(bottomISR) {
    if(mode == 0) return;
    else if(mode == 1) {
        Btriggered = 1;
    } else {
        tripRelay(4);
    }
}

CY_ISR(FloorISR) {
    // reads 1 byte - turn on LED when byte received
    uint8 rxData  = UART_FLOOR_RXDATA_REG;
    //LED_Write(1);
    if(UART_FLOOR_RX_STS_FIFO_NOTEMPTY != 0u && mode == 3) {
        //rxData = UART_FLOOR_RXDATA_REG;
        //UART_SIDE_TXDATA_REG = rxData;
        UART_SIDE_PutChar(rxData);
        if(rxData <=250) {
            floorRecvd = 1;
            targetX = (int)(rxData / (float)(piRangeX) * pwmRangeX);
            if (sideRecvd == 1) {
                setMotion(targetX, targetY, speedRamp);
                floorRecvd = 0;
                sideRecvd = 0;
                speedRamp = speedRamp +speedInc;
            }
        }
    }
}

CY_ISR(SideISR) {
    // reads 1 byte - turn on LED when byte received
    uint8 rxData = UART_SIDE_RXDATA_REG;
    //LED_Write(1);
    if(UART_SIDE_RX_STS_FIFO_NOTEMPTY != 0u && mode == 3) {
        //rxData 
        UART_FLOOR_PutChar(rxData);
        if(rxData <=250) {
            sideRecvd = 1;
            targetY = (int)((rxData) / (float)(piRangeY) * pwmRangeY);
            if (floorRecvd == 1) {
                setMotion(targetX, targetY, speedRamp);
                floorRecvd = 0;
                sideRecvd = 0;
                speedRamp = speedRamp + speedInc;
            }
        } else {
            if(rxData == 253) {
                mode = 2;
                //UART_FLOOR_TXDATA_REG = rxData;
                speedRamp = speedRampInit;
                //CyDelay(2000);
                setMotion(pwmRangeX/2, pwmRangeY/2,50);
                //waitMotion();
            }
        }
        
    }
    else if(mode == 2) {
        //rxData = UART_SIDE_RXDATA_REG;
        if (rxData == 252) {
            UART_FLOOR_PutChar(rxData);
            mode = 3;
        }
        else if (rxData == 254) {
            calibrate = 1;
            goNext++;
        }
        else if (rxData == 255) {
            calibrate = 1;
            goNext = 0;
        }
    }
}

CY_ISR(LeftCountISR) {
    pwm_left_on_Write(0);
    leftReached = 1;
}

CY_ISR(RightCountISR) {
    pwm_right_on_Write(0);
    rightReached = 1;
}

CY_ISR(eStopISR) {
    if(mode!=0) {
        tripRelay2();
    }
}

void setup() {
    // assign interrupt service routines to interrupts
    isr_left_limit_StartEx(leftISR);
    isr_right_limit_StartEx(rightISR);
    isr_top_limit_StartEx(topISR);
    isr_bottom_limit_StartEx(bottomISR);
    isr_floorpi_StartEx(FloorISR);
    isr_sidepi_StartEx(SideISR);
    //isr_left_count_StartEx(LeftCountISR);
    //isr_right_count_StartEx(RightCountISR);
    isr_estop_StartEx(eStopISR);
    
    // start hardware components
    UART_SIDE_Start();
    UART_FLOOR_Start();
    Clock_PWM_Start();
    Clock_COUNT_Start();
    Counter_left_Start();
    Counter_right_Start();
    PWM_right_Start();
    PWM_left_Start();
}

void home() {
    mode = 1;
    int speed= 1;
    // send carriage far left
    setMotion(-20000, 0, speed);
    while(Ltriggered != 1) { CyDelay(1); } // wait for left limit switch
    stopPWMs();
    clearCounters();
    currentX = -rangeOffset; // NOTE: add an offset

    // send carriage far right
    setMotion(20000, 0, speed);
    while(Rtriggered != 1) { CyDelay(1); } // wait for right limit switch
    stopPWMs();
    pwmRangeX = Counter_left_ReadCounter()-(2*rangeOffset); // read pwm count from counter
    clearCounters();
    currentX = pwmRangeX+rangeOffset; // NOTE: add an offset
    
    // travel to horizontal mid point
    setMotion(pwmRangeX/2, 0, speed*2);
    //waitMotion();
    CyDelay(2000);

    // send carriage far bot
    setMotion(pwmRangeX/2, -20000, speed);
    while(Ttriggered != 1) { CyDelay(1); } // wait for top limit switch
    stopPWMs();
    clearCounters();
    currentY = -rangeOffset; // NOTE: add an offset
    
    // send carriage far top
    setMotion(pwmRangeX/2, 20000, speed);
    while(Btriggered != 1) { CyDelay(1); } // wait for bottom limit switch
    stopPWMs();
    pwmRangeY = Counter_left_ReadCounter()-(2*rangeOffset); // read pwm count from counter
    clearCounters();
    currentY = pwmRangeY+rangeOffset; // NOTE: add an offset
    
    // clear limit switch trigger status
    Ltriggered = 0;
    Rtriggered = 0;
    Ttriggered = 0;
    Btriggered = 0;
    
    // travel to centre
    setMotion(pwmRangeX/2, pwmRangeY/2, speed*2);
    targetX = pwmRangeX/2;
    targetY = pwmRangeY/2;
    //waitMotion();
    CyDelay(2000);
    mode = 2;
    uint8 txData = 251;
    UART_FLOOR_PutChar(txData);
    UART_SIDE_PutChar(txData);
    
}


void setMotion(int x, int y, int speed) { // note: currently deals in motor pulses
    // speed range 0 to 244
    if(mode != 1) {
        if(speed>240) speed = 240;
        if(speed<0) speed = 0;
        if(x<0) x = 0;
        if(y<0) y = 0;
        if(x>pwmRangeX) x = pwmRangeX;
        if(y>pwmRangeY) y = pwmRangeY;
    } else {
        speed = 1;
    }
    if (mode != 4) { // do not run if in emergency stop mode
        pwm_left_on_Write(0);
        pwm_right_on_Write(0);
        updatePos();
        leftReached = 0;
        rightReached = 0;
        int x_travel = x - currentX;
        int y_travel = y - currentY;
        // check for out of bounds movement here
        int leftPulses = x_travel + y_travel;
        int rightPulses = x_travel - y_travel;
        if(leftPulses > 0) {
            Pin_left_dir_Write(0);
            leftDir = 1;
        } 
        else {
            Pin_left_dir_Write(1);
            leftDir = -1;
        }
        if(rightPulses > 0) {
            Pin_right_dir_Write(0);
            rightDir = 1;
        }
        else {
            Pin_right_dir_Write(1);
            rightDir = -1;
        }
        leftPulses = abs(leftPulses);
        rightPulses = abs(rightPulses);
        // set hardware PWM pulse counts, and start/update pwms here
        Counter_left_WriteCompare(leftPulses);
        Counter_right_WriteCompare(rightPulses);
        
        int period = 255 - speed;
        
        PWM_right_WritePeriod(period);
        PWM_right_WriteCompare(1+period/2);
        PWM_left_WritePeriod(period);
        PWM_left_WriteCompare(1+period/2);
        pwm_left_on_Write(1);
        pwm_right_on_Write(1);
        
        
    }
}

void updatePos() {
    // Reads counters to update current position and clears counters
    int left_counter = Counter_left_ReadCounter() * leftDir;
    int right_counter = Counter_right_ReadCounter() * rightDir;
    currentX += (left_counter + right_counter) /2;
    currentY += (left_counter - right_counter) /2;
    clearCounters();
}

void stopPWMs() {
    pwm_left_on_Write(0);
    pwm_right_on_Write(0);
}

void clearCounters() {
    Counter_left_Stop();
    Counter_right_Stop();
    Counter_left_WriteCounter(0);
    Counter_right_WriteCounter(0);
    Counter_left_Start();
    Counter_right_Start();
    //counter_left_reset_Write(1);
    //counter_right_reset_Write(1);
    //CyDelay(1);
    //counter_left_reset_Write(0);
    //counter_right_reset_Write(0);
}


void waitMotion() {
    while (!(left_cnt_compare_Read() &&  right_cnt_compare_Read())) {
        //CyDelay(1);
    }
}

void tripRelay(int i) {
    // trip power relay
    Pin_relay_Write(0);
    pwm_left_on_Write(0);
    pwm_right_on_Write(0);
    mode = 4;
    PWM_left_Stop();
    PWM_right_Stop();
    while(1){
        for (int j = 0; j<i;j++) {
            LED_Write(1);
            CyDelay(200);
            LED_Write(0);
            CyDelay(200);
        }
        CyDelay(1000);
    }
}

void tripRelay2() {
    // trip power relay
    Pin_relay_Write(0);
    pwm_left_on_Write(0);
    pwm_right_on_Write(0);
    mode = 4;
    PWM_left_Stop();
    PWM_right_Stop();
    while(1){
        LED_Write(1);
        CyDelay(100);
        LED_Write(0);
        CyDelay(100);
    }
}




void triangulate(float v1[3], float v2[3], float c1[3], float c2[3], float *point) {
    
    //[v1 -v2]' * [v1 -v2]
    float step1[2][2] = {0};
    step1[0][0] = v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2];
    step1[0][1] = -v1[0]*v2[0] - v1[1]*v2[1] - v1[2]*v2[2];
    step1[1][0] = step1[0][1];
    step1[1][1] = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    
    // inverse step 1
    float invStep1[2][2] = {0};
    float det = 1/(step1[0][0]*step1[1][1] - step1[0][1]*step1[1][0]);
    invStep1[0][0] = det*step1[1][1];
    invStep1[0][1] = -det*step1[0][1];
    invStep1[1][0] = -det*step1[1][0];
    invStep1[1][1] = det*step1[0][0];
    
    // c2 - c1 !note: this is the same every time so maybe pre calculate
    float deltaC[3] = {0};
    deltaC[0] = c2[0] - c1[0];
    deltaC[1] = c2[1] - c1[1];
    deltaC[2] = c2[2] - c1[2];
    
    // [v1 -v2]' * deltaC
    float step2[2] = {0};
    step2[0] = v1[0]*deltaC[0] + v1[1]*deltaC[1] + v1[2]*deltaC[2];
    step2[1] = -v2[0]*deltaC[0] - v2[1]*deltaC[1] - v2[2]*deltaC[2];
    
    // multiply step 1 inv and step 2
    float s[2] = {0};
    s[0] = invStep1[0][0]*step2[0] + invStep1[0][1]*step2[1];
    s[1] = invStep1[1][0]*step2[0] + invStep1[1][1]*step2[1];
    
    // calculate points on each line
    float p1[3] = {0};
    p1[0] = c1[0] + s[0]*v1[0];
    p1[1] = c1[1] + s[0]*v1[1];
    p1[2] = c1[2] + s[0]*v1[2];
    
    float p2[3] = {0};
    p2[0] = c2[0] + s[1]*v2[0];
    p2[1] = c2[1] + s[1]*v2[1];
    p2[2] = c2[2] + s[1]*v2[2];
    
    // p = (p1+p2) / 2
    point[0] = (p1[0] + p2[0]) / 2;
    point[1] = (p1[1] + p2[1]) / 2;
    point[2] = (p1[2] + p2[2]) / 2;
}
/* [] END OF FILE */
