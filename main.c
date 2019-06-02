/* ========================================
 *
 * Cameron Brown 2019
 * PSoC code for Mechatronics Final Year Project - Ball Catching Robot
 * S1 2019 Monash University
 *
 * Project partner: James Rowe
 *
 * ========================================
*/
#include "project.h"
#include "stdlib.h"

// Function declarations
void setup(); // Assigns interrupt service routines and starts hardware components
void home(); // Homing routine that runs every time psoc is reset. Measures range of motion of system, in motor pulses.
void setMotion(int, int, int); // Sends motion system to target location (x, y, speed)
void updatePos(); // Reads the pwm counters and calculates the current position of the motion system
void stopPWMs(); // Stops sending position commands to motors
void clearCounters(); // Resets the motor pulse counters
void STOP(int);

// Global variables
int mode; // System state: 0 = startup, 1 = homing, 2 = ready to catch, 3 = catching, 4 = stop
int currentX; // Current position of motion system as measured in motor pulse units
int currentY;
int targetX; // Target position of motion system as measured in motor pulse units
int targetY;
int pwmRangeX; // Horizontal width of range of motion as measured in motor pulse units
int pwmRangeY; // Vertical width of range of motion  as measured in motor pulse units
int piRangeX = 250; // Maximum range of UART values used for prediction messages from Raspberry Pis
int piRangeY = 250;
int leftDir; // Direction control variables for motor commands
int rightDir;
int Ltriggered=0, Rtriggered=0, Ttriggered=0, Btriggered=0; // Stores whether or not the left,right,top,or bottom limit switches have been triggered
int rangeOffset = 700; // Safety distance between limit switches and software range-of-motion (measured in motor pulses)
int floorRecvd = 0; // Stores whether or not a new prediction has been recieved from floor camera
int sideRecvd = 0; // Stores whether or not a new prediction has been recieved from side camera
int catchSpeedInit = 220; // Initial starting speed of catch movements
int catchSpeed = catchSpeedInit; // Current speed of catch movements - this increases for the first few predictions up to max speed (240)
int speedInc = 5; // Speed increment after each new prediction

// UART control signals: 251 = finished homing, 252 = begin catching, 253 = throw ended

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    // Flash LED once
    LED_Write(1);
    CyDelay(1000);
    LED_Write(0);
    
    mode = 0;
    currentX = 0;
    currentY = 0;
    targetX = 0;
    targetY = 0;  
  
    setup();
    home();
   
    while(1) {
      CyDelay(1);
    }
}

// Left limit switch interrupt
CY_ISR(leftISR) {
    if(mode == 0) return;
    else if(mode == 1) { // During homing routine
        Ltriggered = 1;
    } else {  // Unexpected behaviour
        STOP(1); // Emergency Stop mode with error code 1 - left limit switch
    }
}

// Right limit switch interrupt
CY_ISR(rightISR) {
    if(mode == 0) return;
    else if(mode == 1) { // During homing routine
        Rtriggered = 1;
    } else {
        STOP(2); // Emergency Stop mode with error code 2 - right limit switch
    }
}

// Top limit switch interrupt
CY_ISR(topISR) {
    if(mode == 0) return;
    else if(mode == 1) { // During homing routine
        Ttriggered = 1;
    } else {
        STOP(3); // Emergency Stop mode with error code 3 - top limit switch
    }
}

// Bottom limit switch interrupt
CY_ISR(bottomISR) {
    if(mode == 0) return;
    else if(mode == 1) { // During homing routine
        Btriggered = 1;
    } else {
        STOP(4); // Emergency Stop mode with error code 4 - bottom limit switch
    }
}

// UART recieve interrupt for floor Pi
CY_ISR(FloorISR) {
    uint8 rxData = UART_FLOOR_RXDATA_REG;
    if(UART_FLOOR_RX_STS_FIFO_NOTEMPTY != 0u && mode == 3) { // If system is in catching mode
        UART_SIDE_PutChar(rxData); // Forward received message to Side pi
        if(rxData <= piRangeX) { // Prediction message
            floorRecvd = 1;
            targetX = (int)(rxData / (float)(piRangeX) * pwmRangeX); // Convert to motor pulse units
            if (sideRecvd == 1) { // If a corresponding prediction has already been recieved from Side pi
                setMotion(targetX, targetY, catchSpeed); // Send motion system to predicted location
                floorRecvd = 0;
                sideRecvd = 0;
                catchSpeed = catchSpeed + speedInc; // Increment catch speed
            }
        }
    }
}

// UART recieve interrupt for Side Pi
CY_ISR(SideISR) {
    uint8 rxData = UART_SIDE_RXDATA_REG;
    if(UART_SIDE_RX_STS_FIFO_NOTEMPTY != 0u && mode == 3) { // If system is in catching mode
        UART_FLOOR_PutChar(rxData); // Forward received message to Floor pi
        if(rxData <= piRangeY) { // Prediction message
            sideRecvd = 1;
            targetY = (int)((rxData) / (float)(piRangeY) * pwmRangeY); // Convert to motor pulse units
            if (floorRecvd == 1) { // If a corresponding prediction has already been recieved from Floor pi
                setMotion(targetX, targetY, catchSpeed); // Send motion system to predicted location
                floorRecvd = 0;
                sideRecvd = 0;
                catchSpeed = catchSpeed + speedInc; // Increment catch speed
            }
        } else { // Control message
            if(rxData == 253) { // Throw ended
                mode = 2; // Set system mode back to 'ready to catch'
                catchSpeed = catchSpeedInit; // Reset catch speed back to initial
                setMotion(pwmRangeX/2, pwmRangeY/2,50); // Return motion system to centre
            }
        }
    }
    else if(mode == 2) { // If system is in 'ready to catch' mode
        if (rxData == 252) { // begin catching
            UART_FLOOR_PutChar(rxData); // forward message to floor Pi
            mode = 3; // set system state to 'catching'
        }
    }
}

// E-Stop button interrupt
CY_ISR(eStopISR) {
    if(mode!=0) { // if not in setup mode
        STOP(0); //Emergency Stop mode with error code 0 - Estop button
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
    int speed = 1;
  
    // send carriage far left
    setMotion(-20000, 0, speed);
    while(Ltriggered != 1) { CyDelay(1); } // wait for left limit switch
    stopPWMs();
    clearCounters();
    currentX = -rangeOffset; // Set current horizontal location to be just outside left software boundry

    // send carriage far right
    setMotion(20000, 0, speed);
    while(Rtriggered != 1) { CyDelay(1); } // wait for right limit switch
    stopPWMs();
    pwmRangeX = Counter_left_ReadCounter()-(2*rangeOffset); // Read travel distance in motor pulses - subtract saftey offset from each end
    clearCounters();
    currentX = pwmRangeX+rangeOffset; // Set current horizontal location to be just outside right software boundry
    
    // travel to horizontal mid point
    setMotion(pwmRangeX/2, 0, speed);
    CyDelay(2000);

    // send carriage far bottom
    setMotion(pwmRangeX/2, -20000, speed);
    while(Ttriggered != 1) { CyDelay(1); } // wait for top limit switch
    stopPWMs();
    clearCounters();
    currentY = -rangeOffset; // Set current vertical location to be just below bottom software boundry
    
    // send carriage far top
    setMotion(pwmRangeX/2, 20000, speed);
    while(Btriggered != 1) { CyDelay(1); } // wait for bottom limit switch
    stopPWMs();
    pwmRangeY = Counter_left_ReadCounter()-(2*rangeOffset); // Read travel distance in motor pulses - subtract saftey offset from each end
    clearCounters();
    currentY = pwmRangeY+rangeOffset; // Set current vertical location to be just above top software boundry
    
    // clear limit switch trigger status
    Ltriggered = 0;
    Rtriggered = 0;
    Ttriggered = 0;
    Btriggered = 0;
    
    // travel to centre
    targetX = pwmRangeX/2;
    targetY = pwmRangeY/2;
    setMotion(targetX, targetY, speed*2);
    CyDelay(2000);
    mode = 2; // Ready to catch mode
    uint8 txData = 251; // Tell raspberry pis that homing has finished
    UART_FLOOR_PutChar(txData);
    UART_SIDE_PutChar(txData);
}

// This function sends the motion system to an input location
void setMotion(int x, int y, int speed) { // x,y are measured in motor pulse units
    // speed range 0 to 244 (but capped at 240)
    if(mode != 1) { // if not homing
        // limit input speeds and locations to be within acceptable ranges
        if(speed>240) speed = 240;
        if(speed<0) speed = 0;
        if(x<0) x = 0;
        if(y<0) y = 0;
        if(x>pwmRangeX) x = pwmRangeX;
        if(y>pwmRangeY) y = pwmRangeY;
    } else { // during homing routine force slowest speed
        speed = 1;
    }
    if (mode != 4) { // prevents any motion from being able to occur if system is in stop mode
        pwm_left_on_Write(0); // disconnect motors from PWMs
        pwm_right_on_Write(0);
        updatePos(); // calculate current motion system position
      
        // calculate relative motion required
        int x_travel = x - currentX;
        int y_travel = y - currentY;
        
        // convert from cartesian coordinates to motor control coordinates
        int leftPulses = x_travel + y_travel;
        int rightPulses = x_travel - y_travel;
        
        // set required direction pins of motors
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
        
        // Set required amount of pulses in each hardware counter
        Counter_left_WriteCompare(leftPulses);
        Counter_right_WriteCompare(rightPulses);
        
        // update period of PWM generators to change motor speed
        int period = 255 - speed;
        PWM_right_WritePeriod(period);
        PWM_right_WriteCompare(1+period/2);
        PWM_left_WritePeriod(period);
        PWM_left_WriteCompare(1+period/2);
      
        // Reconnect motors with PWM outputs
        pwm_left_on_Write(1);
        pwm_right_on_Write(1);  
    }
}

// This function reads how many motor pulses have been sent to each motor since the last position update
// and then calculates the current position of the motion system in cartesian motor pulse units
void updatePos() {
    // Read hardware counters and account for direction
    int left_counter = Counter_left_ReadCounter() * leftDir;
    int right_counter = Counter_right_ReadCounter() * rightDir;
    
    // convert from motor rotation coordinates to cartesian coordinates
    currentX += (left_counter + right_counter) /2;
    currentY += (left_counter - right_counter) /2;
    
    // reset counters
    clearCounters();
}

// This function disconnects motors from PWM generators
void stopPWMs() {
    pwm_left_on_Write(0);
    pwm_right_on_Write(0);
}

// This function stops the counters, sets them to zero, then restarts them
void clearCounters() {
    Counter_left_Stop();
    Counter_right_Stop();
    Counter_left_WriteCounter(0);
    Counter_right_WriteCounter(0);
    Counter_left_Start();
    Counter_right_Start();
}


// This function is called when the system needs to go into emergency stop mode
void STOP(int i) {
    // trip power relay
    Pin_relay_Write(0);
  
    // disconnect motors from PWM generators
    pwm_left_on_Write(0);
    pwm_right_on_Write(0);
  
    // set software state
    mode = 4;
  
    // turn off PWM generators
    PWM_left_Stop();
    PWM_right_Stop();
    
    // If stop is due to E-stop button being pressed, continually flash LED
    if(i == 0) {
      while(1){
          LED_Write(1);
          CyDelay(100);
          LED_Write(0);
          CyDelay(100);
      }
    } else {
    // flash LED with error code
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
}

/* [] END OF FILE */
