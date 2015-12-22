#include "LineSensor.h"

/*
 * Constructor
 * Assigns 4 Pins to Line Sensor. Sets Min and Max of Readings. And Threshold
 */
LineSensor::LineSensor(uint8_t pin1,uint8_t pin2,uint8_t pin3, uint8_t pin4) {
    _pins[0] = pin1; _pins[1] = pin2; _pins[2] = pin3; _pins[3] = pin4;
    //TODO Why not just run init_calibrate function here?
    int i;
    for(i = 0; i < 4; i++){
        _max[i] = 0;    //Max is 0 as Prior calibration
        _min[i] = 1023; //Max is Largest Num as Prior calibration
        _threshold[i] = 950; //TODO why 950
    }
}

/*
 * Gets Analog Readings from Sensor Pins
 * TODO What does it return? The Direction to adjust towards?
 * Check where the Line is under the 4 Aligned Sensors. (E.g Dead centre? To The Right? Left?)
 * Allows the Ability to Realign the Robot so that the line is Center of the 4 Sensors
 */
line_pos_t LineSensor::get_line() {
    int curLine[4];
    int tripCount = 0, sum = 0, i = 0;
    for(i = 0;i < 4; i++) {
        //1 if Under Threshold, 0 if Over Threshold
        curLine[i] = ( analogRead(_pins[i]) < _threshold[i] ); 
        tripCount += curLine[i];
        curLine[i] *= (2 * i) - 3; //-3, -1, 1, 3
        sum += curLine[i]; 
    }
    if (tripCount == 0) {
        //If All the Readings on the 4 Sensors are over* the threshold (950)
        // TODO Line Detected? Does that mean no need to readjust
        return NONE;
    } else if(sum < 0) {
        return LEFT;
    } else if(sum > 0) {
        return RIGHT;
    } else {
        return CENTRE;
    }
}

/*
 * Combine with "calibrate" function?
 */
void LineSensor::init_calibrate(){
    int i;
    for(i = 0; i < 4; i++){
        _max[i] = 0;
        _min[i] = 1023;
        _threshold[i] = 950;
    }
}

/*
 * Calibrate Max and Min of the Sensors
 * TODO Probably activate calibrate when Sensor is on Line and when it is not on Line
 * Room lighting env must stay same?
 */
void LineSensor::calibrate(){
    int i;
    unsigned long val;
    for(i = 0; i < 4; i++){
        val = analogRead(_pins[i]);
        _max[i] = max(_max[i],val);
        _min[i] = min(_min[i],val);
        _threshold[i] = (_max[i]+_min[i])/2;
    }
}