
#ifndef LineSensor_h
#define LineSensor_h

//This is needed to Align the Robot to The Lines. Which allows them to
//Detect positions they can move in at intersections, and when to Stop.
typedef enum {LEFT,CENTRE,RIGHT,NONE} line_pos_t;

class LineSensor {
public:
    LineSensor(uint8_t pin1,uint8_t pin2,uint8_t pin3, uint8_t pin4);
    line_pos_t get_line();
    void init_calibrate();
    void calibrate();
private:
    uint8_t _pins[4];
    uint16_t _min[4];
    uint16_t _max[4];
    uint16_t _threshold[4];
};

#endif 