// values returned from calculation
typedef unsigned char byte;
#define RESULT_XMIN      0x0001
#define RESULT_XMAX      0x0002
#define RESULT_YMIN      0x0004
#define RESULT_YMAX      0x0008
#define RESULT_SENSORA   0x0010
#define RESULT_SENSORB   0x0020
#define RESULT_SENSORC   0x0040
#define RESULT_SENSORD   0x0080
#define RESULT_READ      RESULT_SENSORA|RESULT_SENSORB|RESULT_SENSORC|RESULT_SENSORD
#define RESULT_MISREAD   0x1000
#define RESULT_ERROR     0x2000

#define UI_TOP 0
#define UI_BOTTOM 1


void UISetup();
void UIReport(byte which, unsigned int result, byte row, byte col, unsigned long milliseconds);
void UIRun(unsigned long milliseconds);


