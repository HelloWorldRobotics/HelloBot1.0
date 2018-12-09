# HelloBot1.0 & 2.0

CAN BUS

Data:
00 01 00 00 20 00 01 0X1 00 X2 0Y1 00 Y2

Left Wheel:
X1-> 0 or 1
0 for forward, 1 for backward
X2-> 00 to FF (0~255) speed value

Right Wheel:
Y1-> 0 or 1
0 for forward, 1 for backward
Y2-> 00 to FF (0~255) speed value

UART

Data:
02 00 0X 00 0Y

X-> A or C
A for right wheel, C for left wheel

Y-> 0 or 1
0 for forward, 1 for backward

Data:
02 00 0X 00 YY

X -> B or D
B for right wheel, D for left wheel

YY-> 00 to FF (0~255) speed value

Data read from STM32:
0X 00 XX 0Y 00 YY

X -> 0 or 1 for right wheel direction
XX->00 to FF (0~255) speed value for right wheel speed

Y -> 0 or 1 for left wheel direction
YY->00 to FF (0~255) speed value for left wheel speed

