#include "bt_hc05.h"

void main (void) {
    bt_init(9600);
    bt_write_string("BT ready\r\n");
}