#ifndef __REMOTE_CONTROL__
#define __REMOTE_CONTROL__

#include "stdint.h"
#include "main.h"

#define WAIT_W 0
#define WAIT_S 1
#define WAIT_A 2
#define WAIT_D 3
#define WAIT_Q 4
#define WAIT_E 5
#define WAIT_G 6
#define WAIT_X 7
#define WAIT_Z 8
#define WAIT_C 9
#define WAIT_B 10
#define WAIT_V 11
#define WAIT_F 12
#define WAIT_R 13
#define WAIT_CTRL 14
#define WAIT_SHIFT 15
#define WAIT_CTRL_Q 16
#define WAIT_CTRL_E 17
#define WAIT_MOUSE_Z 18
#define WAIT_PRESSED_RIGH 19
#define WAIT_SREIAL_STOP 20

#define SIZE_OF_WAIT 25 // 消抖数组大小，要大于上面那些数

typedef PACKED_STRUCT()
{
    struct
    {
        int16_t ch[5]; // 摇杆数据
        char s[2];     // 拨杆数据
    } rc;
    struct
    {
        int16_t x;         // x轴移动速度
        int16_t y;         // y轴移动速度
        int16_t z;         // z轴移动速度
        uint8_t press_l;   // 左键是否按下
        uint8_t press_r;   // 右键是否按下
        uint8_t press_mid; // 中键是否按下
    } mouse;
    struct
    {
        uint16_t v; // 键盘按键数据
    } key;
    struct
    {
        uint8_t custom_data[30]; // 自定义控制器数据
    } custom_robot;
    enum Sreial_mode_e
    {
        Sreial_RC,  // 遥控器
        Sreial_KEY, // 键鼠
    } Sreial_mode;
    int online;
}
RC_ctrl_t;

/*外部函数调用*/
void Remote_Tasks(void);
extern RC_ctrl_t RC_data;


/* ------------------------------ 键盘按键定义 ------------------------------ */
#ifndef KEY_PRESSED_OFFSET_W
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
#endif

#ifndef IF_KEY_PRESSED
#define IF_KEY_PRESSED         (  RC_data.key.v  )
#define IF_KEY_PRESSED_W       ( (RC_data.key.v & KEY_PRESSED_OFFSET_W)     != 0 )
#define IF_KEY_PRESSED_S       ( (RC_data.key.v & KEY_PRESSED_OFFSET_S)     != 0 )
#define IF_KEY_PRESSED_A       ( (RC_data.key.v & KEY_PRESSED_OFFSET_A)     != 0 )
#define IF_KEY_PRESSED_D       ( (RC_data.key.v & KEY_PRESSED_OFFSET_D)     != 0 )
#define IF_KEY_PRESSED_Q       ( (RC_data.key.v & KEY_PRESSED_OFFSET_Q)     != 0 )
#define IF_KEY_PRESSED_E       ( (RC_data.key.v & KEY_PRESSED_OFFSET_E)     != 0 )
#define IF_KEY_PRESSED_G       ( (RC_data.key.v & KEY_PRESSED_OFFSET_G)     != 0 )
#define IF_KEY_PRESSED_X       ( (RC_data.key.v & KEY_PRESSED_OFFSET_X)     != 0 )
#define IF_KEY_PRESSED_Z       ( (RC_data.key.v & KEY_PRESSED_OFFSET_Z)     != 0 )
#define IF_KEY_PRESSED_C       ( (RC_data.key.v & KEY_PRESSED_OFFSET_C)     != 0 )
#define IF_KEY_PRESSED_B       ( (RC_data.key.v & KEY_PRESSED_OFFSET_B)     != 0 )
#define IF_KEY_PRESSED_V       ( (RC_data.key.v & KEY_PRESSED_OFFSET_V)     != 0 )
#define IF_KEY_PRESSED_F       ( (RC_data.key.v & KEY_PRESSED_OFFSET_F)     != 0 )
#define IF_KEY_PRESSED_R       ( (RC_data.key.v & KEY_PRESSED_OFFSET_R)     != 0 )
#define IF_KEY_PRESSED_CTRL    ( (RC_data.key.v & KEY_PRESSED_OFFSET_CTRL)  != 0 )
#define IF_KEY_PRESSED_SHIFT   ( (RC_data.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )
#endif

#endif // !__REMOTE_CONTROL__
