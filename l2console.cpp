#include "l2console.h"
#include "assert.h"
#include "radio_drv.h"
#include "cdc_drv.h"
#include "uart.h"

#include "FreeRTOS.h"
#include "task.h"
//~ #include "FreeRTOS_CLI.h"
//~ #include "vt100.h"

//~ #include <stdint.h>
//~ #include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>


#define L2CONSOLE_STACK_SIZE (2048 / sizeof(portSTACK_TYPE))
#define L2CONSOLE_PRIORITY (tskIDLE_PRIORITY + 1)

TaskHandle_t xRadioRecvTask = NULL;


#define PING_TASK_STACK_SIZE (512 / sizeof(portSTACK_TYPE))
#define PING_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

static TaskHandle_t xL2ConsoleTask;
void l2console_task(void *p);



//implement missing Newlib syscalls, linker problems
extern "C" {
    int _read(int file, char *ptr, int len) {
        configASSERT("_read() Not implemented");
        return (1);
    } int _write(int file, char *ptr, int len) {
        //~ int i;
        configASSERT("_write() Not implemented");
        return len;
    }
}

class VT100 {
  public:
    enum TextAttributes {
        ATTR_OFF = 0,
        BOLD = 1,
        USCORE = 4,
        BLINK = 5,
        REVERSE = 7,
        BOLD_OFF = 21,
        USCORE_OFF = 24,
        BLINK_OFF = 25,
        REVERSE_OFF = 27,
    };

    enum Colors {
        BLACK = 0,
        RED = 1,
        GREEN = 2,
        BROWN = 3,
        BLUE = 4,
        MAGENTA = 5,
        CYAN = 6,
        WHITE = 7,
    };
#define KEY_TAB                 '\t'    // TAB key
#define KEY_CR                  '\r'    // RETURN key
#define KEY_BACKSPACE           '\b'    // Backspace key
#define KEY_ESCAPE              0x1B    // ESCAPE (pressed twice)

#define KEY_DOWN                0x80    // Down arrow key
#define KEY_UP                  0x81    // Up arrow key
#define KEY_LEFT                0x82    // Left arrow key
#define KEY_RIGHT               0x83    // Right arrow key
#define KEY_HOME                0x84    // Home key
#define KEY_DC                  0x85    // Delete character key
#define KEY_IC                  0x86    // Ins char/toggle ins mode key
#define KEY_NPAGE               0x87    // Next-page key
#define KEY_PPAGE               0x88    // Previous-page key
#define KEY_END                 0x89    // End key
#define KEY_BTAB                0x8A    // Back tab key
#define KEY_F1                  0x8B    // Function key F1
#define KEY_F(n)                (KEY_F1+(n)-1)  // Space for additional 12 function keys
#define MAX_KEYS                ((KEY_F1 + 12) - 0x80)

    const char *function_keys[MAX_KEYS] = {
        "B",                    // KEY_DOWN                 0x80                // Down arrow key
        "A",                    // KEY_UP                   0x81                // Up arrow key
        "D",                    // KEY_LEFT                 0x82                // Left arrow key
        "C",                    // KEY_RIGHT                0x83                // Right arrow key
        "1~",                   // KEY_HOME                 0x84                // Home key
        "3~",                   // KEY_DC                   0x85                // Delete character key
        "2~",                   // KEY_IC                   0x86                // Ins char/toggle ins mode key
        "6~",                   // KEY_NPAGE                0x87                // Next-page key
        "5~",                   // KEY_PPAGE                0x88                // Previous-page key
        "4~",                   // KEY_END                  0x89                // End key
        "Z",                    // KEY_BTAB                 0x8A                // Back tab key
#if 1                           // VT400: 2017/08/04  2017/08/04 by Tamalichi
        //#if 0 // VT400:
        "11~",                  // KEY_F(1)                 0x8B                // Function key F1
        "12~",                  // KEY_F(2)                 0x8C                // Function key F2
        "13~",                  // KEY_F(3)                 0x8D                // Function key F3
        "14~",                  // KEY_F(4)                 0x8E                // Function key F4
        "15~",                  // KEY_F(5)                 0x8F                // Function key F5
#else                           // Linux console
        "[A",                   // KEY_F(1)                 0x8B                // Function key F1
        "[B",                   // KEY_F(2)                 0x8C                // Function key F2
        "[C",                   // KEY_F(3)                 0x8D                // Function key F3
        "[D",                   // KEY_F(4)                 0x8E                // Function key F4
        "[E",                   // KEY_F(5)                 0x8F                // Function key F5
#endif
        "17~",                  // KEY_F(6)                 0x90                // Function key F6
        "18~",                  // KEY_F(7)                 0x91                // Function key F7
        "19~",                  // KEY_F(8)                 0x92                // Function key F8
        "20~",                  // KEY_F(9)                 0x93                // Function key F9
        "21~",                  // KEY_F(10)                0x94                // Function key F10
        "23~",                  // KEY_F(11)                0x95                // Function key F11
        "24~"                   // KEY_F(12)                0x96                // Function key F12
    };

#define STRING_STACK_LIMIT 80

    VT100() {
        // initializes terminal to "power-on" settings
        // ESC c
        this->printf("\x1B\x63");
    }

    void ClearScreen(uint8_t param) {
        // ESC [ Ps J
        // 0    Clear screen from cursor down
        // 1    Clear screen from cursor up
        // 2    Clear entire screen 
        this->printf("\x1B[%dJ", param);
    }

    void ClearLine(uint8_t param) {
        // ESC [ Ps K
        // 0    Erase from the active position to the end of the line, inclusive (default)
        // 1    Erase from the start of the screen to the active position, inclusive
        // 2    Erase all of the line, inclusive
        this->printf("\x1B[%dK", param);
    }

    //~ vt100.Scroll(3,8,1); //removes line 3, row 4 to 8(incl) scrolled 1 up. empty line at 9
    void Scroll(uint8_t linestart, uint8_t linestop, uint8_t n) {
        this->printf("\x1B[%d;%dr", linestart, linestop);
        //~ this->printf( "\x1B""D", linestart, linestop );
        this->SetCursorPos(linestop, 0);
        //~ this->printf( "\033M" );
        //~ this->printf( "\033M" );
        for (int i = 0; i < n; i++) {
            this->printf("\033E");
        }

        this->printf("\x1B[r", linestart, linestop);
    }

    void SetAttribute(uint8_t attr) {
        // ESC [ Ps;...;Ps m
        this->printf("\x1B[%dm", attr);
    }

    void SetAttribute(uint8_t attr, uint8_t fgcolor, uint8_t bgcolor) {
        // ESC [ Ps;...;Ps m
        this->printf("\x1B[%d;%d;%dm", attr, fgcolor + 30, bgcolor + 40);
    }

    void SetCursorMode(uint8_t visible) {
        if (visible == true) {
            // ESC [ ? 25 h
            this->printf("\x1B[?25h");
        }
        else {
            // ESC [ ? 25 l
            this->printf("\x1B[?25l");
        }
    }

    void SetCursorPos(uint8_t line, uint8_t col) {
        x = col;
        y = line;
        // ESC [ Pl ; Pc H
        this->printf("\x1B[%d;%dH", line, col);
    }

    void GetCursorPos(uint8_t * line, uint8_t * col) {
        *col = x;
        *line = y;
    }

    void PutStringAt(uint8_t line, uint8_t col, const char *s) {
        this->SetCursorPos(line, col);
        DEBUG_PRINT(s);
        this->printf("%s", s);
    }

    void PutCharAt(uint8_t line, uint8_t col, uint8_t c) {
        this->SetCursorPos(line, col);
        this->printf("%c", c);
    }

    void PutHexAt(uint8_t line, uint8_t col, uint16_t n) {
        this->SetCursorPos(line, col);
        this->printf("%X", n);
    }

    void PutBoxDrawingChar(uint8_t c) {
        this->printf("\x1B(0%c\x1b(B", c);
    }

    uint8_t GetChar(void) {
        return this->getch();
    }

    void insch(uint8_t ch) {
        this->printf("\033[4h");
        this->putch(ch);
    }

    void delch(void) {
        this->printf("\033[P");
    }

    /*
     * mcurses.
     */
    /** Read a char from the serial port
     *
     * @returns The char read from the serial port
     */
//~ #define SEQ_INSERT_MODE                         PSTR("\033[4h")                 // set insert mode
//~ #define SEQ_REPLACE_MODE                        PSTR("\033[4l")                 // set replace mode

    uint8_t getch(void) {
        char buf[4];
        uint8_t ch;
        uint8_t idx;
        char printbuf[20];

        while (!cdc_read(&ch, 1, portMAX_DELAY)) 
            ;

        if (ch == 0x7F)         // BACKSPACE on VT200 sends DEL char
        {
            ch = KEY_BACKSPACE; // map it to '\b'
        }
        else if (ch == '\033')  // ESCAPE
        {
            while (!cdc_read(&ch, 1, portMAX_DELAY));

            if (ch == '\033')   // 2 x ESCAPE
            {
                return KEY_ESCAPE;
            }
            else if (ch == '[') {
                for (idx = 0; idx < 3; idx++) {
                    while (!cdc_read(&ch, 1, portMAX_DELAY))
                        ;
                    buf[idx] = ch;

                    if ((ch >= 'A' && ch <= 'Z') || ch == '~') {
                        idx++;
                        break;
                    }
                }

                buf[idx] = '\0';

                for (idx = 0; idx < MAX_KEYS; idx++) {
                    if (!strcmp(buf, function_keys[idx])) {
                        ch = idx + 0x80;
                        break;
                    }
                }

                if (idx == MAX_KEYS) {
                    ch = -1;
                }
            }
            else {
                ch = -1;
            }
        }
        sprintf(printbuf, "[%02X]", ch);
        DEBUG_PRINT(printbuf);

        return ch;
    }

    /** Write a char to the serial port
     *
     * @param c The char to write
     *
     * @returns The written char or -1 if an error occured
     */
    int putch(int c) {
        if (!cdc_write((uint8_t *) & c, 1))
            return -1;
        return 1;
    }

    /** Write a string to the serial port
     *
     * @param str The string to write
     *
     * @returns 0 if the write succeeds, EOF for error
     */
    int puts(const char *str) {
        while (*str)
            putch(*str++);
        return 0;
    }

    void printchar(char **str, int c)
    {
        //extern int putchar(int c);
        
        if (str) {
            **str = (char)c;
            ++(*str);
        }
        else
        { 
            (void)putch(c);
        }
    }

    #define PAD_RIGHT 1
    #define PAD_ZERO 2

    int prints(char **out, const char *string, int width, int pad)
    {
        register int pc = 0, padchar = ' ';

        if (width > 0) {
            register int len = 0;
            register const char *ptr;
            for (ptr = string; *ptr; ++ptr) ++len;
            if (len >= width) width = 0;
            else width -= len;
            if (pad & PAD_ZERO) padchar = '0';
        }
        if (!(pad & PAD_RIGHT)) {
            for ( ; width > 0; --width) {
                printchar (out, padchar);
                ++pc;
            }
        }
        for ( ; *string ; ++string) {
            printchar (out, *string);
            ++pc;
        }
        for ( ; width > 0; --width) {
            printchar (out, padchar);
            ++pc;
        }

        return pc;
    }

    /* the following should be enough for 32 bit int */
    #define PRINT_BUF_LEN 12

    int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
    {
        char print_buf[PRINT_BUF_LEN];
        register char *s;
        register int t, neg = 0, pc = 0;
        register unsigned int u = (unsigned int)i;

        if (i == 0) {
            print_buf[0] = '0';
            print_buf[1] = '\0';
            return prints (out, print_buf, width, pad);
        }

        if (sg && b == 10 && i < 0) {
            neg = 1;
            u = (unsigned int)-i;
        }

        s = print_buf + PRINT_BUF_LEN-1;
        *s = '\0';

        while (u) {
            t = (unsigned int)u % b;
            if( t >= 10 )
                t += letbase - '0' - 10;
            *--s = (char)(t + '0');
            u /= b;
        }

        if (neg) {
            if( width && (pad & PAD_ZERO) ) {
                printchar (out, '-');
                ++pc;
                --width;
            }
            else {
                *--s = '-';
            }
        }

        return pc + prints (out, s, width, pad);
    }

    int print( char **out, const char *format, va_list args )
    {
        register int width, pad;
        register int pc = 0;
        char scr[2];

        for (; *format != 0; ++format) {
            if (*format == '%') {
                ++format;
                width = pad = 0;
                if (*format == '\0') break;
                if (*format == '%') goto out;
                if (*format == '-') {
                    ++format;
                    pad = PAD_RIGHT;
                }
                while (*format == '0') {
                    ++format;
                    pad |= PAD_ZERO;
                }
                for ( ; *format >= '0' && *format <= '9'; ++format) {
                    width *= 10;
                    width += *format - '0';
                }
                if( *format == 's' ) {
                    register char *s = (char *)va_arg( args, int );
                    pc += prints (out, s?s:"(null)", width, pad);
                    continue;
                }
                if( *format == 'd' ) {
                    pc += printi (out, va_arg( args, int ), 10, 1, width, pad, 'a');
                    continue;
                }
                if( *format == 'x' ) {
                    pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'a');
                    continue;
                }
                if( *format == 'X' ) {
                    pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'A');
                    continue;
                }
                if( *format == 'u' ) {
                    pc += printi (out, va_arg( args, int ), 10, 0, width, pad, 'a');
                    continue;
                }
                if( *format == 'c' ) {
                    /* char are converted to int then pushed on the stack */
                    scr[0] = (char)va_arg( args, int );
                    scr[1] = '\0';
                    pc += prints (out, scr, width, pad);
                    continue;
                }
            }
            else {
            out:
                printchar (out, *format);
                ++pc;
            }
        }
        if (out) **out = '\0';
        va_end( args );
        return pc;
    }

    int printf(const char *format, ...)
    {
        va_list args;
        
        va_start( args, format );
        return print( 0, format, args );
    }

    int sprintf(char *out, const char *format, ...)
    {
        va_list args;
        
        va_start( args, format );
        return print( &out, format, args );
    }


    //~ int snprintf( char *buf, unsigned int count, const char *format, ... )
    //~ {
        //~ va_list args;
        
        //~ ( void ) count;
        
        //~ va_start( args, format );
        //~ return print( &buf, format, args );
    //~ }

  private:
    char temp[STRING_STACK_LIMIT];
    int x = 0, y = 0;
};

typedef enum {
    MSG_TYPE_SENT = 0,
    MSG_TYPE_NOTIFY,
    MSG_TYPE_COMMAND,
    MSG_TYPE_RECV,
    MSG_TYPE_ERROR
} msg_t;

#define MAX_PACKET_SIZE 40

extern "C" { void recv_task_wrapper(void*); };
extern "C" { void ping_task_wrapper(void*); };
    
class mainWin {
    #define MAX_COLS 80
    #define RECV_TASK_STACK_SIZE (1024 / sizeof(portSTACK_TYPE))
    #define RECV_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

  public:
    mainWin(int rows, int columns) {
        this->rows = rows;
        this->columns = columns;
        //~ this->echo = false;
    } 
    
    void init(void) {
        if (xTaskCreate
            (recv_task_wrapper, "Recv", RECV_TASK_STACK_SIZE, (void *) this, RECV_TASK_PRIORITY, &xRecvTask) != pdPASS)
            //~ ((void (*)(void*))&mainWin::recv_task, "Recv", RECV_TASK_STACK_SIZE, (void *) this, RECV_TASK_PRIORITY, &xRecvTask) != pdPASS)
            configASSERT(false);
        if (xTaskCreate
            (ping_task_wrapper, "Ping", PING_TASK_STACK_SIZE, (void *) this, PING_TASK_PRIORITY, &xPingTask) != pdPASS)
            //~ ((void (*)(void*))&mainWin::recv_task, "Recv", RECV_TASK_STACK_SIZE, (void *) this, RECV_TASK_PRIORITY, &xRecvTask) != pdPASS)
            configASSERT(false);
        vTaskSuspend(xPingTask);
    }

    void printgrid(void) {
        vt100.ClearScreen(2);
        for (int i = 1; i < rows; i++) {
            //~ if(i%2)
            //~ vt100.SetAttribute(VT100::REVERSE);
            //~ else
            //~ vt100.SetAttribute(VT100::ATTR_OFF);
            for (int j = 1; j < columns; j++)
                vt100.PutCharAt(i, j, '0' + (((i - 1 % 10) + (j - 1 % 10)) % 10));
        }
        vt100.SetCursorPos(10, 10);
        vt100.Scroll(8, 12, 4);
    }

    void add_message_row(char *msg, msg_t type) {
        uint8_t restorex, restorey;
        int column = 0;
        DEBUG_PRINT("add row");
        if (strlen(msg) > 42)
            msg[42] = 0;
        int msg_rows = rows - 2;
        if (msg_idx > msg_rows) {
            msg_idx = msg_rows;
            vt100.Scroll(1, msg_rows, 1);
            //~ DEBUG_PRINT("scroll");
        }
        //~ vt100.SetCursorPos(msg_idx, 0);
        switch(type) {
            case MSG_TYPE_SENT:
                column = columns - strlen(msg); //align right
                vt100.SetAttribute(VT100::ATTR_OFF, VT100::CYAN, VT100::BLACK);
                break;
            case MSG_TYPE_COMMAND:
                vt100.SetAttribute(VT100::ATTR_OFF, VT100::GREEN, VT100::BLACK);
                break;
            case MSG_TYPE_NOTIFY:
                vt100.SetAttribute(VT100::ATTR_OFF, VT100::BLUE, VT100::BLACK);
                break;
            case MSG_TYPE_RECV:
                vt100.SetAttribute(VT100::ATTR_OFF, VT100::MAGENTA, VT100::BLACK);
            case MSG_TYPE_ERROR:
                vt100.SetAttribute(VT100::ATTR_OFF, VT100::RED, VT100::BLACK);
                break;
            default:
                vt100.SetAttribute(VT100::ATTR_OFF, VT100::WHITE, VT100::BLACK);
        }
        vt100.GetCursorPos(&restorey, &restorex);
        vt100.PutStringAt(msg_idx, column, msg);
        vt100.SetCursorPos(restorey, restorex);
        msg_idx++;
    }

    void print_status(void) {
        uint8_t restorex, restorey;
        int status_row = rows - 1;

        vt100.GetCursorPos(&restorey, &restorex);
        vt100.SetAttribute(VT100::ATTR_OFF, VT100::WHITE, VT100::BLUE);
        vt100.SetCursorPos(status_row, 10);
        vt100.printf("Status Line");
        //~ radio_status(status_buffer);
        //~ vt100.printf(status_buffer);
        vt100.SetAttribute(VT100::ATTR_OFF, VT100::WHITE, VT100::BLACK);
        vt100.SetCursorPos(restorey, restorex);
    }

    uint16_t readline(char *str, int line, int column, int len) {
        uint8_t ch;
        uint8_t curlen = 0;
        uint8_t curpos = 0;
        uint8_t starty = line;
        uint8_t startx = column;
        uint8_t maxlen = len;
        uint8_t i;
        uint8_t init = true;

        vt100.SetCursorPos(line, column + 1);
        vt100.ClearLine(0);
        vt100.SetAttribute(VT100::ATTR_OFF, VT100::WHITE, VT100::BLACK);

        maxlen--;               // reserve one byte in order to store '\0' in last position

        while ((ch = vt100.getch()) != KEY_CR) {
            if (init) {
                if (ch == KEY_UP) {     //one line history
                    curlen = strlen(str);
                    curpos = strlen(str);
                    vt100.SetCursorPos(starty, startx + 1);
                    vt100.printf(str);
                }
                else
                    memset(str, 0, maxlen);
                init = false;
            }
            switch (ch) {
            case KEY_LEFT:
                if (curpos > 0) {
                    curpos--;
                }
                break;
            case KEY_RIGHT:
                if (curpos < curlen) {
                    curpos++;
                }
                break;
            case KEY_HOME:
                curpos = 0;
                break;
            case KEY_END:
                curpos = curlen;
                break;
            case KEY_BACKSPACE:
                if (curpos > 0) {
                    curpos--;
                    curlen--;
                    vt100.SetCursorPos(starty, startx + curpos + 1);

                    for (i = curpos; i < curlen; i++) {
                        str[i] = str[i + 1];
                    }
                    str[i] = '\0';
                    vt100.delch();
                }
                break;

            case KEY_DC:
                if (curlen > 0) {
                    curlen--;
                    for (i = curpos; i < curlen; i++) {
                        str[i] = str[i + 1];
                    }
                    str[i] = '\0';
                    vt100.delch();
                }
                break;

            default:
                //                if (curlen < maxlen && (ch & 0x7F) >= 32 && (ch & 0x7F) < 127)      // printable ascii 7bit or printable 8bit ISO8859
                if (curlen < maxlen && (ch & 0x7F) >= 32)       // 2017/08/04  2017/08/04 by Tamalichi
                {
                    for (i = curlen; i > curpos; i--) {
                        str[i] = str[i - 1];
                    }
                    vt100.insch(ch);
                    str[curpos] = ch;
                    curpos++;   //TODO: overflow
                    curlen++;
                }
            }
            vt100.SetCursorPos(starty, startx + curpos + 1);
        }
        str[curlen] = '\0';
        return curlen;
    }
    
    bool cli_command(char * command, char * outbuf) {
        char *arg = command; //TODO bounds
        while(!isspace(*arg++))
            ;
        if(!strncmp("/echo", command, command-arg)){
            if(echo) {
                echo=false;
                add_message_row((char *)"echo OFF", MSG_TYPE_NOTIFY);
            }
            else {
                echo=true;
                add_message_row((char *)"echo ON", MSG_TYPE_NOTIFY);
            }
        }
        else if(!strncmp("/ping", command, command-arg)){
            if(eTaskGetState(xPingTask) == eSuspended) {
                vTaskResume(xPingTask);
                add_message_row((char *)"ping ON", MSG_TYPE_NOTIFY);
            }
            else {
                vTaskSuspend(xPingTask);
                add_message_row((char *)"ping OFF", MSG_TYPE_NOTIFY);
            }
        }
        else if(!strncmp("/status", command, command-arg)){
            print_status();
        }
        else {
            add_message_row((char *)"command not found", MSG_TYPE_ERROR);
        }
        return true;
    }
    
    void cli(void) {
        //~ portBASE_TYPE xReturned;
        char outbuf[MAX_COLS];
        int cli_row = rows;
        vt100.SetCursorPos(cli_row, 0);
        vt100.printf("CLI>");
        add_message_row((char *)"static hello welcome", MSG_TYPE_NOTIFY);

        while (1) {
            readline(cli_buf, cli_row, 4, columns - 4);
            if (strlen(cli_buf)) {
                if (cli_buf[0] == '/') {
                    //~ add_message_row(outbuf, MSG_TYPE_COMMAND);
                    cli_command(cli_buf, outbuf);
                }
                else {
                    add_message_row(cli_buf, MSG_TYPE_SENT);
                    if (strlen(cli_buf) > MAX_PACKET_SIZE)
                        radio_write((uint8_t *) cli_buf, MAX_PACKET_SIZE);
                    else
                        radio_write((uint8_t *) cli_buf, strlen(cli_buf));
                }
            }
        }
    }

    void recv_task(void *p) {
        int len;
        mainWin *thetask = (mainWin *) p;
        uint8_t buffer[100];
        int16_t rssi;
        int8_t snr;

        while (1) {
            if ((len = radio_read(buffer, 100, &rssi, &snr, portMAX_DELAY))) {
                DEBUG_PRINT("recv...");
                //~ if (strlen((char *) buffer)) {
                    //~ buffer[len] = 0;    //terminate
                    //~ add_message_row((char *) buffer, MSG_TYPE_RECV);
                    //~ if(thetask->echo)
                        //~ radio_write(buffer, len);
                //~ }
            }
            else
                vTaskDelay(10); //CHECKME: why? driver bug?
        }
    }

    void ping_task(void *p)
    {
        DEBUG_PRINT("ping...");
        while (1) {
            if (radio_write((uint8_t *) "PING", 4)) {
                DEBUG_PRINT("ping sent");
            }
            else {
                DEBUG_PRINT("ping fail");
            }
            vTaskDelay(300);
        }
    }

    void run(void) {
        vt100.ClearScreen(2);
        //~ print_status();
        cli();
    }

  private:
    TaskHandle_t xRecvTask = NULL, xPingTask = NULL; 
    char cli_buf[MAX_COLS];
    char status_buffer[MAX_COLS];

    int rows, columns;
    int msg_idx = 1;
    VT100 vt100;
    uint8_t echo = false;
};

extern "C" void recv_task_wrapper(void* parm) {

    (static_cast<mainWin*>(parm))->recv_task(parm);

}

extern "C" void ping_task_wrapper(void* parm) {

    (static_cast<mainWin*>(parm))->ping_task(parm);

}

uint16_t readstring(uint8_t * outbuf, uint16_t maxlen)
{
    int cmd_idx = 0;

    while (1) {
        cdc_read(&(outbuf[cmd_idx]), 1, portMAX_DELAY);
        cdc_write(&(outbuf[cmd_idx]), 1);
        if ((outbuf[cmd_idx] == '\r') || (outbuf[cmd_idx] == '\n')) {   //CR
            outbuf[cmd_idx] = '\0';     //string termination
            cmd_idx = 0;
            return cmd_idx;
        }
        cmd_idx++;              //TODO: overflow
    }

}

void writestring(char *str)
{
    int len = strlen(str);
    cdc_write((uint8_t *) str, len);

}

void start_l2console(void)
{
    if (xTaskCreate
        (l2console_task, "L2Console", L2CONSOLE_STACK_SIZE, NULL, L2CONSOLE_PRIORITY, &xL2ConsoleTask) != pdPASS)
        configASSERT(0);
}

void l2console_task(void *p)
{
    (void) p;
    int columns = 42, rows = 18;

    cdc_init();
    radio_init();

    cdc_wait_CDC();
    cdc_wait_DTR();
    
mainWin win(rows, columns);

    vTaskDelay(300);
    //~ vRegisterCLICommands();

    win.init();

    //~ win.printgrid();
    win.run();
}
