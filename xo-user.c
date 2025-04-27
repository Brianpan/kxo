#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>

#include "game.h"

#define XO_STATUS_FILE "/sys/module/kxo/initstate"
#define XO_DEVICE_FILE "/dev/kxo"
#define XO_DEVICE_ATTR_FILE "/sys/class/kxo/kxo/kxo_state"

char *hash_to_table(int hash)
{
    char *table = malloc(sizeof(char) * N_GRIDS);
    if (!table)
        return NULL;
    for (int i = N_GRIDS - 1; i >= 0; i--) {
        table[i] = " OX"[hash % 3];
        hash /= 3;
    }

    return table;
}

static bool status_check(void)
{
    FILE *fp = fopen(XO_STATUS_FILE, "r");
    if (!fp) {
        printf("kxo status : not loaded\n");
        return false;
    }

    char read_buf[20];
    fgets(read_buf, 20, fp);
    read_buf[strcspn(read_buf, "\n")] = 0;
    if (strcmp("live", read_buf)) {
        printf("kxo status : %s\n", read_buf);
        fclose(fp);
        return false;
    }
    fclose(fp);
    return true;
}

static struct termios orig_termios;

static void raw_mode_disable(void)
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

static void raw_mode_enable(void)
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(raw_mode_disable);
    struct termios raw = orig_termios;
    raw.c_iflag &= ~IXON;
    raw.c_lflag &= ~(ECHO | ICANON);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

static bool read_attr, end_attr;

static void print_moves(uint64_t moves)
{
    // left most 4 bits might be  number of moves
    int move_no = (moves >> 60) & 0x0F;
    printf("Total moves: %d\n", move_no);
    bool has_zero = false, has_fifteen = false;

    for (int m = 0; m < move_no; m++) {
        int step = (moves >> (m << 2)) & 0x0F;
        if (step == 0)
            has_zero = true;
        if (step == 15)
            has_fifteen = true;
        printf("%c%d", 'A' + GET_COL(step), 1 + GET_ROW(step));
        if (m != move_no - 1)
            printf(" -> ");
    }
    // special case for whether 15 is included in first 15 moves
    // if not included, print 15
    if (move_no == (N_GRIDS - 1)  && !has_fifteen) {
        printf(" -> ");
        printf("%c%c", 'A' + GET_COL(15), '1' + GET_ROW(15));
    } else {
        int step = (moves >> (move_no << 2)) & 0x0F;
        // has 2 zero means move_no is correct
        if (step == 0 && has_zero)
            return;
        int next_step = (moves >> ((move_no + 1) << 2)) & 0x0F;
        if (next_step == 0 && step == 0)
            return;
        // this case of full board case
        for (int i = move_no; i < N_GRIDS; i++) {
            int step = moves >> (i << 2) & 0x0F;
            printf(" -> ");
            printf("%c%c", 'A' + GET_COL(step), '1' + GET_ROW(step));
        }
    } 
}

static int print_game_board(int game_no, char *table)
{
    while (game_no > 0) {
        uint64_t game_moves;
        memcpy(&game_moves, table, sizeof(uint64_t));
        printf("Moves: ");
        print_moves(game_moves);
        printf("\n");
        game_no--;
        table += sizeof(uint64_t);
    }
}
static void listen_keyboard_handler(void)
{
    int attr_fd = open(XO_DEVICE_ATTR_FILE, O_RDWR);
    char input;

    if (read(STDIN_FILENO, &input, 1) == 1) {
        char buf[4096];
        printf("\033[8;1H\033[2K"); // move to row 6
        printf("\033[2K"); // clean up current line 
        switch (input) {
        case 16: /* Ctrl-P */
            read(attr_fd, buf, 4096);
            buf[0] = (buf[0] - '0') ? '0' : '1';
            read_attr ^= 1;
            write(attr_fd, buf, 6);
            if (!read_attr)
                printf("Stopping to display the chess board...\n");
            break;
        case 17: /* Ctrl-Q */
            read(attr_fd, buf, 4096);
            int game_no;
            memcpy(&game_no, buf + 12, sizeof(int));
            print_game_board(game_no, buf + 12 + sizeof(int));
            buf[4] = '1';
            read_attr = false;
            end_attr = true;
            write(attr_fd, buf, 6);
            printf("game no: %d, Stopping the kernel space tic-tac-toe game...\n", game_no);
            break;
        }
    }
    close(attr_fd);
}

static char time_buf[32];
static void draw_time()
{
    time_t now = time(NULL);
    struct tm *tm_now = localtime(&now);
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tm_now);
    printf("\x1b[0;0H");
    printf("\x1b[0m");
    printf("\x1b[1;37m");
    printf("\x1b[40m");
    printf(" %s ", time_buf);
    printf("\x1b[0m");
    printf("\x1b[0;0H");
    printf("\x1b[0m");
    printf("\x1b[1;37m");
    printf("\x1b[40m\n");
}

static void draw_board(int hash)
{
    printf("\033[2;1H\033[2K"); // move to second line
    char *table = hash_to_table(hash);
    if (!table)
        return;
    for (int i = 0; i < BOARD_SIZE; i++) {
        if (BOARD_SIZE < 10)
            printf("%2d | ", i + 1);
        else if (BOARD_SIZE >= 10 && BOARD_SIZE < 100)
            printf("%3d | ", i + 1);
        else
            printf("%4d | ", i + 1);

        for (int j = 0; j < BOARD_SIZE; j++) {
            // make background color alter between high-intensity and standard
            if ((i + j) & 1U)
                printf("\x1b[47m");
            else
                printf("\x1b[107m");

            switch (table[GET_INDEX(i, j)]) {
            case 'O':
                printf("\x1b[31m");
                printf(" ○ ");
                printf("\x1b[39m");
                break;
            case 'X':
                printf("\x1b[34m");
                printf(" × ");
                printf("\x1b[39m");
                break;
            default:
                printf("   ");
                break;
            }
            printf("\x1b[49m");
        }
        printf("\n");
    }
    if (BOARD_SIZE >= 10)
        printf("-");
    if (BOARD_SIZE >= 100)
        printf("-");
    printf("---+-");
    for (int i = 0; i < BOARD_SIZE; i++)
        printf("---");
    printf("\n");
    if (BOARD_SIZE >= 10)
        printf(" ");
    if (BOARD_SIZE >= 100)
        printf(" ");
    printf("    ");
    for (int i = 0; i < BOARD_SIZE; i++)
        printf(" %2c", 'A' + i);
    printf("\n");
}
// timer to print the time
static void alrm_handler(int signum, siginfo_t *info, void *ucontext)
{
    printf("\033[1;1H\033[2K"); /* ASCII escape code to move to first line */
    draw_time();
}

static void timer_init()
{
    struct sigaction sa = {
        .sa_handler = (void (*)(int)) alrm_handler,
        .sa_flags = SA_SIGINFO,
    };
    sigfillset(&sa.sa_mask);
    sigaction(SIGALRM, &sa, NULL);
}

static void timer_create2(unsigned int usecs)
{
    ualarm(usecs, usecs);
}

static void timer_cancel(void)
{
    ualarm(0, 0);
}

int main(int argc, char *argv[])
{
    if (!status_check())
        exit(1);

    raw_mode_enable();
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    char display_buf[DRAWBUFFER_SIZE];
    char table[sizeof(int)];

    fd_set readset;
    int device_fd = open(XO_DEVICE_FILE, O_RDONLY);
    int max_fd = device_fd > STDIN_FILENO ? device_fd : STDIN_FILENO;
    read_attr = true;
    end_attr = false;

    printf("\033[H\033[J"); /* ASCII escape code to clear the screen */

    // timer 
    timer_init();
    timer_create2(500000);

    while (!end_attr) {
        FD_ZERO(&readset);
        FD_SET(STDIN_FILENO, &readset);
        FD_SET(device_fd, &readset);

        int result = select(max_fd + 1, &readset, NULL, NULL, NULL);
        if (result < 0) {
            printf("Error with select system call\n");
            exit(1);
        }

        if (FD_ISSET(STDIN_FILENO, &readset)) {
            FD_CLR(STDIN_FILENO, &readset);
            listen_keyboard_handler();
        } else if (read_attr && FD_ISSET(device_fd, &readset)) {
            FD_CLR(device_fd, &readset);
            // print the chess board
            // printf("\033[H\033[J"); /* ASCII escape code to clear the screen */
            // read(device_fd, display_buf, DRAWBUFFER_SIZE);
            // display_buf[DRAWBUFFER_SIZE - 1] = '\0';
            read(device_fd, table, sizeof(int));
            int hash = *(int *)table;
            draw_board(hash);
        }
    }

    raw_mode_disable();
    fcntl(STDIN_FILENO, F_SETFL, flags);

    close(device_fd);

    // clean up timer
    timer_cancel();
    return 0;
}
