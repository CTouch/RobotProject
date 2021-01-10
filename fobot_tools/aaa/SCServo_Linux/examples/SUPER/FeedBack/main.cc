#include <thread>
#include <curses.h>
#include "../../../SCServo.h"

#define CP_ERROR 1
#define CP_TITLE 2
#define CP_KEY 3

SMSBL sm;
bool stop = false;

void update_data() {
    clear();
    move(0, 0);
    for (auto i = 0; i < 6; i++)
    {
        if (sm.FeedBack(i) == -1) {
            attron(COLOR_PAIR(CP_ERROR) | A_BOLD);
            printw("Read Error!\n");
            for (auto j = 0; j < 5; j++) printw("\n");
            attroff(COLOR_PAIR(CP_ERROR) | A_BOLD);
        }
        attron(COLOR_PAIR(CP_TITLE) | A_BOLD);
        printw("Joint %d:\n", i);
        attroff(COLOR_PAIR(CP_TITLE) | A_BOLD);

        attron(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("Pos: ");
        attroff(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("%d\n", sm.ReadPos(-1));

        attron(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("Speed: ");
        attroff(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("%d\n", sm.ReadSpeed(-1));

        attron(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("Load: ");
        attroff(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("%d\n", sm.ReadLoad(-1));

        attron(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("Temper: ");
        attroff(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("%d\n", sm.ReadTemper(-1));

        attron(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("Move: ");
        attroff(COLOR_PAIR(CP_KEY) | A_DIM);
        printw("%d\n", sm.ReadMove(-1));
    }
    refresh();
}

void refresh_timer() {
    while (true) {
        update_data();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

int main(int argc, char **argv)
{
    if (!sm.begin(1000000, argc < 2 ? "/dev/ttyUSB0" : argv[1])) {
        printf("Failed to read serial\n");
        return -1;
    }

    // Init
    initscr();
    start_color();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    init_pair(CP_ERROR, COLOR_RED, COLOR_BLACK);
    init_pair(CP_TITLE, COLOR_GREEN, COLOR_BLACK);
    init_pair(CP_KEY, COLOR_CYAN, COLOR_BLACK);

    std::thread timer_thread(refresh_timer);

    getch();
    stop = true;
    timer_thread.join();

    sm.end();
    endwin();
    return 0;
}

