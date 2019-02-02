#include <stdio.h>
#include <stdlib.h>

#include <iostream> // std::cout
#include <thread>   // std::thread

void dasd() {
    std::cout << "hello thread" << std::endl;
}

/*
 * ===  FUNCTION  =========================================================
 *         Name:  main
 *  Description:  program entry routine.
 * ========================================================================
 */
int main(int argc, const char *argv[])
{
    std::thread t(dasd);
    std::thread
    std::thread t(dasd);
    std::thread t(dasd);
    t.dsa();

    return EXIT_SUCCESS;
}  /* ----------  end of function main  ---------- */