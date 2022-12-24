#include<stdio.h>
#include "./candata.c"
#include "./can_wrap.h"

int main()
{
    const char canType[]="vcan0";
    const bool fd = true;

    const int canConnection= can_connect(canType,fd);

    printf("%d",canConnection);
    return 0;
}