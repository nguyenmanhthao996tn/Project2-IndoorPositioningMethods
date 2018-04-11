#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char *argv[])
{
    struct timespec time;
    char *time_string;

    while (1)
    {    
        if (clock_gettime(CLOCK_REALTIME, &time) == -1)
        {
            printf("Get time ERROR!");
            return 1;
        }

        time_string = ctime(&(time.tv_sec));

        printf("%s\n", time_string); 
        // free(time_string);
    }

    return 0;
}
