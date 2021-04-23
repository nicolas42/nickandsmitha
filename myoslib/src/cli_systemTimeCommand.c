/**
******************************************************************************
* @file      myoslib/src/cli_systemTimeCommand.c
* @author    Smitha Ratnam - 44425782
* @date      15032021
* @brief     Prints the elapsed system time since boot up.
*         REFERENCE: zephyr shell library api and documentation.
*****************************************************************************
*                 EXTERNAL FUNCTIONS
*****************************************************************************
* There are no external functions for this command as all variables are local
* and no special initialisation is required apart from setting the right CONFIG 
* options.
*/

#include "cli_systemTimeCommand.h"


/* @brief - Handler for the shell command to print the elapsed system time since boot up.
*  Usage - time f - To display time in hrs : mins : seconds
*          time  - To display time in seconds.         
*/
static int cmd_timer_elapsed(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
    ARG_UNUSED(argv);
        
    uint32_t time_elapsed_min = 0;
    uint32_t time_elapsed_hrs = 0; 
    uint64_t time_elapsed_ms = k_uptime_get();	
	uint32_t time_elapsed_sec = time_elapsed_ms/1000;
       

    if (argc == 1) { 

            shell_print(shell, "Elapsed time in seconds: %d", time_elapsed_sec);

    }
    else {

        if (argc == 2) {

            uint32_t rem = 0;

            uint32_t el_time_sec = 0;

            if (strcmp(argv[1],"f") == 0) {

                time_elapsed_hrs = (time_elapsed_sec/3600);

                rem = (time_elapsed_sec%3600);

                time_elapsed_min = (rem/60);

                el_time_sec = (rem%60);

                shell_print(shell, "%d:%d:%d",time_elapsed_hrs, time_elapsed_min, el_time_sec);

            }
            else {

                shell_error(shell, "Wrong option.\nUsage: time f - for time in hrs:minutes:seconds OR \ntime - for time in seconds");

            }
        }
    }

    return 0;
}

/* Register the command with the shell backend */
SHELL_CMD_REGISTER(time, NULL, "Elapsed time", cmd_timer_elapsed);


