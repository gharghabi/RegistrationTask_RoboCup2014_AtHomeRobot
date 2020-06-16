#include "ros/ros.h"
#include <iostream>
#include <stdio.h>

#define LINE_BUFSIZE 128

int main(int argc, char **argv)
{
	std::cout<<"Salam"<<std::endl;
	 char line[LINE_BUFSIZE];
    int linenr;
    FILE *pipe;
    
    /* Get a pipe where the output from the scripts comes in */
    pipe = popen("/home/bkhezry/catkin_ws/src/testPack/src/bash/sendFiles.sh", "r");
    if (pipe == NULL) {  /* check for errors */
        perror(argv[0]); /* report error message */
        return 1;        /* return with exit code indicating error */
    }

    /* Read script output from the pipe line by line */
    linenr = 1;
    while (fgets(line, LINE_BUFSIZE, pipe) != NULL) {
        printf("Script output line %d: %s", linenr, line);
        ++linenr;
    }
    
    /* Once here, out of the loop, the script has ended. */
    pclose(pipe); /* Close the pipe */
   
	return 0;
}

