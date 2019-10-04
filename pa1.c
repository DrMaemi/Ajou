/**********************************************************************
 * Copyright (c) 2019
 *  Sang-Hoon Kim <sanghoonkim@ajou.ac.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTIABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 **********************************************************************/

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <ctype.h>
#include <signal.h>
#include <errno.h>

#include "types.h"
#include "parser.h"

/*====================================================================*/
/*          ****** DO NOT MODIFY ANYTHING FROM THIS LINE ******       */
/**
 * String used as the prompt (see @main()). You may change this to
 * change the prompt */
static char __prompt[MAX_TOKEN_LEN] = "$";

/**
 * Time out value. It's OK to read this value, but ** DO NOT CHANGE
 * DIRECTLY **. Instead, use @set_timeout() function below.
 */
static unsigned int __timeout = 2;

static void set_timeout(unsigned int timeout)
{
	__timeout = timeout;

	if (__timeout == 0) {
		fprintf(stderr, "Timeout is disabled\n");
	} else {
		fprintf(stderr, "Timeout is set to %d second%s\n",
				__timeout,
				__timeout >= 2 ? "s" : "");
	}
}
/*          ****** DO NOT MODIFY ANYTHING UP TO THIS LINE ******      */
/*====================================================================*/


/***********************************************************************
 * run_command()
 *
 * DESCRIPTION
 *   Implement the specified shell features here using the parsed
 *   command tokens.
 *
 * RETURN VALUE
 *   Return 1 on successful command execution
 *   Return 0 when user inputs "exit"
 *   Return <0 on error
 */
int isStringInteger(char* s){
	size_t size = strlen(s);
	if (size == 0) return 0;

	for(int i=0; i< (int)size; i++)
		if((s[i] < 0x30) || (s[i] > 0x39))
			return 0;
	return 1;
}

int times_of_for;
pid_t pid;
bool is_master_process = true;
bool need_to_suicide = false;

char* sigterm_app;
struct sigaction newact_sigalrm, oldact_sigalrm;

void sigalrm_handler(int signo){
	int i = 0;
	do{
		if (kill(pid+i, 0) == -1) break;
		fprintf(stderr, "%s is timed out\n", sigterm_app);
		kill(pid+i, SIGKILL);
		i++;
	} while (i < times_of_for);
	
	sigaction(SIGALRM, &oldact_sigalrm, NULL);
	if (need_to_suicide) exit(0);
}


static int run_command(int nr_tokens, char *tokens[])
{
	fflush(stdin);
	need_to_suicide = true;
	pid = 0;
	
	int state_loc = 0;
	int shift = 0;
	int i=0;
	times_of_for = 0;

	newact_sigalrm.sa_handler = sigalrm_handler;
	sigaction(SIGALRM, &newact_sigalrm, &oldact_sigalrm);

	sigterm_app = tokens[shift];
	alarm(__timeout);
	
	pid = fork();
	switch (pid) {
		case -1:
			fprintf(stderr, "ERROR : fork() has been failed\n");
			return 0;
		case 0:
			alarm(0);
			is_master_process = false;
			break;
		default:
			;
	}

	if (strcmp(tokens[shift], "for") == 0){
		if (is_master_process) {
			do {
				if (!isStringInteger(tokens[shift+1])) {
					fprintf(stderr, "for : invalid argument\n");
					alarm(0); return 1;
				}
				if (atoi(tokens[shift+1]) > 1) {
					if (times_of_for == 0)
						times_of_for += atoi(tokens[shift+1]);
					else
						times_of_for *= atoi(tokens[shift+1]);
				}
				shift += 2;
			} while (strcmp(tokens[shift], "for") == 0);
		}
		else {
			shift = 0;
			do {
				if (!isStringInteger(tokens[shift+1])){
					fprintf(stderr, "for : invalid argument\n");
					return 0;
				}
				else{
					if (atoi(tokens[shift+1])>1){
						for(i=0; i<atoi(tokens[shift+1])-1; i++){
							pid=fork();
							switch(pid){
								case -1:
									fprintf(stderr, "fork() has been failed\n");
									break;
								case 0:
									continue;
								default:
									while((wait(&state_loc) == -1)&&(errno = EINTR));
							}
						break;
						}
					}
				}
			shift+=2;
			}while(strcmp(tokens[shift], "for") == 0);
		}
	}

	sigterm_app = tokens[shift];


	if (strcmp(tokens[shift], "exit") == 0) {
		return 0;
	}

	else if (strcmp(tokens[shift], "prompt") == 0){
		if (!is_master_process) return 0;

		if (nr_tokens == shift+2) {			
			strncpy(__prompt, tokens[shift+1], sizeof(tokens[shift+1]));
			__prompt[sizeof(tokens[shift+1])] = '\0';
		}
		else 
			fprintf(stderr, "prompt : invalid argument\n");
			
	}	

	else if (strcmp(tokens[shift], "timeout") == 0) {
		if (!is_master_process) return 0;

		if (tokens[shift+1])
			set_timeout(atoi(tokens[shift+1]));

		else
			fprintf(stderr, "Current timeout is %d second%s\n",
							__timeout, __timeout >= 2? "s" : "");
	}

	else if (strcmp(tokens[shift], "cd") == 0) {
		if (!is_master_process) return 0;

		if (nr_tokens == shift+1)
			chdir(getenv("HOME"));
		else if (nr_tokens == shift+2){
			if (strcmp(tokens[shift+1], "~") == 0)
				chdir(getenv("HOME"));
			else{
				int i=0;
				do{
					if (chdir(tokens[shift+1]) == -1)
						fprintf(stderr, "cd : No such file or directory\n");
					i++;
				}while(i<times_of_for);
			}
		}
		else
			fprintf(stderr, "cd : too many arguments\n");	
	}

	else{
		if (!is_master_process) {
			if (execvp(tokens[shift], tokens+shift) == -1)
				fprintf(stderr, "No such file or directory\n");
		}
	}	 

	if (!is_master_process)
		return 0;

	need_to_suicide = false;
	while ((wait(&state_loc) == -1) && (errno == EINTR));

	alarm(0);
	return 1;
}


/***********************************************************************
 * iniialize()
 *
 * DESCRIPTION
 *   Call-back function for your own initialization code. It is OK to
 *   leave blank if you don't need any initialization.
 *
 * RETURN VALUE
 *   Return 0 on successful initialization.
 *   Return other value on error, which leads the program to exit.
 */
static int initialize(int argc, char * const argv[])
{
	return 0;
}


/***********************************************************************
 * finalize()
 *
 * DESCRIPTION
 *   Callback function for finalizing your code. Like @initialize(),
 *   you may leave this function blank.
 */
static void finalize(int argc, char * const argv[])
{

}


/*====================================================================*/
/*          ****** DO NOT MODIFY ANYTHING BELOW THIS LINE ******      */

static bool __verbose = true;
static char *__color_start = "[0;31;40m";
static char *__color_end = "[0m";

/***********************************************************************
 * main() of this program.
 */
int main(int argc, char * const argv[])
{
	char command[MAX_COMMAND_LEN] = { '\0' };
	int ret = 0;
	int opt;

	while ((opt = getopt(argc, argv, "qm")) != -1) {
		switch (opt) {
		case 'q':
			__verbose = false;
			break;
		case 'm':
			__color_start = __color_end = "\0";
			break;
		}
	}

	if ((ret = initialize(argc, argv))) return EXIT_FAILURE;

	if (__verbose)
		fprintf(stderr, "%s%s%s ", __color_start, __prompt, __color_end);

	while (fgets(command, sizeof(command), stdin)) {	
		char *tokens[MAX_NR_TOKENS] = { NULL };
		int nr_tokens = 0;

		if (parse_command(command, &nr_tokens, tokens) == 0)
			goto more; /* You may use nested if-than-else, however .. */

		ret = run_command(nr_tokens, tokens);
		
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			fprintf(stderr, "Error in run_command: %d\n", ret);
		}
		
more:
		if (__verbose)
			fprintf(stderr, "%s%s%s ", __color_start, __prompt, __color_end);
	}

	finalize(argc, argv);



	return EXIT_SUCCESS;
}
