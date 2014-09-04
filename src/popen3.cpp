#include "popen3.h"

#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define NUM_PIPES 3

#define IN_PIPE 0
#define OUT_PIPE 1
#define ERR_PIPE 2

#define READ_FD 0
#define WRITE_FD 1

#define CLOSEALL(p) \
        do { \
            int i; \
            for (i = 0; i < NUM_PIPES; ++i) { \
                close(p[i][READ_FD]); \
                close(p[i][WRITE_FD]); \
            } \
        } while (0)

process_t popen3 (const char* cmd) {
    int pipes[NUM_PIPES][2] = { { 0 } };
    process_t proc = { -1, NULL, NULL, NULL };

    /* Lay some pipe. */
    if (-1 == pipe(pipes[IN_PIPE]) ||
        -1 == pipe(pipes[OUT_PIPE]) ||
        -1 == pipe(pipes[ERR_PIPE])) {
        CLOSEALL(pipes);
        return proc;
    }

    proc.pid = fork();

    if (-1 == proc.pid) {
        CLOSEALL(pipes);
        return proc;
    }
    else if (0 == proc.pid) {
        /* CHILD PROCESS */
        dup2(pipes[IN_PIPE][READ_FD], STDIN_FILENO);
        dup2(pipes[OUT_PIPE][WRITE_FD], STDOUT_FILENO);
        dup2(pipes[ERR_PIPE][WRITE_FD], STDERR_FILENO);

        CLOSEALL(pipes);

        execl("/bin/sh", "/bin/sh", "-c", cmd, NULL);
        /* Never reached. */
    }

    /* Close the file descriptors that the child will be duping. */
    close(pipes[IN_PIPE][READ_FD]);
    close(pipes[OUT_PIPE][WRITE_FD]);
    close(pipes[ERR_PIPE][WRITE_FD]);

    /* Create FILE streams for our client to interact with. */
    proc.in = fdopen(pipes[IN_PIPE][WRITE_FD], "w");
    proc.out = fdopen(pipes[OUT_PIPE][READ_FD], "r");
    proc.err = fdopen(pipes[ERR_PIPE][READ_FD], "r");

    /* Until something better can be thunk up. Kill process, return -1? */
    assert(proc.in && proc.out && proc.err);

    return proc;
}

int pclose3 (process_t proc) {
    fclose(proc.in);
    fclose(proc.out);
    fclose(proc.err);

    proc.in = NULL;
    proc.out = NULL;
    proc.err = NULL;

    int status;
    pid_t child = waitpid(proc.pid, &status, 0);

    if (-1 == child) {
        return -1;
    }
    else {
        assert(proc.pid == child);
    }

    proc.pid = -1;

    return WEXITSTATUS(status);
}
