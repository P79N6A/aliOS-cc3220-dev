/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "cli_conf.h"
#include "cli_api.h"
#include "cli_adapt.h"

#include "k_config.h"
#if (RHINO_CONFIG_USER_SPACE > 0)
#include "uapp.h"
#endif

#define RET_CHAR '\n'
#define END_CHAR '\r'
#define PROMPT   "# "
#define EXIT_MSG "exit"

#if (CLI_MINIMUM_MODE > 0)
#undef CLI_INBUF_SIZE
#define CLI_INBUF_SIZE 128

#undef CLI_OUTBUF_SIZE
#define CLI_OUTBUF_SIZE 32

#undef CLI_MAX_COMMANDS
#define CLI_MAX_COMMANDS 64

#undef CLI_MAX_ARG_NUM
#define CLI_MAX_ARG_NUM 8

#undef CLI_MAX_ONCECMD_NUM
#define CLI_MAX_ONCECMD_NUM 1
#endif

struct cli_status {
    int32_t  inited;
    uint32_t num;
    int32_t  echo_disabled;
    uint32_t bp; /* Buffer pointer */

    char  inbuf[CLI_INBUF_SIZE];
    char *outbuf;

    const struct cli_command_st *cmds[CLI_MAX_COMMANDS];

#if (RHINO_CONFIG_USER_SPACE > 0)
    /* user cli: pid = 1,2,3..*/
    const struct cli_command_st *u_cmds[MAX_APP_BINS + 1][CLI_MAX_COMMANDS];
    uint32_t     u_num[MAX_APP_BINS + 1];
    kqueue_t    *u_cli_queue[MAX_APP_BINS + 1];
#endif

#if (CLI_MINIMUM_MODE <= 0)
    int32_t his_idx;
    int32_t his_cur;
    char    history[CLI_INBUF_SIZE];
#endif
};

extern int32_t cli_register_default_commands(void);

static struct cli_status *g_cli = NULL;

static int32_t volatile g_cli_exit = 0;

static char    g_cli_tag[64] = {0};
static uint8_t g_cli_tag_len =  0;

#if (RHINO_CONFIG_USER_SPACE > 0)
static uint32_t lookup_command_pid(const char *name)
{
    uint32_t pid_i = 1, j = 0, user_cmd_num = 0;

    while (pid_i < (MAX_APP_BINS + 1)) {

        user_cmd_num = g_cli->u_num[pid_i];

        for (j = 0; j < user_cmd_num; j++) {
            if (!strcmp(g_cli->u_cmds[pid_i][j]->name, name))
                return pid_i;
        }

        pid_i++;
    }
    return 0;
}
#endif

static const struct cli_command_st *lookup_command(char *name, int len)
{
    int i = 0;
    int n = 0;

    while (i < CLI_MAX_COMMANDS && n < g_cli->num) {
        if (g_cli->cmds[i]->name == NULL) {
            i++;
            continue;
        }
        /* See if partial or full match is expected */
        if (len != 0) {
            if (!strncmp(g_cli->cmds[i]->name, name, len)) {
                return g_cli->cmds[i];
            }
        } else {
            if (!strcmp(g_cli->cmds[i]->name, name)) {
                return g_cli->cmds[i];
            }
        }

        i++;
        n++;
    }

    return NULL;
}

static int32_t proc_onecmd(int argc, char *argv[])
{
    int32_t i = 0;
    uint8_t tmp = 0;

    const char *p = NULL;

    const struct cli_command_st *command = NULL;

#if (RHINO_CONFIG_USER_SPACE > 0)
    uint32_t  pid;
    kqueue_t *cli_q;
#endif

    if (argc < 1) {
        return 0;
    }

    if (!g_cli->echo_disabled) {
        tmp = g_cli_tag_len;
        g_cli_tag_len = 0;
        cli_printf("\r\n");

        g_cli_tag_len = tmp;
    }

    /*
     * Some comamands can allow extensions like foo.a, foo.b and hence
     * compare commands before first dot.
     */
    i = ((p = strchr(argv[0], '.')) == NULL) ? 0 : (p - argv[0]);

    command = lookup_command(argv[0], i);
    if (command == NULL) {
        return 1;
    }

#if (RHINO_CONFIG_USER_SPACE > 0)
    pid = lookup_command_pid(command->name);

    if ((pid > 0) && (pid < (MAX_APP_BINS + 1))) {
        cli_q = g_cli->u_cli_queue[pid];
        krhino_queue_back_send(cli_q, (void*)command);
        return 0;
    }
#endif

    g_cli->outbuf = cli_malloc(CLI_OUTBUF_SIZE);
    if (NULL == g_cli->outbuf) {
        cli_printf("Error! cli alloc mem fail!\r\n");
        return 1;
    }
    memset(g_cli->outbuf, 0, CLI_OUTBUF_SIZE);

    command->function(g_cli->outbuf, CLI_OUTBUF_SIZE, argc, argv);
    cli_printf("%s", g_cli->outbuf);

    cli_free(g_cli->outbuf);
    g_cli->outbuf = NULL;
    return 0;
}

static int32_t cli_handle_input(char *inbuf)
{
    struct
    {
        unsigned inArg : 1;
        unsigned inQuote : 1;
        unsigned done : 1;
    } stat;
    static char *argvall[CLI_MAX_ONCECMD_NUM][CLI_MAX_ARG_NUM];
    int32_t      argcall[CLI_MAX_ONCECMD_NUM] = { 0 };

    int32_t  cmdnum = 0;
    int32_t *pargc  = &argcall[0];
    int32_t  i      = 0;
    int32_t  ret    = 0;

    memset((void *)&argvall, 0, sizeof(argvall));
    memset((void *)&argcall, 0, sizeof(argcall));
    memset(&stat, 0, sizeof(stat));

    do {
        switch (inbuf[i]) {
            case '\0':
                if (stat.inQuote) {
                    return 2;
                }
                stat.done = 1;
                break;

            case '"':
                if (i > 0 && inbuf[i - 1] == '\\' && stat.inArg) {
                    memcpy(&inbuf[i - 1], &inbuf[i], strlen(&inbuf[i]) + 1);
                    --i;
                    break;
                }
                if (!stat.inQuote && stat.inArg) {
                    break;
                }
                if (stat.inQuote && !stat.inArg) {
                    return 2;
                }

                if (!stat.inQuote && !stat.inArg) {
                    stat.inArg   = 1;
                    stat.inQuote = 1;
                    (*pargc)++;
                    argvall[cmdnum][(*pargc) - 1] = &inbuf[i + 1];
                } else if (stat.inQuote && stat.inArg) {
                    stat.inArg   = 0;
                    stat.inQuote = 0;
                    inbuf[i]     = '\0';
                }
                break;

            case ' ':
                if (i > 0 && inbuf[i - 1] == '\\' && stat.inArg) {
                    memcpy(&inbuf[i - 1], &inbuf[i], strlen(&inbuf[i]) + 1);
                    --i;
                    break;
                }
                if (!stat.inQuote && stat.inArg) {
                    stat.inArg = 0;
                    inbuf[i]   = '\0';
                }
                break;

            case ';':
                if (i > 0 && inbuf[i - 1] == '\\' && stat.inArg) {
                    memcpy(&inbuf[i - 1], &inbuf[i], strlen(&inbuf[i]) + 1);
                    --i;
                    break;
                }
                if (stat.inQuote) {
                    return 2;
                }
                if (!stat.inQuote && stat.inArg) {
                    stat.inArg = 0;
                    inbuf[i]   = '\0';

                    if (*pargc) {
                        if (++cmdnum < CLI_MAX_ONCECMD_NUM) {
                            pargc = &argcall[cmdnum];
                        }
                    }
                }

                break;

            default:
                if (!stat.inArg) {
                    stat.inArg = 1;
                    (*pargc)++;
                    argvall[cmdnum][(*pargc) - 1] = &inbuf[i];
                }
                break;
        }
    } while (!stat.done && ++i < CLI_INBUF_SIZE && cmdnum < CLI_MAX_ONCECMD_NUM &&
             (*pargc) < CLI_MAX_ARG_NUM);

    if (stat.inQuote) {
        return 2;
    }

    for (i = 0; i <= cmdnum && i < CLI_MAX_ONCECMD_NUM; i++) {
        ret |= proc_onecmd(argcall[i], argvall[i]);
    }

    return ret;
}

/**
 * @brief Perform basic tab-completion on the input buffer
 *
 * @param[in] inbuf the input buffer
 * @param[in] bp    the current buffer pointer
 *
 * @return none
 *
 */
static void cli_tab_complete(char *inbuf, unsigned int *bp)
{
    int32_t i, n, m;

    const char *fm = NULL;

    i = n = m = 0;

    cli_printf("\r\n");

    /* show matching commands */
    for (i = 0; i < CLI_MAX_COMMANDS && n < g_cli->num; i++) {
        if (g_cli->cmds[i]->name != NULL) {
            if (!strncmp(inbuf, g_cli->cmds[i]->name, *bp)) {
                m++;
                if (m == 1) {
                    fm = g_cli->cmds[i]->name;
                } else if (m == 2)
                    cli_printf("%s %s ", fm, g_cli->cmds[i]->name);
                else
                    cli_printf("%s ", g_cli->cmds[i]->name);
            }
            n++;
        }
    }

    /* there's only one match, so complete the line */
    if (m == 1 && fm) {
        n = strlen(fm) - *bp;
        if (*bp + n < CLI_INBUF_SIZE) {
            memcpy(inbuf + *bp, fm + *bp, n);
            *bp += n;
            inbuf[(*bp)++] = ' ';
            inbuf[*bp]     = '\0';
        }
    }
    if (m >= 2) {
        cli_printf("\r\n");
    }

    /* just redraw input line */
    cli_printf("%s%s", PROMPT, inbuf);
}

#if (CLI_MINIMUM_MODE <= 0)

static void cli_history_input(void)
{
    char    *inbuf    = g_cli->inbuf;
    int32_t  charnum  = strlen(g_cli->inbuf) + 1;
    int32_t  his_cur  = g_cli->his_cur;
    int32_t  left_num = CLI_INBUF_SIZE - his_cur;

    char    lastchar;
    int32_t tmp_idx;

    g_cli->his_idx = his_cur;

    if (left_num >= charnum) {
        tmp_idx  = his_cur + charnum - 1;
        lastchar = g_cli->history[tmp_idx];
        strncpy(&(g_cli->history[his_cur]), inbuf, charnum);

    } else {
        tmp_idx  = (his_cur + charnum - 1) % CLI_INBUF_SIZE;
        lastchar = g_cli->history[tmp_idx];

        strncpy(&(g_cli->history[his_cur]), inbuf, left_num);
        strncpy(&(g_cli->history[0]), inbuf + left_num, charnum - left_num);
    }
    tmp_idx = (tmp_idx + 1) % CLI_INBUF_SIZE;

    g_cli->his_cur = tmp_idx;

    /*overwrite*/
    if ('\0' != lastchar) {

        while (g_cli->history[tmp_idx] != '\0') {
            g_cli->history[tmp_idx] = '\0';

            tmp_idx = (tmp_idx + 1) % CLI_INBUF_SIZE;
        }
    }
}

static void cli_up_history(char *inaddr)
{
    int index;
    int lastindex = 0;

    lastindex = g_cli->his_idx;
    index     = (g_cli->his_idx - 1 + CLI_INBUF_SIZE) % CLI_INBUF_SIZE;

    while ((g_cli->history[index] == '\0') && (index != g_cli->his_idx)) {
        index = (index - 1 + CLI_INBUF_SIZE) % CLI_INBUF_SIZE;
    }
    if (index != g_cli->his_idx) {
        while (g_cli->history[index] != '\0') {
            index = (index - 1 + CLI_INBUF_SIZE) % CLI_INBUF_SIZE;
        }
        index = (index + 1) % CLI_INBUF_SIZE;
    }
    g_cli->his_idx = index;

    while (g_cli->history[lastindex] != '\0') {

        *inaddr++ = g_cli->history[lastindex];
        lastindex = (lastindex + 1) % CLI_INBUF_SIZE;
    }
    *inaddr = '\0';

    return;
}

static void cli_down_history(char *inaddr)
{
    int index;
    int lastindex = 0;

    lastindex = g_cli->his_idx;
    index     = g_cli->his_idx;

    while ((g_cli->history[index] != '\0')) {
        index = (index + 1) % CLI_INBUF_SIZE;
    }
    if (index != g_cli->his_idx) {
        while (g_cli->history[index] == '\0') {
            index = (index + 1) % CLI_INBUF_SIZE;
        }
    }
    g_cli->his_idx = index;

    while (g_cli->history[lastindex] != '\0') {
        *inaddr++ = g_cli->history[lastindex];
        lastindex = (lastindex + 1) % CLI_INBUF_SIZE;
    }

    *inaddr = '\0';

    return;
}

#endif

/**
 * @brief Get an input line
 *
 * @param[in/out] inbuf poiner to the input buffer
 * @param[out]    bp    the current buffer pointer
 *
 * @return 1 if there is input, 0 if the line should be ignored
 *
 */
static int32_t cli_get_input(char *inbuf, uint32_t *bp)
{
    char c;
    int32_t esc  =  0;
    int32_t key1 = -1;
    int32_t key2 = -1;
    uint8_t tmp  =  0;

    if (inbuf == NULL) {
        cli_printf("input null\r\n");
        return 0;
    }

    while (cli_getchar(&c) == 1) {
        if (c == RET_CHAR || c == END_CHAR) { /* end of input line */
            inbuf[*bp] = '\0';
            *bp = 0;
            return 1;
        }

        if (c == 0x1b) { /* escape sequence */
            esc  = 1;
            key1 = -1;
            key2 = -1;
            continue;
        }

        if (esc) {
            if (key1 < 0) {
                key1 = c;
                if (key1 != 0x5b) {
                    /* not '[' */
                    inbuf[*bp] = 0x1b;
                    (*bp)++;

                    inbuf[*bp] = key1;
                    (*bp)++;

                    if (!g_cli->echo_disabled) {
                        tmp = g_cli_tag_len;
                        g_cli_tag_len = 0;

                        cli_printf("\x1b%c", key1); /* Ignore the cli tag */
                        g_cli_tag_len = tmp;
                    }
                    esc = 0;
                }
                continue;
            }

            if (key2 < 0) {
                key2 = c;
                if (key2 == 't') {
                    g_cli_tag[0]  = 0x1b;
                    g_cli_tag[1]  = key1;
                    g_cli_tag_len = 2;
                }
            }

            if (key2 != 0x41 && key2 != 0x42 && key2 != 't') {
                inbuf[*bp] = 0x1b;
                (*bp)++;

                inbuf[*bp] = key1;
                (*bp)++;

                inbuf[*bp] = key2;
                (*bp)++;

                g_cli_tag[0]  = '\x0';
                g_cli_tag_len = 0;
                esc           = 0;

                if (!g_cli->echo_disabled) {
                    cli_printf("\x1b%c%c", key1, key2);
                }
                continue;
            }

#if CLI_MINIMUM_MODE > 0
            if (key2 == 0x41 || key2 == 0x42) {
                tmp = g_cli_tag_len;
                g_cli_tag_len = 0;

                /* Ignore the cli tag */
                cli_printf("\r\n" PROMPT
                        "Warning! mini cli mode do not support history cmds!");
                g_cli_tag_len = tmp;
            }
#else
            if (key2 == 0x41) {
                cli_up_history(inbuf);

                *bp           = strlen(inbuf);
                g_cli_tag[0]  = '\x0';
                g_cli_tag_len = 0;
                esc           = 0;

                cli_printf("\r\n" PROMPT "%s", inbuf);
                continue;
            }

            if (key2 == 0x42) {
                cli_down_history(inbuf);

                *bp           = strlen(inbuf);
                g_cli_tag[0]  = '\x0';
                g_cli_tag_len = 0;
                esc           = 0;

                cli_printf("\r\n" PROMPT "%s", inbuf);
                continue;
            }
#endif
            /* ESC_TAG */
            if (g_cli_tag_len >= sizeof(g_cli_tag)) {
                g_cli_tag[0]  = '\x0';
                g_cli_tag_len = 0;
                esc           = 0;

                cli_printf("Error: cli tag buffer overflow\r\n");
                continue;
            }

            g_cli_tag[g_cli_tag_len++] = c;
            if (c == 'm') {
                g_cli_tag[g_cli_tag_len++] = '\x0';
                tmp = g_cli_tag_len;
                g_cli_tag_len = 0;

                if (!g_cli->echo_disabled) {
                    tmp = g_cli_tag_len;
                    g_cli_tag_len = 0;

                    cli_printf("%s", g_cli_tag);
                    g_cli_tag_len = tmp;
                }
                esc = 0;
            }
            continue;
        }

        inbuf[*bp] = c;
        if ((c == 0x08) || (c == 0x7f)) {
            if (*bp > 0) {
                (*bp)--;

                if (!g_cli->echo_disabled) {
                    tmp = g_cli_tag_len;
                    g_cli_tag_len = 0;

                    cli_printf("%c %c", 0x08, 0x08);
                    g_cli_tag_len = tmp;
                }
            }
            continue;
        }

        if (c == '\t') {
            inbuf[*bp] = '\0';
            cli_tab_complete(inbuf, bp);
            continue;
        }

        if (!g_cli->echo_disabled) {
            tmp = g_cli_tag_len;
            g_cli_tag_len = 0;

            cli_printf("%c", c);
            g_cli_tag_len = tmp;
        }

        (*bp)++;
        if (*bp >= CLI_INBUF_SIZE) {
            cli_printf("Error: input buffer overflow\r\n");
            cli_printf(PROMPT);
            *bp = 0;
            return 0;
        }
    }

    return 0;
}

/**
 * @brief Print out a bad command string
 *
 * @param[in] cmd_string the command string
 *
 * @return none
 *
 * @Note print including a hex representation of non-printable characters.
 * Non-printable characters show as "\0xXX".
 */
static void cli_print_bad_command(char *cmd_string)
{
    if (cmd_string != NULL) {
        cli_printf("command '%s' not found\r\n", cmd_string);
    }
}


/**
 * @brief Main CLI processing loop
 *
 * @param[in] data pointer to the process arguments
 *
 * @return none
 *
 * @Note Waits to receive a command buffer pointer from an input collector,
 * and then process. it must cleanup the buffer when done with it.
 * Input collectors handle their own lexical analysis and must pass complete
 * command lines to CLI.
 *
 */
void cli_main(void *data)
{
    int32_t ret;

    char *msg = NULL;

    while (!g_cli_exit) {
        if (cli_get_input(g_cli->inbuf, &g_cli->bp) != 0) {
            msg = g_cli->inbuf;

#if (CLI_MINIMUM_MODE <= 0)
            if (strlen(g_cli->inbuf) > 0) {
                cli_history_input();
            }
#endif

            ret = cli_handle_input(msg);
            if (ret == CLI_ERR_BADCMD) {
                cli_print_bad_command(msg);
            } else if (ret == CLI_ERR_SYNTAX) {
                cli_printf("syntax error\r\n");
            }

            cli_printf("\r\n");
            g_cli_tag[0]  = '\x0';
            g_cli_tag_len = 0;
            cli_printf(PROMPT);
        }
    }

    cli_printf("CLI exited\r\n");
    cli_free(g_cli);
    g_cli = NULL;

    cli_task_exit();
}

int32_t cli_init(void)
{
    int32_t ret;

    g_cli = (struct cli_status *)cli_malloc(sizeof(struct cli_status));
    if (g_cli == NULL) {
        return CLI_ERR_NOMEM;
    }

    memset((void *)g_cli, 0, sizeof(struct cli_status));

    ret = cli_task_create("cli", cli_main, NULL, CLI_STACK_SIZE, CLI_TASK_PRIORITY);
    if (ret != CLI_OK) {
        cli_printf("Error: Failed to create cli thread: %d\r\n", ret);
        goto init_err;
    }

    g_cli->inited        = 1;
    g_cli->echo_disabled = 0;

    ret = cli_register_default_commands();
    if (ret != CLI_OK) {
        cli_printf("Error: register built-in commands failed");
        goto init_err;
    }

    return CLI_OK;

init_err:
    if (g_cli != NULL) {
        cli_free(g_cli);
        g_cli = NULL;
    }

    return ret;
}

int32_t cli_stop(void)
{
    g_cli_exit = 1;

    return CLI_OK;
}

char *cli_tag_get(void)
{
    return g_cli_tag;
}

int32_t cli_register_command(const struct cli_command_st *cmd)
{
    int32_t i = 0;

#if (RHINO_CONFIG_USER_SPACE > 0)
    uint32_t  pid = 0;
    void     *cli_q;
    ktask_t  *cur_task, *cur_proc;
#endif

    if (g_cli == NULL) {
        return CLI_ERR_DENIED;
    }

    if (!cmd->name || !cmd->function) {
        return CLI_ERR_INVALID;
    }

    if (g_cli->num >= CLI_MAX_COMMANDS) {
        return CLI_ERR_NOMEM;
    }

    /*
     * Check if the command has already been registered.
     * Return 0, if it has been registered.
     */
    for (i = 0; i < g_cli->num; i++) {
        if (g_cli->cmds[i] == cmd) {
            return CLI_OK;
        }
    }

    g_cli->cmds[g_cli->num++] = cmd;

#if (RHINO_CONFIG_USER_SPACE > 0)
    cur_task = krhino_cur_task_get();
    cur_proc = cur_task->proc_addr;

    if (cur_proc == NULL)
        return 0;

    pid   = cur_proc->pid;
    cli_q = cur_proc->cli_q;

    if (!cli_q)
        return CLI_ERR_INVALID;

    /*user app pid:1,2,3..*/
    if ((pid > 0) && (pid < (MAX_APP_BINS + 1))) {
        g_cli->u_cmds[pid][g_cli->u_num[pid]] = cmd;
        g_cli->u_num[pid] ++;
        if (g_cli->u_cli_queue[pid] == NULL)
            g_cli->u_cli_queue[pid] = (kqueue_t *)cli_q;
    }
#endif

    return CLI_OK;
}

int32_t cli_unregister_command(const struct cli_command_st *cmd)
{
    int32_t remaining_cmds;
    int32_t i = 0;

    if (g_cli == NULL) {
        return CLI_ERR_DENIED;
    }

    if (!cmd->name || !cmd->function) {
        return CLI_ERR_INVALID;
    }

    for (i = 0; i < g_cli->num; i++) {
        if (g_cli->cmds[i] == cmd) {
            g_cli->num--;

            remaining_cmds = g_cli->num - i;
            if (remaining_cmds > 0) {
                memmove(&g_cli->cmds[i], &g_cli->cmds[i + 1],
                        (remaining_cmds * sizeof(struct cli_command_st *)));
            }

            g_cli->cmds[g_cli->num] = NULL;

            return CLI_OK;
        }
    }

    return CLI_ERR_NOMEM;
}

int32_t cli_register_commands(const struct cli_command_st *cmds, int32_t num)
{
    int32_t i, err;

    for (i = 0; i < num; i++) {
        if ((err = cli_register_command(cmds++)) != 0) {
            return err;
        }
    }

    return CLI_OK;
}

int32_t cli_unregister_commands(const struct cli_command_st *cmds, int32_t num)
{
    int32_t i, err;

    for (i = 0; i < num; i++) {
        if ((err = cli_unregister_command(cmds++)) != 0) {
            return err;
        }
    }

    return CLI_OK;
}

int32_t cli_printf(const char *buffer, ...)
{
    va_list ap;

    int32_t sz, len;

    char *pos     = NULL;
    char *message = NULL;

    message = (char *)cli_malloc(CLI_OUTBUF_SIZE);
    if (message == NULL) {
        return CLI_ERR_NOMEM;
    }

    memset(message, 0, CLI_OUTBUF_SIZE);

    sz = 0;
    if (g_cli_tag != NULL) {
        len = strlen(g_cli_tag);
        strncpy(message, g_cli_tag, len);
        sz = len;
    }

    pos = message + sz;

    va_start(ap, buffer);
    len = vsnprintf(pos, CLI_OUTBUF_SIZE - sz, buffer, ap);
    va_end(ap);

    if (len <= 0) {
        cli_free(message);

        return CLI_OK;
    }

    cli_putstr(message);
    cli_free(message);

    return CLI_OK;
}

int32_t cli_get_commands_num(void)
{
    return g_cli->num;
}

struct cli_command_st *cli_get_command(int32_t index)
{
    return (struct cli_command_st *)(g_cli->cmds[index]);
}

int32_t cli_get_echo_status(void)
{
    return g_cli->echo_disabled;
}

int32_t cli_set_echo_status(int32_t status)
{
    g_cli->echo_disabled = status;

    return CLI_OK;
}
