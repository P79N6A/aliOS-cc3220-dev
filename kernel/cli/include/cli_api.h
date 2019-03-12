/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef CLI_API_H
#define CLI_API_H

#include <stdint.h>

#include "cli_err.h"

#ifdef __cplusplus
extern "C" {
#endif

struct cli_command_st {
    const char *name;
    const char *help;

    void (*function)(char *outbuf, int32_t len, int32_t argc, char **argv);
};

/**
 * @brief Initialize the CLI module
 *
 * @return 0 on success, otherwise failed
 *
 */
int32_t cli_init(void);

/**
 * @brief Stop the CLI task and carry out the cleanup
 *
 * @return 0 on success, otherwise failed
 *
 */
int32_t cli_stop(void);

/**
 * @brief Get CLI tag string for print
 *
 * @return CLI tag storing buffer
 *
 */
char *cli_tag_get(void);

/**
 * @brief This function registers a command with the command-line interface
 *
 * @param[in] cmd the structure to regiter one CLI command
 *
 * @return 0 on success, otherwise failed
 *
 */
int32_t cli_register_command(const struct cli_command_st *cmd);

/**
 * @brief This function unregisters a command from the command-line interface
 *
 * @param[in] cmd the structure to unregister one CLI command
 *
 * @return 0 on success, otherwise failed
 *
 */
int32_t cli_unregister_command(const struct cli_command_st *cmd);

/**
 * @brief This function registers a batch of CLI commands
 *
 * @param[in] cmds pointer to an array of commands
 * @param[in] num  number of commands in the array
 *
 * @return 0 on success, otherwise failed
 *
 */
int32_t cli_register_commands(const struct cli_command_st *cmds, int32_t num);

/**
 * @brief This function unregisters a batch of CLI commands
 *
 * @param[in] cmds pointer to an array of commands
 * @param[in] num  number of command in the array
 *
 * @return 0 on success, otherwise failed
 *
 */
int32_t cli_unregister_commands(const struct cli_command_st *cmds, int32_t num);

/**
 * @brief Print CLI message
 *
 * @param[in] buffer pointer to a char * buffer
 *
 * @return 0 on success, otherwise failed
 */
int32_t cli_printf(const char *buffer, ...);

/**
 * @brief Get the total number of CLI commands
 *
 * @return the total number
 *
 */
int32_t cli_get_commands_num(void);

/**
 * @brief Get the CLI command by index
 *
 * @param[in] index the command index
 *
 * @return the CLI command
 *
 */
struct cli_command_st *cli_get_command(int32_t index);

/**
 * @brief Get echo support status
 *
 * @return echo support status, 1: disable, 0: enable
 *
 */
int32_t cli_get_echo_status(void);

/**
 * @brief Set echo support status
 *
 * @param[in] status echo support status
 *
 * @return 0 on success, otherwise failed
 *
 */
int32_t cli_set_echo_status(int32_t status);

/**
* @brief CLI task entry
*
*/
void cli_main(void *data);

#ifdef __cplusplus
}
#endif

#endif /* CLI_API_H */
