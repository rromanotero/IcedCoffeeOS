/**
 * @file    syscalls.h
 * @author
 * @version
 *
 * @brief System Calls Interface header file
 *
 */

#ifndef SYSCALLS_H_
#define SYSCALLS_H_

void syscalls_init(void);

/**
*   System call numbers
*/
enum{
    SVCDummy                = 0,
};

#endif /* SYSCALLS_H_ */
