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
    SVCPutc                 = 0,
    SVCPutPixel             = 1,
    SVCGetc                 = 2,
    SVCGetcNonBlocking      = 3,
    SVCDummy                = 11,
    SVCLoadBinEntry         = 13,
    SVCDirContent           = 14,
    SVCOpenFile             = 15,
    SVCReadFile             = 16,
};

#endif /* SYSCALLS_H_ */
