#ifndef STDIO
#define STDIO

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void *memcpy(void *dest, void *src, size_t n);
bool isprint(unsigned char c);
bool isspace(char c);
char toupper(char ch);
void trim_right(char *str);
void trim_whitespace(char **str);
int strcmp(const char *p1, const char *p2);

#endif
