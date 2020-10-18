#include "util.h"

void *memcpy(void *dest, void *src, size_t n){
    uint8_t *my_dest = (uint8_t *)dest;
    uint8_t *my_src = (uint8_t *)src;

    while (n--) {
        *my_dest++ = *my_src++;
    }

    return dest;
}

// check if character is printable
bool isprint(unsigned char c) {
    return 0x20 <= c && c <= 0x7e;
}

bool isspace(char c) {
   return c == ' ' || c == '\t';
}

bool isalpha(char x){
    return (x>=65 && x<=90) || (x>=97 && x<=122);
}

// This assumes ascii but I doubt we have to worry about any other encoding
char toupper(char ch) {
    return ('a' <= ch & ch <= 'z') << 5 ^ ch;
}

void trim_right(char *str) {
    char *s = str;
    while (*s) ++s;
    if (s > str) {
        // null the whitespace after the last non-whitespace char
        while (isspace(*(--s)));
        s[1] = '\0';
    }
}

void trim_whitespace(char **str) {
    char *s = *str;
    // find first non-whitespace character
    while (isspace(*s)) ++s;
    // make the string start with it
    *str = s;

    trim_right(s);
}

// strcmp copied from glibc
int strcmp(const char *p1, const char *p2) {
    const unsigned char *s1 = (const unsigned char *) p1;
    const unsigned char *s2 = (const unsigned char *) p2;
    unsigned char c1, c2;
    do {
        c1 = (unsigned char) *s1++;
        c2 = (unsigned char) *s2++;
        if (c1 == '\0')
            return c1 - c2;
    } while (c1 == c2);
    return c1 - c2;
}

