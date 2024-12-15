##This is a practice problem from my teamlead:

Let's have a look at the simple STRCMP() function/macros written in C:

#define STRCMP(s1, s2) ({ \
    const char *str1 = (s1); \
    const char *str2 = (s2); \
    int result = 0; \
    while (*str1 && *str2) { \
        if (*str1 != *str2) { \
            result = (unsigned char)(*str1) - (unsigned char)(*str2); \
            break; \
        } \
        str1++; \
        str2++; \
    } \
    if (!result && (*str1 || *str2)) { \
        result = (unsigned char)(*str1) - (unsigned char)(*str2); \
    } \
    result; \
})

Assume the correct pin is 073310:
  If we provide the incorrect sequence say 1xxxxx instead of 0xxxxx, we can notice that the run with the 0xxxxx takes slightly more time than 1xxxxx. 
  It's  because the macros returns the result(non-zero) right away if the incorrect sequence is given, but 0xxxxx proceeds to check the following character.
  The same logic applies to the consequent digits, and instead of brute-forcing our PIN with 10^n combinations, we can guess the PIN in just 10*n attempts.
How do we measure time in-between checks?
  Measuring time between the last UART tx and the first rx is unreliable(buffering, slower clocks), therefore we measure time between the tx's last rising edge 
  and the indicator(turns on incorrect PIN) led's gpio rising edge
The emulated MCU has 15s cool-down after each incorrect input, therefore "r" reset command is sent to the second UART
