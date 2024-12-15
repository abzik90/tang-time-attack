Practice Problem from My Team Lead
STRCMP Function/Macro in C

Let's take a look at a simple STRCMP() function/macro written in C:

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

Pin Guessing with STRCMP

Assume the correct PIN is 073310. If we provide an incorrect sequence (e.g., 1xxxxx instead of 0xxxxx), we notice that the run with 0xxxxx takes slightly more time than 1xxxxx. This is because the STRCMP() macro returns a non-zero result right away if the incorrect sequence is given, but 0xxxxx proceeds to check the following character.

The same logic applies to each digit. Instead of brute-forcing the PIN with all possible combinations (10^n), we can guess the PIN in just 10 * n attempts, where n is the number of digits in the PIN.
How to Measure Time Between Checks

To measure the time between checks, we need to consider how the MCU handles the timing:

    Measuring between UART TX and RX: Measuring the time between the last UART TX and the first RX is unreliable because of UART buffering and slower clocks. The buffering and timing delays can introduce variability in the measurement.

    Better Measurement: Instead of relying on UART RX timing, we measure the time between the last UART TX rising edge and the rising edge of the GPIO pin connected to an indicator LED (which is turned on when an incorrect PIN is entered). This is more reliable because GPIO timing is generally faster and more precise compared to UART.

Cool-Down and Reset Mechanism

The emulated MCU has a 15-second cool-down after each incorrect input. Therefore, to proceed with the next guess, a reset command (r) is sent via the second UART after each failed attempt. This ensures that the system does not accept further inputs until the cool-down period has passed.
