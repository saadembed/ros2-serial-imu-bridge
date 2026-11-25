/* Minimal assert-style test harness, single header, no dependencies. */
#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdio.h>
#include <string.h>

static int th_checks = 0;
static int th_failures = 0;
static int th_tests = 0;
static int th_failed_tests = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        th_checks++;                                                         \
        if (!(cond)) {                                                       \
            th_failures++;                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                    \
    } while (0)

#define CHECK_EQ_INT(a, b)                                                   \
    do {                                                                     \
        long th_a = (long)(a), th_b = (long)(b);                             \
        th_checks++;                                                         \
        if (th_a != th_b) {                                                  \
            th_failures++;                                                   \
            printf("  FAIL %s:%d: %s == %s (%ld vs %ld, 0x%lX vs 0x%lX)\n",  \
                   __FILE__, __LINE__, #a, #b, th_a, th_b,                   \
                   (unsigned long)th_a, (unsigned long)th_b);                \
        }                                                                    \
    } while (0)

#define CHECK_MEM(a, b, n)                                                   \
    do {                                                                     \
        th_checks++;                                                         \
        if (memcmp((a), (b), (n)) != 0) {                                    \
            th_failures++;                                                   \
            printf("  FAIL %s:%d: memcmp(%s, %s, %s)\n",                     \
                   __FILE__, __LINE__, #a, #b, #n);                          \
        }                                                                    \
    } while (0)

#define RUN_TEST(fn)                                                         \
    do {                                                                     \
        int th_before = th_failures;                                         \
        th_tests++;                                                          \
        fn();                                                                \
        if (th_failures != th_before) {                                      \
            th_failed_tests++;                                               \
            printf("[FAIL] %s\n", #fn);                                      \
        } else {                                                             \
            printf("[ OK ] %s\n", #fn);                                      \
        }                                                                    \
    } while (0)

#define TEST_SUMMARY(name)                                                   \
    (printf("%s: %d/%d tests passed, %d checks, %d failed checks\n", (name), \
            th_tests - th_failed_tests, th_tests, th_checks, th_failures),   \
     th_failures == 0 ? 0 : 1)

#endif /* TEST_HARNESS_H */
