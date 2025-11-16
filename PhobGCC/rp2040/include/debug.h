#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

//#define DEBUG_ENABLED 1

inline void debug_print(const char* str) {
#if (DEBUG_ENABLED)
  printf(str);
#endif
}
inline void debug_print(char c) {
#if (DEBUG_ENABLED)
  printf(&c);
#endif
}
inline void debug_print(int i, int x = 0) {
#if (DEBUG_ENABLED)
  printf("%d", i);
#endif
}
inline void debug_print(long l, int x = 0) {
#if (DEBUG_ENABLED)
  printf("%ld", l);
#endif
}
inline void debug_print(double d, int x = 0) {
#if (DEBUG_ENABLED)
  printf("%f", d);
#endif
}

inline void debug_println(const char* str) {
#if (DEBUG_ENABLED)
  printf(str);
  printf("\n");
#endif
}
inline void debug_println(char c) {
#if (DEBUG_ENABLED)
  printf(&c);
  printf("\n");
#endif
}
inline void debug_println(int i, int x = 0) {
#if (DEBUG_ENABLED)
  printf("%d", i);
  printf("\n");
#endif
}
inline void debug_println(long l, int x = 0) {
#if (DEBUG_ENABLED)
  printf("%ld", l);
  printf("\n");
#endif
}
inline void debug_println(double d, int x = 0) {
#if (DEBUG_ENABLED)
  printf("%f", d);
  printf("\n");
#endif
}
inline void debug_println() {
#if (DEBUG_ENABLED)
  printf("\n");
#endif
}

#endif //DEBUG_H
