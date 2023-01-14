#ifndef DEBUG_H
#define DEBUG_H

#define DEBUG_ENABLED   0

inline void debug_print(const char* str) {
#if (DEBUG_ENABLED == 1)
  //print(str);
#endif
}
inline void debug_print(char c) {
#if (DEBUG_ENABLED == 1)
  //print(c);
#endif
}
inline void debug_print(int i, int x = DEC) {
#if (DEBUG_ENABLED == 1)
  //print(i, x);
#endif
}
inline void debug_print(long l, int x = DEC) {
#if (DEBUG_ENABLED == 1)
  //print(l, x);
#endif
}
inline void debug_print(double d, int x = 2) {
#if (DEBUG_ENABLED == 1)
  //print(d, x);
#endif
}

inline void debug_println(const char* str) {
#if (DEBUG_ENABLED == 1)
  //println(str);
#endif
}
inline void debug_println(char c) {
#if (DEBUG_ENABLED == 1)
  //println(c);
#endif
}
inline void debug_println(int i, int x = DEC) {
#if (DEBUG_ENABLED == 1)
  //println(i, x);
#endif
}
inline void debug_println(long l, int x = DEC) {
#if (DEBUG_ENABLED == 1)
  //println(l, x);
#endif
}
inline void debug_println(double d, int x = 2) {
#if (DEBUG_ENABLED == 1)
  //println(d, x);
#endif
}
inline void debug_println() {
#if (DEBUG_ENABLED == 1)
  //println();
#endif
}

#endif //DEBUG_H
