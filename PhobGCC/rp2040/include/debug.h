#ifndef DEBUG_H
#define DEBUG_H

#define DEBUG_ENABLED 0

inline void debug_print(const char* str) {
#if (DEBUG_ENABLED)
  //print(str);
#endif
}
inline void debug_print(char c) {
#if (DEBUG_ENABLED)
  //print(c);
#endif
}
inline void debug_print(int i, int x = 0) {
#if (DEBUG_ENABLED)
  //print(i, x);
#endif
}
inline void debug_print(long l, int x = 0) {
#if (DEBUG_ENABLED)
  //print(l, x);
#endif
}
inline void debug_print(double d, int x = 0) {
#if (DEBUG_ENABLED)
  //print(d, x);
#endif
}

inline void debug_println(const char* str) {
#if (DEBUG_ENABLED)
  //println(str);
#endif
}
inline void debug_println(char c) {
#if (DEBUG_ENABLED)
  //println(c);
#endif
}
inline void debug_println(int i, int x = 0) {
#if (DEBUG_ENABLED)
  //println(i, x);
#endif
}
inline void debug_println(long l, int x = 0) {
#if (DEBUG_ENABLED)
  //println(l, x);
#endif
}
inline void debug_println(double d, int x = 0) {
#if (DEBUG_ENABLED)
  //println(d, x);
#endif
}
inline void debug_println() {
#if (DEBUG_ENABLED)
  //println();
#endif
}

#endif //DEBUG_H
