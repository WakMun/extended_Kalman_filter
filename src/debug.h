#ifndef __debug_h__
#define __debug_h__

#ifdef DEBUG
#  define DEBUGOUT(x) do {std::cerr << x << std::endl;} while(0);
#else
#  define DEBUGOUT(x) do {} while (0);
#endif




#endif  /* __debug_h__*/