#ifndef BAROBO_LOGGING_H
#define BAROBO_LOGGING_H

#ifdef DEBUG
#ifdef _WIN32
#define bDebug(...) _ftprintf(__VA_ARGS__)
#else
#define bDebug(...) fprintf(__VA_ARGS__)
#endif
#else
#define bDebug(...) 0
#endif

#ifdef VERBOSE
#ifdef _WIN32
#define bInfo(...) _ftprintf(__VA_ARGS__)
#else
#define bInfo(...) fprintf(__VA_ARGS__)
#endif
#else
#define bInfo(...) 0
#endif

#endif
