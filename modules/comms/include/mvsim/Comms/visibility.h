#pragma once

// From https://gcc.gnu.org/wiki/Visibility
#if defined(_WIN32) || defined(__CYGWIN__)
// Assume we are always building a library for now to simplify
// #ifdef MVSIM_BUILDING_LIBRARY
#if defined(__GNUC__) || defined(__clang__)
#define MVSIM_PUBLIC __attribute__((dllexport))
#else
#define MVSIM_PUBLIC \
	__declspec(dllexport)  // Note: actually gcc seems to also supports this syntax.
#endif
// #else
//   #if defined(__GNUC__) || defined(__clang__)
//     #define MVSIM_PUBLIC __attribute__ ((dllimport))
//   #else
//     #define MVSIM_PUBLIC __declspec(dllimport) // Note: actually gcc seems to also supports this
//     syntax.
//   #endif
// #endif
#define MVSIM_LOCAL
#else
#if __GNUC__ >= 4 || defined(__clang__)
#define MVSIM_PUBLIC __attribute__((visibility("default")))
#define MVSIM_LOCAL __attribute__((visibility("hidden")))
#else
#define MVSIM_PUBLIC
#define MVSIM_LOCAL
#endif
#endif