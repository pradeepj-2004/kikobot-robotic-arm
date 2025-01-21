#ifndef MESKO_HARDWARE__VISIBILITY_CONTROL_H_
#define MESKO_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MESKO_HARDWARE_EXPORT __attribute__((dllexport))
#define MESKO_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define MESKO_HARDWARE_EXPORT __declspec(dllexport)
#define MESKO_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef MESKO_HARDWARE_BUILDING_DLL
#define MESKO_HARDWARE_PUBLIC MESKO_HARDWARE_EXPORT
#else
#define MESKO_HARDWARE_PUBLIC MESKO_HARDWARE_IMPORT
#endif
#define MESKO_HARDWARE_PUBLIC_TYPE MESKO_HARDWARE_PUBLIC
#define MESKO_HARDWARE_LOCAL
#else
#define MESKO_HARDWARE_EXPORT __attribute__((visibility("default")))
#define MESKO_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define MESKO_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define MESKO_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define MESKO_HARDWARE_PUBLIC
#define MESKO_HARDWARE_LOCAL
#endif
#define MESKO_HARDWARE_PUBLIC_TYPE
#endif

#endif  // MESKO_HARDWARE__VISIBILITY_CONTROL_H_