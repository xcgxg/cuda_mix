
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the CORRELATOR_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// CORRELATOR_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef CORRELATOR_EXPORTS
#define CORRELATOR_API __declspec(dllexport)
#else
#define CORRELATOR_API __declspec(dllimport)
#endif

