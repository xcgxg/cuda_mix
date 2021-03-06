
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the CORRELATOR_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// CORRELATOR_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef GLO1_CORRELATOR_EXPORTS
#define GLO1_CORRELATOR_API __declspec(dllexport)
#else
#define GLO1_CORRELATOR_API __declspec(dllimport)
#endif

