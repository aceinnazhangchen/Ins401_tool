#include "ins_compile.h"
#include <stdint.h>

#define MACRO_TO_STRING_BUILD(x)    #x
#define MACRO_TO_STRING(x)          MACRO_TO_STRING_BUILD(x)
uint8_t ins_lib_compile_version[] = "INS_LIB_v_" MACRO_TO_STRING(VERSION_INS_LIB);
uint8_t ins_lib_compile_time[] = MACRO_TO_STRING(TIME_INS_LIB);
uint8_t ins_lib_compile_author[] = MACRO_TO_STRING(AUTHOR_INS_LIB);
uint8_t ins_lib_compile_commit[] = MACRO_TO_STRING(COMPILE_COMMIT);

uint8_t* get_ins_compile_version()
{
	return ins_lib_compile_version;
}

uint8_t* get_ins_compile_time()
{
	return ins_lib_compile_time;
}

uint8_t* get_ins_compile_author()
{
	return ins_lib_compile_author;
}

uint8_t* get_ins_compile_commit()
{
	return ins_lib_compile_commit;
}
