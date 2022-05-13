#ifndef INS_VERSION_H_
#define INS_VERSION_H_
#include <stdio.h>
#include<string.h>
#include "ins_interface_API.h"

//��������
//�汾��
//��Ҫ����
//��������
const char* const APP = "INS\n";
const char* const VERSION = "1.1.2\n";
const char* const CONTENT = "1:GNSS/INS/ODO FUSION;2:ONLINE RBV ESTIMATE\n";
const char* const CORRECTION = "1、continent 2 Export： last updated observations；Odometer is good or not；Static judgment\n";

const char* const get_APPNAME()
{
	return APP;
};

const char* const get_VERSIONNUMBER()
{
	return VERSION;
};

const char* const get_CONTENT()
{
	return CONTENT;
};

const char* const get_CORRECTION()
{
	return CORRECTION;
};

int8_t get_versionstr(char* str,int32_t length)
{
	int8_t ret = 0;
	const char* const  app = get_APPNAME();
	const char* const version = get_VERSIONNUMBER();
	const char* const content = get_CONTENT();
	const char* const correction = get_CORRECTION();
	int32_t len = strlen(app) + strlen(version) + strlen(content) + strlen(correction);
	if (len > length)
	{
		ret = -1;
	}
	else
	{
		strcat(str, app);
		strcat(str, version);
		strcat(str, content);
		strcat(str, correction);
		ret = 1;
	}
	return ret;
}




#endif // !INS_VERSION_H_
