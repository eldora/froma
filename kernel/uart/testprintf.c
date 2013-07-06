#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define printf test_printf
#define sprintf test_sprintf

#define PRINTF_FLOAT_SUPPORT
#include "printf-stdarg.c"

#undef printf
#undef sprintf

#define MaximumLength 32
#define MaximumPrecision 32
#define TestsPerFormat 64
#define MaximumNumberOfErrors 100
#define NumberOfFloatDigitsToTest 15

static void TestFormat(const char *format,char type);
static void TestIntegerFormat(const char *format,char type);
static void TestFloatFormatWithIntegers(const char *format,char type);
static void TestFloatFormatWithFloats(const char *format,char type);
static void TestFloatFormatWithSpecials(const char *format,char type);

static bool CompareFloatStringsFuzzily(const char *s1,const char *s2,int numberofdigits);

static void SeedRandom(uint32_t seed);
static uint32_t RandomInteger();
static int32_t RandomExponentialSignedInteger(int maxlength);

int main()
{
	char format[32];
	char types[]={'d','u','x','X','o','f','F','e','E','g','G'};

	for(int typeindex=0;typeindex<sizeof(types);typeindex++)
	{
		char type=types[typeindex];

		for(int flags=0;flags<31;flags++)
		{
			char flagsstring[6];
			char *flagsptr=flagsstring;

			if(flags&1) *flagsptr++='#';
			if(flags&2) *flagsptr++='0';
			if(flags&4) *flagsptr++='-';
			if(flags&8) *flagsptr++=' ';
			if(flags&16) *flagsptr++='+';
			*flagsptr=0;

			sprintf(format,"%%%s%c",flagsstring,type);
			TestFormat(format,type);

			for(int length=0;length<MaximumLength;length++)
			{
				sprintf(format,"%%%s%d%c",flagsstring,length,type);
				TestFormat(format,type);
			}

			for(int precision=0;precision<MaximumPrecision;precision++)
			{
				sprintf(format,"%%%s.%d%c",flagsstring,precision,type);
				TestFormat(format,type);
			}

			for(int length=0;length<MaximumLength;length++)
			{
				for(int precision=0;precision<MaximumPrecision;precision++)
				{
					sprintf(format,"%%%s%d.%d%c",flagsstring,length,precision,type);
					TestFormat(format,type);
				}
			}
		}
	}
}

static int numerrors=0;

static void TestFormat(const char *format,char type)
{
	SeedRandom(1234);

	switch(type)
	{
		case 'd':
		case 'u':
		case 'x':
		case 'X':
		case 'o':
			TestIntegerFormat(format,type);
		break;

		case 'f':
		case 'F':
		case 'e':
		case 'E':
		case 'g':
		case 'G':
			TestFloatFormatWithIntegers(format,type);
			TestFloatFormatWithFloats(format,type);
			TestFloatFormatWithSpecials(format,type);
		break;
	}
}

static void TestIntegerFormat(const char *format,char type)
{
	char test[1024];
	char correct[1024];
	char description[1024];
	static int numerrors=0;

	for(int i=0;i<TestsPerFormat;i++)
	{
		uint32_t bits=RandomInteger();
		uint32_t length=(RandomInteger()&31)+1;
		uint32_t sign=RandomInteger()&0x80000000;

		uint32_t bitmask;
		if(length==32) bitmask=0xffffffff;
		else bitmask=(1<<length)-1;

		int value=(bits&bitmask)^sign;
		test_sprintf(test,format,value);
		sprintf(correct,format,value);
		sprintf(description,"%d",value);

		if(strcmp(test,correct)!=0)
		{
			printf("Mismatch for format \"%s\" with value %s: Should be \"%s\", is \"%s\"\n",format,description,correct,test);
			numerrors++;
			if(numerrors>=MaximumNumberOfErrors) exit(1);
		}
	}
}

static void TestFloatFormatWithIntegers(const char *format,char type)
{
	char test[1024];
	char correct[1024];
	char description[1024];

	for(int i=0;i<TestsPerFormat;i++)
	{
		double value=RandomExponentialSignedInteger(31);

		test_sprintf(test,format,value);
		sprintf(correct,format,value);
		sprintf(description,"%f",value);

		//if(strcmp(test,correct)!=0)
		if(!CompareFloatStringsFuzzily(test,correct,NumberOfFloatDigitsToTest))
		{
			printf("Mismatch for format \"%s\" with value %s: Should be \"%s\", is \"%s\"\n",format,description,correct,test);
			numerrors++;
			if(numerrors>=MaximumNumberOfErrors) exit(1);
		}
	}
}

static void TestFloatFormatWithFloats(const char *format,char type)
{
	char test[1024];
	char correct[1024];
	char description[1024];

	for(int i=0;i<TestsPerFormat;i++)
	{
		int exponent=RandomExponentialSignedInteger(5);
		double mantissa=(double)RandomExponentialSignedInteger(31)/(double)(1<<31);
		double value=mantissa*pow(10,exponent);

		test_sprintf(test,format,value);
		sprintf(correct,format,value);
		sprintf(description,"%f",value);

		if(!CompareFloatStringsFuzzily(test,correct,NumberOfFloatDigitsToTest))
		{
			printf("Mismatch for format \"%s\" with value %s: Should be \"%s\", is \"%s\"\n",format,description,correct,test);
			numerrors++;
			if(numerrors>=MaximumNumberOfErrors) exit(1);
		}
	}
}

static void TestFloatFormatWithSpecials(const char *format,char type)
{
	char test[1024];
	char correct[1024];
	char description[1024];

	double specials[]={ 0.0, -0.0, 1.0/0.0, -1.0/0.0, 0.0/0.0, -0.0/0.0 };

	for(int i=0;i<sizeof(specials)/sizeof(specials[0]);i++)
	{
		double value=specials[i];

		test_sprintf(test,format,value);
		sprintf(correct,format,value);
		sprintf(description,"%f",value);

		//if(strcmp(test,correct)!=0)
		if(!CompareFloatStringsFuzzily(test,correct,NumberOfFloatDigitsToTest))
		{
			printf("Mismatch for format \"%s\" with value %s: Should be \"%s\", is \"%s\"\n",format,description,correct,test);
			numerrors++;
			if(numerrors>=MaximumNumberOfErrors) exit(1);
		}
	}
}

static bool CompareFloatStringsFuzzily(const char *s1,const char *s2,int numberofdigits)
{
	// Compare two strings with numbers in them, ignoring difference in numbers
	// (but not other characters) after numberofdigts numbers have been encountered.
	// Also accept differences in one in the last checked decimal point, such as
	// 1.00000 -> 0.99999.
	uint64_t i1=0,i2=0;
	while(*s1 && *s2)
	{
		char c1=*s1++,c2=*s2++;

		if(c1>='0'&&c1<='9'&&c2>='0'&&c2<='9')
		{
			if(numberofdigits)
			{
				i1=i1*10+c1-'0';
				i2=i2*10+c2-'0';
				numberofdigits--;
			}
		}
		else
		{
			if(c1!=c2) return false;
		}
	}

	return *s1==0 && *s2==0 && (i1==i2 || i1==i2+1 || i1==i2-1);
}

static uint32_t s1=0xc7ff5f16,s2=0x0dc556ae,s3=0x78010089;

static void SeedRandom(uint32_t seed)
{
	s1=seed*1664525+1013904223|0x10;
	s2=seed*1103515245+12345|0x1000;
	s3=seed*214013+2531011|0x100000;
}

static uint32_t RandomInteger()
{
	s1=((s1&0xfffffffe)<<12)^(((s1<<13)^s1)>>19);
	s2=((s2&0xfffffff8)<<4)^(((s2<<2)^s2)>>25);
	s3=((s3&0xfffffff0)<<17)^(((s3<<3)^s3)>>11);
	return s1^s2^s3;
}

static int32_t RandomExponentialSignedInteger(int maxlength)
{
	uint32_t bits=RandomInteger();
	uint32_t length=(RandomInteger()%(maxlength+1));
	uint32_t sign=RandomInteger()&1;

	if(length==0) return 0;

	int bitmask=(1<<(length-1))-1;
	int value=(1<<(length-1))|(bits&bitmask);

	if(sign) return -value;
	else return value;
}

