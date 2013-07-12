#include <common/Defenitions.h>

#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <cxxabi.h>

#define BACKTRACE_ARRAY_COUNT   100
#define FUNCTION_MAX_SIZE  200


namespace KSRobot
{
namespace common
{

void PrintStackTrace()
{
    void *array[BACKTRACE_ARRAY_COUNT];
    size_t stack_depth;
    
    stack_depth = backtrace(array, BACKTRACE_ARRAY_COUNT);
    
    //backtrace_symbols_fd(array, num_entries, STDOUT_FILENO);
    
    char** strings;
    strings = backtrace_symbols(array, stack_depth);
    
    //now demangle names
    
    size_t function_name_size = FUNCTION_MAX_SIZE;
    char *function_name = (char*)malloc(function_name_size);
    for(size_t i = 0; i < stack_depth; i++)
    {
        char *begin = 0, *end = 0;
        for(char* j = strings[i]; *j; j++)
        {
            if( *j == '(' )
                begin = j;
            if( *j == '+' )
            {
                end = j;
                break;
            }
        }
        
        if( begin && end )
        {
            begin++;
            char tmpEnd = *end;
            *end = '\0';
            
            int stat;
            char* ret = abi::__cxa_demangle(begin, function_name, &function_name_size, &stat);
            *end = tmpEnd;
            if( ret )
            {
                function_name = ret;
                printf("\t%s\n", ret);
                //free(ret);
            }
            else
            {
                //demangle failed. print whole
                printf("\t%s\n", strings[i]);
            }
        }
        else
        {
            // name cannot be demangled
            printf("\t%s\n", strings[i]);
        }
    }

    free(function_name);
    free(strings);
}

#ifndef NDEBUG

static void SigSegvHandler(int sig)
{
    (void)sig;
    printf("\n\n\nERROR, Segmentation fault:\n");
    PrintStackTrace();
}
#endif

void RegisterDebugModeStackTracePrinter()
{
#ifndef NDEBUG
    signal(SIGSEGV, SigSegvHandler);
#endif
}


}
}
