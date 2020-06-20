/* ------------------------------------------------------------------------------------------------ */
/* FILE :		WebServer.h																			*/
/*																									*/
/* CONTENTS :																						*/
/*				Code for the lwIP webserver with netconn / BSD sockets								*/
/*				This file contains all the webservices management									*/
/*																									*/
/*																									*/
/* Copyright (c) 2013-2014, Code-Time Technologies Inc. All rights reserved.						*/
/*																									*/
/* Code-Time Technologies retains all right, title, and interest in and to this work				*/
/*																									*/
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS							*/
/* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF										*/
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL							*/
/* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR								*/
/* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,							*/
/* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR							*/
/* OTHER DEALINGS IN THE SOFTWARE.																	*/
/*																									*/
/*																									*/
/*	$Revision: 1.8 $																				*/
/*	$Date: 2013/10/03 15:55:16 $																	*/
/*																									*/
/* ------------------------------------------------------------------------------------------------ */

#ifndef __WEBSERVER_H__
#define __WEBSERVER_H__

#include <string.h>
#include "lwip/def.h"
#include "lwip/opt.h"
#include "alt_ethernet.h"
#include "alt_generalpurpose_io.h"

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef WEBSERVER_PRIO
  #define WEBSERVER_PRIO				(OS_PRIO_MIN-1)	/* Priority of the HTTP webserver task		*/
#endif
#ifndef WEBSERVER_STACKSIZE
  #define WEBSERVER_STACKSIZE			8192		/* Stack size of the HTTP webserver task		*/
#endif
#ifndef LWIP_HTTP_MAX_CGI
  #define LWIP_HTTP_MAX_CGI				16			/* Maximum number of diffrent CGIs handlers		*/
#endif
#ifndef LWIP_HTTP_MAX_POST
  #define LWIP_HTTP_MAX_POST			16			/* Maximum number of diffrent POSTs handlers	*/
#endif
#ifndef LWIP_HTTP_MAX_CGI_POST_PARAM
  #define LWIP_HTTP_MAX_CGI_POST_PARAM	16			/* Maximum number of Param=Value in a request	*/
#endif
#ifndef LWIP_HTTP_MAX_VARIABLE
  #define LWIP_HTTP_MAX_VARIABLE		64			/* Maximum number of server variables for SSI	*/
#endif
#ifndef LWIP_HTTP_MAX_SSI
  #define LWIP_HTTP_MAX_SSI				64			/* Maximum number of SSI update functions		*/
#endif

#define GPIO_DIR(Port,Dir)	alt_gpio_port_datadir_set(alt_gpio_bit_to_pid(Port), 					\
                                                      1<<alt_gpio_bit_to_port_pin(Port),			\
							                          ((Dir)==0)?0:1<<alt_gpio_bit_to_port_pin(Port))

#define GPIO_SET(Port,Val)	alt_gpio_port_data_write (alt_gpio_bit_to_pid(Port), 					\
                                                      1<<alt_gpio_bit_to_port_pin(Port),			\
							                          ((Val)==0)?0:1<<alt_gpio_bit_to_port_pin(Port))
#define SW_GET(Switch)		alt_gpio_port_data_read  (2, 1<<Switch)

#define LED_0	ALT_GPIO_1BIT_44
#define LED_1	ALT_GPIO_1BIT_43
#define LED_2	ALT_GPIO_1BIT_42
#define LED_3	ALT_GPIO_1BIT_41

#define SW_0	(24)
#define SW_1	(23)
#define SW_2	(22)
#define SW_3	(21)

/* ------------------------------------------------------------------------------------------------ */

typedef u16_t       (*SSIhandler_t)(int Index, char *Insert, int InsertLen);
typedef const char *(*CGIhandler_t)(int NumParams, char *Param[], char *Value[]);

typedef struct {
    const char  *Name;
    CGIhandler_t Handler;
} CGI_t;

enum VerbTypes {V_GET,   V_HEAD,    V_POST,    V_PUT,   V_DELETE,
                V_TRACE, V_OPTIONS, V_CONNECT, V_PATCH, V_UNKNOWN_};

struct Methods {									/* Association of text to verbs					*/
	char          *Text;
	enum VerbTypes Verb;
};

struct LocalVar {									/* Local variable information holding			*/
	char VarName[32];								/* Name of the variable							*/
	char Value[32];									/* Value of the variable: always text			*/
	char *(*Command)(char *);    					/* Command associated when is an exec variable	*/
};

#if (((OS_DEMO) == 10) || ((OS_DEMO) == 11))
  #include "fs.h"
  typedef struct fs_file	*FILE_DSC_t;
  #define F_INIT_FDSC(x)	((x)=NULL)
  #define F_MNT()                               0
  #define F_OPEN(F_Dsc, F_Name, F_Access)		(NULL != ((F_Dsc)=fs_open(F_Name)))		
  #define F_CLOSE(F_Dsc)						fs_close(F_Dsc)
  #define F_READ(F_Dsc, Buf, BufSize, Nread)												\
						if (-1 == (*(Nread)=fs_read((F_Dsc), (char *)Buf, (BufSize)))) {	\
							*(Nread) = 0;													\
						}
  #define FA_READ								0
#endif
#if (((OS_DEMO) == 12) || ((OS_DEMO) == 13))
  #include "ff.h"
  typedef FIL				FILE_DSC_t;
  #define F_INIT_FDSC(x)
  #define F_MNT()                             f_mount(&g_FileSys, "0", 1)
  #define F_OPEN(F_Dsc, F_Name, F_Access)	  (FR_OK == (f_open(&(F_Dsc),(F_Name), (F_Access))))
  #define F_CLOSE(F_Dsc)					  f_close(&(F_Dsc))
  #define F_READ(F_Dsc, Buf, BufSize, Nread)  f_read(&(F_Dsc),(void *)(Buf),(BufSize),(UINT *)(Nread))
#endif

/* ------------------------------------------------------------------------------------------------ */

void  CGIinit(void);
char *CGIprocess(char *buf);
int   CGInewHandler(const char *Fname, const CGIhandler_t Handler);
void  POSTinit(void);
char *POSTprocess(char *uri, char *Data);
int   SSIexpand(char *buf, const char *uri);
int   POSTnewHandler(const char *Fname, const CGIhandler_t Handler);
char *SSIgetVar(const char *Name);
void  SSIinit(void);
int   SSInewUpdate(void (*FctPtr)(void));
int   SSInewVar(const char *Name, char *Value, char *(*Command)(char *));
int   SSIvarUpdate(void);
void  WebServerInit(void);

#ifdef __cplusplus
}
#endif

#endif

/* EOF */

