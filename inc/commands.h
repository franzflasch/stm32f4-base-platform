/*
 * commands.h
 *
 *  Created on: Mar 23, 2013
 *      Author: franz
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <commandline.h>

#define USERFLASH_MEMORY_BASE 0x080E0000

/*
 * Commandfunction prototypes:
 */
portBASE_TYPE simpleHelloCmd( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE simpleParamCmd( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE storeInFlash( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE getFromFlash( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/*
 * Available Commands:
 */
static const xCommandLineInput xHelloCLI =
{
    (int8_t*)"hello",
    (int8_t*)"hello: Sends simple hello back for testing. \r\n",
    simpleHelloCmd,
    0
};

static const xCommandLineInput xSimpleParamCLI =
{
   (int8_t*)"param",
   (int8_t*)"param: Simple parameter command. \r\n",
   simpleParamCmd,
   1
};

static const xCommandLineInput xStoreInFlash =
{
   (int8_t*)"store",
   (int8_t*)"store: Store value in flashmemory. \r\n",
   storeInFlash,
   1
};

static const xCommandLineInput xgetFromFlash =
{
   (int8_t*)"get",
   (int8_t*)"get: Get value from flashmemory. \r\n",
   getFromFlash,
   0
};


#endif /* COMMANDS_H_ */
