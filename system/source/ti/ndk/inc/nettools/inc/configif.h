/*
 * Copyright (c) 2012-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/*
 * ======== configif.h ========
 *
 * Configuration Manager Interface functions
 *
 */

#ifndef _CONFIGIF_H
#define _CONFIGIF_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------- */
/* Config API */
/*----------------------------------------------------------------------- */

/*----------------------------------------------------------------------- */
/* CfgNew() */
/* Create a new configuration */
/*----------------------------------------------------------------------- */
extern void *CfgNew();

/*----------------------------------------------------------------------- */
/* CfgFree( void *hCfg ) */
/* Destroy a configuration */
/*----------------------------------------------------------------------- */
extern void CfgFree( void *hCfg );

/*----------------------------------------------------------------------- */
/* CfgSetDefault() */
/* Set the default configuration */
/*----------------------------------------------------------------------- */
extern void CfgSetDefault( void *hCfg );

/*----------------------------------------------------------------------- */
/* CfgGetDefault() */
/* Get the default configuration */
/*----------------------------------------------------------------------- */
extern void *CfgGetDefault();

/*----------------------------------------------------------------------- */
/* CfgLoad() */
/* Load a configuration from memory buffer */
/* Returns the number of bytes processed, or <0 on error */
/*----------------------------------------------------------------------- */
extern int  CfgLoad( void *hCfg, int Size, unsigned char *pData );

/*----------------------------------------------------------------------- */
/* CfgSave() */
/* Save configuration to a memory buffer */
/* *pSize is set to the size of the supplied buffer, or zero to get */
/* required size (the pointer pSize must be valid, but the value at */
/* the pointer can be zero). */
/* Returns the number of bytes written (0 on size check, <0 on error) */
/* *pSize is set to the number of bytes required. */
/*----------------------------------------------------------------------- */
extern int  CfgSave( void *hCfg, int *pSize, unsigned char *pData );

/*----------------------------------------------------------------------- */
/* CfgSetExecuteOrder() */
/* Establishes the order in which Tags are loaded and unloaded when */
/* the Execute status changes. */
/* When a configuration is first created, the order set in ascending */
/* Tag value order. */
/* The valute of Tags must be set to the EXACT number of tags in the */
/* configuration system. */
/* Returns 0 on success, or <0 on error */
/*----------------------------------------------------------------------- */
extern int CfgSetExecuteOrder( void *hCfg, uint32_t Tags,
                        uint32_t *pOpenOrder, uint32_t *pCloseOrder );

/*----------------------------------------------------------------------- */
/* CfgExecute() */
/* Executes the configuration - loads all loadable entries */
/* When a configuration is first created, config changes do not */
/* alter the state of the system. Once the configuration is executed, */
/* all past settings take effect, and any new settings are immediately */
/* invoked. */
/* When fExecute is set to 0, all invoked entries are shutdown. */
/* Returns 0 on success, or <0 on error */
/*----------------------------------------------------------------------- */
extern int  CfgExecute( void *hCfg, uint32_t fExecute );

/*----------------------------------------------------------------------- */
/* CfgSetService() */
/* Sets the service function for a particular config TAG. Service */
/* functions default to NULL, and when so, no service is performed. */
/* When invoked, the service callback function is passed back information */
/* about the affected entry. */
/*   int CbSrv( void *hCfg, uint32_t Tag, uint32_t Item, uint32_t Op, */
/*              void *hCfgEntry )                                        */
/*                                                                        */
/*        hCfg      = void *to Config */
/*        Tag       = Tag value of entry changed */
/*        Item      = Item value of entry changed */
/*        Op        = CFGCOP_ADD or CFGOP_REMOVE */
/*        hCfgEntry = Non-Ref'd void *to entry added or removed */
/*    Returns 1 on success, 0 on "pass", and <0 on error. */
/* Note: The config entry handle passed to the callback is NOT Ref'd, */
/*       in that its scope expires when the callback function returns. */
/* Returns 0 on success, <0 on error. */
/*----------------------------------------------------------------------- */
extern int  CfgSetService( void *hCfg, uint32_t Tag,
                           int (*pCb) (void *, uint32_t, uint32_t, uint32_t,
                           void *) );
#define CFGOP_REMOVE    0
#define CFGOP_ADD       1

/*----------------------------------------------------------------------- */
/* CfgAddEntry() */
/* Add a configuration entry to a configuration. */
/* When the pointer phCfgEntry is non-zero, this function write a */
/* referenced void *to this location. */
/* When finished with a referenced entry HANDLE, an application must */
/* DeRef it by calling one of the following functions: */
/*   CfgEntryDeRef()          - Stop using the entry */
/*   CfgRemoveEntry()         - Stop using entry and remove it from cfg */
/*   CfgGetNextEntry()        - Stop using entry and get next entry */
/* Returns 1 on successful add and processing. */
/* Returns 0 on successful add with no processing. */
/* Returns <0 but > CFGERROR_SERVICE on configuration error */
/* Returns <= CFGERROR_SERVICE on successful add, but service error */
/*----------------------------------------------------------------------- */
extern int  CfgAddEntry( void *hCfg, uint32_t Tag, uint32_t Item,
                         uint32_t Mode, int Size, unsigned char *pData,
                         void **phCfgEntry );

/* Add Entry Flags */
#define CFG_ADDMODE_UNIQUE      0x0001  /* Replace all previous instances */
#define CFG_ADDMODE_DUPLICATE   0x0002  /* Allow duplicate data entry */
#define CFG_ADDMODE_NOSAVE      0x0004  /* Don't include this entry in CfgSave */

/*----------------------------------------------------------------------- */
/* CfgRemoveEntry() */
/* Performs a single DeRef on a configuration entry, and removes it from */
/* the configuration structure. */
/* Returns 0 on success, or <0 on error */
/*----------------------------------------------------------------------- */
extern int  CfgRemoveEntry( void *hCfg, void *hCfgEntry );

/*----------------------------------------------------------------------- */
/* CfgGetEntryCnt() */
/* Returns the number of entries on a specific tag/item, or <0 on error */
/*----------------------------------------------------------------------- */
extern int  CfgGetEntryCnt( void *hCfg, uint32_t Tag, uint32_t Item );

/*----------------------------------------------------------------------- */
/* CfgGetEntry() */
/* Get a referenced void *to a configuration entry */
/* Index is a relative value (the "n'th" 1-based entry in a list) */
/* DO NOT use the index value to enumerate entry entry in the list. */
/* The index is valid only at the time of the call as an item can */
/* move up and down in the list as config changes are made. To */
/* enumerate every entry for a Tag/Item pair, start with index 1 and */
/* then use GetNextEntry() to get additional entries. */
/* When finished with this entry, an application must deref it */
/* by calling one of the following functions: */
/*   CfgEntryDeRef()          - Stop using the entry */
/*   CfgRemoveEntry()         - Stop using entry and remove it from cfg */
/*   CfgGetNextEntry()        - Stop using entry and get next entry */
/* Function returns 1 on success, 0 on "not found", and <0 on error. */
/*----------------------------------------------------------------------- */
extern int  CfgGetEntry( void *hCfg, uint32_t Tag, uint32_t Item,
                         uint32_t Index, void **phCfgEntry );

/*----------------------------------------------------------------------- */
/* CfgGetNextEntry() */
/* DeRef supplied entry void *and get referenced void *of */
/* next configuration entry in the enumerated list. */
/* When finished with this entry, an application must deref it */
/* by calling one of the following functions: */
/*   CfgEntryDeRef()          - Stop using the entry */
/*   CfgRemoveEntry()         - Stop using entry and remove it from cfg */
/*   CfgGetNextEntry()        - Stop using entry and get next entry */
/* Function returns 1 on success, 0 on "not found", and <0 on error. */
/*----------------------------------------------------------------------- */
extern int  CfgGetNextEntry( void *hCfg, void *hCfgEntry,
                             void **phCfgEntryNext );

/*----------------------------------------------------------------------- */
/* CfgGetImmediate() */
/* This function is intened for when an entry is known to most likely */
/* exist and is of a fixed size. It looks-up the entry, copies the data, */
/* and de-refs the entry all in one call. */
/* Returns the number of bytes copied. */
/*----------------------------------------------------------------------- */
extern int CfgGetImmediate( void *hCfg, uint32_t Tag, uint32_t Item,
                            uint32_t Instance, int MaxSize, unsigned char *pData );

/*----------------------------------------------------------------------- */
/* CfgEntryRef() */
/* Add a reference a configuration entry handle */
/* This function is called by an application when it intends to use an */
/* entry handle beyond the scope of the function which obtained it */
/* from the configuration. */
/* When finished with this entry, an application must deref it */
/* by calling one of the following functions: */
/*   CfgEntryDeRef()          - Stop using the entry */
/*   CfgRemoveEntry()         - Stop using entry and remove it from cfg */
/*   CfgGetNextEntry()        - Stop using entry and get next entry */
/* Returns 0 on success, <0 on error. */
/*----------------------------------------------------------------------- */
extern int  CfgEntryRef( void *hCfgEntry );

/*----------------------------------------------------------------------- */
/* CfgEntryDeRef(); */
/* Dereference a configuration entry handle */
/* Returns 0 on success, <0 on error. */
/*----------------------------------------------------------------------- */
extern int  CfgEntryDeRef( void *hCfgEntry );

/*----------------------------------------------------------------------- */
/* CfgEntryGetData() */
/* Get configuration entry user data */
/* the pointer can be zero). */
/* Returns the number of bytes written. If the supplied size is ZERO or */
/* too small, the function returns 0 and *pSize is set to the number of */
/* bytes required. Returns <0 on non-size related error. */
/*----------------------------------------------------------------------- */
extern int  CfgEntryGetData( void *hCfgEntry, int *pSize, unsigned char *pData );

/*----------------------------------------------------------------------- */
/* CfgEntrySetData() */
/* Set configuration entry user data */
/* Size is set to the size of item to replace. It must be an exact */
/* match for the current entry size. Also, no processing is done on */
/* the altered data. */
/* * USE WITH CARE * */
/* Returns the number of bytes written. If the supplied size doesn't */
/* match the old size, function returns 0. */
/* Returns <0 on non-size related error. */
/*----------------------------------------------------------------------- */
extern int  CfgEntrySetData( void *hCfgEntry, int Size, unsigned char *pData );

/*----------------------------------------------------------------------- */
/* CfgEntryInfo() */
/* Get configuration entry user data info */
/* Returns configuration size and data pointer. When either size or */
/* data information is not required, the pointer arguments can be NULL. */
/* Returns ZERO on successm or <0 on error */
/*----------------------------------------------------------------------- */
extern int  CfgEntryInfo( void *hCfgEntry, int *pSize, unsigned char **ppData );

/*----------------------------------------------------------------------- */
/* Config API Error Codes */
/*----------------------------------------------------------------------- */
#define CFGERROR_BADHANDLE      -1      /* Invalid Cfg handle */
#define CFGERROR_BADPARAM       -2      /* Invalid function parameter */
#define CFGERROR_RESOURCES      -3      /* Memory allocation error */
#define CFGERROR_REFERROR       -4      /* Reference count mismatch */
#define CFGERROR_ALREADY        -5      /* Already in desired state */
#define CFGERROR_SERVICE        -100    /* First service error */

#define CFG_MAKE_CFGERROR(x)     ((x)+CFGERROR_SERVICE)
#define CFG_GET_SERVICE_ERROR(x) ((x)-CFGERROR_SERVICE)
#define CFG_IS_SERVICE_ERROR(x)  ((x)<=CFGERROR_SERVICE)

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
