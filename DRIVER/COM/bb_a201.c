/*********************  P r o g r a m  -  M o d u l e ***********************
 *
 *         Name: bb_a201.c
 *      Project: A201/B201/B202 board handler
 *
 *       Author: uf
 *
 *  Description: A201/B201/B202 board handler routines
 *
 *
 *     Required: OSS, DESC libs
 *     Switches: _ONE_NAMESPACE_PER_DRIVER_
 *               A201_NTSW	: NT variant with swapped access
 *				 B201		: make B201 BBIS handler
 *				 B202		: make B202 BBIS handler
 *---------------------------[ Public Functions ]----------------------------
 *  
 *  A201_GetEntry          Gets the entry points of the BBIS handler functions. 
 *  B201_GetEntry          Gets the entry points of the BBIS handler functions. 
 *  B202_GetEntry          Gets the entry points of the BBIS handler functions. 
 *  
 * Note: functions will be renamed if B201/B202 switch is set
 *---------------------------------------------------------------------------
 * Copyright 1997-2019, MEN Mikro Elektronik GmbH
 ******************************************************************************/
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <MEN/mdis_com.h>
#include <MEN/men_typs.h>

/* pass debug definitions to dbg.h */
#define DBG_MYLEVEL		a201Hdl->debugLevel
#include <MEN/dbg.h>

#ifdef A201_NTSW
#	define MAC_MEM_MAPPED
#	define MAC_BYTESWAP
#	define _WIN_PCI2VME_BRIDGE_
#endif

/* swapped access */
#ifdef MAC_BYTESWAP
# define ID_SW
#endif

#include <MEN/oss.h>
#include <MEN/desc.h>
#include <MEN/mdis_err.h>
#include <MEN/maccess.h>
#include <MEN/mdis_api.h>
#include <MEN/modcom.h>

#include <MEN/bb_defs.h>
#include <MEN/bb_entry.h>
#include <MEN/bb_a201.h>

#ifdef B201
    static const char IdentString[]=MENT_XSTR_SFX(MAK_REVISION,B201);
#else
    #ifdef B202
        static const char IdentString[]=MENT_XSTR_SFX(MAK_REVISION,B202);
    #else
        #ifdef A201
            static const char IdentString[]=MENT_XSTR_SFX(MAK_REVISION,A201);
        #else
            static const char IdentString[]=MENT_XSTR(MAK_REVISION);
        #endif
    #endif
#endif 

/*-----------------------------------------+
|  DEFINES & CONST                         |
+------------------------------------------*/
#ifdef B201
# define A201_NBR_OF_SLOTS      1   /* number of module slots */
# define BBNAME					"B201"
#endif

#ifdef B202
# define A201_NBR_OF_SLOTS      2   /* number of module slots */
# define BBNAME					"B202"
#endif

#ifdef A201
# define A201_NBR_OF_SLOTS      4   /* number of module slots */
# define BBNAME					"A201"
#endif


#define A201_NBR_OF_RESOURCES  A201_NBR_OF_SLOTS /* number of used resources */
#define A201_ADDRSPACE OSS_ADDRSPACE_MEM	/*addres space type */

#ifdef _WIN_PCI2VME_BRIDGE_
#	define A201_BUSTYPE	OSS_BUSTYPE_PCI		/* PCI bustype for windows */
#else
#	define A201_BUSTYPE	OSS_BUSTYPE_VME		/* default: VME bustype */
#endif

#define BUS_0                  0

#define A201_CTRL_REG_1        0x100    /* offset of the first control register */
#define A201_CTRL_REG_SIZE     4

#define A201_CTRL_OFFS         0x00
#define A201_VECT_OFFS         0x02

#define A201_CTRL_DEFAULT      0x00            /* irq disable, level 0 */
#define A201_VECT_DEFAULT      0x0f            /* uninitialized interrupt */

#define A201_CTRL_IRQ_ENABLE   0x10

/*------------------ address spaces --------------------------*/
#define A201_SIZE               0x800
#define A201_M_A08_SIZE         0x100            /* A08 size */

/* debug handle */
#define DBH		a201Hdl->debugHdl

/*-----------------------------------------+
|  TYPEDEFS                                |
+------------------------------------------*/
typedef struct
{
    int32       ownMemSize;         /* own memory size */
    OSS_HANDLE  *osHdl;             /* os specific handle */
    u_int32     hardAddr;           /* no conversion from bus to local view */
    void        *physAddr;          /* a201 physical address - local view */
    u_int32     dataWidth;          /* VMEbus data width */
	                        /* mapped module control addresses - local view */
    void        *virtModCtrlAddr[A201_NBR_OF_SLOTS];    
                                                          
    u_int8      irqVector[A201_NBR_OF_SLOTS];    	/* irq vectors */
    u_int8      irqLevel[A201_NBR_OF_SLOTS];    	/* irq levels */
    u_int8      irqPriority[A201_NBR_OF_SLOTS];    	/* irq priorities */
    u_int8      forceFound[A201_NBR_OF_SLOTS];    	/* forces that modue was found */
#ifdef _WIN_PCI2VME_BRIDGE_
    u_int8      irqWin[A201_NBR_OF_SLOTS];    		/* irq for windows */
#endif
    u_int32     debugLevel;     /* debug level for BBIS */
	DBG_HANDLE* debugHdl;       /* debug handle */
	MDIS_IDENT_FUNCT_TBL idFuncTbl;	/* id function table */
    u_int32     busNbr;				/* bus number */
    OSS_RESOURCES res[A201_NBR_OF_RESOURCES]; /* assigned resources */
	int			resourcesAssigned;	/* flag: resources assigned */
#ifdef OSS_HAS_MAP_VME_ADDR
	void		*addrWinHdl;		/* address window handle */ 
#endif
} A201_BRD_STRUCT;

/*-----------------------------------------+
|  GLOBALS                                 |
+------------------------------------------*/

/*-----------------------------------------+
|  PROTOTYPES                              |
+------------------------------------------*/
static char* A201_Ident( void );

/* init/exit */
static int32 A201_Init( OSS_HANDLE  *osHdl,
                            DESC_SPEC   *descSpec,
                            BBIS_HANDLE **brdHdlP );

static int32 A201_BrdInit( BBIS_HANDLE *brdHdl );
static int32 A201_BrdExit( BBIS_HANDLE *brdHdl );
static int32 A201_Exit(    BBIS_HANDLE **brdHdlP );

/* info */
static int32 A201_BrdInfo(u_int32 code, ...);
static int32 A201_CfgInfo( BBIS_HANDLE *brdHdl,
                               u_int32 code,
                               ...);
/* interrupt handling */
static int32 A201_IrqEnable( BBIS_HANDLE *brdHdl,
                                 u_int32     mSlot,
                                 u_int32     enable );

static int32 A201_IrqSrvInit( BBIS_HANDLE *brdHdl,
                                  u_int32     mSlot   );


static void  A201_IrqSrvExit( BBIS_HANDLE *brdHdl,
                                  u_int32     mSlot   );

static int32 A201_ExpEnable( BBIS_HANDLE *brdHdl,
                                 u_int32     mSlot,
                                 u_int32     enable );

static int32 A201_ExpSrv( BBIS_HANDLE *brdHdl,
                                  u_int32 mSlot   );

/* get module address */
static int32 A201_SetMIface(  BBIS_HANDLE     *brdHdl,
                                  u_int32         mSlot,
                                  u_int32         addrMode,
                                  u_int32         dataMode );

static int32 A201_ClrMIface(  BBIS_HANDLE     *brdHdl,
                                  u_int32         mSlot );

static int32 A201_GetMAddr(   BBIS_HANDLE     *brdHdl,
                                  u_int32         mSlot,
                                  u_int32         addrMode,
                                  u_int32         dataMode,
                                  void            **mAddr,
                                  u_int32         *mSize );

/* getstat/setstat */
static int32 A201_SetStat(  BBIS_HANDLE     *brdHdl,
                                u_int32         mSlot,
                                int32           code,
                                INT32_OR_64     value32_or_64 );


static int32 A201_GetStat(  BBIS_HANDLE     *brdHdl,
                                u_int32         mSlot,
                                int32           code,
                                INT32_OR_64     *value32_or_64P );

static int32 A201_Unused( void );


static int32 CfgInfoSlot( BBIS_HANDLE *brdHdl, va_list argptr );

/*****************************  A201_Ident  *********************************
 *
 *  Description:  Gets the pointer to ident string.
 *
 *
 *---------------------------------------------------------------------------
 *  Input......:  -
 *
 *  Output.....:  return  pointer to ident string
 *
 *  Globals....:  -
 ****************************************************************************/
static char* A201_Ident( void )
{
	return( (char*) IdentString );
}

/**************************** A201_GetEntry *********************************
 *
 *  Description:  Gets the entry points of the BBIS handler functions.
 *
 *
 *---------------------------------------------------------------------------
 *  Input......:  bbisP  pointer to the inititialized bbis entry struct
 *
 *  Output.....:  *bbisP inititialized bbis entry struct
 *
 *  Globals....:  -
 *
 ****************************************************************************/
#ifdef _ONE_NAMESPACE_PER_DRIVER_
	extern void BBIS_GetEntry( BBIS_ENTRY *bbisP )
#else
    extern void __A201_GetEntry( BBIS_ENTRY *bbisP ) 
#endif
{
    bbisP->init         =   A201_Init;
    bbisP->brdInit      =   A201_BrdInit;
    bbisP->brdExit      =   A201_BrdExit;
    bbisP->exit         =   A201_Exit;
    bbisP->fkt04        =   A201_Unused;

    bbisP->brdInfo      =   A201_BrdInfo;
    bbisP->cfgInfo      =   A201_CfgInfo;
    bbisP->fkt07        =   A201_Unused;
    bbisP->fkt08        =   A201_Unused;
    bbisP->fkt09        =   A201_Unused;

    bbisP->irqEnable    =   A201_IrqEnable;
    bbisP->irqSrvInit   =   A201_IrqSrvInit;
    bbisP->irqSrvExit   =   A201_IrqSrvExit;
    bbisP->setIrqHandle =   NULL;
    bbisP->fkt14        =   A201_Unused;

    bbisP->expEnable    =   A201_ExpEnable;
    bbisP->expSrv       =   A201_ExpSrv;
    bbisP->fkt17        =   A201_Unused;
    bbisP->fkt18        =   A201_Unused;
    bbisP->fkt19        =   A201_Unused;

    bbisP->fkt20        =   A201_Unused;
    bbisP->fkt21        =   A201_Unused;
    bbisP->fkt22        =   A201_Unused;
    bbisP->fkt23        =   A201_Unused;
    bbisP->fkt24        =   A201_Unused;

    bbisP->setMIface    =   A201_SetMIface;
    bbisP->getMAddr     =   A201_GetMAddr;
    bbisP->setStat      =   A201_SetStat;
    bbisP->getStat      =   A201_GetStat;

    bbisP->clrMIface    =   A201_ClrMIface;
    bbisP->fkt30        =   A201_Unused;
    bbisP->fkt31        =   A201_Unused;
}/*A201_GetEntry*/


/****************************** A201_Init ***********************************
 *
 *  Description:  Allocates and initializes the board structure,
 *                decodes and stores descriptor data,
 *                assignes and maps the needed resources.
 *
 *                Use only PHYS_ADDR or VME_A16_ADDR or VME_A24_ADDR.
 *
 *                deskriptor key        default          range
 *
 *                DEBUG_LEVEL           DBG_OFF     	 see oss.h
 *
 *                PHYS_ADDR             (none)           see A201 and CPU description
 *                                                       no bus to local address
 *                                                       conversion supplied
 *
 *                VME_A16_ADDR          (none)           see A201 description
 *                                                       base addr of module slot 0
 *
 *                VME_A24_ADDR          (none)           see A201 description
 *                                                       base addr of module slot 0
 *
 *                IRQ_VECTOR            (none)           array of irq vectors
 *                                                       slot 0..4
 *
 *                IRQ_LEVEL             (none)           array of irq levels
 *                                                       slot 0..4
 *
 *                IRQ_PRIORITY          0,0,0,0          array of irq priorities
 *                                                       slot 0..4
 *
 *                PNP_FORCE_FOUND		0,0,0,0			 array of "force found"
 *                                                       slot 0..4
 *                  0: only a recognized M-Module will be reported
 *                  1: an M-Module will be reported for the slot
 *                     regardless if there is one plugged
 *
 *
 *                If switch _WIN_PCI2VME_BRIDGE_ is set, the following additional
 *                descriptor keys will be used for windows.
 *
 *                WIN_BUSNBR			(none)			windows bus number
 *
 *                WIN_IRQ				(none)			windows IRQ
 *
 *---------------------------------------------------------------------------
 *  Input......:  osHdl    pointer to os specific structure
 *                descSpec pointer to os specific descriptor specifier
 *                brdHdlP  pointer to variable where handle stored
 *
 *  Output.....:  *brdHdlP created handle
 *                return   0 | error code
 *
 *  Globals....:  -
 *
 ****************************************************************************/
static int32 A201_Init
(
    OSS_HANDLE      *osHdl,
    DESC_SPEC       *descSpec,
    BBIS_HANDLE     **brdHdlP
)
{
    u_int32     gotsize;
    int32       retCode;
    void 		*physAddr;  	        /* a201 physical address - local view */
    u_int32		temp;
    u_int32     a24Addr;    	    	/* a201 physical address A24 - VMEbus view*/
    u_int32     a16Addr;        		/* a201 physical address A16 - VMEbus view*/
    u_int32     dataWidth;          	/* VMEbus data width */
    u_int32     i;
    u_int8      defaultArr[A201_NBR_OF_SLOTS];
    u_int32     len;
    u_int32     debugLevelDesc;
    A201_BRD_STRUCT     *a201Hdl;
    DESC_HANDLE         *descHdlP;      /* descriptor handle pointer */

    /*-------------------------------+
    | initialize the board structure |
    +-------------------------------*/
    /* get memory for the board structure */
    a201Hdl = (A201_BRD_STRUCT*)( OSS_MemGet( osHdl, sizeof(A201_BRD_STRUCT),
                                              &gotsize ) );

    /* set board handle to the board structure */
    *brdHdlP = (BBIS_HANDLE*) a201Hdl;

    if ( a201Hdl == NULL )
        return( ERR_OSS_MEM_ALLOC );

    /* fill turkey with 0 */
    OSS_MemFill( osHdl, gotsize, (char*) a201Hdl, 0 );

    /* fill defaultArr with 0 */
    OSS_MemFill( osHdl, sizeof(defaultArr), (char*)defaultArr, 0 );

	/* prepare debugging */
	DBG_MYLEVEL = OSS_DBG_DEFAULT;	
	DBGINIT((NULL,&DBH));

    DBGWRT_1((DBH,"BB - A201_Init\n") );

    /* store data into the board structure */
    a201Hdl->ownMemSize = gotsize;
    a201Hdl->osHdl = osHdl;
	a201Hdl->busNbr = BUS_0;

    /*------------------------------+
    |  init id function table       |
    +------------------------------*/
	/* drivers ident function */
	a201Hdl->idFuncTbl.idCall[0].identCall = A201_Ident;
	/* libraries ident functions */
	a201Hdl->idFuncTbl.idCall[1].identCall = DESC_Ident;
	a201Hdl->idFuncTbl.idCall[2].identCall = OSS_Ident;
	/* terminator */
	a201Hdl->idFuncTbl.idCall[3].identCall = NULL;

    /*-----------------------------------+
    | get data from the board descriptor |
    +-----------------------------------*/
    /* init descHdl */
    retCode = DESC_Init( descSpec, osHdl, &descHdlP );
    if( retCode ) goto CLEANUP;

    /* get DEBUG_LEVEL_DESC */
    retCode = DESC_GetUInt32( descHdlP, DBG_OFF, &debugLevelDesc,
                             "DEBUG_LEVEL_DESC");
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND ) goto CLEANUP;
	DESC_DbgLevelSet(descHdlP, debugLevelDesc);

    /* get DEBUG_LEVEL */
    retCode = DESC_GetUInt32( descHdlP, DBG_OFF, &a201Hdl->debugLevel,
                             "DEBUG_LEVEL");
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND ) goto CLEANUP;

    /* hard address use - no conversion from VME bus view to local view */
    retCode = DESC_GetUInt32( descHdlP, 0, &temp, "PHYS_ADDR");
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND ) goto CLEANUP;
	physAddr = (void*)(U_INT32_OR_64)temp;

    /* A24 base address */
    retCode = DESC_GetUInt32( descHdlP, 0, &a24Addr, "VME_A24_ADDR");
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND ) goto CLEANUP;

    /* A16 base address */
    retCode = DESC_GetUInt32( descHdlP, 0, &a16Addr, "VME_A16_ADDR");
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND ) goto CLEANUP;

    /* VMEbus data width */
	if ((a24Addr || a16Addr) && 
        (retCode = DESC_GetUInt32( descHdlP, 0, (u_int32*)(&dataWidth), 
								  "VME_DATA_WIDTH")))
		goto CLEANUP;

    /* check one of the addr must be defined */
    if( !physAddr && !a24Addr && !a16Addr )
    {
        DBGWRT_ERR((DBH," *** %s_Init: no address in descriptor\n",BBNAME) );
        retCode = ERR_DESC_KEY_NOTFOUND;
        goto CLEANUP;
    }/*if*/

    retCode = 0; /* overwrite ERR_DESC_KEY_NOTFOUND*/

    if( physAddr )
    {
        a201Hdl->hardAddr = BBIS_BUS_TO_LOCAL_NO;
        DBGWRT_2((DBH,"   BBIS_BUS_TO_LOCAL_NO\n") );
    }
    else
    {
		u_int32	maxDataMode;
        a201Hdl->hardAddr = BBIS_BUS_TO_LOCAL;
        DBGWRT_2((DBH,"   BBIS_BUS_TO_LOCAL\n") );

		/* check and map dataWidth */
		/* 2=D24 not supported! */
		switch( dataWidth ){
			case 1 : maxDataMode = OSS_VME_DM_16; break;
			case 3 : maxDataMode = OSS_VME_DM_32; break;
			default:
				DBGWRT_ERR((DBH," *** %s_Init: illegal VME_DATA_WIDTH=%d\n",
					BBNAME, dataWidth) );
				retCode = ERR_OSS_VME_ILL_SPACE;
				goto CLEANUP;
		}

        if( a24Addr )
        {
            DBGWRT_2((DBH,"   A24 range\n") );
#ifdef OSS_HAS_MAP_VME_ADDR
			retCode = OSS_MapVmeAddr( osHdl, (u_int64)a24Addr, OSS_VME_AM_24UD,
							maxDataMode, A201_SIZE,	0x00,
							&physAddr, &a201Hdl->addrWinHdl );
#else
            retCode = OSS_BusToPhysAddr( osHdl, A201_BUSTYPE, &physAddr, a24Addr,
                                         OSS_VME_A24+(dataWidth<<8), A201_SIZE );
#endif
			if( retCode != 0 )
			{
				DBGWRT_ERR((DBH," *** %s_Init: OSS_MapVmeAddr()/OSS_BusToPhysAddr() failed\n",
							BBNAME) );
				goto CLEANUP;
			}/*if*/
        }
        else
        {
            DBGWRT_2((DBH,"   A16 range\n") );
#ifdef OSS_HAS_MAP_VME_ADDR
			retCode = OSS_MapVmeAddr( osHdl, (u_int64)a16Addr, OSS_VME_AM_16UD,
							maxDataMode, A201_SIZE,	0x00,
							&physAddr, &a201Hdl->addrWinHdl );
#else
            retCode = OSS_BusToPhysAddr( osHdl, A201_BUSTYPE, &physAddr, a16Addr,
                                         OSS_VME_A16+(dataWidth<<8), A201_SIZE );
#endif
            if( retCode != 0 )
            {
                DBGWRT_ERR((DBH," *** %s_Init: OSS_BusToPhysAddr() failed\n",
							BBNAME) );
                goto CLEANUP;
            }/*if*/
        }/*if*/
    }/*if*/

    a201Hdl->physAddr = physAddr;
    a201Hdl->dataWidth = dataWidth;

    /* irqVect of slot */
	len = A201_NBR_OF_SLOTS;
    retCode = DESC_GetBinary( descHdlP, defaultArr, len, a201Hdl->irqVector, 
							 &len, "IRQ_VECTOR");
    if( retCode != 0 || len != A201_NBR_OF_SLOTS )
    {
        DBGWRT_ERR((DBH," *** :DESC_GetBinary() IRQ_VECTOR ***\n") );
        goto CLEANUP;
    }/*if*/

    /* irqLevel of slot */
	len = A201_NBR_OF_SLOTS;
    retCode = DESC_GetBinary( descHdlP, defaultArr, len, a201Hdl->irqLevel, 
							 &len, "IRQ_LEVEL");
    if( retCode != 0 || len != A201_NBR_OF_SLOTS )
    {
        DBGWRT_ERR((DBH," *** :DESC_GetBinary() IRQ_LEVEL ***\n") );
        goto CLEANUP;
    }/*if*/

    /* irqPriority of slot (optional) */
	len = A201_NBR_OF_SLOTS;
    retCode = DESC_GetBinary( descHdlP, defaultArr, len, a201Hdl->irqPriority, 
							 &len, "IRQ_PRIORITY");
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND ) goto CLEANUP;

    /* PNP_FORCE_FOUND (optional) */
	len = A201_NBR_OF_SLOTS;
    retCode = DESC_GetBinary( descHdlP, defaultArr, len, a201Hdl->forceFound, 
							 &len, "PNP_FORCE_FOUND");
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND ) goto CLEANUP;

/* descriptor parameters required by windows (if PCI2VME bridge) */
#ifdef _WIN_PCI2VME_BRIDGE_
    /* get WIN_BUSNBR */
    retCode = DESC_GetUInt32( descHdlP, 0, &a201Hdl->busNbr,
                             "WIN_BUSNBR");
    if( retCode != 0 ) goto CLEANUP;

    /* get WIN_IRQ */
	len = A201_NBR_OF_SLOTS;
    retCode = DESC_GetBinary( descHdlP, defaultArr, len, a201Hdl->irqWin, 
							 &len, "WIN_IRQ");
    if( retCode != 0 || len != A201_NBR_OF_SLOTS )
    {
        DBGWRT_ERR((DBH," *** :DESC_GetBinary() WIN_IRQ ***\n") );
        goto CLEANUP;
    }/*if*/
#endif

    /* exit descHdl */
    retCode = DESC_Exit( &descHdlP );
    if( retCode )
    {
        DBGWRT_ERR((DBH," *** :DESC_Exit() ***\n") );
        goto CLEANUP;
    }/*if*/

    /*---------------------------+
    | assign the resources       |
    +----------------------------*/
    /* store memmory addresses in resource struct */
	for( i=0; i<A201_NBR_OF_SLOTS; i++ ) {
		a201Hdl->res[i].type = OSS_RES_MEM;
		a201Hdl->res[i].u.mem.physAddr = (void*) 
			((u_int8*)physAddr + A201_CTRL_REG_1 + i * 0x200);
		a201Hdl->res[i].u.mem.size = A201_CTRL_REG_SIZE;
	}

	/* assign the resources */
    retCode = OSS_AssignResources(osHdl, A201_BUSTYPE, a201Hdl->busNbr, 
								  A201_NBR_OF_RESOURCES, a201Hdl->res);
    if (retCode) {
		DBGWRT_ERR((DBH," *** %s_Init: OSS_AssignResources() failed (0x%x)\n",
					BBNAME,retCode));
		goto CLEANUP;
    }
	a201Hdl->resourcesAssigned = TRUE;

    /*----------------------------------------+
    | compute and map control and vector reg  |
    +----------------------------------------*/
    for( i=0; i<A201_NBR_OF_SLOTS; i++ )
    {
        retCode = OSS_MapPhysToVirtAddr( osHdl,
                                         (void*) ((u_int8*)physAddr + A201_CTRL_REG_1 + i * 0x200),
                                         A201_CTRL_REG_SIZE,
										 A201_ADDRSPACE,
                                         A201_BUSTYPE,
                                         a201Hdl->busNbr,
                                         &(a201Hdl->virtModCtrlAddr[i])
                                       );
        DBGWRT_2((DBH,"   slot %d  mapped to 0x%08x\n",
                                          i ,a201Hdl->virtModCtrlAddr[i] ) );
        if( retCode )
        {
            DBGWRT_ERR((DBH," *** %s_Init: OSS_MapPhysToVirtAddr() failed\n",
						BBNAME) );
            goto CLEANUP;
        }/*if*/
    }/*for*/


    return( retCode );

CLEANUP:
	if( descHdlP )
		DESC_Exit( &descHdlP );

	A201_Exit( brdHdlP );

	return( retCode );
}/*A201_Init*/

/****************************** A201_BrdInit ********************************
 *
 *  Description:  Initializes the base board.
 *                (control = 0x00 and vector = 0xff register)
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl   pointer to board handle structure
 *  Output.....:  0 | error code
 *  Globals....:  -
 ****************************************************************************/
static int32 A201_BrdInit
(
    BBIS_HANDLE     *brdHdl
)
{
    DBGCMD( A201_BRD_STRUCT  *a201Hdl = (A201_BRD_STRUCT*)brdHdl; )
    DBGWRT_1( (DBH,"BB - %s_BrdInit\n",BBNAME) );

    /*------------------------------+
    | identify board with EEPROM Id |
    +-------------------------------*/
    /* there is no id-prom */

    /*------------------------------+
    | init global registers         |
    +-------------------------------*/
    /* there are no global registers */

    return( 0 );
}/*A201_BrdInit*/


/****************************** A201_BrdExit ********************************
 *
 *  Description:  Deinitialize the board.
 *
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl   pointer to board handle structure
 *  Output.....:  0 | error code
 *  Globals....:  -
 ****************************************************************************/
static int32 A201_BrdExit
(
    BBIS_HANDLE     *brdHdl
)
{
    DBGCMD( A201_BRD_STRUCT  *a201Hdl = (A201_BRD_STRUCT*)brdHdl; )
    DBGWRT_1( (DBH,"BB - %s_BrdExit\n",BBNAME) );

    /*------------------------------+
    | cleanup global registers      |
    +-------------------------------*/
    /* there are no global registers */

    return( 0 );
}/*A201_BrdExit*/


/****************************** A201_Exit ***********************************
 *
 *  Description:  Releases memory of the board structure and
 *                unmaps control register addresses.
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdlP  pointer to variable where handle stored
 *
 *  Output.....:  *brdHdlP NULL
 *                return   0 | error code
 *
 *  Globals....:  -
 *
 ****************************************************************************/
static int32 A201_Exit
(
    BBIS_HANDLE     **brdHdlP
)
{
    A201_BRD_STRUCT *a201Hdl = (A201_BRD_STRUCT*) *brdHdlP;
    u_int32         i;

    DBGWRT_1( (DBH,"BB - %s_Exit\n",BBNAME) );

    /*----------------------------------------+
    | unmap control and vector reg            |
    +----------------------------------------*/
    for( i=0; i<A201_NBR_OF_SLOTS; i++ )
    {
        if( a201Hdl->virtModCtrlAddr[i] )
           OSS_UnMapVirtAddr( a201Hdl->osHdl, &a201Hdl->virtModCtrlAddr[i],
							A201_CTRL_REG_SIZE, A201_ADDRSPACE );
    }/*for*/

#ifdef OSS_HAS_UNASSIGN_RESOURCES
	if( a201Hdl->resourcesAssigned ){
		/* unassign the resources */
		OSS_UnAssignResources( a201Hdl->osHdl, OSS_BUSTYPE_VME,a201Hdl->busNbr,
							   A201_NBR_OF_RESOURCES, a201Hdl->res );
	}
#endif /* OSS_HAS_UNASSIGN_RESOURCES */

#ifdef OSS_HAS_MAP_VME_ADDR
	if( a201Hdl->addrWinHdl )
		OSS_UnMapVmeAddr( a201Hdl->osHdl, a201Hdl->addrWinHdl );
#endif

	/* cleanup debug */
	DBGEXIT((&DBH));

    /* release memory for the board handle */
    OSS_MemFree( a201Hdl->osHdl, (int8*)*brdHdlP, a201Hdl->ownMemSize);

    *brdHdlP = NULL;

    return 0;
}/*A201_Exit*/


/****************************** A201_BrdInfo ********************************
 *
 *  Description:  Get information about hardware and driver requirements.
 *
 *                Following info codes are supported:
 *
 *                Code                      Description
 *                ------------------------  -----------------------------
 *                BBIS_BRDINFO_BUSTYPE      board bustype
 *                BBIS_BRDINFO_DEVBUSTYPE   device bustype
 *                BBIS_BRDINFO_FUNCTION     used optional functions
 *                BBIS_BRDINFO_NUM_SLOTS    number of slots
 *				  BBIS_BRDINFO_INTERRUPTS   interrupt characteristics
 *                BBIS_BRDINFO_ADDRSPACE    address characteristic
 *				  BBIS_BRDINFO_BRDNAME		name of the board hardware
 *
 *                The BBIS_BRDINFO_BUSTYPE code returns the bustype of
 *                the specified board. (here always PCI)
 *
 *                The BBIS_BRDINFO_DEVBUSTYPE code returns the bustype of
 *                the specified device - not the board bus type. 
 * 				  (here always NONE)
 *
 *                The BBIS_BRDINFO_FUNCTION code returns the information
 *                if an optional BBIS function is supported or not.
 *
 *                The BBIS_BRDINFO_NUM_SLOTS code returns the number of
 *                devices used from the driver. (CHAMELEON BBIS: always 16)
 *
 *                The BBIS_BRDINFO_INTERRUPTS code returns the supported
 *                interrupt capability (BBIS_IRQ_DEVIRQ/BBIS_IRQ_EXPIRQ)
 *                of the specified device.
 *
 *                The BBIS_BRDINFO_ADDRSPACE code returns the address
 *                characteristic (OSS_ADDRSPACE_MEM/OSS_ADDRSPACE_IO)
 *                of the specified device.
 *
 *				  The BBIS_BRDINFO_BRDNAME code returns the short hardware
 *                name and type of the board without any prefix or suffix.
 *                The hardware name must not contain any non-printing
 *                characters. The length of the returned string, including
 *                the terminating null character, must not exceed
 *                BBIS_BRDINFO_BRDNAME_MAXSIZE.
 *                Examples: D201 board, PCI device, Chameleon FPGA 
 *
 *---------------------------------------------------------------------------
 *  Input......:  code      reference to the information we need    
 *                ...       variable arguments                      
 *  Output.....:  *...      variable arguments
 *                return    0 | error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 A201_BrdInfo
(
    u_int32 code,
    ...
)
{
    va_list     argptr;
    int32       retCode;

    retCode = 0;
    va_start(argptr,code);

    switch( code )
    {
        /* bus type */
        case BBIS_BRDINFO_BUSTYPE:
        {
            u_int32 *busType = va_arg( argptr, u_int32* );

            *busType = A201_BUSTYPE;
            break;
        }
		/* device bus type */
		case BBIS_BRDINFO_DEVBUSTYPE:
		{
			u_int32 mSlot       = va_arg( argptr, u_int32 );
			u_int32 *devBusType = va_arg( argptr, u_int32* );

			mSlot = mSlot; /* dummy access to avoid compiler warning */
			*devBusType = OSS_BUSTYPE_MMODULE;
			break;
		}
        /* supported functions */
        case BBIS_BRDINFO_FUNCTION:
        {
            u_int32 funcCode = va_arg( argptr, u_int32 );
            u_int32 *statusP = va_arg( argptr, u_int32* );

			switch( funcCode )
			{
				/* supported */
				case BBIS_FUNC_IRQENABLE:
				case BBIS_FUNC_IRQSRVINIT:
				case BBIS_FUNC_IRQSRVEXIT:
	                *statusP = TRUE;
	                break;

	            default:
	            	*statusP = FALSE;
            		break;
            }
            break;
        }
        /* number of module slots */
        case BBIS_BRDINFO_NUM_SLOTS:
        {
            u_int32 *numSlotP = va_arg( argptr, u_int32* );

            *numSlotP = A201_NBR_OF_SLOTS;
            break;
        }
        /* interrupt capability */
        case BBIS_BRDINFO_INTERRUPTS:
        {
            u_int32 mSlot = va_arg( argptr, u_int32 );
            u_int32 *irqP = va_arg( argptr, u_int32* );

			*irqP = BBIS_IRQ_DEVIRQ;
			mSlot = mSlot;  /* dummy access to suppress compiler warning */
            break;
        }
        /* address space type */
        case BBIS_BRDINFO_ADDRSPACE:
        {
            u_int32 mSlot      = va_arg( argptr, u_int32 );
            u_int32 *addrSpace = va_arg( argptr, u_int32* );

			*addrSpace = A201_ADDRSPACE;
			mSlot = mSlot;  /* dummy access to suppress compiler warning */
            break;
        }
		/* board name */
		case BBIS_BRDINFO_BRDNAME:
		{
			char	*brdName = va_arg( argptr, char* );
			char	*from;

			/* 
			 * build hw name (e.g. A201 board)
			 */  
			from = BBNAME;
			while( (*brdName++ = *from++) );	/* copy string */
			from = " board";
			while( (*brdName++ = *from++) );	/* copy string */
			
			break;
		}

        /* error */
        default:
            va_end( argptr );
            retCode = ERR_BBIS_UNK_CODE;
    }/*if*/

    va_end( argptr );

    return( retCode );
}

/****************************** A201_CfgInfo ********************************
 *
 *  Description:  Get information about board configuration.
 *
 *                Following info codes are supported:
 *
 *                Code                      Description
 *                ------------------------  ------------------------------
 *                BBIS_CFGINFO_BUSNBR       bus number
 *                BBIS_CFGINFO_IRQ          interrupt parameters
 *                BBIS_CFGINFO_EXP          exception interrupt parameters
 *                BBIS_CFGINFO_SLOT			slot information
 *
 *                The BBIS_CFGINFO_BUSNBR code returns the number of the
 *                bus on which the board resides.
 *
 *                The BBIS_CFGINFO_IRQ code returns the device interrupt
 *                vector, level and mode of the specified device.
 *
 *                The BBIS_CFGINFO_EXP code returns the exception interrupt
 *                vector, level and mode of the specified device.
 *
 *                The BBIS_CFGINFO_SLOT code returns the following
 *                information about the specified device slot:
 *                The slot is occupied or empty, the device id and device
 *                revision of the plugged device, the name of the slot and
 *                the name of the plugged device.
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure       
 *                code      reference to the information we need    
 *                ...       variable arguments                      
 *  Output.....:  ...       variable arguments
 *                return    0 | error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 A201_CfgInfo
(
    BBIS_HANDLE     *brdHdl,
    u_int32         code,
    ...
)
{
    va_list             argptr;
    A201_BRD_STRUCT     *a201Hdl = (A201_BRD_STRUCT*)brdHdl;
    int32				retCode;

    retCode = 0;
    DBGWRT_1((DBH,"BB - %s_CfgInfo\n",BBNAME) );

    va_start(argptr,code);

    switch( code )
    {
        /* bus number */
        case BBIS_CFGINFO_BUSNBR:
        {
            u_int32 *busNbrP = va_arg( argptr, u_int32* );
            *busNbrP = a201Hdl->busNbr;
            break;
        }

        /* interrupt information */
        case BBIS_CFGINFO_IRQ:
        {
            u_int32 mSlot   = va_arg( argptr, u_int32 );
            u_int32 *vectorP= va_arg( argptr, u_int32* );
            u_int32 *levelP = va_arg( argptr, u_int32* );
            u_int32 *modeP  = va_arg( argptr, u_int32* );

#ifdef _WIN_PCI2VME_BRIDGE_
            *vectorP = a201Hdl->irqWin[mSlot];
            *levelP  = a201Hdl->irqWin[mSlot];
#else
            *vectorP = a201Hdl->irqVector[mSlot];
            *levelP  = a201Hdl->irqLevel[mSlot];
#endif
            *modeP   = BBIS_IRQ_EXCLUSIVE;
            break;
        }

        /* exception interrupt information */
        case BBIS_CFGINFO_EXP:
        {
            u_int32 mSlot    = va_arg( argptr, u_int32 );
            u_int32 *vectorP = va_arg( argptr, u_int32* );
            u_int32 *levelP  = va_arg( argptr, u_int32* );
            u_int32 *modeP   = va_arg( argptr, u_int32* );

            /* no extra exception interrupt */
            *modeP = 0;
            mSlot    = mSlot;    /* dummy access to suppress compiler warning */
            *vectorP = *vectorP; /* dummy access to suppress compiler warning */
            *levelP  = *levelP;  /* dummy access to suppress compiler warning */
            break;
        }

		/* slot information for PnP support*/
		case BBIS_CFGINFO_SLOT:
		{
			retCode = CfgInfoSlot( brdHdl, argptr ); 
			break;
		}

        default:
            DBGWRT_ERR((DBH," *** %s_CfgInfo: code=0x%x not supported\n",
						BBNAME,code));
            retCode = ERR_BBIS_UNK_CODE;
    }/*switch*/

    va_end( argptr );
    return( retCode );
}/*A201_CfgInfo*/


/****************************** A201_IrqEnable *********************************
 *
 *  Description:  En/disable interrupt for module slot 'mSlot' and
 *                writes vector and control register
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure
 *                mSlot     module slot number
 *                enable    interrupt setting
 *  Output.....:  return    0 | error code
 *  Globals....:  ---
 ****************************************************************************/
static int32 A201_IrqEnable
(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot,
    u_int32         enable
)
{
    u_int16          control = 0;
    A201_BRD_STRUCT  *a201Hdl = (A201_BRD_STRUCT*)brdHdl;


    DBGWRT_1( (DBH,"BB - %s_IrqEnable: slot=%d enable=%d\n", 
			   BBNAME,mSlot, enable) );

    if (mSlot >= A201_NBR_OF_SLOTS)
        return( ERR_BBIS_ILL_SLOT );

    control = a201Hdl->irqLevel[mSlot];

    if( enable )
    {
       control |=  A201_CTRL_IRQ_ENABLE;          /* enable irq */
    }/*if*/

    MWRITE_D16( (a201Hdl->virtModCtrlAddr[mSlot]), A201_VECT_OFFS, a201Hdl->irqVector[mSlot] );
    MWRITE_D16( (a201Hdl->virtModCtrlAddr[mSlot]), A201_CTRL_OFFS, control );

    return( 0 );
}/*A201_IrqEnable*/


/****************************** A201_IrqSrvInit *****************************
 *
 *  Description:  This function is called at the beginning of an interrupt
 *                handler.
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure
 *                mSlot     module slot number
 *
 *  Output.....:  return    BBIS_IRQ_UNK
 *
 *  Globals....:  ---
 ****************************************************************************/
static int32 A201_IrqSrvInit
(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot
)
{
    DBGCMD( A201_BRD_STRUCT  *a201Hdl = (A201_BRD_STRUCT*)brdHdl );
    DBGWRT_1( (DBH,"BB - %s_IrqSrvInit: slot=%d\n", BBNAME,mSlot) );

    return( BBIS_IRQ_UNK );
}/*A201_IrqSrvInit*/


/****************************** A201_IrqSrvExit *****************************
 *
 *  Description:  This function is called at the end of an interrupt
 *                handler.
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure
 *                mSlot     module slot number
 *  Output.....:  ---
 *  Globals....:  ---
 ****************************************************************************/
static void A201_IrqSrvExit
(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot
)
{
    DBGCMD( A201_BRD_STRUCT  *a201Hdl = (A201_BRD_STRUCT*)brdHdl; )
    DBGWRT_1( (DBH,"BB - %s_IrqSrvExit: slot=%d\n", BBNAME,mSlot) );
}/*A201_IrqSrvExit*/


/****************************** A201_ExpEnable ******************************
 *
 *  Description:  Exception interrupt enable / disable.
 *
 *                Do nothing
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure   
 *                mSlot     module slot number                  
 *                enable    interrupt setting                   
 *  Output.....:  return    0
 *  Globals....:  ---
 ****************************************************************************/
static int32 A201_ExpEnable(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot,
	u_int32			enable)
{
    DBGCMD( A201_BRD_STRUCT  *a201Hdl = (A201_BRD_STRUCT*)brdHdl; )
    DBGWRT_1( (DBH,"BB - %s_ExpEnable: slot=%d\n", BBNAME,mSlot) );

	return 0;
}/*A201_ExpEnable*/


/****************************** A201_ExpSrv *********************************
 *
 *  Description:  Called at the beginning of an exception interrupt.
 *
 *                Do nothing
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure   
 *                mSlot     module slot number                  
 *  Output.....:  return    BBIS_IRQ_NO
 *  Globals....:  ---
 ****************************************************************************/
static int32 A201_ExpSrv(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot )
{
    DBGCMD( A201_BRD_STRUCT  *a201Hdl = (A201_BRD_STRUCT*)brdHdl; )
    DBGWRT_1( (DBH,"BB - %s_ExpSrv: slot=%d\n", BBNAME,mSlot) );

	return BBIS_IRQ_NO;
}/*A201_ExpSrv*/


/****************************** A201_SetMIface ******************************
 *
 *  Description:  This function configure the M-Module interface for slot
 *                'mSlot'according to the M-Module characteristic.
 *                Call this function once per slot.
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure
 *                mSlot     module slot number
 *                addrMode  MDIS_MODE_A08
 *                dataMode  MDIS_MODE_D08 | MDIS_MODE_D16
 *  Output.....:  0 | error code
 *  Globals....:  -
 ****************************************************************************/
static int32 A201_SetMIface
(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot,
    u_int32         addrMode,
    u_int32         dataMode
)
{
    A201_BRD_STRUCT *a201Hdl = (A201_BRD_STRUCT*)brdHdl;

    DBGWRT_1( (DBH,"BB - %s_SetMIface: slot=%d\n", BBNAME, mSlot) );

    if (mSlot >= A201_NBR_OF_SLOTS)
        return( ERR_BBIS_ILL_SLOT );

    /*-------------------------------+
    | set data mode                  |
    +-------------------------------*/
    /* D16 module or D8 module*/
    if( (dataMode & MDIS_MD16) || (dataMode & MDIS_MD08) )
    {
    }
    else
    {
        /* only D32 : we have a problem */
        DBGWRT_ERR((DBH," *** %s_SetMIface: data mode=0x%x not supported\n", 
				  BBNAME,dataMode) );
       return( ERR_BBIS_ILL_DATAMODE );
    }/*if*/


    /*-------------------------------+
    | set address mode               |
    +-------------------------------*/
    /* A08 module */
    if( (addrMode & MDIS_MA08) )
    {
    }
    else
    {
        DBGWRT_ERR((DBH," *** %s_SetMIface: addr mode=0x%x not supported\n",
					BBNAME,addrMode) );
        return( ERR_BBIS_ILL_ADDRMODE );
    }/*if*/

    /*------------------------------+
    | init slot registers           |
    +-------------------------------*/
    /* disable interrupts / default vector */
    MWRITE_D16( (a201Hdl->virtModCtrlAddr[mSlot]), A201_CTRL_OFFS, A201_CTRL_DEFAULT );
    MWRITE_D16( (a201Hdl->virtModCtrlAddr[mSlot]), A201_VECT_OFFS, A201_VECT_DEFAULT );

    /* all was ok */
    return( 0 );
}/*A201_SetMIface*/


/****************************** A201_ClrMIface ******************************
 *
 *  Description:  Cleanup the M-Module interface for slot 'mSlot'
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure
 *                mSlot     module slot number
 *  Output.....:  0 | error code
 *  Globals....:  -
 ****************************************************************************/
static int32 A201_ClrMIface
(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot
)
{
    A201_BRD_STRUCT *a201Hdl = (A201_BRD_STRUCT*)brdHdl;

    DBGWRT_1( (DBH,"BB - %s_ClrMIface: slot=%d\n", BBNAME,mSlot) );

    if (mSlot >= A201_NBR_OF_SLOTS)
        return( ERR_BBIS_ILL_SLOT );

    /*------------------------------+
    | cleanup slot registers        |
    +-------------------------------*/
    /* disable interrupts / default vector */
    MWRITE_D16( (a201Hdl->virtModCtrlAddr[mSlot]), A201_CTRL_OFFS, A201_CTRL_DEFAULT );
    MWRITE_D16( (a201Hdl->virtModCtrlAddr[mSlot]), A201_VECT_OFFS, A201_VECT_DEFAULT );

    /* all was ok */
    return( 0 );
}/*A201_ClrMIface*/


/****************************** A201_GetMAddr *******************************
 *
 *  Description:  This function suply a pointer to the physical address space
 *                and the size of this address space for the specified module
 *                slot, address mode and data mode.
 *
 *---------------------------------------------------------------------------
 *  Input......:  brdHdl    pointer to board handle structure
 *                mSlot     module slot number
 *                addrMode  MDIS_MODE_A08
 *                dataMode  MDIS_MODE_D08 | MDIS_MODE_D16
 *
 *  Output.....:  mAddrP    pointer to address space
 *                mSizeP    size of address space
 *                return    0 | error code
 *
 *  Globals....:  -
 *
 ****************************************************************************/
static int32 A201_GetMAddr
(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot,
    u_int32         addrMode,
    u_int32         dataMode,
    void            **mAddrP,
    u_int32         *mSizeP
)
{

    A201_BRD_STRUCT *a201Hdl = (A201_BRD_STRUCT*)brdHdl;

    DBGWRT_1( (DBH,"BB - %s_GetMAddr: slot=%d\n", BBNAME, mSlot) );

    if (mSlot >= A201_NBR_OF_SLOTS)
        return( ERR_BBIS_ILL_SLOT );

    *mAddrP  = NULL;
    *mSizeP  = 0;

    /*---------------------------+
    | depend on address mode     |
    +---------------------------*/
    switch( addrMode )
    {
        /* A08 */
        case MDIS_MA08 :
            *mSizeP = A201_M_A08_SIZE;
            break;

        default:
            DBGWRT_ERR((DBH," *** %s_GetMAddr: addr mode=0x%x not supported\n",
						BBNAME, addrMode) );
            return( ERR_BBIS_ILL_ADDRMODE );
    }/*switch*/


    /*------------------------+
    | depend on data mode     |
    +------------------------*/
    switch( dataMode )
    {
        /* D16 or D08 */
        case MDIS_MD08 :
        case MDIS_MD16 :
            DBGWRT_2((DBH," supply D08/D16 data mode\n") );
            break;

        default:
            DBGWRT_ERR((DBH," *** %s_GetMAddr: data mode=0x%x not supported\n",
						BBNAME,dataMode) );
            return( ERR_BBIS_ILL_DATAMODE );
    }/*switch*/

    *mAddrP = (void*) ( (u_int8*)a201Hdl->physAddr + (0x200 * mSlot) );

    return( 0 );
}/*A201_GetMAddr*/

/****************************** A201_SetStat ********************************
 *
 *  Description:  Sets the board state.
 *
 *     supported  codes                     values
 *                M_BB_DEBUG_LEVEL          see oss.h
 *
 *---------------------------------------------------------------------------
 *
 *  Input......:  brdHdl  			handle to board data
 *                mSlot   			modul index
 *                code    			setstat code
 *                value32_or_64   	setstat data or blocksetstat data pointer
 *
 *  Output.....:  return  0 | error code
 *
 *  Globals....:  -
 *
 ****************************************************************************/
static int32 A201_SetStat
(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot,
    int32           code,
    INT32_OR_64     value32_or_64
)
{
    A201_BRD_STRUCT *a201Hdl = (A201_BRD_STRUCT*)brdHdl;
    int32 			value = (int32) value32_or_64;  /* 32bit value */

    DBGWRT_1((DBH,"BB - %s_SetStat: mSlot=%d code=0x%04x\n", BBNAME,
			  mSlot, code) );

    if (mSlot >= A201_NBR_OF_SLOTS)
        return( ERR_BBIS_ILL_SLOT );

    switch(code)
    {
        /* set debug level */
        case M_BB_DEBUG_LEVEL:
            a201Hdl->debugLevel = value;
            break;

        /* unknown */
        default:
            return( ERR_BBIS_UNK_CODE );
    }/*switch*/

    return( 0 );
}/*switch*/


/****************************** A201_GetStat ********************************
 *
 *  Description:  Gets the board state.
 *
 *     supported  codes                     values
 *                M_BB_IRQ_VECT				0..255
 *                M_BB_IRQ_LEVEL			0..255
 *                M_BB_IRQ_PRIORITY			0..255
 *                M_BB_ID_CHECK             0
 *                M_BB_DEBUG_LEVEL          see oss.h
 *                M_MK_BLK_REV_ID           -
 *
 *---------------------------------------------------------------------------
 *
 *  Input......:  brdHdl  handle to board data
 *                mSlot   modul index
 *                code    getstat code
 *
 *  Output.....:  value32_or_64P  getstat data pointer or blockgetstat data pointer
 *                return  0 | error code
 *
 *  Globals....:  -
 *
 ****************************************************************************/
static int32 A201_GetStat 
(
    BBIS_HANDLE     *brdHdl,
    u_int32         mSlot,
    int32           code,
    INT32_OR_64     *value32_or_64P
)
{
    A201_BRD_STRUCT *a201Hdl = (A201_BRD_STRUCT*)brdHdl;
    int32			*valueP  = (int32*) value32_or_64P; /*pointer to 32 bit val*/

    DBGWRT_1((DBH,"BB - %s_GetStat: mSlot=%d code=0x%04x\n", BBNAME, 
			  mSlot, code) );

    if (mSlot >= A201_NBR_OF_SLOTS)
        return( ERR_BBIS_ILL_SLOT );

    switch( code )
    {
        /*------------------+
        |   IRQ vector      |
        +------------------*/
        case M_BB_IRQ_VECT:
            *valueP = a201Hdl->irqVector[mSlot];
            break;

        /*------------------+
        |   IRQ level       |
        +------------------*/
        case M_BB_IRQ_LEVEL:
            *valueP = a201Hdl->irqLevel[mSlot];
            break;

        /*------------------+
        |   IRQ priority    |
        +------------------*/
        case M_BB_IRQ_PRIORITY:
            *valueP = a201Hdl->irqPriority[mSlot];
            break;

        /*------------------+
        |  debug level      |
        +------------------*/
        case M_BB_DEBUG_LEVEL:
            *valueP = a201Hdl->debugLevel;
            break;

        /*------------------+
        |  id check enabled |
        +------------------*/
        case M_BB_ID_CHECK:
            *valueP = 0;
            break;

        /*--------------------+
        |  ident table        |
        +--------------------*/
        case M_MK_BLK_REV_ID:
           *value32_or_64P = (INT32_OR_64)&a201Hdl->idFuncTbl;
           break;

        /* unknown */
        default:
            return ERR_BBIS_UNK_CODE;
    }/*switch*/

    return( 0 );
}/*A201_GetStat*/


/****************************** A201_Unused *********************************
 *
 *  Description:  Dummy function.
 *
 *
 *---------------------------------------------------------------------------
 *  Input......:  -
 *  Output.....:  return 0
 *  Globals....:  -
 ****************************************************************************/
static int32 A201_Unused
(
    void
)
{
    return( ERR_BBIS_ILL_FUNC );
}/*A201_Unused*/

/********************************* CfgInfoSlot ******************************
 *
 *  Description:  Fulfils the BB_CfgInfo(BBIS_CFGINFO_SLOT) request
 * 
 *				  The variable-argument list (argptr) contains the following
 *                parameters in the given order:
 *
 *                Input
 *                -----
 *                mSlot (u_int32) - device slot number  
 *
 *                Output
 *                ------
 *                occupied (u_int32*) - occupied information
 *                 - pluggable device: 
 *                   BBIS_SLOT_OCCUP_YES if slot is occupied
 *                   or BBIS_SLOT_OCCUP_NO if slot is empty
 *                 - onboard device: 
 *                   BBIS_SLOT_OCCUP_ALW if device is enabled
 *                   or BBIS_SLOT_OCCUP_DIS if device is disabled
 *
 *                devId (u_int32*) - device id (4-byte hex value)
 *                  The device id should identify the type of the device
 *                  but should not contain enough information to differentiate
 *                  between two identical devices. If the device id is unknown
 *                  BBIS_SLOT_NBR_UNK must be returned. 
 *                  - M-Module:
 *                    id-prom-magic-word << 16 | id-prom-module-id
 *                    Example: 0x53460024
 *                    Note: The returned device id must be identical to the
 *                          "autoid" value in the device drivers xml file.  
 *
 *                devRev (u_int32*) - device revision (4-byte hex value)
 *                  - M-Module: id-prom-layout-revision << 16 |
 *                              id-prom-product-variant
 *                              example: 0x01091400       
 *                    or BBIS_SLOT_NBR_UNK if device revision is unknown
 *
 *                slotName (char*) - slot name
 *                  The slot name should consist of the slot type and the
 *                  slot label but must not contain any non-printing
 *                  characters.
 *                  The length of the returned string, including the
 *                  terminating null character, must not exceed
 *                  BBIS_SLOT_STR_MAXSIZE.
 *                  format : "<slot type> <slot label>"
 *
 *                  Examples:
 *                  - M-Module:		"M-Module slot 0"
 *
 *                devName (char*) - device name
 *                  The device name should identify the type of the device
 *                  but should not contain enough information to differentiate
 *                  between two identical devices. Furthermore, the device
 *                  name should refer to the appropriate device driver name if
 *                  possible.
 *
 *                  The returned string must not contain any non-printing
 *                  characters or blanks. The length of the returned string,
 *                  including the terminating null character, must not exceed
 *                  BBIS_SLOT_STR_MAXSIZE.
 * 
 *                  Examples:
 *                  - M-Module:		"M34", "MS9"
 *                  
 *                  If the device name is unknown BBIS_SLOT_STR_UNK must
 *                  be returned.
 *
 *                  Note: The returned device name must be identical to the
 *                        "hwname" value in the device drivers xml file.
 *
 *---------------------------------------------------------------------------
 *  Input......: brdHdl			handle
 *				 argptr			argument pointer
 *  Output.....: return			error code
 *  Globals....: -
 ****************************************************************************/
static int32 CfgInfoSlot( BBIS_HANDLE *brdHdl, va_list argptr )	/* nodoc */
{
	int32	status, status2;
	u_int32	modAddrSize;
	void	*physModAddr, *virtModAddr;
	u_int32 modType;

    A201_BRD_STRUCT *a201Hdl = (A201_BRD_STRUCT*)brdHdl;

	u_int32 mSlot     = va_arg( argptr, u_int32 );
    u_int32 *occupied = va_arg( argptr, u_int32* );
    u_int32 *devId    = va_arg( argptr, u_int32* );
    u_int32 *devRev   = va_arg( argptr, u_int32* );
	char	*slotName = va_arg( argptr, char* );
	char	*devName  = va_arg( argptr, char* );

	/* clear parameters to return */
	*occupied = BBIS_SLOT_OCCUP_DIS;
	*devId    = BBIS_SLOT_NBR_UNK;
	*devRev   = BBIS_SLOT_NBR_UNK;
	*slotName = '\0';
	*devName  = '\0';

	/* illegal slot? */
	if( mSlot > (A201_NBR_OF_SLOTS-1) ){
		DBGWRT_ERR((DBH,"*** %s_CfgInfoSlot: wrong module slot number=0x%x\n",
					BBNAME,mSlot));
		return ERR_BBIS_ILL_SLOT;
	}

	/*
	 * build slot name
	 */
	OSS_Sprintf( a201Hdl->osHdl, slotName, "M-Module slot %d", mSlot);

	/* force found? */
	if( a201Hdl->forceFound[mSlot] ){
		*occupied = BBIS_SLOT_OCCUP_YES;
		return ERR_SUCCESS;
	}

	/*------------------------------+
	| get module information        |
	+------------------------------*/
	/* get physical module addr */
	if( (status = A201_GetMAddr( brdHdl, mSlot, MDIS_MA08, MDIS_MD16,
								 &physModAddr, &modAddrSize )) ){
		return status;
	}
	
	/* init module slot */
	if( (status = A201_SetMIface( brdHdl, mSlot, MDIS_MA08, MDIS_MD16 )) ){
		return status;
	}
	
	/*
	 * map module address space
	 * (quick and dirty without resource assignment)
	 */ 
	if( (status = OSS_MapPhysToVirtAddr( a201Hdl->osHdl, physModAddr, modAddrSize,
										 OSS_ADDRSPACE_MEM, OSS_BUSTYPE_PCI,
										 a201Hdl->busNbr, &virtModAddr )) ){
		A201_ClrMIface( brdHdl, mSlot );
		return status;
	}

	/* now get the information */
	m_getmodinfo( (U_INT32_OR_64)virtModAddr, &modType,
				  devId, devRev, devName );

	DBGWRT_2((DBH," modType=%d, devId=0x%08x, devRev=0x%08x, devName=%s\n",
				  modType, *devId, *devRev, devName ));

	/*
	 * MEN M-Module detected?
	 * Note: modType=MODCOM_MOD_THIRD (magic-id <> 0x5346) is not safe for the
	 *       detection without a DTACK timeout interrupt. Therefore, we support
	 *       here only MEN M-Modules (magic-id=0x5346).
	 */
	if( modType == MODCOM_MOD_MEN )
		*occupied = BBIS_SLOT_OCCUP_YES;	
	/* no MEN M-Module detected */
	else
		*occupied = BBIS_SLOT_OCCUP_NO;

	/* unmap module address space */
	status = OSS_UnMapVirtAddr(a201Hdl->osHdl, &virtModAddr,
							   modAddrSize, OSS_ADDRSPACE_MEM);
	/* clear module slot */
	status2 = A201_ClrMIface( brdHdl, mSlot );

	/* return on error */ 
	if( status )
		return status;
	if( status2 )
		return status2;

	/* return on success */ 
	return ERR_SUCCESS;
}













