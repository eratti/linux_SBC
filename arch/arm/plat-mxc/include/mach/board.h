
#define BASE_CS	  0x0900
/* CS id */
#define CS_ID_928 0x28
#define CS_ID_962 0x62
#define CS_ID_984 0x84
#define CS_ID_A62 0x01

/* PCB revision */
#define PCB_REV_A 0x00
#define PCB_REV_B 0x01
#define PCB_REV_C 0x02
#define PCB_REV_D 0x03

#define PCB_REV_A_STRING "RevA"
#define PCB_REV_B_STRING "RevB"
#define PCB_REV_C_STRING "RevC"
#define PCB_REV_D_STRING "RevD"

/* Board revision */
#define B_REV_0 0x00
#define B_REV_1 0x01

/* byte mask */
#define MASK_PCB_REV   0xF0
#define MASK_BOARD_REV 0x0F
#define MASK_CS_ID     0xFF


#define GET_CS_ID(x)		(BASE_CS | (x))

#define _GET_PCB_REV(x)		(((x) & MASK_PCB_REV) >> 4)
#define _GET_BOARD_REV(x)	((x) & MASK_BOARD_REV)
#define _GET_BOARD_CS_ID(x)	((x) & MASK_CS_ID )

extern unsigned char secoboardrev[4];

#define GET_PCB_REV			(_GET_PCB_REV(secoboardrev[1]))
#define PCB_IS_REV_A 		(GET_PCB_REV == PCB_REV_A ? 1 : 0)
#define PCB_IS_REV_B 		(GET_PCB_REV == PCB_REV_B ? 1 : 0)
#define PCB_IS_REV_C 		(GET_PCB_REV == PCB_REV_C ? 1 : 0)
#define PCB_IS_REV_D 		(GET_PCB_REV == PCB_REV_D ? 1 : 0)

#define GET_BOARD_REV		(_GET_BOARD_REV(secoboardrev[1]))

#define GET_BOARD_CS_ID		(_GET_BOARD_CS_ID(secoboardrev[0]))

#define BOARD_CS_IS_928		(GET_BOARD_CS_ID == CS_ID_928 ? 1 : 0)
#define BOARD_CS_IS_962		(GET_BOARD_CS_ID == CS_ID_962 ? 1 : 0)
#define BOARD_CS_IS_984		(GET_BOARD_CS_ID == CS_ID_984 ? 1 : 0)
#define BOARD_CS_IS_A62         (GET_BOARD_CS_ID == CS_ID_A62 ? 1 : 0)
