/***************************************************************************
*
* This is FREESCALE SOFTWARE LICENSE AGREEMENT copy of ReleaseNotes.rtf from
* Standard Software Driver (SSD) for C55 Flash module v1.1.0.
*___________________________________________________________________________
*
* IMPORTANT. Read the following Freescale Software License Agreement
* ("Agreement") completely. By selecting the "I Accept" button at the end of
* this page, you indicate that you accept the terms of this Agreement. You
* may then download the file.
*
*                    FREESCALE SOFTWARE LICENSE AGREEMENT
*
* This is a legal agreement between you (either as an individual or as anauthorized representative
* of your employer) and Freescale Semiconductor, Inc. ("Freescale"). It concerns your rights to use
* this file and any accompanying written materials (the "Software"). In consideration for Freescale
* allowing you to access the Software, you are agreeing to be bound by the terms of this
* Agreement. If you do not agree to all of the terms of this Agreement, do not download the
* Software. If you change your mind later, stop using the Software and delete all copies of the
* Software in your possession or control. Any copies of the Software that you have already
* distributed, where permitted, and do not destroy will continue to be governed by this Agreement.
* Your prior use will also continue to be governed by this Agreement.
*
* LICENSE GRANT. Freescale grants to you, free of charge, the non-exclusive, non-transferable
* right (1) to use the Software, (2) to reproduce the Software, (3) to prepare derivative works of the
* Software, (4) to distribute the Software and derivative works thereof in source (human-readable)
* form and object (machine-readable) form, and (5) to
* sublicense to others the right to use the distributed Software. If you violate any of the terms or
* restrictions of this Agreement, Freescale may immediately terminate this Agreement, and require
* that you stop using and delete all copies of the Software in your possession or control.
*
* COPYRIGHT. The Software is licensed to you, not sold. Freescale owns the Software, and
* United States copyright laws and international treaty provisions protect the Software. Therefore,
* you must treat the Software like any other copyrighted material (e.g., a book or musical
* recording). You may not use or copy the Software for any other purpose than what is described in
* this Agreement. Except as expressly provided herein, Freescale does not grant to you any express
* or implied rights under any Freescale or third party patents, copyrights, trademarks, or trade
* secrets. Additionally, you must reproduce and apply any copyright or other proprietary rights
* notices included on or embedded in the Software to any copies or derivative works made thereof,
* in whole or in part, if any.
*
* SUPPORT. Freescale is NOT obligated to provide any support, upgrades or new releases of the
* Software. If you wish, you may contact Freescale and report problems and provide suggestions
* regarding the Software. Freescale has no obligation whatsoever to respond in any way to such a
* problem report or suggestion. Freescale may make changes to the Software at any time, without
* any obligation to notify or provide updated versions of the Software to you.
*
* NO WARRANTY. TO THE MAXIMUM EXTENT PERMITTED BY LAW,  FREESCALE
* EXPRESSLY DISCLAIMS ANY WARRANTY FOR THE SOFTWARE. THE SOFTWARE IS
* PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR
* IMPLIED, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR NON-
* INFRINGEMENT. YOU ASSUME THE ENTIRE RISK ARISING OUT OF THE USE OR
* PERFORMANCE OF THE SOFTWARE, OR ANY SYSTEMS YOU DESIGN USING THE
* SOFTWARE (IF ANY). NOTHING IN THIS AGREEMENT MAY BE CONSTRUED AS A
* WARRANTY OR REPRESENTATION BY FREESCALE THAT THE SOFTWARE OR ANY
* DERIVATIVE WORK DEVELOPED WITH OR INCORPORATING THE SOFTWARE WILL
* BE FREE FROM INFRINGEMENT OF THE INTELLECTUAL PROPERTY RIGHTS OF
* THIRD PARTIES.
*
* INDEMNITY. You agree to fully defend and indemnify Freescale from any and all claims,
* liabilities, and costs (including reasonable attorney's fees) related to (1) your use (including your
* sublicensee's use, if permitted) of the Software or (2) your violation of the terms and conditions
* of this Agreement.
*
* LIMITATION OF LIABILITY. IN NO EVENT WILL FREESCALE BE LIABLE, WHETHER
* IN CONTRACT, TORT, OR OTHERWISE, FOR ANY INCIDENTAL, SPECIAL, INDIRECT,
* CONSEQUENTIAL OR PUNITIVE DAMAGES, INCLUDING, BUT NOT LIMITED TO,
* DAMAGES FOR ANY LOSS OF USE, LOSS OF TIME, INCONVENIENCE, COMMERCIAL
* LOSS, OR LOST PROFITS, SAVINGS, OR REVENUES TO THE FULL EXTENT SUCH
* MAY BE DISCLAIMED BY LAW.
*
* COMPLIANCE WITH LAWS; EXPORT RESTRICTIONS. You must use the Software in
* accordance with all applicable U.S. laws, regulations and statutes. You agree that neither you nor
* your licensees (if any) intend to or will, directly or indirectly, export or transmit the Software to
* any country in violation of U.S. export restrictions.
*
* GOVERNMENT USE. Use of the Software and any corresponding documentation, if
* any, is provided with RESTRICTED RIGHTS. Use, duplication or disclosure by the Government
* is subject to restrictions as set forth in subparagraph (c)(1)(ii) of The Rights in Technical Data
* and Computer Software clause at DFARS 252.227-7013 or subparagraphs (c)(l) and (2) of the
* Commercial Computer Software--Restricted Rights at 48 CFR 52.227-19, as applicable.
* Manufacturer is Freescale, Inc., 6501 William Cannon Drive West, Austin, TX, 78735.
*
* HIGH RISK ACTIVITIES. You acknowledge that the Software is not fault tolerant and is not
* designed, manufactured or intended by Freescale for incorporation into products intended for use
* or resale in on-line control equipment in hazardous, dangerous to life or potentially life-
* threatening environments requiring fail-safe performance, such as in the operation of nuclear
* facilities, aircraft navigation or communication systems, air traffic control, direct life support
* machines or weapons systems, in which the failure of products could lead directly to death,
* personal injury or severe physical or environmental damage ("High Risk Activities"). You
* specifically represent and warrant that you will not use the Software or any derivative work of the
* Software for High Risk Activities.
*
* CHOICE OF LAW; VENUE; LIMITATIONS. You agree that the statutes and laws of
* the United States and the State of Texas, USA, without regard to conflicts of laws principles, will
* apply to all matters relating to this Agreement or the Software, and you agree that any litigation
* will be subject to the exclusive jurisdiction of the state or federal courts in Texas, USA. You
* agree that regardless of any statute or law to the contrary, any claim or cause of action arising out
* of or related to this Agreement or the Software must be filed within one (1) year after such claim or cause of action arose
* or be forever barred.
*
* PRODUCT LABELING. You are not authorized to use any Freescale trademarks, brand names,
* or logos.
*
* ENTIRE AGREEMENT. This Agreement constitutes the entire agreement between you and
* Freescale regarding the subject matter of this Agreement, and supersedes all prior
* communications, negotiations, understandings, agreements or representations, either written or
* oral, if any. This Agreement may only be amended in written form, executed by you and
* Freescale.
*
* SEVERABILITY. If any provision of this Agreement is held for any reason to be invalid or
* unenforceable, then the remaining provisions of this Agreement will be unimpaired and, unless a
* modification or replacement of the invalid or unenforceable provision is further held to deprive
* you or Freescale of a material benefit, in which case the Agreement will immediately terminate,
* the invalid or unenforceable provision will be replaced with a provision that is valid and
* enforceable and that comes closest to the intention underlying the invalid or unenforceable
* provision.
*
* NO WAIVER. The waiver by Freescale of any breach of any provision of this Agreement will
* not operate or be construed as a waiver of any other or a subsequent breach of the same or a
* different provision.
*
**********************************************************************/

#include "fnet.h"

#if FNET_MPC && FNET_CFG_CPU_MPC5744P && FNET_CFG_COMP_GHS

/*For writing SPR*/
#define mtspr(rn, v)    __asm__ volatile("mtspr " #rn ",%0" : : "r" (v))

/*For reading SPR*/
#define mfspr(reg, spr)    __asm__ volatile("mfspr %0, " # spr : "=r"  (reg))

#define E200CORE_SYNC()                __asm__("msync")        /* Memory synchronize  */
#define E200CORE_ISYNC()               __asm__("se_isync" )    /* Instruction synchronize */
#define E200CORE_SPR_GET(lhs, reg)     mfspr( lhs, reg)        /* read from special register*/
#define E200CORE_SPR_SET(reg, val)     mtspr( reg, val)        /* write to special register*/
#define E200CORE_L1CSR0                1010                    /* L1 Cache Control and Status Register 0 */
#define E200CORE_L1CSR0_CE             0x00000001              /* Data cache enable */
#define E200CORE_L1CSR0_CWM            0x00100000              /* Data cache Write mode */

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : dcache_enable
* Returned Value   : none
* Comments         :
*   This function enables the data cache
*
*END*----------------------------------------------------------------------*/

void fnet_mpc5744p_dcache_enable(
   void
){ /* Body */
   register fnet_uint32_t i;
   /* Read L1CSR0 */
   E200CORE_SPR_GET(i,E200CORE_L1CSR0); 
   /* enable copy-back mode,enable cache */
   i |= E200CORE_L1CSR0_CE | E200CORE_L1CSR0_CWM;
   /* Memory Synchronize */
   E200CORE_SYNC();
   /* Instruction Synchronize */
   E200CORE_ISYNC();
   /* Write to L1CSR0 */
   E200CORE_SPR_SET(E200CORE_L1CSR0,i);      
   /* Instruction Synchronize */
   E200CORE_ISYNC();
} /* Endbody */

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : dcache_disable
* Returned Value   : none
* Comments         :
*   This function disables the data cache (if possible)
*
*END*----------------------------------------------------------------------*/

void fnet_mpc5744p_dcache_disable(
   void
){ /* Body */
   register fnet_uint32_t val;

   E200CORE_SPR_GET(val,E200CORE_L1CSR0); /* L1CSR0 */
   val &= ~E200CORE_L1CSR0_CE;
   /* Memory Synchronize */
   E200CORE_SYNC();
   /* Instruction Synchronize */
   E200CORE_ISYNC();
   E200CORE_SPR_SET(E200CORE_L1CSR0,val);
   /* Instruction Synchronize */
   E200CORE_ISYNC();
} /* Endbody */

#endif
