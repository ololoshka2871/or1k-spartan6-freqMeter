#include "mem_map.h"
#include "syscall.h"

#include "cache.h"


//-----------------------------------------------------------------
// cache_dflush:
//-----------------------------------------------------------------
void cache_dflush(void)
{
	unsigned long sr = mfspr(SPR_SR);    
    mtspr(SPR_SR, sr | SPR_SR_DCACHE_FLUSH);    
}
//-----------------------------------------------------------------
// cache_iflush:
//-----------------------------------------------------------------
void cache_iflush(void)
{
	unsigned long sr = mfspr(SPR_SR);    
    mtspr(SPR_SR, sr | SPR_SR_ICACHE_FLUSH);    
}
//-----------------------------------------------------------------
// cache_flush:
//-----------------------------------------------------------------
void cache_flush(void)
{
	unsigned long sr = mfspr(SPR_SR);    
    mtspr(SPR_SR, sr | SPR_SR_ICACHE_FLUSH | SPR_SR_DCACHE_FLUSH);    
}
