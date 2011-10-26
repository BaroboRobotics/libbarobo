#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

void baswap(bdaddr_t *dst, const bdaddr_t *src);
bdaddr_t *strtoba(const char *str);
char *batostr(const bdaddr_t *ba);
int ba2str(const bdaddr_t *ba, char *str);
int str2ba(const char *str, bdaddr_t *ba);
int ba2oui(const bdaddr_t *ba, char *oui);
int bachk(const char *str);

int baprintf(const char *format, ...);
int bafprintf(FILE *stream, const char *format, ...);
int basprintf(char *str, const char *format, ...);
int basnprintf(char *str, size_t size, const char *format, ...);

void *bt_malloc(size_t size);
void bt_free(void *ptr);

int bt_error(uint16_t code);
char *bt_compidtostr(int id);

#ifdef _CH_
#pragma package <chbluetooth>
#include <chdl.h>
LOAD_CHDL(bluetooth)
#endif

#endif
