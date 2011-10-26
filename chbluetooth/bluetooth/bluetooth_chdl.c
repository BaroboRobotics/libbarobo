#include <bluetooth/bluetooth.h>
#include <ch.h>

EXPORTCH void baswap_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    bdaddr_t *dst;
    bdaddr_t *src;

    Ch_VaStart(interp, ap, varg);
    dst = Ch_VaArg(interp, ap, bdaddr_t *);
    src = Ch_VaArg(interp, ap, bdaddr_t *);
    baswap(dst, src);
    Ch_VaEnd(interp, ap);
}

EXPORTCH bdaddr_t * strtoba_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    char *str;
    bdaddr_t *retval;

    Ch_VaStart(interp, ap, varg);
    str = Ch_VaArg(interp, ap, char *);
    retval = strtoba(str);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH char * batostr_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    bdaddr_t *ba;
    char *retval;

    Ch_VaStart(interp, ap, varg);
    ba = Ch_VaArg(interp, ap, bdaddr_t *);
    retval = batostr(ba);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int ba2str_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    bdaddr_t *ba;
    char *str;
    int retval;

    Ch_VaStart(interp, ap, varg);
    ba = Ch_VaArg(interp, ap, bdaddr_t *);
    str = Ch_VaArg(interp, ap, char *);
    retval = ba2str(ba, str);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int str2ba_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    char *str;
    bdaddr_t *ba;
    int retval;

    Ch_VaStart(interp, ap, varg);
    str = Ch_VaArg(interp, ap, char *);
    ba = Ch_VaArg(interp, ap, bdaddr_t *);
    retval = str2ba(str, ba);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int ba2oui_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    bdaddr_t *ba;
    char *oui;
    int retval;

    Ch_VaStart(interp, ap, varg);
    ba = Ch_VaArg(interp, ap, bdaddr_t *);
    oui = Ch_VaArg(interp, ap, char *);
    retval = ba2oui(ba, oui);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int bachk_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    char *str;
    int retval;

    Ch_VaStart(interp, ap, varg);
    str = Ch_VaArg(interp, ap, char *);
    retval = bachk(str);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int baprintf_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    char *format;
    ChVaList_t ap_ch;
    void *ap_c;
    void *memhandle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    format = Ch_VaArg(interp, ap, char *);
    ap_ch = Ch_VaArg(interp, ap, ChVaList_t);
    ap_c = Ch_VaVarArgsCreate(interp, ap_ch, &memhandle);
    retval = vbaprintf(format, ap_c);
    Ch_VaVarArgsDelete(interp, memhandle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int bafprintf_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    FILE *stream;
    char *format;
    ChVaList_t ap_ch;
    void *ap_c;
    void *memhandle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    stream = Ch_VaArg(interp, ap, FILE *);
    format = Ch_VaArg(interp, ap, char *);
    ap_ch = Ch_VaArg(interp, ap, ChVaList_t);
    ap_c = Ch_VaVarArgsCreate(interp, ap_ch, &memhandle);
    retval = vbafprintf(stream, format, ap_c);
    Ch_VaVarArgsDelete(interp, memhandle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int basprintf_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    char *str;
    char *format;
    ChVaList_t ap_ch;
    void *ap_c;
    void *memhandle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    str = Ch_VaArg(interp, ap, char *);
    format = Ch_VaArg(interp, ap, char *);
    ap_ch = Ch_VaArg(interp, ap, ChVaList_t);
    ap_c = Ch_VaVarArgsCreate(interp, ap_ch, &memhandle);
    retval = vbasprintf(str, format, ap_c);
    Ch_VaVarArgsDelete(interp, memhandle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH int basnprintf_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    char *str;
    size_t size;
    char *format;
    ChVaList_t ap_ch;
    void *ap_c;
    void *memhandle;
    int retval;

    Ch_VaStart(interp, ap, varg);
    str = Ch_VaArg(interp, ap, char *);
    size = Ch_VaArg(interp, ap, size_t);
    format = Ch_VaArg(interp, ap, char *);
    ap_ch = Ch_VaArg(interp, ap, ChVaList_t);
    ap_c = Ch_VaVarArgsCreate(interp, ap_ch, &memhandle);
    retval = vbasnprintf(str, size, format, ap_c);
    Ch_VaVarArgsDelete(interp, memhandle);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH void * bt_malloc_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    size_t size;
    void *retval;

    Ch_VaStart(interp, ap, varg);
    size = Ch_VaArg(interp, ap, size_t);
    retval = bt_malloc(size);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH void bt_free_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    void *ptr;

    Ch_VaStart(interp, ap, varg);
    ptr = Ch_VaArg(interp, ap, void *);
    bt_free(ptr);
    Ch_VaEnd(interp, ap);
}

EXPORTCH int bt_error_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    uint16_t code;
    int retval;

    Ch_VaStart(interp, ap, varg);
    code = Ch_VaArg(interp, ap, uint16_t);
    retval = bt_error(code);
    Ch_VaEnd(interp, ap);
    return retval;
}

EXPORTCH char * bt_compidtostr_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    int id;
    char *retval;

    Ch_VaStart(interp, ap, varg);
    id = Ch_VaArg(interp, ap, int);
    retval = bt_compidtostr(id);
    Ch_VaEnd(interp, ap);
    return retval;
}
