#ifndef _BAROBO_CONFIG_FIL_H_
#define _BAROBO_CONFIG_FIL_H_

#include <mxml.h>

/* Sample xml file:
<?xml version="1.0" encoding="utf-8"?>
<BaroboData>
  <addresses>
    <child type="bluetooth">
      00:06:66:45:12:21
    </child>
    <child type="bluetooth">
      00:12:32:12:33:12
    </child>
    <child type="zigbee">
      A082
    </child>
  </addresses>

  <dongles>
    <dongle>
      COM1
    </dongle>
    <dongle>
      /dev/ttyACM2
    </dongle>
  </dongles>
</BaroboData>
*/

typedef struct bcf_s
{
  char* filename;
  mxml_node_t* root;
  int num;
  char** entries;
  int numDongles;
  char** dongles;
} bcf_t;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _MSYS
#define strdup(x) BCF_strdup(x)
char* BCF_strdup(const char* str);
#endif

bcf_t* BCF_New();
int BCF_Destroy(bcf_t* bcf);

int BCF_Read(bcf_t* bcf, const char* filename);
int BCF_Write(bcf_t* bcf, /* OPT */ const char* filename);
const char* BCF_GetIndex(bcf_t* bcf, int index);
int BCF_GetNum(bcf_t* bcf);
const char* BCF_GetDongle(bcf_t* bcf, int index);
int BCF_RemoveDongle(bcf_t* bcf, int index);
int BCF_PrependDongle(bcf_t* bcf, const char* entry);
int BCF_GetNumDongles(bcf_t* bcf);
int BCF_Prepend(bcf_t* bcf, const char* entry);
int BCF_Append(bcf_t* bcf, const char* entry);
int BCF_Insert(bcf_t* bcf, const char* entry, int index);
int BCF_Remove(bcf_t* bcf, int index);
int BCF_MoveUp(bcf_t* bcf, int index);
int BCF_MoveDown(bcf_t* bcf, int index);

#ifdef __cplusplus
}
#endif

#endif
