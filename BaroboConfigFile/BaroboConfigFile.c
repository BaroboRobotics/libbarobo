#include <stdio.h>
#include "BaroboConfigFile.h"

char* BCF_strdup(const char* str)
{
  char* s;
  s = (char*)malloc(sizeof(char)*(strlen(str)+1));
  strcpy(s, str);
  return s;
}

bcf_t* BCF_New()
{
  bcf_t* bcf = (bcf_t*)malloc(sizeof(bcf_t));
  memset(bcf, 0, sizeof(bcf_t));
  bcf->entries = (char**)malloc(sizeof(char*)*256);
  bcf->dongles = (char**)malloc(sizeof(char*)*256);
  return bcf;
}

int BCF_Destroy(bcf_t* bcf)
{
  int i;
  if(bcf->filename != NULL) {
    free(bcf->filename);
  }

  if(bcf->root != NULL) {
    mxmlDelete(bcf->root);
  }

  for(i = 0; i < bcf->num; i++) {
    free(bcf->entries[i]);
  }
  free(bcf->entries);
  for(i = 0; i < bcf->numDongles; i++) {
    free(bcf->dongles[i]);
  }
  free(bcf->dongles);

  free(bcf);
  return 0;
}

int BCF_Read(bcf_t* bcf, const char* filename)
{
  FILE *fp;
  mxml_node_t *node, *text;
  int i;

  if(bcf->filename != NULL) {
    free(bcf->filename);
  }
  bcf->filename = strdup(filename);

  fp = fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error opening config file %s for reading.\n", filename);
    return -1;
  }
  bcf->root = mxmlLoadFile(NULL, fp, MXML_TEXT_CALLBACK);
  fclose(fp);

  /* Now walk through the file and get all the children */
  for(i = 0, node = mxmlFindElement(bcf->root, bcf->root,
        "child",
        NULL,
        NULL,
        MXML_DESCEND);
        node != NULL;
        node = mxmlFindElement(node, bcf->root,
        "child",
        NULL,
        NULL,
        MXML_DESCEND), i++
     )
  {
    text = mxmlGetFirstChild(node);
    bcf->entries[i] = strdup(mxmlGetText(text, NULL));
  }
  bcf->num = i;

  /* Now read all the dongles */
  for(i = 0, node = mxmlFindElement(bcf->root, bcf->root,
        "dongle",
        NULL,
        NULL,
        MXML_DESCEND);
        node != NULL;
        node = mxmlFindElement(node, bcf->root,
        "dongle",
        NULL,
        NULL,
        MXML_DESCEND), i++
     )
  {
    text = mxmlGetFirstChild(node);
    bcf->dongles[i] = strdup(mxmlGetText(text, NULL));
  }
  bcf->numDongles = i;

  /* Clean everything up */
  mxmlDelete(bcf->root);
  bcf->root = NULL;

  return 0;
}

int BCF_Write(bcf_t* bcf, const char* filename)
{
  FILE *fp;
  int i;
  mxml_node_t *baroboData, *addresses, *dongles, *iter;
  if(filename == NULL) {
    filename = bcf->filename;
  }
  fp = fopen(filename, "w");
  if(fp == NULL) {
    fprintf(stderr, "Error opening file %s for writing.\n", filename);
    return -1;
  }
  /* Write everything */
  bcf->root = mxmlNewXML("1.0");
  baroboData = mxmlNewElement(bcf->root, "BaroboData");
  addresses = mxmlNewElement(baroboData, "addresses");
  mxmlElementSetAttrf(
      addresses,
      "num",
      "%d", bcf->num);
  dongles = mxmlNewElement(baroboData, "dongles");
  mxmlElementSetAttrf(
      dongles,
      "num", "%d", bcf->numDongles);
  
  for(i = 0; i < bcf->num; i++) {
    iter = mxmlNewElement(addresses, "child");
    mxmlNewText(iter, 0, bcf->entries[i]);
  }
  for(i = 0; i < bcf->numDongles; i++) {
    iter = mxmlNewElement(dongles, "dongle");
    mxmlNewText(iter, 0, bcf->dongles[i]);
  }
  mxmlSaveFile(bcf->root, fp, MXML_NO_CALLBACK);
  fclose(fp);
  return 0;
}

const char* BCF_GetIndex(bcf_t* bcf, int index)
{
  if(index > bcf->num) {
    return NULL;
  }
  return bcf->entries[index];
}

int BCF_GetEntryIndex(bcf_t* bcf, const char* entry)
{
  int i;
  for(i = 0; i < bcf->num; i++) {
    if(!strcasecmp(entry, bcf->entries[i])) {
      return i;
    }
  }
  return -1;
}

int BCF_GetNum(bcf_t* bcf)
{
  return bcf->num;
}

const char* BCF_GetDongle(bcf_t* bcf, int index)
{
  if (index >= bcf->numDongles) {
    return NULL;
  }
  return bcf->dongles[index];
}

int BCF_GetNumDongles(bcf_t* bcf)
{
  return bcf->numDongles;
}

int BCF_Prepend(bcf_t* bcf, const char* entry)
{
  /* Move all entries up one */
  int i;
  for(i = bcf->num-1; i >= 0; i--)
  {
    bcf->entries[i+1] = bcf->entries[i];
  }
  bcf->entries[0] = strdup(entry);
  bcf->num++;
  return 0;
}

int BCF_Append(bcf_t* bcf, const char* entry)
{
  bcf->entries[bcf->num] = strdup(entry);
  bcf->num++;
  return 0;
}

int BCF_Insert(bcf_t* bcf, const char* entry, int index)
{
  int i;
  if(index > bcf->num) {
    return -1;
  }
  /* Move everything up one including the index */

  for(i = bcf->num-1; i >= index; i--)
  {
    bcf->entries[i+1] = bcf->entries[i];
  }
  bcf->entries[index] = strdup(entry);
  bcf->num++;
  return 0;
}

int BCF_Remove(bcf_t* bcf, int index)
{
  int i;
  if(index > bcf->num) {
    return -1;
  }
  free(bcf->entries[index]);
  for(i = index; i < bcf->num-1; i++) {
    bcf->entries[i] = bcf->entries[i+1];
  }
  bcf->num--;
  return 0;
}

int BCF_MoveUp(bcf_t* bcf, int index)
{
  char *tmp;
  if( (index < 1) || (index > bcf->num-1)) {
    return -1;
  }
  /* Swap index with one above */
  tmp = bcf->entries[index-1];
  bcf->entries[index-1] = bcf->entries[index];
  bcf->entries[index] = tmp;
  return 0;
}

int BCF_MoveDown(bcf_t* bcf, int index)
{
  char* tmp;
  if( (index < 0) || (index > bcf->num-2)) {
    return -1;
  }
  /* Swap with index below */
  tmp = bcf->entries[index+1];
  bcf->entries[index+1] = bcf->entries[index];
  bcf->entries[index] = tmp;
  return 0;
}

int BCF_RemoveDongle(bcf_t* bcf, int index)
{
  int i;
  if( (index < 0) || (index >= bcf->numDongles)) {
    return -1;
  }
  free(bcf->dongles[index]);

  for(i = index; i < bcf->numDongles-1; i++) {
    bcf->dongles[i] = bcf->dongles[i+1];
  }
  bcf->numDongles--;
  return 0;
}

int BCF_PrependDongle(bcf_t* bcf, const char* entry)
{
  /* Move all the dongles down */
  int i;
  for(i = bcf->numDongles-1; i >= 0; i--) {
    bcf->dongles[i+1] = bcf->dongles[i];
  }
  bcf->dongles[0] = strdup(entry);
  bcf->numDongles++;
  return 0;
}
