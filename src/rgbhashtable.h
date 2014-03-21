/* rgbhashtable.h
 *
 * A hashtable for storing RGB values for a Linkbot LED
 * Chaining is used to resolve collisions 
 * RGB values need to be stored as an array of 3 values
 * An address of an array of three values is returned for the RGB color values
 *
 * Dawn Hustig-Schultz
 * 2013/12/20
 *
 */

#ifndef RGBHASHTABLE_H
#define RGBHASHTABLE_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TABLE_SIZE 150
#define MAX_KEYLEN 25
#define RGB_LEN 3

//One entry in the table
struct rgbNode{
  char * key;
  int values[RGB_LEN]; 
  struct rgbNode *next;     
};

//The table itself
typedef struct rgbHashTable rgbHashTable;
struct rgbHashTable{
  struct rgbNode **table;
  int size;
};

rgbHashTable * HT_Create(); //Allocate memory for the table
void HT_Destroy(rgbHashTable * rgbHT); //Deallocate the memory
int HT_Get(rgbHashTable * rgbHT, char * key, int * rgbArray);            // retrieve entry
void HT_Add(rgbHashTable * rgbHT, char * key, int values[]);  // store entry 
void HT_Remove(rgbHashTable * rgbHT, char * key);          // remove entry
int HT_GetKey(rgbHashTable * rgbHT, int values[], char color[]); //reverse look-up

#ifdef __cplusplus
}
#endif

#endif /* RGBHASHTABLE_H */
