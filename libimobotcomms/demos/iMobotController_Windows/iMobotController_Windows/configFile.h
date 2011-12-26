#ifndef _CONFIG_FILE_H_
#define _CONFIG_FILE_H_

class ConfigFile
{
  public:
    ConfigFile();
    ~ConfigFile();
    int read(const char* path);
    int numEntries();
    const char* getEntry(int index);
    int addEntry(const char* entry);
    int moveEntryDown(int index);
    int moveEntryUp(int index);
    int remove(int index);
    int write();

  private:
    int _numEntries;
    char *_addresses[100];
    char *_path;
};


#endif


