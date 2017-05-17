#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include "serialize.h"

int Serialize_FindSync(int Stream, uint32_t Sync)                                                // find the Sync word in the stream
{ uint32_t Buffer=0; if(read(Stream, &Buffer, sizeof(uint32_t))!=sizeof(uint32_t)) return -1;    // read the very first word
  if(Buffer==Sync) return 0;                                                                     // if this is our Sync then we are there !
  int Total=0;                                                                                   // Sync not found immediately: we need to skip data and hunt for it
  for( ; ; )
  { uint8_t Byte; if(read(Stream, &Byte, 1)!=1) return -1;                                       // read one byte
    Total++;                                                                                     // count the number os bytes we had to skip
    Buffer = (Buffer>>8) | ((uint32_t)Byte<<24);                                                 // slide the received wod by one byte and add the byte we just read
    if(Buffer==Sync) break; }                                                                    // if this is our Sync then we are there !
  return Total; }                                                                                // return the number of skipped bytes

int Serialize_FindSync(FILE *Stream, uint32_t Sync)
{ uint32_t Buffer=0; if(fread(&Buffer, 1, sizeof(uint32_t), Stream)!=sizeof(uint32_t)) return -1;
  if(Buffer==Sync) return 0;
  int Total=0;
  for( ; ; )
  { uint8_t Byte; if(fread(&Byte, 1, 1, Stream)!=1) return -1;
    Total++;
    Buffer = (Buffer>>8) | ((uint32_t)Byte<<24);
    if(Buffer==Sync) break; }
  return Total; }

int Serialize_ReadName(int Stream, char *Name, int MaxBytes)
{ int Len=0;
  for( ; ; )
  { if(Len>=MaxBytes) return -1;
    if(read(Stream, Name+Len, 1)!=1) return -1;
    if(Name[Len]==0) break;
    Len++; }
  return Len; }

int Serialize_ReadName(FILE *Stream, char *Name, int MaxBytes)
{ int Len=0;
  for( ; ; )
  { if(Len>=MaxBytes) return -1;
    if(fread(Name+Len, 1, 1, Stream)!=1) return -1;
    if(Name[Len]==0) break;
    Len++; }
  return Len; }

