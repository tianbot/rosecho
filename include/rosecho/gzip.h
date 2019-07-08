#ifndef _GZIP_H_
#define _GZIP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "zlib.h"
int gzcompress(Bytef *data, uLong ndata, Bytef *zdata, uLong *nzdata);
               
int gzdecompress(Byte *zdata, uLong nzdata, Byte *data, uLong *ndata);
//int gzdecompress(unsigned char* zdata, unsigned long nzdata, unsigned char* data, unsigned long* ndata);
#ifdef __cplusplus
}
#endif

#endif /* _GZIP_H_*/
