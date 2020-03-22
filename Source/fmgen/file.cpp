//	$Id: file.cpp,v 1.1 2001/04/23 22:25:33 kaoru-k Exp $

#include <ctype.h>
#include "headers.h"
#include "file.h"

// ---------------------------------------------------------------------------
//	構築/消滅
// ---------------------------------------------------------------------------
static FIL hfile @"NonCacheable";


FileIO::FileIO()
{
	flags = 0;
}

FileIO::FileIO(const char* filename, uint flg)
{
	flags = 0;
	Open(filename, flg);
}

FileIO::~FileIO()
{
	Close();
}

// ---------------------------------------------------------------------------
//	ファイルを開く
// ---------------------------------------------------------------------------

bool FileIO::Open(const char* filename, uint flg)
{
	FRESULT error;
        
	Close();

	strncpy(path, filename, 255);
        
	error = f_open(&hfile, filename, FA_READ);
	if (error == FR_OK) 
	  flags = open;
	      
	return !!(flags & open);
}

// ---------------------------------------------------------------------------
//	ファイルがない場合は作成
// ---------------------------------------------------------------------------

bool FileIO::CreateNew(const char* filename)
{
	Close();

	return !!(flags & open);
}

// ---------------------------------------------------------------------------
//	ファイルを作り直す
// ---------------------------------------------------------------------------

bool FileIO::Reopen(uint flg)
{
	if (!(flags & open)) return false;
	if ((flags & readonly) && (flg & create)) return false;

	if (flags & readonly) flg |= readonly;

	Close();

	return !!(flags & open);
}

// ---------------------------------------------------------------------------
//	ファイルを閉じる
// ---------------------------------------------------------------------------

void FileIO::Close()
{
	if (GetFlags() & open)
	{
		f_close(&hfile);
                memset(&hfile, sizeof(FIL), 0);
		flags = 0;
	}
}

// ---------------------------------------------------------------------------
//	ファイル殻の読み出し
// ---------------------------------------------------------------------------

int32 FileIO::Read(void* dest, int32 size)
{
	if (!(GetFlags() & open))
		return -1;
	
	size_t readsize;
	f_read (&hfile, dest, size, (unsigned int *)&readsize);

	return (int32)readsize;
}

// ---------------------------------------------------------------------------
//	ファイルへの書き出し
// ---------------------------------------------------------------------------

int32 FileIO::Write(const void* dest, int32 size)
{
	if (!(GetFlags() & open) || (GetFlags() & readonly))
		return -1;
	
	return 0;
}

// ---------------------------------------------------------------------------
//	ファイルをシーク
// ---------------------------------------------------------------------------

bool FileIO::Seek(int32 pos, SeekMethod method)
{
	if (!(GetFlags() & open))
		return false;
	
	int spos;
	switch (method) {
	case begin:	
                spos = pos;
		break;
	case current:	
	        spos = f_tell(&hfile) + pos;
		break;
	case end:		
		spos = f_size(&hfile) - pos; 
		break;
	default:
		return false;
	}

	return f_lseek(&hfile, spos) != FR_OK;
}

// ---------------------------------------------------------------------------
//	ファイルの位置を得る
// ---------------------------------------------------------------------------

int32 FileIO::Tellp()
{
	if (!(GetFlags() & open))
		return 0;

	return (int32)f_tell(&hfile);
}

// ---------------------------------------------------------------------------
//	現在の位置をファイルの終端とする
// ---------------------------------------------------------------------------

bool FileIO::SetEndOfFile()
{
	if (!(GetFlags() & open))
		return false;
        
	return f_lseek (&hfile, f_size(&hfile)) != FR_OK;
}
