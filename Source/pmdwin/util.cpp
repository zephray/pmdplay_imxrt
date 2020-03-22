﻿//=============================================================================
//			Utility Functions
//				Programmed by C60
//=============================================================================

#ifdef _WINDOWS
#include	<windows.h>
#include	<mbstring.h>
#endif
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include	"util.h"
extern "C" {
#include "ff.h"
#include "diskio.h"
}


//=============================================================================
//	ファイル名で示されたファイルのサイズを取得する
//=============================================================================
signed long long GetFileSize_s(char *filename)
{
#ifdef _WINDOWS
	HANDLE	handle;
	WIN32_FIND_DATAA	FindFileData;

	if((handle = FindFirstFileA(filename, &FindFileData)) == INVALID_HANDLE_VALUE) {
		return -1;		// 取得不可
	} else {
		FindClose(handle);
		return (signed long long)((_signed long long)FindFileData.nFileSizeHigh << 32) + FindFileData.nFileSizeLow; 
 	}
#else
	FIL fp;
	int size;

	if (f_open (&fp, filename, FA_READ) != FR_OK)
	  {
		return -1;
	  }

	size = f_size(&fp);
	f_close (&fp);

	return size;
#endif
}


//=============================================================================
//	TAB を SPACE に変換(tabcolumn カラム毎 / ESCシーケンス不可)
//=============================================================================
char *tab2spc(char *dest, const char *src, int tabcolumn)
{
	int		column = 0;
	char	*dest2;

	dest2 = dest;
	while(*src != '\0') {
		if(*src == '\t') {							// TAB
			src++;
			do {
				*dest++ = ' ';
			}while(++column % tabcolumn);
		} else if(*src == 0x0d || *src == 0x0a) {	// 改行
			column = 0;
			*dest++ = *src++;
		} else {
			*dest++ = *src++;
			column++;
		}
	}
	*dest = '\0';
	return dest2;
}


//=============================================================================
//	エスケープシーケンスの除去
//	カーソル移動系エスケープシーケンスには非対応
//=============================================================================
char *delesc(char *dest, const char *src)
{
	char	*dest2;
	uchar	*src2;
	uchar	*src3;
	size_t	i;

	if((src2 = src3 = (uchar *)malloc(strlen(src) + 32)) == NULL) {
		return NULL;
	};

	strcpy((char *)src2, src);

	// 31バイト連続 '\0' にする(8 以上なら OK なはずだけど，念のため）
	for(i = strlen((char *)src2); i < strlen((char *)src2) + 31; i++) {
		src2[i] = '\0';
	}

	dest2 = dest;

	do {
		if(*src2 == 0x1b) {		// エスケープシーケンス
			if(*(src2 + 1) == '[') {
				src2 += 2;
				while(*src2 && (toupper(*src2) < 'A' || toupper(*src2) > 'Z')) {
					if(_ismbblead(*src2)) {
						src2 += 2;
						continue;
					}
					src2++;
				}
				src2++;
			} else if(*(src2 + 1) == '=') {
				src2 += 4;
			} else if(*(src2 + 1) == ')' || *(src2 + 1) == '!') {
				src2 += 3;
			} else {
				src2 += 2;
			}
		} else {
			*dest++ = *src2++;		
		}
	}while(*src2 != '\0');

	free(src3);
	*dest = '\0';
	return dest2;
}


//=============================================================================
//	２バイト半角文字を半角に変換
//=============================================================================
char *zen2tohan(char *dest, const char *src)
{
	char *src2;
	char *src3;
	char *dest2;
	int	len;
	static const char *codetable[] = {
		"!",		/*	8540 	*/
		"\"",		/*	8541 	*/
		"#",		/*	8542 	*/
		"$",		/*	8543 	*/
		"%",		/*	8544 	*/
		"&",		/*	8545 	*/
		"'",		/*	8546 	*/
		"(",		/*	8547 	*/
		")",		/*	8548 	*/
		"*",		/*	8549 	*/
		"+",		/*	854a 	*/
		",",		/*	854b 	*/
		"-",		/*	854c 	*/
		".",		/*	854d 	*/
		"/",		/*	854e 	*/
		"0",		/*	854f 	*/
		"1",		/*	8550 	*/
		"2",		/*	8551 	*/
		"3",		/*	8552 	*/
		"4",		/*	8553 	*/
		"5",		/*	8554 	*/
		"6",		/*	8555 	*/
		"7",		/*	8556 	*/
		"8",		/*	8557 	*/
		"9",		/*	8558 	*/
		":",		/*	8559 	*/
		";",		/*	855a 	*/
		"<",		/*	855b 	*/
		"=",		/*	855c 	*/
		">",		/*	855d 	*/
		"?",		/*	855e 	*/
		"@",		/*	855f 	*/
		"A",		/*	8560 	*/
		"B",		/*	8561 	*/
		"C",		/*	8562 	*/
		"D",		/*	8563 	*/
		"E",		/*	8564 	*/
		"F",		/*	8565 	*/
		"G",		/*	8566 	*/
		"H",		/*	8567 	*/
		"I",		/*	8568 	*/
		"J",		/*	8569 	*/
		"K",		/*	856a 	*/
		"L",		/*	856b 	*/
		"M",		/*	856c 	*/
		"N",		/*	856d 	*/
		"O",		/*	856e 	*/
		"P",		/*	856f 	*/
		"Q",		/*	8570 	*/
		"R",		/*	8571 	*/
		"S",		/*	8572 	*/
		"T",		/*	8573 	*/
		"U",		/*	8574 	*/
		"V",		/*	8575 	*/
		"W",		/*	8576 	*/
		"X",		/*	8577 	*/
		"Y",		/*	8578 	*/
		"Z",		/*	8579 	*/
		"[",		/*	857a 	*/
		"\\",		/*	857b 	*/
		"]",		/*	857c 	*/
		"^",		/*	857d 	*/
		"_",		/*	857e 	*/
		"",			/*	857f 	*/
		"`",		/*	8580 	*/
		"a",		/*	8581 	*/
		"b",		/*	8582 	*/
		"c",		/*	8583 	*/
		"d",		/*	8584 	*/
		"e",		/*	8585 	*/
		"f",		/*	8586 	*/
		"g",		/*	8587 	*/
		"h",		/*	8588 	*/
		"i",		/*	8589 	*/
		"j",		/*	858a 	*/
		"k",		/*	858b 	*/
		"l",		/*	858c 	*/
		"m",		/*	858d 	*/
		"n",		/*	858e 	*/
		"o",		/*	858f 	*/
		"p",		/*	8590 	*/
		"q",		/*	8591 	*/
		"r",		/*	8592 	*/
		"s",		/*	8593 	*/
		"t",		/*	8594 	*/
		"u",		/*	8595 	*/
		"v",		/*	8596 	*/
		"w",		/*	8597 	*/
		"x",		/*	8598 	*/
		"y",		/*	8599 	*/
		"z",		/*	859a 	*/
		"{",		/*	859b 	*/
		"|",		/*	859c 	*/
		"}",		/*	859d 	*/
		"¯",		/*	859e 	*/
		"。",		/*	859f 	*/
		"「",		/*	85a0 	*/
		"」",		/*	85a1 	*/
		"、",		/*	85a2 	*/
		"・",		/*	85a3 	*/
		"ヲ",		/*	85a4 	*/
		"ァ",		/*	85a5 	*/
		"ィ",		/*	85a6 	*/
		"ゥ",		/*	85a7 	*/
		"ェ",		/*	85a8 	*/
		"ォ",		/*	85a9 	*/
		"ャ",		/*	85aa 	*/
		"ュ",		/*	85ab 	*/
		"ョ",		/*	85ac 	*/
		"ッ",		/*	85ad 	*/
		"ー",		/*	85ae 	*/
		"ア",		/*	85af 	*/
		"イ",		/*	85b0 	*/
		"ウ",		/*	85b1 	*/
		"エ",		/*	85b2 	*/
		"オ",		/*	85b3 	*/
		"カ",		/*	85b4 	*/
		"キ",		/*	85b5 	*/
		"ク",		/*	85b6 	*/
		"ケ",		/*	85b7 	*/
		"コ",		/*	85b8 	*/
		"サ",		/*	85b9 	*/
		"シ",		/*	85ba 	*/
		"ス",		/*	85bb 	*/
		"セ",		/*	85bc 	*/
		"ソ",		/*	85bd 	*/
		"タ",		/*	85be 	*/
		"チ",		/*	85bf 	*/
		"ツ",		/*	85c0 	*/
		"テ",		/*	85c1 	*/
		"ト",		/*	85c2 	*/
		"ナ",		/*	85c3 	*/
		"ニ",		/*	85c4 	*/
		"ヌ",		/*	85c5 	*/
		"ネ",		/*	85c6 	*/
		"ノ",		/*	85c7 	*/
		"ハ",		/*	85c8 	*/
		"ヒ",		/*	85c9 	*/
		"フ",		/*	85ca 	*/
		"ヘ",		/*	85cb 	*/
		"ホ",		/*	85cc 	*/
		"マ",		/*	85cd 	*/
		"ミ",		/*	85ce 	*/
		"ム",		/*	85cf 	*/
		"メ",		/*	85d0 	*/
		"モ",		/*	85d1 	*/
		"ヤ",		/*	85d2 	*/
		"ユ",		/*	85d3 	*/
		"ヨ",		/*	85d4 	*/
		"ラ",		/*	85d5 	*/
		"リ",		/*	85d6 	*/
		"ル",		/*	85d7 	*/
		"レ",		/*	85d8 	*/
		"ロ",		/*	85d9 	*/
		"ワ",		/*	85da 	*/
		"ン",		/*	85db 	*/
		"゛",		/*	85dc 	*/
		"゜",		/*	85dd 	*/
		"ヰ",		/*	85de 	*/
		"ヱ",		/*	85df 	*/
		"ヮ",		/*	85e0 	*/
		"ヵ",		/*	85e1 	*/
		"ヶ",		/*	85e2 	*/
		"ヴ",		/*	85e3 	*/
		"ガ",		/*	85e4 	*/
		"ギ",		/*	85e5 	*/
		"グ",		/*	85e6 	*/
		"ゲ",		/*	85e7 	*/
		"ゴ",		/*	85e8 	*/
		"ザ",		/*	85e9 	*/
		"ジ",		/*	85ea 	*/
		"ズ",		/*	85eb 	*/
		"ゼ",		/*	85ec 	*/
		"ゾ",		/*	85ed 	*/
		"ダ",		/*	85ee 	*/
		"ヂ",		/*	85ef 	*/
		"ヅ",		/*	85f0 	*/
		"デ",		/*	85f1 	*/
		"ド",		/*	85f2 	*/
		"バ",		/*	85f3 	*/
		"パ",		/*	85f4 	*/
		"ビ",		/*	85f5 	*/
		"ピ",		/*	85f6 	*/
		"ブ",		/*	85f7 	*/
		"プ",		/*	85f8 	*/
		"ベ",		/*	85f9 	*/
		"ペ",		/*	85fa 	*/
		"ボ",		/*	85fb 	*/
		"ポ"		/*	85fc 	*/
	};

	if((src2 = src3 = (char *)malloc(strlen(src) + 2)) == NULL) {
		return NULL;
	};

	strcpy(src2, src);
	src2[strlen(src2)+1] = '\0';		// 2バイト連続 '\0' にする

	dest2 = dest;
	do {
		if(_ismbblead(*(uchar *)src2)) {		// 漢字１バイト目
			if(*(uchar *)src2 == 0x85 &&
					*(uchar *)(src2+1) >= 0x40 && *(uchar *)(src2+1) <= 0xfc) {	// 2バイト半角
				len = (int)strlen(codetable[*(uchar *)(src2+1) - 0x40]);
				strncpy(dest, codetable[*(uchar *)(src2+1) - 0x40], len);
				src2 += 2;
				dest += len;
			} else {
				*dest++ = *src2++;
				*dest++ = *src2++;
			}
		} else {
			*dest++ = *src2++;
		}
	}while(*src2 != '\0');

	free(src3);
	if(strlen(dest2) > 0) {
		if(*(dest - 1) != '\0') {
			*dest = '\0';
		}
	}
	return dest2;
}
