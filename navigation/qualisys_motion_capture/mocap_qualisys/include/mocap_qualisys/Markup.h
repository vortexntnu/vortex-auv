#ifndef QUALISYS_NBC_NBC_MARKUP_H_INCLUDED
#define QUALISYS_NBC_NBC_MARKUP_H_INCLUDED

// Markup.h: interface for the NBC_CMarkup class.
//
// NBC_CMarkup Release 6.5 Lite
// Copyright (C) 1999-2003 First Objective Software, Inc. All rights reserved
// This entire notice must be retained in this source code
// Redistributing this source code requires written permission
// This software is provided "as is", with no warranty.
// Latest fixes enhancements and documentation at www.firstobject.com

#include <vector>
#include <string>

#ifdef _DEBUG
#define _DS(i) (i?&((const char*)m_csDoc.c_str())[m_aPos[i].nStartL]:0)
#define MARKUP_SETDEBUGSTATE m_pMainDS=_DS(m_iPos); m_pChildDS=_DS(m_iPosChild)
#else
#define MARKUP_SETDEBUGSTATE
#endif

class CMarkup  
{
public:
	CMarkup() { SetDoc(NULL); mnIndent = 4; };
	CMarkup(const char* szDoc) { SetDoc(szDoc); };
	CMarkup(const CMarkup& markup) { *this = markup; };
	void operator=(const CMarkup& markup);
	virtual ~CMarkup() {};

    // Settings
    void SetIndent(int nIndent = 4);

	// Create
	std::string GetDoc() const { return m_csDoc; };
	bool AddElem(const char* szName, const char* szData=NULL) { return x_AddElem(szName,szData,false,false); };
	bool AddChildElem(const char* szName, const char* szData=NULL) { return x_AddElem(szName,szData,false,true); };
	bool AddAttrib(const char* szAttrib, const char* szValue) { return x_SetAttrib(m_iPos,szAttrib,szValue); };
	bool AddChildAttrib(const char* szAttrib, const char* szValue) { return x_SetAttrib(m_iPosChild,szAttrib,szValue); };
	bool SetAttrib(const char* szAttrib, const char* szValue) { return x_SetAttrib(m_iPos,szAttrib,szValue); };
	bool SetChildAttrib(const char* szAttrib, const char* szValue) { return x_SetAttrib(m_iPosChild,szAttrib,szValue); };

	// Navigate
	bool SetDoc(const char* szDoc);
	bool IsWellFormed();
	bool FindElem(const char* szName=NULL);
	bool FindChildElem(const char* szName=NULL);
	bool IntoElem();
	bool OutOfElem();
	void ResetChildPos() { x_SetPos(m_iPosParent,m_iPos,0); };
	void ResetMainPos() { x_SetPos(m_iPosParent,0,0); };
	void ResetPos() { x_SetPos(0,0,0); };
	std::string GetTagName() const;
	std::string GetChildTagName() const { return x_GetTagName(m_iPosChild); };
	std::string GetData() const { return x_GetData(m_iPos); };
	std::string GetChildData() const { return x_GetData(m_iPosChild); };
	std::string GetAttrib(const char* szAttrib) const { return x_GetAttrib(m_iPos,szAttrib); };
	std::string GetChildAttrib(const char* szAttrib) const { return x_GetAttrib(m_iPosChild,szAttrib); };
	std::string GetError() const { return m_csError; };

	static std::string Format(const char *fmt, ...);

	enum MarkupNodeType
	{
		MNT_ELEMENT					= 1,  // 0x01
		MNT_TEXT					= 2,  // 0x02
		MNT_WHITESPACE				= 4,  // 0x04
		MNT_CDATA_SECTION			= 8,  // 0x08
		MNT_PROCESSING_INSTRUCTION	= 16, // 0x10
		MNT_COMMENT					= 32, // 0x20
		MNT_DOCUMENT_TYPE			= 64, // 0x40
		MNT_EXCLUDE_WHITESPACE		= 123,// 0x7b
	};

protected:

#ifdef _DEBUG
	const char* m_pMainDS;
	const char* m_pChildDS;
#endif

	std::string m_csDoc;
	std::string m_csError;

	struct ElemPos
	{
		ElemPos() { Clear(); };
		ElemPos(const ElemPos& pos) { *this = pos; };
		bool IsEmptyElement() const { return (nStartR == nEndL + 1); };
		void Clear()
		{
			nStartL=0; nStartR=0; nEndL=0; nEndR=0; nReserved=0;
			iElemParent=0; iElemChild=0; iElemNext=0;
		};
		void AdjustStart(int n) { nStartL+=n; nStartR+=n; };
		void AdjustEnd(int n) { nEndL+=n; nEndR+=n; };
		int nStartL;
		int nStartR;
		int nEndL;
		int nEndR;
		int nReserved;
		int iElemParent;
		int iElemChild;
		int iElemNext;
	};

    std::vector<ElemPos> m_aPos;
	int m_iPosParent;
	int m_iPos;
	int m_iPosChild;
	int m_iPosFree;
	int m_nNodeType;

	struct TokenPos
	{
		TokenPos(const char* sz) { Clear(); szDoc = sz; };
		bool IsValid() const { return (nL <= nR); };
		void Clear() { nL=0; nR=-1; nNext=0; bIsString=false; };
		bool Match(const char* szName) const;
		int nL;
		int nR;
		int nNext;
		const char* szDoc;
		bool bIsString;
	};

	void x_SetPos(int iPosParent, int iPos, int iPosChild)
	{
		m_iPosParent = iPosParent;
		m_iPos = iPos;
		m_iPosChild = iPosChild;
		m_nNodeType = iPos?MNT_ELEMENT:0;
		MARKUP_SETDEBUGSTATE;
	};

	int x_GetFreePos();
	int x_ReleasePos();
	int x_ParseElem(int iPos);
	int x_ParseError(const char* szError, const char* szName = NULL);
	static bool x_FindChar(const char* szDoc, int& nChar, char c);
	static bool x_FindAny(const char* szDoc, int& nChar);
	static bool x_FindToken(TokenPos& token);
	std::string x_GetToken(const TokenPos& token) const;
	int x_FindElem(int iPosParent, int iPos, const char* szPath);
	std::string x_GetTagName(int iPos) const;
	std::string x_GetData(int iPos) const;
	std::string x_GetAttrib(int iPos, const char* szAttrib) const;
	bool x_AddElem(const char* szName, const char* szValue, bool bInsert, bool bAddChild);
	bool x_FindAttrib(TokenPos& token, const char* szAttrib=NULL) const;
	bool x_SetAttrib(int iPos, const char* szAttrib, const char* szValue);
	void x_LocateNew(int iPosParent, int& iPosRel, int& nOffset, int nLength, int nFlags);
	int x_ParseNode(TokenPos& token);
	void x_DocChange(int nLeft, int nReplace, const std::string& csInsert);
	void x_Adjust(int iPos, int nShift, bool bAfterPos = false);
	std::string x_TextToDoc(const char* szText, bool bAttrib = false) const;
	std::string x_TextFromDoc(int nLeft, int nRight) const;

protected:
    char mtIndent[ 1000 ];
    int  mnIndent;

private:
    std::string Mid(const std::string &tStr, int nFirst) const;
    std::string Mid(const std::string &tStr, int nFirst, int nCount) const;
    char*       GetBuffer(std::string &tStr, int nMinLen = -1) const;
    void        ReleaseBuffer(std::string &tStr, int nNewLen = -1) const;
};

#endif /* QUALISYS_NBC_NBC_MARKUP_H_INCLUDED */
